"""Compact current-plant controller comparison; preserves all frozen studies.

The root geometry/engine contract is required for development or evaluation.
The source-bound soft-pad pilot can run before that geometry is decided.
No bulk arrays, original compile stamps, or original controller files are written.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
import ctypes as ct
from datetime import datetime, timezone
import hashlib
import importlib.util
import itertools
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OLD = ROOT / "acf_simulink/experiments/sea_pad_dynamic_20261006"
OLDVAL = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006"
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/control"
FORECAST = OUT.parent / "storage_forecast.json"
PLAN = HERE / "controller_plan.json"
ROOT_CONTRACT_HASH = "0936fd73866223e0a1f8c884f8ee2eed28627e3882b78adf9127eafff57e521f"
ROOT_CONTRACT = HERE.parent / "study_contract.json"
INITIAL_BANKS = HERE.parent / "initial_banks.json"

_spec = importlib.util.spec_from_file_location("matrix_control_engine_abi", OLD / "engine_api.py")
ABI = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(ABI)

SOURCE_HASHES = {
    "engine.c": "be43185291e39e6348efd5fb687482db29615a13e516fc52cc0bc306ed3de4a5",
    "engine_api.py": "4bc06898fef060577454f0139f2b3012cf6dc81af40ca17a02ceb13af78630c9",
    "study_contract.json": "c2077261ffff69e10682ea524577eb669d2f47641e02154358043ba3dc3e26b6",
    "pad5mm_contract.json": "674b1217279f8a9ca689f55afbcecbd6640b63cf45c4240f63b2549074a5efff",
}
OLD_NAMES = {0: "coil_pad5mm_k50_c0", 1: "sea_pad5mm_k50_c0"}
CONTROL_FIELDS = {
    "force_kp", "force_ki", "velocity_gain", "inertia_gain", "inertia_cutoff",
    "velocity_leak", "force_feedback_cutoff", "servo_hz", "servo_zeta",
    "servo_target_cutoff", "servo_velocity_feedforward",
}
COHORT_FIELDS = ("mass_s", "identified_mass_s", "force_target", "angle_deg", "seed")
METRIC_COLUMNS = ABI.METRIC_FIELDS + [
    "stop_reason", "force_pass", "completed", "static_current_A",
    "continuous_static_current_pass", "continuous_rms_current_pass",
    "copper_loss_average_W", "actuator_ratings_pass", "sensor_ratings_pass",
    "physical_pass", "eligibility_pass", "force_and_eligibility_pass",
    "one_sided_extra_limit_count", "scored_integration_endpoint_count",
]


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def object_sha(value: dict) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def dump_new(path: Path, value: dict) -> None:
    """Immutable reruns reuse exact content; different content needs a new name."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        if read(path) != value:
            raise RuntimeError(f"Preserve existing evidence: {path} differs.")
        return
    path.write_text(json.dumps(value, sort_keys=True, separators=(",", ":"),
                               allow_nan=False) + "\n")


def retained_bytes() -> int:
    return sum(p.stat().st_size for folder in (HERE, OUT)
               for p in folder.rglob("*") if p.is_file() and "tmp" not in p.parts)


def budget_check(expected_increment: int = 0) -> dict:
    allocation = read(FORECAST)["allocations"]["controller"]
    used = retained_bytes()
    if used + expected_increment > allocation["retained"]:
        raise RuntimeError(f"Controller allocation exceeded: {used}+{expected_increment} > "
                           f"{allocation['retained']} bytes")
    free = shutil.disk_usage(OUT.parent).free
    if free < allocation["temporary"] + expected_increment:
        raise RuntimeError("Insufficient space for the declared controller batch.")
    return {"retained_bytes": used, "retained_cap_bytes": allocation["retained"],
            "temporary_cap_bytes": allocation["temporary"], "filesystem_free_bytes": free,
            "new_bulk_array_bytes": 0, "forecast_sha256": sha(FORECAST)}


def verify_original_inputs() -> None:
    for name, expected in SOURCE_HASHES.items():
        if sha(OLD / name) != expected:
            raise RuntimeError(f"Frozen input changed: {OLD / name}")


def inherited_config(actuator: int) -> tuple[dict, dict]:
    verify_original_inputs()
    path = OLDVAL / "development" / f"{OLD_NAMES[actuator]}_lock.json"
    frozen = read(path)
    source = {"path": str(path.relative_to(ROOT)), "sha256": sha(path),
              "name": frozen["name"]}
    return {**frozen["base_config"], **frozen["gains"]}, source


@contextmanager
def fork_private_engine(source: Path, expected_sha256: str):
    """Use the reviewed fork wrapper, including all rigid-event proof fields."""
    binding = read(OUT / "engine_source_binding.json")
    api_path = HERE.parent / "engine_api.py"
    if sha(source) != expected_sha256 or sha(api_path) != binding["api_sha256"]:
        raise RuntimeError("Reviewed current-plant source or API changed.")
    OUT.mkdir(parents=True, exist_ok=True)
    budget_check()
    temp_parent = OUT / "tmp"
    temp_parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="fork_", dir=temp_parent) as directory:
        tmp = Path(directory)
        (tmp / "engine.c").write_bytes(source.read_bytes())
        module_spec = importlib.util.spec_from_file_location("matrix_control_current_api", api_path)
        api = importlib.util.module_from_spec(module_spec)
        assert module_spec.loader is not None
        module_spec.loader.exec_module(api)
        api.HERE = tmp
        api.OUT = tmp / "build"
        lib = api.compile_engine()
        lib._matrix_api = api
        private_files = list(tmp.rglob("*"))
        binary = Path(lib._name)
        if binary not in private_files:
            private_files.append(binary)
        used = sum(p.stat().st_size for p in private_files if p.is_file())
        metadata = {"source_path": str(source.relative_to(ROOT)), "source_sha256": expected_sha256,
                    "api_path": str(api_path.relative_to(ROOT)), "api_sha256": sha(api_path),
                    "binary_sha256": sha(binary), "gcc_version": subprocess.run(
                        ["gcc", "-dumpfullversion"], capture_output=True, text=True,
                        check=True).stdout.strip(),
                    "flags": ["-O3", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared"],
                    "parameter_bytes": ct.sizeof(api.Parameters), "metric_bytes": ct.sizeof(api.Metrics),
                    "temporary_bytes": used}
        if (ct.sizeof(api.Parameters) != ct.sizeof(ABI.Parameters) or
                ct.sizeof(api.Metrics) != ct.sizeof(ABI.Metrics)):
            api.cleanup_engine(lib)
            raise RuntimeError("Reviewed fork changed the parameter/metric ABI sizes.")
        if used > read(FORECAST)["allocations"]["controller"]["temporary"]:
            api.cleanup_engine(lib)
            raise RuntimeError("Fork compilation exceeded its temporary allocation.")
        try:
            yield lib, metadata
        finally:
            api.cleanup_engine(lib)
    try:
        temp_parent.rmdir()
    except OSError:
        pass


@contextmanager
def private_engine(source: Path = OLD / "engine.c", expected_sha256: str | None = None):
    """Compile original source privately and verify its exposed struct sizes."""
    verify_original_inputs()
    source = source.resolve()
    digest = sha(source)
    if expected_sha256 is not None and digest != expected_sha256:
        raise RuntimeError("Selected engine source differs from the root contract.")
    if source != (OLD / "engine.c").resolve():
        with fork_private_engine(source, digest) as result:
            yield result
        return
    OUT.mkdir(parents=True, exist_ok=True)
    budget_check()
    temp_parent = OUT / "tmp"
    temp_parent.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="engine_", dir=temp_parent) as directory:
        tmp = Path(directory)
        wrapper = tmp / "abi_wrapper.c"
        # JSON quoting is valid for this C include string; no shell interpolation.
        wrapper.write_text('#include ' + json.dumps(str(source)) + '\n'
                           'size_t matrix_parameter_bytes(void){return sizeof(Parameters);}\n'
                           'size_t matrix_metric_bytes(void){return sizeof(Metrics);}\n')
        binary = tmp / "engine.so"
        cmd = ["gcc", "-O3", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC",
               "-shared", str(wrapper), "-lm", "-o", str(binary)]
        proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if proc.returncode:
            raise RuntimeError(proc.stdout + proc.stderr)
        lib = ct.CDLL(str(binary))
        for name, expected in (("matrix_parameter_bytes", ct.sizeof(ABI.Parameters)),
                               ("matrix_metric_bytes", ct.sizeof(ABI.Metrics))):
            fn = getattr(lib, name)
            fn.restype = ct.c_size_t
            if fn() != expected:
                raise RuntimeError(f"Read-only Python ABI differs from C: {name}")
        lib.engine_simulate.argtypes = [ct.POINTER(ABI.Parameters), ct.POINTER(ABI.Metrics),
                                       ct.POINTER(ct.c_double), ct.c_int, ct.c_int]
        lib.engine_simulate.restype = ct.c_int
        lib.engine_initialize.argtypes = [ct.POINTER(ABI.Parameters), ct.POINTER(ct.c_double),
                                         ct.POINTER(ct.c_double)]
        lib.engine_rhs.argtypes = [ct.POINTER(ABI.Parameters), ct.c_double,
                                  ct.POINTER(ct.c_double), ct.c_double, ct.c_double,
                                  ct.POINTER(ct.c_double), ct.POINTER(ct.c_double)]
        metadata = {"source_path": str(source.relative_to(ROOT)), "source_sha256": digest,
                    "binary_sha256": sha(binary), "gcc_version": subprocess.run(
                        ["gcc", "-dumpfullversion"], capture_output=True, text=True,
                        check=True).stdout.strip(), "flags": cmd[1:8],
                    "parameter_bytes": ct.sizeof(ABI.Parameters),
                    "metric_bytes": ct.sizeof(ABI.Metrics),
                    "temporary_bytes": sum(p.stat().st_size for p in tmp.iterdir())}
        if metadata["temporary_bytes"] > read(FORECAST)["allocations"]["controller"]["temporary"]:
            raise RuntimeError("Private compilation exceeded its temporary allocation.")
        yield lib, metadata
    # Remove the empty, owned temporary parent; concurrent users retain their folders.
    try:
        temp_parent.rmdir()
    except OSError:
        pass


def preparation_rejection(p: dict) -> str | None:
    g = p["gravity"] * math.cos(math.radians(p["angle_deg"]))
    fa = p["force_target"] - (p["mass_b"] + p["mass_s"]) * g
    zs = 0 if p["pad_k"] == 0 else -p["force_target"] / p["pad_k"]
    zb = zs - (p["force_target"] - p["mass_s"] * g) / p["cell_k"]
    za = (fa - (p["mass_a"] + (p["nut_mass"] if p["actuator"] else 0)) * g) / p["upper_k"]
    u = za - zb
    if abs(u) > p["tool_travel_limit"]:
        return "initial_flange_stroke"
    if p["pad_compression_limit"] and -zs > p["pad_compression_limit"]:
        return "initial_pad_compression"
    if p["actuator"]:
        q = u + fa / p["sea_k"]
        if abs(q) > p["nut_travel_limit"]:
            return "initial_nut_stroke"
        if abs(fa / p["sea_k"]) > p["spring_deflection_limit"]:
            return "initial_spring_deflection"
    current = fa / p["force_constant"] if not p["actuator"] else (
        (fa - p["nut_mass"] * g) * p["lead"] / (2 * math.pi * p["force_constant"]))
    if abs(current) > p["current_limit"]:
        return "initial_peak_current"
    if abs(current * p["resistance"]) > p["voltage_limit"]:
        return "initial_voltage"
    return None


def validate_parameters(p: dict) -> None:
    ABI.make_parameters(p)
    expected = {"mass_a": 1.5, "mass_b": .6, "upper_k": 20000, "cell_k": 64000000,
                "robot_enabled": 0, "external_current": 0, "external_robot_amplitude": 0,
                "sea_k": 5000, "sea_c": 0, "duration": 2, "noise_enabled": 1}
    if p["actuator"] == 1:
        expected.update(lead=.01, nut_mass=.075, rotor_inertia=7.04e-6)
    for key, value in expected.items():
        if p[key] != value:
            raise ValueError(f"Current-plant contract violation: {key}={p[key]} != {value}")


def run_case(lib, p: dict, identity: str, case: dict, family: int | None = None) -> dict:
    validate_parameters(p)
    row = {"id": identity, "case": case, "config_sha256": object_sha(p)}
    if family is not None:
        row["family"] = family
    rejection = preparation_rejection(p)
    if rejection:
        return {**row, "status": "preparation_rejected", "reason": rejection}
    try:
        metrics, trace = getattr(lib, "_matrix_api", ABI).simulate(lib, p)
        if len(trace):
            raise AssertionError("Bulk traces are disabled in the compact controller runner.")
    except ValueError as exc:
        return {**row, "status": "execution_rejected", "reason": str(exc)}
    if any(isinstance(v, float) and not math.isfinite(v) for v in metrics.values()):
        metrics = {k: (str(v) if isinstance(v, float) and not math.isfinite(v) else v)
                   for k, v in metrics.items()}
        return {**row, "status": "nonfinite_execution", "metrics": metrics}
    metrics["physical_pass"] = metrics["completed"]
    metrics["eligibility_pass"] = bool(metrics["completed"] and metrics["sensor_ratings_pass"]
        and metrics["actuator_ratings_pass"] and metrics["continuous_static_current_pass"] is not False
        and metrics["continuous_rms_current_pass"] is not False)
    metrics["force_and_eligibility_pass"] = metrics["force_pass"] and metrics["eligibility_pass"]
    return {**row, "status": "executed", "metrics": metrics}


def read_contract(path: Path) -> dict:
    spec = read(path)
    if (sha(path) != ROOT_CONTRACT_HASH or
            spec.get("status") != "declared_before_new_hard_execution"):
        raise RuntimeError("Use the exact root contract declared before hard execution.")
    binding_path = OUT / "engine_source_binding.json"
    if not binding_path.exists():
        raise RuntimeError("The engine worker's reviewed source/API hashes must be bound before execution.")
    binding = read(binding_path)
    if binding["contract_sha256"] != sha(path):
        raise RuntimeError("The reviewed engine belongs to a different root contract.")
    spec["engine"] = {"source": spec["expected_private_source_paths"]["engine"],
                      "sha256": binding["engine_source_sha256"],
                      "api": spec["expected_private_source_paths"]["api"],
                      "api_sha256": binding["api_sha256"]}
    if sha(ROOT / spec["engine"]["api"]) != spec["engine"]["api_sha256"]:
        raise RuntimeError("The contracted current-plant API changed.")
    if sha(ROOT / spec["engine"]["source"]) != spec["engine"]["sha256"]:
        raise RuntimeError("The contracted plant source changed.")
    # Normalize the root's field names without changing its frozen document.
    root_ev = spec["evaluation"]
    ev = {"mass_s_kg": root_ev["mass_s"], "force_N": root_ev["force_target"],
          "orientation_deg": root_ev["angle_deg"], "noise_seeds": root_ev["seeds"]}
    spec["evaluation"] = ev
    spec["geometry"] = {
        "soft": {"definition": spec["contacts"]["soft"]["geometry"],
                 "delta": {"pad_k": 50000, "pad_c": 0, "pad_diameter": .077,
                           "pad_thickness": .005, "pad_compression_limit": .0025, "pad_points": 61}},
        "no_pad": {"definition": spec["contacts"]["rigid_plate"]["geometry"],
                   "law": spec["contacts"]["rigid_plate"]["law"],
                   "force_scoring": spec["contacts"]["rigid_plate"]["force_scoring"],
                   "delta": {"pad_k": 0, "pad_c": 0, "pad_diameter": .077,
                             "pad_thickness": 0, "pad_compression_limit": 0, "pad_points": 61}}}
    root_dev = spec["development"]
    spec["development"] = {"noise_seed": root_dev["seed"],
                            "corners": [[c["mass_s"], c["force_target"], c["angle_deg"]]
                                        for c in root_dev["cases_per_family"]]}
    if (ev["mass_s_kg"], ev["force_N"], ev["orientation_deg"]) != (
            [.1, 1.4, 2], [5, 20, 50], [0, 45, 90]):
        raise ValueError("A 54-case current-plant nominal grid is required.")
    seeds = ev["noise_seeds"]
    if len(seeds) != 2 or len(set(seeds)) != 2 or set(seeds) & {
            260600, 260601, 260602, 260611, 260612, 260630, 260651, 260652,
            spec["development"]["noise_seed"]}:
        raise ValueError("Final seeds must be fresh and distinct from development.")
    if spec["geometry"]["soft"]["delta"].get("pad_k") != 50000:
        raise ValueError("Soft branch must retain the current 50 kN/m pad assumption.")
    if spec["geometry"]["no_pad"]["delta"].get("pad_k") != 0:
        raise ValueError("No-pad branch must be the reviewed rigid-contact branch.")
    return spec


def bind_engine(contract_path: Path, engine_source_sha256: str, api_sha256: str) -> dict:
    """Bind reviewed worker hashes before any new rigid execution."""
    if sha(contract_path) != ROOT_CONTRACT_HASH:
        raise RuntimeError("The root geometry contract changed.")
    spec = read(contract_path)
    sources = spec["expected_private_source_paths"]
    if sha(ROOT / sources["engine"]) != engine_source_sha256:
        raise RuntimeError("Engine-worker source hash differs from the supplied reviewed hash.")
    if sha(ROOT / sources["api"]) != api_sha256:
        raise RuntimeError("Engine-worker API hash differs from the supplied reviewed hash.")
    readiness_path = OUT.parent / "plant/readiness.json"
    readiness = read(readiness_path)
    if (readiness["status"] != "ready_for_bounded_development_and_native_replay" or
            readiness["engine_source_sha256"] != engine_source_sha256 or
            readiness["api_sha256"] != api_sha256 or readiness["contract_sha256"] != sha(contract_path) or
            sha(ROOT / readiness["checks_path"]) != readiness["checks_sha256"]):
        raise RuntimeError("Passed plant readiness does not bind the selected source/API/contract.")
    api_spec = importlib.util.spec_from_file_location("matrix_reviewed_abi", ROOT / sources["api"])
    module = importlib.util.module_from_spec(api_spec)
    assert api_spec.loader is not None
    api_spec.loader.exec_module(module)
    for field in ("FIELDS", "METRIC_FIELDS", "TRACE_COLUMNS"):
        if getattr(module, field) != getattr(ABI, field):
            raise RuntimeError(f"Engine fork changed the read-only ABI: {field}")
    binding = {"status": "reviewed_engine_bound_before_hard_execution",
               "contract_sha256": sha(contract_path), "engine_source_sha256": engine_source_sha256,
               "api_sha256": api_sha256, "initial_banks_sha256": sha(INITIAL_BANKS),
               "plant_readiness": {"path": str(readiness_path.relative_to(ROOT)), "sha256": sha(readiness_path)},
               "abi": {"parameter_bytes": ct.sizeof(ABI.Parameters),
                       "metric_bytes": ct.sizeof(ABI.Metrics)},
               "source_scope": "Analytic rigid77mm envelope and transition-side scoring; positive-k plant preserved."}
    dump_new(OUT / "engine_source_binding.json", binding)
    return binding


def root_config(actuator: int, branch: str, spec: dict, gains_delta: dict | None = None) -> dict:
    if branch not in {"soft", "no_pad"}:
        raise ValueError(branch)
    if gains_delta is not None and set(gains_delta) - CONTROL_FIELDS:
        raise ValueError("Only declared causal controller gains can change during development.")
    p, _ = inherited_config(actuator)
    p.update(spec["geometry"][branch]["delta"])
    p.update(gains_delta or {})
    validate_parameters(p)
    return p


def nominal_cases(spec: dict) -> list[dict]:
    ev = spec["evaluation"]
    return [{"mass_s": m, "force_target": f, "angle_deg": a, "seed": s}
            for m, f, a, s in itertools.product(ev["mass_s_kg"], ev["force_N"],
                                               ev["orientation_deg"], ev["noise_seeds"])]


def parameters_for_case(base: dict, case: dict) -> dict:
    return {**base, **case, "identified_mass_s": case["mass_s"]}


def journal_rows(path: Path) -> list[dict]:
    """Decode compact scalar metric columns; no physical histories are stored."""
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        row = json.loads(line)
        if "metric_values" in row:
            values = row.pop("metric_values")
            if len(values) != len(METRIC_COLUMNS):
                raise RuntimeError("Compact metric schema differs from this runner.")
            row["metrics"] = {k: v for k, v in zip(METRIC_COLUMNS, values) if v is not None or
                              k in {"continuous_static_current_pass", "continuous_rms_current_pass"}}
            row["metrics"].update(row.pop("metric_extra", {}))
        rows.append(row)
    return rows


def compact_record(row: dict) -> dict:
    if "metrics" not in row:
        return row
    result = {k: v for k, v in row.items() if k != "metrics"}
    result["metric_values"] = [row["metrics"].get(k) for k in METRIC_COLUMNS]
    extra = {k: v for k, v in row["metrics"].items() if k not in METRIC_COLUMNS}
    if extra:
        result["metric_extra"] = extra
    return result


def gain_families(actuator: int) -> list[dict]:
    """Fixed 18-family bank; declared before any hard-branch development."""
    if actuator == 0:
        def g(kp=1, ff=1, cut=250, ki=0, vg=0, fcut=250):
            return dict(force_kp=kp, force_ki=ki, velocity_gain=vg,
                        inertia_gain=ff, inertia_cutoff=cut, force_feedback_cutoff=fcut)
        return [{}, g(kp=0, ff=0), g(kp=0, cut=80),
                *[g(kp=k, cut=c) for k, c in itertools.product([.25, 1, 3], [80, 250, 500])],
                g(ff=.8), g(ff=1.2), g(ki=20), g(vg=80),
                g(kp=3, cut=500, vg=80, fcut=500), g(kp=5)]
    if actuator != 1:
        raise ValueError(actuator)
    def g(kp=.5, ff=1, servo=60, ki=0, target=300, vg=0):
        return dict(force_kp=kp, force_ki=ki, velocity_gain=vg, inertia_gain=ff,
                    inertia_cutoff=250, servo_hz=servo, servo_zeta=1,
                    servo_target_cutoff=target, servo_velocity_feedforward=2,
                    force_feedback_cutoff=250)
    return [{}, g(kp=0, ff=0),
            *[g(kp=k, ff=f, servo=s) for s, k, f in
              itertools.product([30, 60, 100], [.5, 2], [.5, 1])],
            g(kp=1, servo=160), g(kp=1, servo=100, ki=5),
            g(kp=1, servo=100, target=100), g(kp=1, vg=50)]


def run_batch(lib, requests: list[tuple[dict, str, dict, int | None]], journal: Path,
              workers: int = 4) -> list[dict]:
    """Flush every completed failed or successful trial to one compact journal."""
    if workers < 1 or workers > 4:
        raise ValueError("At most four independent streaming C workers are allocated.")
    previous = journal_rows(journal)
    indexed = {row["id"]: row for row in previous}
    if len(indexed) != len(previous):
        raise RuntimeError("Duplicate retained recording identity.")
    pending = []
    expected_ids = {identity for _, identity, _, _ in requests}
    if len(expected_ids) != len(requests) or set(indexed) - expected_ids:
        raise RuntimeError("Retained batch identities do not match the current batch.")
    for p, identity, case, family in requests:
        if identity in indexed:
            if indexed[identity]["config_sha256"] != object_sha(p):
                raise RuntimeError("A retained recording has different source parameters.")
        else:
            pending.append((p, identity, case, family))
    budget_check(expected_increment=2100 * len(pending))
    journal.parent.mkdir(parents=True, exist_ok=True)
    dump_new(OUT / "metric_schema.json", {"schema_version": 1, "metric_columns": METRIC_COLUMNS,
             "description": "Compact scalar summaries of all scored endpoints, not saved physical-history arrays; journal_rows decodes metric_values into named metrics. metric_extra retains transition-event proof."})
    with journal.open("a") as stream, ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case, lib, *request): request[1] for request in pending}
        for future in as_completed(futures):
            row = future.result()
            stream.write(json.dumps(compact_record(row), sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n")
            stream.flush()
            indexed[row["id"]] = row
            print(f"{row['id']} {row['status']} "
                  f"{row.get('metrics', {}).get('force_peak_error', row.get('reason', ''))}", flush=True)
    budget_check()
    return [indexed[identity] for _, identity, _, _ in requests]


def summarize(rows: list[dict]) -> dict:
    executed = [r for r in rows if r["status"] == "executed"]
    metrics = [r["metrics"] for r in executed]
    failures: dict[str, int] = {}
    for row in rows:
        reason = row["metrics"]["stop_reason"] if row["status"] == "executed" else row["status"]
        failures[reason] = failures.get(reason, 0) + 1
    return {"planned_count": len(rows), "executed_count": len(executed),
            "preparation_rejected_count": sum(r["status"] == "preparation_rejected" for r in rows),
            "execution_rejected_count": sum(r["status"] == "execution_rejected" for r in rows),
            "nonfinite_execution_count": sum(r["status"] == "nonfinite_execution" for r in rows),
            "integration_endpoints_scored": int(sum(m.get("scored_integration_endpoint_count", m["steps"]) for m in metrics)),
            "one_sided_extra_limits_scored": int(sum(m.get("one_sided_extra_limit_count", 0) for m in metrics)),
            "force_pass_count": sum(m["force_pass"] for m in metrics),
            "force_and_eligibility_pass_count": sum(m["force_and_eligibility_pass"] for m in metrics),
            "physical_stop_count": sum(not m["completed"] for m in metrics),
            "peak_current_or_speed_failure_count": sum(not m["actuator_ratings_pass"] for m in metrics),
            "sensor_range_failure_count": sum(not m["sensor_ratings_pass"] for m in metrics),
            "motor_continuous_rms_failure_count": sum(m["continuous_rms_current_pass"] is False for m in metrics),
            "motor_continuous_static_failure_count": sum(m["continuous_static_current_pass"] is False for m in metrics),
            "worst_force_error_N": max((m["force_peak_error"] for m in metrics), default=None),
            "maximum_current_rms_A": max((m["current_rms"] for m in metrics), default=None),
            "maximum_average_hot_copper_loss_W": max((m["copper_loss_average_W"] for m in metrics), default=None),
            "maximum_voltage_request_V": max((m["voltage_peak_request"] for m in metrics), default=None),
            "maximum_voltage_clipped_fraction": max((m["voltage_clipped_fraction"] for m in metrics), default=None),
            "maximum_centering_band_fraction": max((m["centering_band_fraction"] for m in metrics), default=None),
            "stop_and_rejection_counts": failures}


def selection_score(rows: list[dict]) -> tuple:
    result = summarize(rows)
    ineligible = len(rows) - sum(r["status"] == "executed" and r["metrics"]["eligibility_pass"] for r in rows)
    force_failures = len(rows) - result["force_pass_count"]
    return (ineligible, force_failures, result["worst_force_error_N"]
            if result["worst_force_error_N"] is not None else 1e30,
            result["maximum_average_hot_copper_loss_W"] or 0)


def make_lock(name: str, actuator: int, branch: str, contract_path: Path,
              gains_delta: dict | None = None, development: dict | None = None) -> dict:
    spec = read_contract(contract_path)
    p = root_config(actuator, branch, spec, gains_delta)
    _, inherited = inherited_config(actuator)
    lock = {"schema_version": 1, "name": name, "actuator": actuator, "branch": branch,
            "status": "controller_locked_before_final_evaluation", "locked_at_utc": datetime.now(timezone.utc).isoformat(),
            "root_contract": {"path": str(contract_path.resolve().relative_to(ROOT)), "sha256": sha(contract_path)},
            "source_controller_lock": inherited, "geometry": spec["geometry"][branch],
            "controller_delta": gains_delta or {}, "base_parameters_sha256": object_sha(p),
            "engine": spec["engine"], "evaluation_seeds": spec["evaluation"]["noise_seeds"],
            "method": "predeclared_bounded_development" if development else "unchanged_current_soft_pad_controller",
            "development": development,
            "qualification": "Finite axial-force development comparison; no engagement, recentering, thermal or hardware qualification."}
    lock["lock_sha256"] = object_sha(lock)
    return lock


def lock_parameters(lock: dict) -> tuple[dict, dict]:
    expected = lock["lock_sha256"]
    content = {k: v for k, v in lock.items() if k != "lock_sha256"}
    if object_sha(content) != expected:
        raise RuntimeError("Controller lock content changed.")
    contract_path = ROOT / lock["root_contract"]["path"]
    if sha(contract_path) != lock["root_contract"]["sha256"]:
        raise RuntimeError("Root contract changed after the controller lock.")
    source = ROOT / lock["source_controller_lock"]["path"]
    if sha(source) != lock["source_controller_lock"]["sha256"]:
        raise RuntimeError("Inherited source controller changed after locking.")
    spec = read_contract(contract_path)
    p = root_config(lock["actuator"], lock["branch"], spec, lock["controller_delta"])
    if object_sha(p) != lock["base_parameters_sha256"]:
        raise RuntimeError("Reconstructed parameters differ from the controller lock.")
    return p, spec


def develop(actuator: int, contract_path: Path, workers: int = 4) -> dict:
    spec = read_contract(contract_path)
    name = "coil_rigid_plate" if actuator == 0 else "screw_rigid_plate"
    if any(OUT.glob("*_evaluation_summary.json")):
        raise RuntimeError("No development after final outcomes; declare a new study instead.")
    summary_path = OUT / f"{name}_development_summary.json"
    if summary_path.exists():
        return read(summary_path)
    families = gain_families(actuator)
    if len(families) != 18:
        raise AssertionError("The predeclared bank has changed size.")
    cases = [{"mass_s": m, "force_target": f, "angle_deg": a,
              "seed": spec["development"]["noise_seed"]}
             for m, f, a in spec["development"]["corners"]]
    if len(cases) != 4:
        raise ValueError("The bounded development contract requires four declared corners.")
    declaration = {"status": "declared_before_controller_development", "actuator": actuator,
                   "root_contract_sha256": sha(contract_path), "runner_sha256": sha(Path(__file__)),
                   "families": families, "cases": cases, "planned_count": len(families)*len(cases),
                   "selection": "Lexicographic: ineligible-case count, force-failure count, worst full-record force error, largest average hot copper loss. All families and all corners retained."}
    dump_new(OUT / f"{name}_development_declaration.json", declaration)
    requests = []
    for ix, gains in enumerate(families):
        base = root_config(actuator, "no_pad", spec, gains)
        requests.extend((parameters_for_case(base, case), f"{name}_development_g{ix:02d}_c{cx}", case, ix)
                        for cx, case in enumerate(cases))
    started = time.monotonic()
    with private_engine(ROOT / spec["engine"]["source"], spec["engine"]["sha256"]) as (lib, build):
        rows = run_batch(lib, requests, OUT / f"{name}_development_records.jsonl", workers)
    per_family = {ix: [r for r in rows if r["family"] == ix] for ix in range(len(families))}
    selected = min(per_family, key=lambda ix: (*selection_score(per_family[ix]), ix))
    development = {"declaration_path": str((OUT / f"{name}_development_declaration.json").relative_to(ROOT)),
                   "declaration_sha256": sha(OUT / f"{name}_development_declaration.json"),
                   "records_path": str((OUT / f"{name}_development_records.jsonl").relative_to(ROOT)),
                   "records_sha256": sha(OUT / f"{name}_development_records.jsonl"),
                   "selected_family": selected, "selected_summary": summarize(per_family[selected]),
                   "selection_score": selection_score(per_family[selected])}
    lock_path = OUT / f"{name}_lock.json"
    if lock_path.exists():
        lock = read(lock_path)
        lock_parameters(lock)
    else:
        lock = make_lock(name, actuator, "no_pad", contract_path, families[selected], development)
        dump_new(lock_path, lock)
    result = {"status": "executed_bounded_controller_development", "name": name,
              "all_trials": summarize(rows), "selection": development, "lock_path": str(lock_path.relative_to(ROOT)),
              "lock_sha256": sha(lock_path), "build": build, "elapsed_s": time.monotonic()-started,
              "storage": budget_check()}
    dump_new(summary_path, result)
    return result


def selected_banks(contract_path: Path) -> dict:
    """Publish all four source-bound locks before acquisition/final outcomes."""
    path = OUT / "selected_banks.json"
    if path.exists():
        return read(path)
    spec = read_contract(contract_path)
    banks = []
    for name, actuator, branch in (("coil_soft", 0, "soft"), ("screw_soft", 1, "soft"),
                                  ("coil_rigid_plate", 0, "no_pad"), ("screw_rigid_plate", 1, "no_pad")):
        lock_path = OUT / f"{name}_lock.json"
        if not lock_path.exists():
            if branch == "no_pad":
                raise RuntimeError("Both bounded hard developments must finish before publishing selected banks.")
            dump_new(lock_path, make_lock(name, actuator, branch, contract_path))
        lock = read(lock_path)
        config, _ = lock_parameters(lock)
        banks.append({"id": name, "config": config, "config_sha256": object_sha(config),
                      "controller_lock_path": str(lock_path.relative_to(ROOT)),
                      "controller_lock_sha256": sha(lock_path), "controller_delta": lock["controller_delta"],
                      "method": lock["method"], "status": "locked_before_acquisition_and_final_evaluation"})
    result = {"schema_version": 1, "status": "four_banks_locked_before_acquisition_and_final_evaluation",
              "engine_source_sha256": spec["engine"]["sha256"], "api_sha256": spec["engine"]["api_sha256"],
              "contract_sha256": sha(contract_path), "initial_banks_sha256": sha(INITIAL_BANKS),
              "runner_sha256": sha(Path(__file__)), "banks": banks,
              "qualification": False, "storage": budget_check()}
    dump_new(path, result)
    return result


def evaluate(lock_path: Path, workers: int = 4) -> dict:
    lock = read(lock_path)
    base, spec = lock_parameters(lock)
    name = lock["name"]
    summary_path = OUT / f"{name}_evaluation_summary.json"
    if summary_path.exists():
        if read(summary_path)["lock_file_sha256"] != sha(lock_path):
            raise RuntimeError("A final result already exists under a different controller lock.")
        return read(summary_path)
    cases = nominal_cases(spec)
    requests = [(parameters_for_case(base, case), f"{name}_evaluation_{ix:03d}", case, None)
                for ix, case in enumerate(cases)]
    started = time.monotonic()
    with private_engine(ROOT / spec["engine"]["source"], spec["engine"]["sha256"]) as (lib, build):
        journal = OUT / f"{name}_evaluation_records.jsonl"
        rows = run_batch(lib, requests, journal, workers)
    worst = max((r for r in rows if r["status"] == "executed"),
                key=lambda r: r["metrics"]["force_peak_error"], default=None)
    result = {"status": "executed_locked_54_case_axial_development_comparison", "name": name,
              "lock_path": str(lock_path.resolve().relative_to(ROOT)), "lock_file_sha256": sha(lock_path),
              "records_path": str(journal.relative_to(ROOT)), "records_sha256": sha(journal),
              "nominal_grid": spec["evaluation"], "summary": summarize(rows),
              "worst_record_id": worst["id"] if worst else None,
              "by_tool_mass": {str(m): summarize([r for r in rows if r["case"]["mass_s"] == m])
                               for m in spec["evaluation"]["mass_s_kg"]},
              "mechanical_error_floor_N": ({str(m): 1.2*m for m in spec["evaluation"]["mass_s_kg"]}
                                            if lock["branch"] == "no_pad" else None),
              "build": build, "elapsed_s": time.monotonic()-started, "storage": budget_check()}
    dump_new(summary_path, result)
    return result


def final_matrix(contract_path: Path, workers: int = 4) -> dict:
    """All gains and source banks exist before the first fresh final outcome."""
    banks_path = OUT / "selected_banks.json"
    if not banks_path.exists():
        raise RuntimeError("Publish the four selected banks before final evaluation.")
    banks = read(banks_path)
    spec = read_contract(contract_path)
    if banks["contract_sha256"] != sha(contract_path):
        raise RuntimeError("Selected banks belong to a different root contract.")
    groups = ["coil_soft", "screw_soft", "coil_rigid_plate_transfer",
              "screw_rigid_plate_transfer", "coil_rigid_plate", "screw_rigid_plate"]
    for name, actuator in (("coil_rigid_plate_transfer", 0), ("screw_rigid_plate_transfer", 1)):
        path = OUT / f"{name}_lock.json"
        if not path.exists():
            dump_new(path, make_lock(name, actuator, "no_pad", contract_path))
    declaration_path = OUT / "final_matrix_declaration.json"
    declaration = {"status": "all_six_comparisons_locked_before_fresh_final_outcomes",
                   "contract_sha256": sha(contract_path), "selected_banks_sha256": sha(banks_path),
                   "runner_sha256": sha(Path(__file__)), "groups": groups,
                   "controller_lock_sha256": {name: sha(OUT / f"{name}_lock.json") for name in groups},
                   "evaluation": spec["evaluation"], "planned_count": 324}
    dump_new(declaration_path, declaration)
    results = []
    for name in groups:
        results.append(evaluate(OUT / f"{name}_lock.json", workers))
    all_rows = [row for name in groups for row in journal_rows(OUT / f"{name}_evaluation_records.jsonl")]
    expected_cases = {object_sha(c) for c in nominal_cases(spec)}
    for name in groups:
        rows = journal_rows(OUT / f"{name}_evaluation_records.jsonl")
        if (len(rows) != 54 or len({r["id"] for r in rows}) != 54 or
                {object_sha(r["case"]) for r in rows} != expected_cases):
            raise AssertionError(f"Incomplete, duplicate or shifted final coverage: {name}")
        for row in rows:
            _, actual = resolve_record(OUT / f"{name}_lock.json", row["id"])
            if actual != row:
                raise AssertionError("Record reconstruction changed retained metrics.")
    representatives = []
    for bank in banks["banks"]:
        name = bank["id"]
        rows = journal_rows(OUT / f"{name}_evaluation_records.jsonl")
        completed = [r for r in rows if r["status"] == "executed" and r["metrics"]["completed"]]
        if completed:
            selected = max(completed, key=lambda r: r["metrics"]["force_peak_error"])
            representatives.append({"group": name, "identity": selected["id"],
                "controller_lock_path": bank["controller_lock_path"],
                "controller_lock_sha256": bank["controller_lock_sha256"],
                "config_sha256": selected["config_sha256"], "case": selected["case"],
                "scope": "Completed physical history; force failure and rating status remain in original metrics."})
    result = {"status": "complete_current_plant_controller_matrix", "qualification": False,
              "contract_sha256": sha(contract_path), "engine_source_sha256": spec["engine"]["sha256"],
              "api_sha256": spec["engine"]["api_sha256"], "selected_banks_sha256": sha(banks_path),
              "declaration_sha256": sha(declaration_path), "final_summary": summarize(all_rows),
              "development_summary": summarize([row for name in ["coil_rigid_plate", "screw_rigid_plate"]
                for row in journal_rows(OUT / f"{name}_development_records.jsonl")]),
              "groups": [{"name": r["name"], "summary": r["summary"],
                          "by_tool_mass": r["by_tool_mass"], "worst_record_id": r["worst_record_id"],
                          "summary_path": str((OUT / f"{r['name']}_evaluation_summary.json").relative_to(ROOT)),
                          "summary_sha256": sha(OUT / f"{r['name']}_evaluation_summary.json")}
                         for r in results], "native_representatives": representatives,
              "storage": budget_check(), "scope": "2 s loaded-equilibrium fixed-robot axial force; no engagement/recentering/thermal/hardware qualification."}
    dump_new(OUT / "matrix_completion.json", result)
    return result


def resolve_record(lock_path: Path, record_id: str) -> tuple[dict, dict]:
    """Read-only reconstruction for acquisition diagnostics and native replay."""
    lock = read(lock_path)
    base, _ = lock_parameters(lock)
    journal = OUT / f"{lock['name']}_evaluation_records.jsonl"
    matches = [row for row in journal_rows(journal) if row["id"] == record_id]
    if len(matches) != 1:
        raise ValueError("Selected evaluation identity is absent or ambiguous.")
    row = matches[0]
    p = parameters_for_case(base, row["case"])
    if object_sha(p) != row["config_sha256"]:
        raise RuntimeError("Selected recording parameters do not reproduce their hash.")
    return p, row


def soft_pilot() -> dict:
    path = OUT / "soft_pilot_parity.json"
    if path.exists():
        return read(path)
    rows = []
    with private_engine(expected_sha256=SOURCE_HASHES["engine.c"]) as (lib, build):
        for actuator, name in OLD_NAMES.items():
            group = read(OLDVAL / "evaluation" / f"{name}.json")
            old_row = max(group["records"], key=lambda r: r["metrics"]["force_peak_error"])
            p = old_row["config"]
            case = {key: p[key] for key in ("mass_s", "force_target", "angle_deg", "seed")}
            row = run_case(lib, p, f"{name}_private_source_parity", case)
            deltas = {key: row["metrics"][key]-old_row["metrics"][key]
                      for key in ABI.METRIC_FIELDS if key != "runtime_s"}
            if any(deltas.values()):
                raise AssertionError(f"Private compile changed deterministic metrics: {deltas}")
            rows.append({"source_record": old_row["id"], "source_group_path": str(
                (OLDVAL / "evaluation" / f"{name}.json").relative_to(ROOT)),
                "source_group_sha256": sha(OLDVAL / "evaluation" / f"{name}.json"),
                "row": row, "all_deterministic_metrics_bitwise_equal": True})
    result = {"status": "executed_private_compile_soft_pilot_parity", "records": rows,
              "original_hashes": SOURCE_HASHES, "build": build, "storage": budget_check(),
              "scope": "Two existing worst-case identities repeated for source/ABI parity only; no new evaluation or tuning."}
    dump_new(path, result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["pilot", "bind", "develop", "lock", "banks", "evaluate", "final"])
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--actuator", type=int, choices=[0, 1])
    parser.add_argument("--branch", choices=["soft", "no_pad"])
    parser.add_argument("--name")
    parser.add_argument("--lock", type=Path)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--source-sha256")
    parser.add_argument("--api-sha256")
    args = parser.parse_args()
    if args.mode == "pilot":
        result = soft_pilot()
    elif args.mode == "bind":
        if args.contract is None or args.source_sha256 is None or args.api_sha256 is None:
            parser.error("Engine binding requires --contract, --source-sha256 and --api-sha256.")
        result = bind_engine(args.contract, args.source_sha256, args.api_sha256)
    elif args.mode == "develop":
        if args.contract is None or args.actuator is None:
            parser.error("Development requires --contract and --actuator.")
        result = develop(args.actuator, args.contract, args.workers)
    elif args.mode == "lock":
        if args.contract is None or args.actuator is None or args.branch is None or args.name is None:
            parser.error("A transfer lock requires --contract, --actuator, --branch and --name.")
        lock_path = OUT / f"{args.name}_lock.json"
        if lock_path.exists():
            result = read(lock_path)
            lock_parameters(result)
        else:
            result = make_lock(args.name, args.actuator, args.branch, args.contract)
            dump_new(lock_path, result)
    elif args.mode == "banks":
        if args.contract is None:
            parser.error("Selected banks require --contract.")
        result = selected_banks(args.contract)
    elif args.mode == "final":
        if args.contract is None:
            parser.error("Final matrix requires --contract.")
        result = final_matrix(args.contract, args.workers)
    else:
        if args.lock is None:
            parser.error("Evaluation requires --lock.")
        result = evaluate(args.lock, args.workers)
    print(json.dumps(result, sort_keys=True, separators=(",", ":"), allow_nan=False), flush=True)


if __name__ == "__main__":
    main()
