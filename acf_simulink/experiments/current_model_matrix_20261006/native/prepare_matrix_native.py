"""Make bounded physical references in an owned /tmp directory.

The native model receives recorded controller current, never surface preview.
All existing study source and outputs are read-only. The actual run is refused
until the caller supplies the new matrix contract and its exact SHA256.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys

import numpy as np

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE = ROOT / "acf_simulink/experiments/current_model_matrix_20261006"
TEMP_CAP = 600 * 1024**2


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_commands(path: Path) -> tuple[bytes, np.ndarray]:
    raw = gzip.decompress(path.read_bytes()) if path.suffix == ".gz" else path.read_bytes()
    return raw, np.loadtxt(io.BytesIO(raw), delimiter=",", skiprows=1, ndmin=2)


def check_entries(record: dict, contract: Path, digest: str) -> list[dict]:
    if sha256(contract) != digest or record["matrix_contract_sha256"] != digest:
        raise ValueError("Matrix contract identity differs from the frozen selection.")
    entries = record["cases"]
    if not 1 <= len(entries) <= 4:
        raise ValueError("Native batch must have one to four representative cases.")
    if len({e["identity"] for e in entries}) != len(entries):
        raise ValueError("Duplicate native case identity.")
    for e in entries:
        if "controller_lock_path" in e and sha256(Path(e["controller_lock_path"])) != e["controller_lock_sha256"]:
            raise ValueError("Selected controller lock changed.")
        config = e["config"]
        if isinstance(config, str):
            config_path = Path(config)
            if "config_file_sha256" in e and sha256(config_path) != e["config_file_sha256"]:
                raise ValueError("Configuration file differs from the frozen selection.")
            config = json.loads(config_path.read_text())
            e["config"] = config
        config_digest = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        if "config_sha256" in e and e["config_sha256"] != config_digest:
            raise ValueError("Scalar configuration differs from the selected locked record.")
        e["config_sha256"] = config_digest
        if config["duration"] > 2 or config["duration"] <= 0:
            raise ValueError("Native batch is bounded to positive durations <=2s.")
        if config["robot_enabled"] or config["external_robot_amplitude"]:
            raise ValueError("Current native matrix is the fixed-robot comparison.")
        if config["pad_k"] < 0 or config["pad_diameter"] < 0:
            raise ValueError("Negative contact stiffness or diameter is invalid.")
        for name in ("identity", "group"):
            if not e[name] or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for ch in e[name]):
                raise ValueError(f"Unsafe or empty case {name}.")
        if e.get("capture_from_locked_config"):
            if not isinstance(e.get("original_metrics"), dict):
                raise ValueError("Temporary capture requires original full-step metrics.")
            continue
        raw, commands = read_commands(Path(e["commands_csv"]))
        if commands.shape[1] not in (2, 4):
            raise ValueError("Commands must be time,current or time,current,robot,force.")
        expected = np.arange(len(commands)) / config["control_rate"]
        if np.max(np.abs(commands[:, 0] - expected)) >= 1e-10:
            raise ValueError("Commands are not fresh regular controller timestamps.")
        required = round(config["duration"] * config["control_rate"]) + 1
        if len(commands) != required:
            raise ValueError("Representative must have a complete command history.")
        if commands.shape[1] == 4 and np.any(commands[:, 2] != 0):
            raise ValueError("Recorded robot is not fixed.")
        if "commands_uncompressed_sha256" in e and hashlib.sha256(raw).hexdigest() != e["commands_uncompressed_sha256"]:
            raise ValueError("Original controller command bytes changed.")
        e["commands_uncompressed_sha256"] = hashlib.sha256(raw).hexdigest()
    return entries


def prepare(selection: Path, contract: Path, digest: str, temp: Path) -> dict:
    record = json.loads(selection.read_text())
    entries = check_entries(record, contract, digest)
    if temp.parent != Path("/tmp") or not (temp / "owned_matrix_native").is_file():
        raise ValueError("Use an owned direct /tmp/acf_matrix_native_* directory.")
    api_spec = importlib.util.spec_from_file_location("matrix_native_engine_api", SOURCE / "engine_api.py")
    api = importlib.util.module_from_spec(api_spec)
    api_spec.loader.exec_module(api)
    engine = Path(record.get("engine_source", SOURCE / "engine.c"))
    if engine.resolve() != (SOURCE / "engine.c").resolve():
        raise ValueError("This wrapper is bound to the reviewed current-matrix engine.")
    engine_digest = sha256(engine)
    if record["engine_source_sha256"] != engine_digest:
        raise ValueError("Engine source differs from the frozen selection.")
    api_digest = sha256(SOURCE / "engine_api.py")
    if record["api_source_sha256"] != api_digest:
        raise ValueError("Engine API differs from the frozen selection.")
    api.OUT = temp / "engine_build"
    lib = api.compile_engine()
    rows = []
    for entry in entries:
        config = entry["config"]
        capture_audit = dict(status="retained_commands_read_no_capture")
        if entry.get("capture_from_locked_config"):
            metrics, trace = api.simulate(lib, config, trace_rate=config["control_rate"])
            checked = [name for name in api.METRIC_FIELDS if name != "runtime_s"]
            differences = {name: dict(expected=entry["original_metrics"].get(name), actual=metrics[name])
                           for name in checked if entry["original_metrics"].get(name) != metrics[name]}
            if "event_force_limits" in entry["original_metrics"]:
                if entry["original_metrics"]["event_force_limits"] != metrics.get("event_force_limits"):
                    differences["event_force_limits"] = dict(expected=entry["original_metrics"]["event_force_limits"], actual=metrics.get("event_force_limits"))
                checked.append("event_force_limits")
            if differences:
                raise ValueError(f"Temporary capture changed frozen metrics: {differences}")
            commands = trace[:, [0, 6, 17, 1]]
            if len(commands) != round(config["duration"] * config["control_rate"]) + 1:
                raise ValueError("Choose completed representatives; stopped history cannot be extrapolated.")
            text = io.StringIO()
            np.savetxt(text, commands, delimiter=",", header="time_s,current_command_A,robot_R_m,original_force_N", comments="", fmt="%.17g")
            raw = text.getvalue().encode()
            capture_audit = dict(status="temporary_command_capture_executed", original_metrics_identical=True,
                                 compared_fields=checked, source_record_identity=entry.get("source_record_identity"),
                                 points=len(commands), force_metrics=metrics)
            del trace
        else:
            raw, commands = read_commands(Path(entry["commands_csv"]))
        command_file = temp / (entry["identity"] + "_commands.csv")
        command_file.write_bytes(raw)
        replay_file = temp / (entry["identity"] + "_replay_inputs.csv")
        np.savetxt(replay_file, commands[:, :2], delimiter=",", header="time_s,current_command_A", comments="", fmt="%.17g")
        physical = api.physical_replay(lib, config, np.ascontiguousarray(commands[:, 1]))
        count = round(config["duration"] / config["dt"]) + 1
        if physical.shape != (count, 10):
            raise ValueError("Physical replay returned an unexpected output count.")
        reference = temp / (entry["identity"] + "_force.bin")
        np.ascontiguousarray(physical[:, 0]).tofile(reference)
        stride = round(1 / (config["control_rate"] * config["dt"]))
        states = temp / (entry["identity"] + "_states.bin")
        np.ascontiguousarray(physical[::stride, 1:]).tofile(states)
        row = dict(entry, commands_csv=str(replay_file), original_commands_csv=str(command_file),
                   original_commands_columns=commands.shape[1], reference_binary=str(reference),
                   reference_count=count, reference_sha256=sha256(reference),
                   reference_states_binary=str(states), reference_state_stride=stride,
                   reference_states_sha256=sha256(states),
                   regular_C_force_peak_error_N=float(np.max(np.abs(physical[:, 0] - config["force_target"]))),
                   command_capture=capture_audit, commands_uncompressed_sha256=hashlib.sha256(raw).hexdigest())
        if "commands_csv" in entry:
            row["retained_commands_archive_sha256"] = sha256(Path(entry["commands_csv"]))
        row["regular_C_rigid_side_events"] = []
        if config["pad_k"] == 0 and config["pad_diameter"] > 0:
            gravity_axis = config["gravity"] * np.cos(np.deg2rad(config["angle_deg"]))
            acceleration_edge = -24 * config["bump_height"] * (config["scan_speed"] / config["bump_width"]) ** 2
            for transition, sign in (("enter_plateau", -1), ("leave_plateau", 1)):
                event_time = config["bump_start"] + (config["bump_width"] + sign * config["pad_diameter"]) / (2 * config["scan_speed"])
                index = round(event_time / config["dt"])
                if abs(index * config["dt"] - event_time) > 1e-10:
                    raise ValueError("Native event references require these transitions on the base clock.")
                cell = config["cell_k"] * (physical[index, 5] - physical[index, 3])
                left_acceleration, right_acceleration = ((acceleration_edge, 0) if sign == -1 else (0, acceleration_edge))
                row["regular_C_rigid_side_events"].append(dict(
                    transition=transition, time_s=event_time,
                    left_acceleration_m_s2=left_acceleration, right_acceleration_m_s2=right_acceleration,
                    left_force_N=float(cell + config["mass_s"] * (left_acceleration + gravity_axis)),
                    right_force_N=float(cell + config["mass_s"] * (right_acceleration + gravity_axis))))
        if commands.shape[1] == 4:
            row["regular_vs_original_split_event_force_at_controller_ticks_N"] = float(np.max(np.abs(physical[::stride, 0] - commands[:, 3])))
        rows.append(row)
        del physical, commands
    manifest = dict(status="temporary_regular_grid_physical_references_executed", cases=rows,
                    matrix_contract=str(contract.resolve()), matrix_contract_sha256=digest,
                    source_selection=str(selection.resolve()), source_selection_sha256=sha256(selection),
                    engine_source_sha256=engine_digest, api_source_sha256=api_digest,
                    engine_build=json.loads((api.OUT / "build.json").read_text()), native_force_guard_N=1e-5,
                    footprint_guard_N=1e-4, controller_independently_verified=False,
                    temporary_cap_bytes=TEMP_CAP)
    (temp / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    api.cleanup_engine(lib)
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("selection", type=Path)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--temp", type=Path)
    parser.add_argument("--inspect-only", action="store_true")
    args = parser.parse_args()
    if args.inspect_only:
        entries = check_entries(json.loads(args.selection.read_text()), args.contract, args.contract_sha256)
        print(json.dumps(dict(status="selection_schema_checked_no_simulation", cases=len(entries))))
        return
    if args.temp is None:
        parser.error("--temp is required for reference generation")
    result = prepare(args.selection, args.contract, args.contract_sha256, args.temp)
    print(json.dumps(dict(status=result["status"], cases=len(result["cases"]))))


if __name__ == "__main__":
    main()
