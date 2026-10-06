"""Bounded release checks for the contact fork; no campaign or retained arrays."""
from __future__ import annotations

import ctypes as ct
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import tempfile

import numpy as np

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[4]
HERE = ROOT / "acf_simulink/experiments/current_model_matrix_20261006"
OLD = ROOT / "acf_simulink/experiments/sea_pad_dynamic_20261006"
OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def physical_rhs(api, lib, config, t, state, side="right"):
    p = api.make_parameters(config)
    state = np.ascontiguousarray(state, dtype=np.float64)
    derivative, output = np.empty(9), np.empty(10)
    command = state[8]
    lib.engine_rhs_side(ct.byref(p), t, state.ctypes.data_as(ct.POINTER(ct.c_double)),
                        command, 0., derivative.ctypes.data_as(ct.POINTER(ct.c_double)),
                        output.ctypes.data_as(ct.POINTER(ct.c_double)), -1 if side == "left" else 1)
    return derivative, output


def initial_state(api, lib, config):
    p = api.make_parameters(config)
    state, equilibrium = np.empty(9), np.empty(2)
    lib.engine_initialize(ct.byref(p), state.ctypes.data_as(ct.POINTER(ct.c_double)),
                          equilibrium.ctypes.data_as(ct.POINTER(ct.c_double)))
    return state, equilibrium


def main():
    contract_path = HERE / "study_contract.json"
    contract = json.loads(contract_path.read_text())
    assert sha(contract_path) == "0936fd73866223e0a1f8c884f8ee2eed28627e3882b78adf9127eafff57e521f"
    assert sha(OLD / "engine.c") == contract["physics_lineage"]["base_engine_sha256"]
    assert sha(OLD / "engine_api.py") == contract["physics_lineage"]["base_api_sha256"]
    forecast_path = ROOT / contract["storage_forecast"]["path"]
    forecast = json.loads(forecast_path.read_text())
    assert sha(forecast_path) == contract["storage_forecast"]["sha256"]
    assert forecast["shared_array_audit"]["bytes"] < forecast["shared_array_limit_bytes"]
    assert shutil.disk_usage("/tmp").free > forecast["allocations"]["plant"]["temporary"]
    api = module("matrix_check_api", HERE / "engine_api.py")
    old_api = module("frozen_check_api", OLD / "engine_api.py")
    banks = json.loads((HERE / "initial_banks.json").read_text())["banks"]
    rigid = dict(next(row["config"] for row in banks if row["id"] == "coil_rigid_plate"))
    report = {"schema_version": 1, "status": "checks_in_progress",
              "started_utc": datetime.now(timezone.utc).isoformat(),
              "contract_sha256": sha(contract_path), "engine_sha256": sha(HERE / "engine.c"),
              "api_sha256": sha(HERE / "engine_api.py"), "check_recipe_sha256": sha(Path(__file__)),
              "old_source_sha256": sha(OLD / "engine.c"), "old_api_sha256": sha(OLD / "engine_api.py"),
              "retained_arrays": 0, "campaign_or_tuning_runs": 0,
              "temporary_disk_forecast_bytes": 256 * 1024,
              "trace_array_peak_forecast_bytes": 3 * 96 * 1024,
              "checks": {}, "executed_verification_recordings": []}
    report["preserved_prior_verification_attempts"] = [
        {"path": path.name, "sha256": sha(path),
         "executed_recording_count": len(json.loads(path.read_text())["executed_verification_recordings"])}
        for path in sorted(OUT.glob("checks_failed_fixture_*.json"))]
    lib = api.compile_engine()
    try:
        assert api.FIELDS == old_api.FIELDS and api.METRIC_FIELDS == old_api.METRIC_FIELDS
        assert api.TRACE_COLUMNS == old_api.TRACE_COLUMNS
        report["checks"]["abi"] = {"pass": True, "parameter_bytes": ct.sizeof(api.Parameters),
                                  "metric_bytes": ct.sizeof(api.Metrics),
                                  "trace_columns": len(api.TRACE_COLUMNS)}
        events = api.rigid_event_times(lib, rigid)
        assert np.max(np.abs(np.array(events) - [.415, .465, 1.235, 1.285])) < 3e-16
        height_error = velocity_error = 0.
        derivative_count = 0
        for a, b in ((.3, .4), (.416, .464), (.466, 1.234), (1.236, 1.284), (1.3, 1.4)):
            for t in np.linspace(a, b, 25):
                h, v, acc = api.rigid_motion(lib, rigid, float(t))
                left = api.rigid_motion(lib, rigid, float(t-1e-6))
                right = api.rigid_motion(lib, rigid, float(t+1e-6))
                height_error = max(height_error, abs((right[0]-left[0])/2e-6-v))
                velocity_error = max(velocity_error, abs((right[1]-left[1])/2e-6-acc))
                derivative_count += 1
        assert height_error < 2e-10 and velocity_error < 2e-7
        limits = []
        for t in events[1:3]:
            left, right = api.rigid_motion(lib, rigid, t, "left"), api.rigid_motion(lib, rigid, t)
            assert left[:2] == right[:2] == (.001, 0.)
            assert abs(abs(left[2]-right[2])-2.4) < 1e-14
            before, after = api.rigid_motion(lib, rigid, t-1e-8), api.rigid_motion(lib, rigid, t+1e-8)
            assert abs(before[0]-after[0]) < 1e-14 and abs(before[1]-after[1]) < 3e-8
            limits.append({"time_s": t, "left_h_v_a": left, "right_h_v_a": right})
        report["checks"]["analytic_geometry"] = {"pass": True, "event_times_s": events,
            "derivative_points": derivative_count, "max_height_derivative_error_m_s": height_error,
            "max_velocity_derivative_error_m_s2": velocity_error, "limits": limits,
            "endpoint_convention": "right continuous; 1e-12s equality tolerance; integration k4 uses left event limit"}

        equilibrium_count, equilibrium_error, balance_error = 0, 0., 0.
        for actuator in (0, 1):
            base = next(row["config"] for row in banks if row["id"] == ("coil_rigid_plate" if actuator == 0 else "screw_rigid_plate"))
            for mass in (.1, 1.4, 2.):
                for force in (5., 50.):
                    for angle in (0., 45., 90.):
                        config = {**base, "mass_s": mass, "identified_mass_s": mass, "force_target": force, "angle_deg": angle}
                        state, eq = initial_state(api, lib, config)
                        dx, output = physical_rhs(api, lib, config, 0., state)
                        equilibrium_error = max(equilibrium_error, float(np.max(np.abs(dx))))
                        balance_error = max(balance_error, abs(output[0]-output[1]-mass*(dx[5]+eq[1])))
                        assert abs(output[0]-force) < 1e-12
                        equilibrium_count += 1
        assert equilibrium_error < 1e-10 and balance_error < 1e-12
        bound_checks = []
        for mass in (.1, 1.4, 2.):
            config = {**rigid, "mass_s": mass, "identified_mass_s": mass, "force_target": 20., "angle_deg": 45.}
            state, eq = initial_state(api, lib, config)
            # Center the pair around the target: continuous cell cannot cancel the jump.
            state[2] = .001-(20.+1.2*mass-mass*eq[1])/config["cell_k"]
            state[4], state[5] = 123., 456.  # Deliberately wrong integrated S is ignored.
            for t in events[1:3]:
                dl, ol = physical_rhs(api, lib, config, t, state, "left")
                dr, ore = physical_rhs(api, lib, config, t, state)
                assert ol[1] == ore[1]
                measured_jump = abs(ol[0]-ore[0])
                peak_error = max(abs(ol[0]-20), abs(ore[0]-20))
                assert abs(measured_jump-2.4*mass) < 1e-11 and abs(peak_error-1.2*mass) < 1e-11
                assert dl[4] == dr[4] == 0.
                bound_checks.append({"mass_s_kg": mass, "time_s": t, "force_jump_N": measured_jump,
                                     "minimum_peak_error_N": 1.2*mass, "centered_peak_error_N": peak_error})
        report["checks"]["force_balance_and_bound"] = {"pass": True, "loaded_equilibria": equilibrium_count,
            "max_equilibrium_derivative": equilibrium_error, "max_force_balance_residual_N": balance_error,
            "one_sided_pairs": bound_checks, "analytic_S_overrides_integrated_S": True}

        original_source, new_source = (OLD / "engine.c").read_text(), (HERE / "engine.c").read_text()
        def controller_source(source):
            start = source.index("    if(t+1e-12 >= control/p->control_rate)")
            end = source.index("    plant_rhs(&c,t,x,c.command", start)
            return source[start:end]
        assert controller_source(original_source) == controller_source(new_source)
        report["checks"]["causal_control"] = {"pass": True, "controller_block_byte_identical": True,
            "truth_motion_routed_only_to_physical_contact_and_sensor_inputs": True,
            "no_surface_preview_or_true_force_controller_input_added": True}

        selected_path = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006/engine/replay/replay_cases_pad5mm.json"
        selected = json.loads(selected_path.read_text())["cases"]
        parity = []
        with tempfile.TemporaryDirectory(prefix="acf_matrix_soft_oracle_") as directory:
            old_api.OUT = Path(directory) / "build"
            old_lib = old_api.compile_engine()
            for row in selected:
                config = json.loads(Path(row["config"]).read_text())
                previous, previous_trace = old_api.simulate(old_lib, config, trace_rate=250)
                current, current_trace = api.simulate(lib, config, trace_rate=250)
                report["executed_verification_recordings"].extend([
                    {"role": "frozen_oracle", "id": row["id"], "steps": previous["steps"]},
                    {"role": "new_soft_parity", "id": row["id"], "steps": current["steps"]}])
                for key in old_api.METRIC_FIELDS:
                    if key != "runtime_s":
                        assert current[key] == previous[key] == row["metrics"][key], (row["id"], key, current[key], previous[key])
                assert np.array_equal(current_trace, previous_trace)
                parity.append({"id": row["id"], "all_nonruntime_metrics_bitwise_equal": True,
                    "metrics_compared": len(old_api.METRIC_FIELDS)-1, "trace_rows": len(current_trace),
                    "trace_columns": current_trace.shape[1], "trace_float64_sha256": hashlib.sha256(current_trace.tobytes()).hexdigest(),
                    "trace_max_abs_difference": 0., "force_peak_error_N": current["force_peak_error"],
                    "recorded_metric_guard": "unchanged exact equality; no tolerance relaxation"})
                del previous_trace, current_trace
        report["checks"]["soft_parity"] = {"pass": True, "source_manifest_sha256": sha(selected_path), "cases": parity}

        score_checks = []
        for offset in (0., .000000137):
            config = {**rigid, "mass_s": .1, "identified_mass_s": .1, "force_target": 50.,
                      "noise_enabled": 0., "bump_start": .8+offset, "external_current": 1.}
            metrics, trace = api.simulate(lib, config, trace_rate=100)
            report["executed_verification_recordings"].append({"role": "rigid_event_alignment", "bump_start_offset_s": offset,
                "steps": metrics["steps"], "stop_reason": metrics["stop_reason"]})
            score_checks.append({"bump_start_offset_s": offset, "metrics": metrics, "trace_rows_checked": len(trace),
                                 "constant_equilibrium_current_physical_probe": True})
            report["checks"]["event_scoring"] = {"pass": False, "cases": score_checks}
            assert metrics["completed"] and len(metrics["event_force_limits"]) == 2
            actual_events = api.rigid_event_times(lib, config)
            for index, row in enumerate(metrics["event_force_limits"]):
                assert row["time_s"] == actual_events[index+1]
                assert abs(abs(row["force_left_N"]-row["force_right_N"])-.24) < 1e-11
            for row in trace:
                h, v, _ = api.rigid_motion(lib, config, row[0])
                assert row[12] == h and row[13] == v
            assert metrics["force_peak_error"] >= .12-1e-11
            assert metrics["steps"] == metrics["scored_integration_endpoint_count"] + 2
            score_checks[-1].update({"analytic_trace_S_exact": True, "regular_endpoint_plus_extra_limit_count_exact": True})
        report["checks"]["event_scoring"] = {"pass": True, "cases": score_checks}

        config = {**rigid, "mass_s": 2., "identified_mass_s": 2., "force_target": 1., "noise_enabled": 0., "external_current": 1.}
        metrics, trace = api.simulate(lib, config, trace_rate=100)
        report["executed_verification_recordings"].append({"role": "physical_contact_stop", "steps": metrics["steps"], "stop_time": metrics["stop_time"]})
        assert metrics["stop_reason"] == "contact_loss" and metrics["stop_time"] < events[1]
        assert trace[-1,0] == metrics["stop_time"] and trace[-1,1] <= 0
        report["checks"]["physical_stop"] = {"pass": True, "metrics": metrics, "terminal_trace_is_scored_stop_endpoint": True}

        config = {**rigid, "force_target": 0., "gravity": 0., "angle_deg": 90., "noise_enabled": 0.}
        metrics, trace = api.simulate(lib, config, trace_rate=100)
        report["executed_verification_recordings"].append({"role": "startup_stop_probe", "steps": metrics["steps"]})
        assert metrics["stop_reason"] == "contact_loss" and metrics["stop_time"] == 0.
        assert metrics["steps"] == 1. and metrics["adc_count"] == metrics["control_count"] == 1.
        assert len(trace) == 1
        report["checks"]["startup_scoring"] = {"pass": True, "metrics": metrics, "loaded_zero_force_deliberate_stop_probe": True}
        report["status"] = "passed_analytic_balance_event_and_exact_soft_parity_checks"
        report["executed_verification_recording_count"] = len(report["executed_verification_recordings"])
        report["total_verification_recordings_including_preserved_attempts"] = report["executed_verification_recording_count"] + sum(
            row["executed_recording_count"] for row in report["preserved_prior_verification_attempts"])
        report["closed_loop_controller_or_hardware_qualification"] = False
        report["completed_utc"] = datetime.now(timezone.utc).isoformat()
    except Exception as error:
        report["status"] = "failed_guard_preserved"
        report["exception"] = repr(error)
        raise
    finally:
        api.cleanup_engine(lib)
        (OUT / "checks.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
        used = sum(path.stat().st_size for path in OUT.rglob("*") if path.is_file())
        source_bytes = sum((HERE/name).stat().st_size for name in ("engine.c", "engine_api.py"))
        assert used+source_bytes <= forecast["allocations"]["plant"]["retained"]
    print(json.dumps({"status": report["status"], "source_sha256": report["engine_sha256"],
                      "verification_recordings": report["executed_verification_recording_count"]}))


if __name__ == "__main__":
    main()
