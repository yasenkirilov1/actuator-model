"""Hold actor requests while the unchanged native graph advances at 100 us.

This adapter aggregates measurements, not physics. A 1 ms actor decision can
hold its SI requests over ten native action intervals; acquisition, servo,
observers, safety gate and robot queue continue on their original clocks.
The final native observation is returned for the next recurrent decision.
"""
from __future__ import annotations

import operator

import numpy as np

from native_api import StepResult


_SUM_COUNTS = ("native_samples", "voltage_clip_samples", "current_limit_samples", "force_outside_samples")
_MEAN_SQUARES = ("mean_squared_normal_error_N2", "mean_squared_travel_error_m2", "mean_squared_current_A2")
_MINIMUM = ("min_normal_N", "min_travel_m")
_MAXIMUM = ("max_normal_N", "max_travel_m", "peak_normal_error_N", "peak_current_A", "peak_voltage_V")
_FLAGS = ("terminated", "nonfinite", "contact_loss", "travel_stop")


def _is_terminal(result: StepResult) -> bool:
    return any(bool(result.metrics.get(name, False)) for name in _FLAGS)


def _aggregate(parts: list[StepResult]) -> StepResult:
    """Aggregate equal-clock native samples, retaining cumulative endpoints."""
    if not parts:
        raise ValueError("aggregation requires at least one native result")
    final = parts[-1]
    metrics = dict(final.metrics)
    for name in _SUM_COUNTS:
        metrics[name] = sum(int(result.metrics[name]) for result in parts)
    sample_count = metrics["native_samples"]
    for name in _MEAN_SQUARES:
        # Partial failure intervals have fewer physical samples. Do not
        # average interval averages equally or reuse zero-sample stopped calls.
        squared_sum = sum(float(result.metrics[name]) * int(result.metrics["native_samples"])
                          for result in parts if int(result.metrics["native_samples"]) > 0)
        metrics[name] = squared_sum / sample_count if sample_count else 0.0
    for name in _MINIMUM:
        metrics[name] = min(float(result.metrics[name]) for result in parts)
    for name in _MAXIMUM:
        metrics[name] = max(float(result.metrics[name]) for result in parts)
    for name in _FLAGS:
        metrics[name] = any(bool(result.metrics.get(name, False)) for result in parts)
    metrics["terminated"] = any(metrics[name] for name in _FLAGS)
    metrics["elapsed_s"] = sum(float(result.metrics["elapsed_s"]) for result in parts)
    metrics["start_s"] = float(parts[0].metrics["start_s"])
    metrics["end_s"] = float(final.metrics["end_s"])
    # record_samples and physical_tick are cumulative native counters; summing
    # them would double-count history. last_normal_N also belongs to the end.
    return StepResult(final.obs, final.truth_last, final.comparator, metrics)


def repeat_native_step(plant, contexts, actions, repeat: int = 1) -> list[StepResult]:
    """Hold current/robot SI requests for repeat native 100 us intervals.

    Results retain the input context order. A context that terminates during
    this decision is excluded from later native calls. Its final observation
    and partial native statistics are preserved, and context.result receives
    the aggregate. Repeat1 is an exact delegation to the original API.
    """
    repeat = operator.index(repeat)
    if repeat < 1:
        raise ValueError("repeat must be a positive integer")
    if repeat == 1:
        return plant.step_batch(contexts, actions)
    contexts = list(contexts)
    count = len(contexts)
    if not count:
        return []
    held = np.array(actions, dtype=np.float64, order="C", copy=True)
    if held.shape != (count, 2):
        raise ValueError("actions must have shape [contexts,2] in SI current/robot units")
    parts: list[list[StepResult]] = [[] for _ in contexts]
    active = list(range(count))
    for _ in range(repeat):
        results = plant.step_batch([contexts[index] for index in active], held[active])
        if len(results) != len(active):
            raise RuntimeError("native batch returned the wrong number of results")
        next_active = []
        for index, result in zip(active, results, strict=True):
            parts[index].append(result)
            if not _is_terminal(result):
                next_active.append(index)
        active = next_active
        if not active:
            break
    aggregated = [_aggregate(row) for row in parts]
    for context, result in zip(contexts, aggregated, strict=True):
        context.result = result
    return aggregated


def _fixture_checks() -> dict:
    """Uneven-sample terminal fixtures; no simulated mechanics are defined."""
    from types import SimpleNamespace

    def result(number, samples=30, **flags):
        metrics = {"elapsed_s": samples / 300000.0, "start_s": number * 1e-4,
                   "end_s": number * 1e-4 + samples / 300000.0,
                   "native_samples": samples, "min_normal_N": 5-number,
                   "max_normal_N": 5+number, "peak_normal_error_N": number,
                   "mean_squared_normal_error_N2": number+1.0,
                   "min_travel_m": .001-number*.0001, "max_travel_m": .001+number*.0001,
                   "mean_squared_travel_error_m2": (number+1)*1e-6,
                   "peak_current_A": .1*number, "peak_voltage_V": number+2.0,
                   "mean_squared_current_A2": .01*(number+1),
                   "voltage_clip_samples": number, "current_limit_samples": number+1,
                   "force_outside_samples": number+2, "contact_loss": False,
                   "travel_stop": False, "nonfinite": False, "terminated": False,
                   "record_samples": 31+30*number, "physical_tick": 30*(number+1),
                   "last_normal_N": 5+number}
        metrics.update(flags)
        return StepResult(np.full(18, number, np.float64), np.full(27, number, np.float64),
                          np.full(3, number, np.float64), metrics)

    fixtures = [[result(0), result(1, 7, contact_loss=True)],
                [result(0), result(1), result(2, 3, travel_stop=True)],
                [result(0), result(1, 0, nonfinite=True)],
                [result(0), result(1), result(2), result(3)]]
    contexts = [SimpleNamespace(index=i, result=None) for i in range(len(fixtures))]

    class FixturePlant:
        def __init__(self):
            self.positions = [0] * len(fixtures)
            self.calls = []

        def step_batch(self, active, actions):
            self.calls.append([context.index for context in active])
            returned = []
            for context in active:
                index, position = context.index, self.positions[context.index]
                if position >= len(fixtures[index]):
                    raise AssertionError("terminated context was called again")
                assert np.array_equal(actions[len(returned)], [index + .2, index * .001])
                context.result = fixtures[index][position]
                returned.append(context.result)
                self.positions[index] += 1
            return returned

    plant = FixturePlant()
    actions = np.array([[i+.2, i*.001] for i in range(len(contexts))])
    combined = repeat_native_step(plant, contexts, actions, 4)
    assert plant.calls == [[0, 1, 2, 3], [0, 1, 2, 3], [1, 3], [3]]
    assert [row.native_samples for row in combined] == [37, 63, 30, 120]
    assert combined[0].mean_squared_normal_error_N2 == (30+2*7)/37
    assert combined[1].mean_squared_normal_error_N2 == (30+2*30+3*3)/63
    assert combined[2].mean_squared_normal_error_N2 == 1.0
    assert all(row.terminated for row in combined[:3]) and not combined[3].terminated
    assert combined[0].contact_loss and combined[1].travel_stop and combined[2].nonfinite
    assert combined[3].record_samples == 121 and combined[3].physical_tick == 120
    assert all(context.result is row for context, row in zip(contexts, combined, strict=True))
    sentinel = object()
    class Delegate:
        def step_batch(self, received_contexts, received_actions):
            assert received_contexts is contexts and received_actions is actions
            return sentinel
    assert repeat_native_step(Delegate(), contexts, actions, 1) is sentinel
    return {"status": "passed", "inner_call_contexts": plant.calls,
            "sample_counts": [row.native_samples for row in combined],
            "repeat1_exact_delegation": True, "uneven_sample_weighting": True,
            "terminal_contact_travel_nonfinite_or": True, "cumulative_counters_not_summed": True}


def _native_checks(build_dir, output_dir) -> dict:
    """Compare direct and held native calls using training scenarios only."""
    import json
    from pathlib import Path
    from native_api import NativePlant
    build = Path(build_dir)
    if not (build / "libacf_native.so").exists():
        raise FileNotFoundError("acceptance reuses an existing native library; no build mutation is allowed")
    plant = NativePlant(build)
    output = Path(output_dir)
    train_ids = [i for i, row in enumerate(plant.scenarios) if row.get("split") == "train"]
    if not train_ids:
        raise ValueError("acceptance must use a recorded training identity")
    scenario = train_ids[0]
    seed = 813001
    direct = plant.create(scenario, seed=seed, baseline=False, record=True)
    repeated = plant.create(scenario, seed=seed, baseline=False, record=True)
    checked = 0
    errors = {"obs": 0.0, "truth": 0.0, "comparator": 0.0, "metrics": 0.0}
    try:
        equilibrium = float(direct.result.comparator[2])
        for block in range(12):
            request = np.array([[equilibrium + .001*np.sin(block), .001*(-1)**block]])
            parts = [plant.step_batch([direct], request)[0] for _ in range(10)]
            assert not any(_is_terminal(row) for row in parts)
            held = repeat_native_step(plant, [repeated], request, 10)[0]
            # Independently reconstruct expected measurement statistics.
            counts = np.array([part.native_samples for part in parts], np.int64)
            assert held.native_samples == int(counts.sum())
            for name in _MEAN_SQUARES:
                expected = float(np.dot(counts, [part.metrics[name] for part in parts])/counts.sum())
                errors["metrics"] = max(errors["metrics"], abs(held.metrics[name]-expected))
            assert held.start_s == parts[0].start_s and held.end_s == parts[-1].end_s
            assert held.elapsed_s == sum(part.elapsed_s for part in parts)
            for name in _MINIMUM:
                assert held.metrics[name] == min(part.metrics[name] for part in parts)
            for name in _MAXIMUM:
                assert held.metrics[name] == max(part.metrics[name] for part in parts)
            for name in _SUM_COUNTS:
                assert held.metrics[name] == sum(part.metrics[name] for part in parts)
            for name, expected in (("obs", parts[-1].obs), ("truth", parts[-1].truth_last),
                                   ("comparator", parts[-1].comparator)):
                actual = held.truth_last if name == "truth" else getattr(held, name)
                errors[name] = max(errors[name], float(np.abs(actual-expected).max()))
                assert np.array_equal(actual, expected)
            checked += 1
        one, two = direct.take_record(), repeated.take_record()
        for name in one:
            assert np.array_equal(one[name], two[name]), name
        np.savez_compressed(output/"direct_held_native.npz", **one)
        np.savez_compressed(output/"repeat10_native.npz", **two)
        trace_samples = len(one["time"])
    finally:
        direct.close(); repeated.close()

    # A real physical stop partway through one repeated decision must have
    # exactly the same full recording as immediately stopping direct calls.
    direct = plant.create(scenario, seed=seed+1, baseline=False, record=True)
    repeated = plant.create(scenario, seed=seed+1, baseline=False, record=True)
    failure_parts = []
    try:
        action = np.array([[-3.5, 0.0]])
        for _ in range(20):
            row = plant.step_batch([direct], action)[0]
            failure_parts.append(row)
            if _is_terminal(row):
                break
        held = repeat_native_step(plant, [repeated], action, 20)[0]
        one, two = direct.take_record(), repeated.take_record()
        assert all(np.array_equal(one[name], two[name]) for name in one)
        assert held.native_samples == sum(part.native_samples for part in failure_parts)
        assert held.terminated == any(_is_terminal(part) for part in failure_parts)
        assert held.terminated and len(failure_parts) < 20
        np.savez_compressed(output/"direct_failure_native.npz", **one)
        np.savez_compressed(output/"repeat20_failure_native.npz", **two)
        failure = {"terminated": bool(held.terminated), "contact_loss": bool(held.contact_loss),
                   "travel_stop": bool(held.travel_stop), "repeat_requested": 20,
                   "inner_intervals": len(failure_parts), "later_intervals_skipped": 20-len(failure_parts),
                   "native_samples": held.native_samples, "record_samples": len(one["time"])}
    finally:
        direct.close(); repeated.close()
    if errors["metrics"] > 1e-12 or max(errors["obs"],errors["truth"],errors["comparator"]) != 0:
        raise AssertionError("native repeated action parity failed")
    return {"status": "passed", "scenario": scenario, "identity": plant.scenarios[scenario].get("identity"),
            "seed": seed, "split": "train", "blocks": checked, "repeat": 10,
            "native_record_samples": trace_samples, "native_record_bit_identical": True,
            "errors": errors, "real_failure": failure}


def main():
    import argparse
    import hashlib
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-build", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "scope": "repeat measurement adapter acceptance; training scenarios only",
              "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    target = args.output/"results.json"
    try:
        report["fixtures"] = _fixture_checks()
        if args.native_build:
            report["native"] = _native_checks(args.native_build, args.output)
        report["status"] = "complete"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        target.write_text(json.dumps(report, indent=2)+"\n")
        raise
    report["files"] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                       for path in args.output.glob("*.npz")}
    target.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"status": report["status"], "result": str(target)}))


if __name__ == "__main__":
    main()
