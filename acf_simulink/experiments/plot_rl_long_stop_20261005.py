"""Plot the locked long-run physical stop from complete native recordings.

Only retained results are read. Hashes, case/seed pairing and every native
sample are checked. Streaming reduces memory use; min/max bins are display
envelopes, never the data used to calculate metrics.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
STUDY = ROOT / "acf_simulink/validation/rl_budget3h_20261005"
BLUE, ORANGE, RED, GRAY = "#1768a6", "#c35c13", "#b72c40", "#596475"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.labelcolor": "#26374b", "text.color": "#26374b",
    "axes.titleweight": "bold", "axes.titlesize": 11,
    "svg.fonttype": "none", "savefig.facecolor": "white",
    "figure.facecolor": "white", "axes.grid": True, "grid.alpha": .16,
})


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def npy_header(source):
    version = np.lib.format.read_magic(source)
    if version == (1, 0):
        return np.lib.format.read_array_header_1_0(source)
    if version == (2, 0):
        return np.lib.format.read_array_header_2_0(source)
    raise ValueError(f"Unsupported retained NPY format: {version}")


def streamed_history(row, scenario, maximum_bins=1800):
    """Recompute full native metrics while retaining only display envelopes."""
    path = Path(row["nativeHistoryFile"])
    assert sha(path) == row["nativeHistorySha256"], path
    reference, midpoint = scenario["force_N"], scenario["midTravel_m"]
    radius = row["travelBandRadius_m"]
    n = int(row["nativeSamples"])
    width = max(1, math.ceil(n / maximum_bins))
    bins = math.ceil(n / width)
    minimum = np.full((3, bins), np.inf)
    maximum = np.full((3, bins), -np.inf)
    stats = {"nativeSamples": n, "peakNormalForceError_N": 0.0,
             "normalSquaredSum_N2": 0.0, "maxAbsTravelError_m": 0.0,
             "forceOutsideSamples": 0, "travelOutsideBandSamples": 0,
             "contactLoss": False, "travelStop": False}
    with zipfile.ZipFile(path) as archive:
        with archive.open("time.npy") as source:
            time = np.lib.format.read_array(source, allow_pickle=False)
        assert time.shape == (n,) and time.dtype == np.float64
        assert np.isfinite(time).all() and time[0] == 0 and np.all(np.diff(time) > 0)
        with archive.open("truth.npy") as source:
            shape, fortran, dtype = npy_header(source)
            assert shape == (n, 27) and not fortran and dtype == np.float64
            for start in range(0, n, 65536):
                count = min(65536, n - start)
                data = source.read(count * 27 * dtype.itemsize)
                if len(data) != count * 27 * dtype.itemsize:
                    raise ValueError("Retained native history ended early")
                truth = np.frombuffer(data, dtype=dtype).reshape(count, 27)
                assert np.isfinite(truth).all()
                force = truth[:, 0] - reference
                travel = truth[:, 2] - midpoint
                velocity = truth[:, 8]
                stats["peakNormalForceError_N"] = max(stats["peakNormalForceError_N"], float(np.max(np.abs(force))))
                stats["normalSquaredSum_N2"] += float(np.sum(force * force, dtype=np.float64))
                stats["maxAbsTravelError_m"] = max(stats["maxAbsTravelError_m"], float(np.max(np.abs(travel))))
                stats["forceOutsideSamples"] += int(np.count_nonzero(np.abs(force) > .5))
                stats["travelOutsideBandSamples"] += int(np.count_nonzero(np.abs(travel) > radius + 1e-12))
                stats["contactLoss"] |= bool(np.any(truth[:, 18] < .5))
                stats["travelStop"] |= bool(np.any(truth[:, 21] > .5))
                group = (start + np.arange(count)) // width
                local = np.r_[0, np.flatnonzero(group[1:] != group[:-1]) + 1]
                indices = group[local]
                for channel, values in enumerate((travel * 1000, force, velocity * 1000)):
                    minimum[channel, indices] = np.minimum(minimum[channel, indices], np.minimum.reduceat(values, local))
                    maximum[channel, indices] = np.maximum(maximum[channel, indices], np.maximum.reduceat(values, local))
                if start + count == n:
                    final = [float(travel[-1] * 1000), float(force[-1]), float(velocity[-1] * 1000)]
            assert source.read(1) == b""
    stats["rmsNormalForceError_N"] = math.sqrt(stats.pop("normalSquaredSum_N2") / n)
    stats["endTime_s"] = float(time[-1])
    stats["travelWithinBandFraction"] = 1 - stats["travelOutsideBandSamples"] / n
    for field in ("peakNormalForceError_N", "rmsNormalForceError_N", "maxAbsTravelError_m", "endTime_s", "travelWithinBandFraction"):
        assert math.isclose(stats[field], row[field], rel_tol=1e-11, abs_tol=1e-12), (field, stats[field], row[field])
    for field in ("forceOutsideSamples", "travelOutsideBandSamples", "contactLoss", "travelStop"):
        assert stats[field] == row[field], (field, stats[field], row[field])
    starts = np.arange(0, n, width)
    stops = np.minimum(starts + width - 1, n - 1)
    centers = (time[starts] + time[stops]) / 2
    return {"time": centers, "minimum": minimum, "maximum": maximum,
            "final": final, "stats": stats, "nativeHistorySha256": row["nativeHistorySha256"]}


def draw(ax, history, channel, color, label):
    lo, hi = history["minimum"][channel], history["maximum"][channel]
    ax.fill_between(history["time"], lo, hi, color=color, alpha=.26, linewidth=0)
    ax.plot(history["time"], (lo + hi) / 2, color=color, linewidth=.85, label=label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, default=STUDY / "final_long4/results.json")
    parser.add_argument("--output", type=Path, default=STUDY / "figures/final")
    args = parser.parse_args()
    document = read(args.results)
    assert document["status"] == "complete" and document["lockedInputsUnchanged"]
    assert document["completedRecordings"] == 8
    lock = args.results.parent / "input_lock.json"
    assert sha(lock) == document["inputLockSha256"]
    consumed = args.results.parent / "frozen_inputs/consumed_scenarios.json"
    assert sha(consumed) == document["contract"]["frozenFiles"]["consumed_scenarios.json"]
    scenario = read(consumed)[1]
    rows = {phase: next(row for row in document["outcomes"][phase]["rows"] if row["scenario"] == 1)
            for phase in ("baseline", "selected")}
    baseline, selected = rows["baseline"], rows["selected"]
    assert baseline["identity"] == selected["identity"] == scenario["identity"]
    assert baseline["seed"] == selected["seed"] == scenario["seed"]
    assert baseline["completed"] and not baseline["physicalFailure"]
    assert selected["travelStop"] and selected["physicalFailure"] and not selected["completed"]
    assert all(row["fullNativeHistoryScored"] and row["retainedHistory"] for row in rows.values())
    histories = {phase: streamed_history(row, scenario) for phase, row in rows.items()}
    end = selected["endTime_s"]
    midpoint, radius = scenario["midTravel_m"], selected["travelBandRadius_m"] * 1000
    stops = [(scenario["travelMin_m"] - midpoint) * 1000, (scenario["travelMax_m"] - midpoint) * 1000]

    fig, axes = plt.subplots(3, 1, figsize=(11.7, 8.8), sharex=True)
    fig.subplots_adjust(left=.092, right=.96, top=.825, bottom=.14, hspace=.32)
    fig.suptitle("A long-run travel stop remains a system failure", x=.092, y=.975, ha="left", fontsize=17, fontweight="bold")
    fig.text(.092, .934,
             f"Same case and seed: S = {scenario['massS_kg']:g} kg · {scenario['force_N']:g} N · "
             f"{np.degrees(scenario['angle_rad']):g}° · sustained axial robot drift · seed {scenario['seed']}", fontsize=10)
    fig.text(.092, .899, f"PI + feedforward: completes 30 s; maximum travel {baseline['maxAbsTravelError_m'] * 1000:.3f} mm", color=BLUE, fontsize=11)
    fig.text(.092, .869, f"Learned policy: reaches travel stop at {end:.5f} s; maximum travel {selected['maxAbsTravelError_m'] * 1000:.3f} mm", color=ORANGE, fontsize=11)

    axes[0].axhspan(-radius, radius, color=BLUE, alpha=.065, zorder=0)
    for stop in stops:
        axes[0].axhline(stop, color=RED, linestyle="--", linewidth=1)
    for limit in (-radius, radius):
        axes[0].axhline(limit, color=GRAY, linestyle=":", linewidth=.8)
    axes[0].set(ylim=(stops[0] - 1, stops[1] + 1), ylabel="Travel from center (mm)")
    axes[0].set_title("Accepted region ±7.5 mm; physical stops ±12.5 mm", loc="left")
    axes[1].axhspan(-.5, .5, color=BLUE, alpha=.045, zorder=0)
    for value in (-.5, .5):
        axes[1].axhline(value, color=RED, linestyle="--", linewidth=.9)
    axes[1].set(ylim=(-.55, .55), ylabel="Normal-force error (N)")
    axes[1].set_title(f"Recorded peaks: baseline {baseline['peakNormalForceError_N']:.4f} N; learned {selected['peakNormalForceError_N']:.4f} N", loc="left")
    axes[2].set(ylabel="Actual robot velocity (mm/s)", xlabel="Time (s)")
    axes[2].set_title("Robot-side axial motion; each trace ends with its recording", loc="left")
    for phase, color, label in (("baseline", BLUE, "PI + feedforward"), ("selected", ORANGE, "Unified learned policy")):
        for channel, ax in enumerate(axes):
            draw(ax, histories[phase], channel, color, label)
    for ax in axes:
        ax.axvline(end, color=RED, linestyle="--", linewidth=.9, alpha=.85)
        ax.set_xlim(0, baseline["endTime_s"])
    axes[0].scatter([end], [histories["selected"]["final"][0]], color=RED, s=36, zorder=5)
    axes[0].legend(loc="lower right", ncol=2, fontsize=9, framealpha=.95)
    axes[0].annotate("Travel stop — recording ends", xy=(end, stops[1]), xytext=(end - 7.5, 10.6),
                     fontsize=9, color=RED, arrowprops={"arrowstyle": "->", "color": RED, "lw": .8})
    fig.text(.092, .038,
             "Every native sample is scored, including startup and the endpoint. Display envelopes retain each bin's minimum/maximum.\n"
             "The learned recording fails at the stop despite remaining within ±0.5 N before termination. Both traces exceed the soft travel band.",
             fontsize=9, color=GRAY, linespacing=1.5)
    args.output.mkdir(parents=True, exist_ok=True)
    stem = "rl_3h_long_stop"
    svg, png = args.output / (stem + ".svg"), args.output / (stem + ".png")
    fig.savefig(svg)
    fig.savefig(png, dpi=135)
    plt.close(fig)
    sources = [args.results, lock, consumed, *[Path(row["nativeHistoryFile"]) for row in rows.values()]]
    manifest = {
        "status": "complete", "plotScriptSha256": sha(Path(__file__)),
        "checkpointSha256": document["contract"]["checkpointSha256"],
        "caseIdentity": scenario["identity"], "scenario": 1, "seed": scenario["seed"],
        "cohortRecordings": document["completedRecordings"], "cohortNativeSamples": document["totalNativeSamples"],
        "plottedRecordings": 2, "plottedNativeSamples": sum(row["nativeSamples"] for row in rows.values()),
        "metrics": {phase: history["stats"] for phase, history in histories.items()},
        "sourceHashes": {str(path.resolve().relative_to(ROOT)): sha(path) for path in sources},
        "files": {svg.name: sha(svg), png.name: sha(png)},
        "display": "At most1800 min/max bins per history; full-native metrics recomputed independently.",
        "failureSemantics": "Physical travel stop fails the selected run; neither free flight nor subsequent impacts are simulated.",
        "newPlantEpisodesExecuted": 0,
    }
    (args.output / (stem + "_manifest.json")).write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": "complete", "plottedNativeSamples": manifest["plottedNativeSamples"],
                      "metrics": manifest["metrics"], "svg": str(svg)}, indent=2))


if __name__ == "__main__":
    main()
