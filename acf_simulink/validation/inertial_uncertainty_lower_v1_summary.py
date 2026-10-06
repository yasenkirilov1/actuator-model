#!/usr/bin/env python3
"""Assemble retained native one-factor evidence; never rescore plot samples."""
from __future__ import annotations

import hashlib
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
STEM = "inertial_uncertainty_lower_v1_summary"
COHORTS = {
    "core": "inertial_uncertainty_lower_v1_core/inertial_uncertainty_core_sensitivity_20260930_results.json",
    "identified_holder": "inertial_uncertainty_lower_v1_holder_serial/inertial_uncertainty_holder_mass_20260930_results.json",
    "identified_scan_speed": "inertial_uncertainty_lower_v1_scan_serial/inertial_uncertainty_scan_speed_20260930_results.json",
}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    files = {name: ROOT / path for name, path in COHORTS.items()}
    cohorts = {name: json.loads(path.read_text()) for name, path in files.items()}
    assert all(c["status"] == "complete" for c in cohorts.values())
    hashes = {(c["sourceModelSha256"], c["candidateSha256"]) for c in cohorts.values()}
    assert len(hashes) == 1, "Do not combine different source/candidate versions."
    rows = [dict(r, cohort=name) for name, c in cohorts.items() for r in c["rows"]]
    assert all(r["executed"] and r["nativeReturned"] for r in rows)
    assert all(c["sourceStillMatches"] and c["privateSourceUnmodified"] and
               c["sourceHelpersStillMatch"] for c in cohorts.values())
    scenarios = {}
    for row in rows:
        scenarios.setdefault(row["scenario"], []).append(row)
    groups = []
    for name, group in scenarios.items():
        worst = max(group, key=lambda r: r["peakNormalForceError_N"])
        groups.append(dict(scenario=name, executed=len(group), passed=sum(r["passed"] for r in group),
                           parameterValues=sorted({r["value"] for r in group}),
                           worstNormalError_N=worst["peakNormalForceError_N"],
                           worstRow=worst))
    failures = [r for r in rows if not r["passed"]]
    clipping = [r for r in rows if r["voltageClipFraction"] or r["effortClipFraction"]]
    summary = dict(
        scope="112 four-second one-factor native recordings on the lower-series legacy-sensor candidate. "
              "This is sensitivity evidence, not a joint uncertainty allowance or full-range qualification.",
        metric="max(abs(normalContact.data(:,1)-reference)) over every native sample including startup; "
               "contact/travel/overcurrent/incomplete/impact and configured spring packing also fail.",
        sourceModelSha256=next(iter(hashes))[0], candidateSha256=next(iter(hashes))[1],
        evidence=[dict(cohort=name, file=str(path.relative_to(ROOT)), sha256=sha(path),
                       executed=cohorts[name]["executed"], passed=cohorts[name]["passed"],
                       failed=cohorts[name]["failedNative"], rejected=cohorts[name]["rejectedBeforeMetrics"])
                  for name, path in files.items()],
        executed=len(rows), passed=sum(r["passed"] for r in rows), failed=len(failures), rejected=0,
        scenarios=groups, failedRows=failures, clippingRows=clipping,
        noContactLoss=not any(r["lostContact"] for r in rows),
        noTravelStop=not any(r["strokeStop"] for r in rows),
        allCompleted=all(r["completed"] for r in rows),
        maximumForceBalanceResidual_N=max(r["axialForceBalanceResidual_N"] for r in rows),
        maximumNormalBalanceResidual_N=max(r["normalBalanceResidual_N"] for r in rows),
        configuredPreload_N=33.6,
        allConfiguredPreloadsPass=all(r["configuredCommonPreloadPassed"] and r["packingPassed"] for r in rows),
        commonFeasiblePreloadIntersection_N=[max(r["feasibleCommonPreloadMin_N"] for r in rows),
                                            min(r["feasibleCommonPreloadMax_N"] for r in rows)],
        combinedFreeLengthMax_mm=max(r["combinedFreeLengthMax_mm"] for r in rows),
        combinedInstalledLengthMax_mm=max(r["combinedInstalledLengthMax_mm"] for r in rows),
        assumptions="B=0.6 kg; lower spring19746 N/m plus damper60 Ns/m; upper rigid; "
                    "64 MN/m massless series cell; physical startup current uses actual masses/Kf; "
                    "force/acceleration100us EMA plus100us transport; controller PI0.3/20, "
                    "1kHz Butterworth feedback, proper identified lower-link inverse1.5kHz; "
                    "issued robot predictor80/60ms and recenter gain0.735294/s held fixed.",
        interpretation="Only actual robot transport mismatch produced normal-force failures in this cohort. "
                       "Large extra IMU noise can pass force tolerance while briefly clipping voltage/current command. "
                       "Do not infer bias/scale/timing tolerances from output RMS noise.",
    )
    (ROOT / f"{STEM}.json").write_text(json.dumps(summary, indent=2) + "\n")
    labels = {
        "baseline": "Nominal seeded endpoints", "accelerometer_bias": "Acceleration bias ±0.05 m/s²",
        "accelerometer_scale": "Acceleration gain ±1%", "axial_force_bias": "Axial force bias ±0.1 N",
        "force_vector_scale": "Force vector gain ±0.2%", "force_vector_noise": "Force RMS noise up to 0.1 N",
        "accelerometer_noise": "Acceleration RMS noise up to 0.05 m/s²", "identified_S_mass": "Identified S mass ±1%",
        "position_noise": "Position RMS noise 3 µm", "position_scale": "Position gain ±1%",
        "robot_actual_delay": "Actual robot delay 60/100/120 ms", "robot_actual_response": "Actual robot response 40/100 ms",
        "identified_B_mass": "Identified B mass ±1%", "identified_scan_speed": "Issued scan-speed gain ±5%",
    }
    shown = [g for g in groups if g["scenario"] in labels]
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(10.5, 7.5), constrained_layout=True)
    y = np.arange(len(shown))
    values = [g["worstNormalError_N"] for g in shown]
    ax.barh(y, values, color=["#b91c1c" if g["passed"] != g["executed"] else "#0369a1" for g in shown], height=.65)
    ax.set_yticks(y, [labels[g["scenario"]] for g in shown])
    ax.invert_yaxis(); ax.axvline(.5, color="#b91c1c", linestyle="--", linewidth=1.4)
    for i, g in enumerate(shown):
        ax.text(g["worstNormalError_N"] + .01, i, f"{g['worstNormalError_N']:.3f} N · {g['passed']}/{g['executed']} pass", va="center", fontsize=9)
    ax.set_xlim(0, 1.05); ax.set_xlabel("Worst absolute normal force error over all native samples (N)")
    ax.set_title("Lower-series candidate: separate sensitivities\nOne-factor results do not establish combined tolerances", loc="left", pad=14)
    fig.savefig(ROOT / f"{STEM}.svg"); fig.savefig(ROOT / f"{STEM}.png", dpi=160); plt.close(fig)
    diagnostic = ROOT / "inertial_uncertainty_lower_v1_core/inertial_uncertainty_robot_delay_peak_decomposition.csv"
    native = np.genfromtxt(diagnostic, delimiter=",", names=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    axes[0].plot(native["time_s"], native["normalError_N"], label="Actual workpiece normal error", color="#b91c1c")
    axes[0].plot(native["time_s"], native["estimatedError_N"], label="Controller normal error estimate", color="#0369a1")
    axes[0].axhline(.5, color="#b91c1c", linestyle="--"); axes[0].axhline(-.5, color="#b91c1c", linestyle="--")
    axes[0].set_ylabel("Force error (N)"); axes[0].legend(loc="upper left", fontsize=9)
    axes[1].plot(native["time_s"], native["robotVelocity_m_s"] * 1000, label="Actual robot normal speed", color="#b91c1c")
    axes[1].plot(native["time_s"], native["predictedRobotVelocity_m_s"] * 1000, label="Issued-command predictor", color="#0369a1")
    axes[1].set_ylabel("Speed (mm/s)"); axes[1].legend(loc="upper right", fontsize=9)
    axes[2].plot(native["time_s"], native["trueSlope"], label="Physical surface slope (audit only)", color="#b91c1c")
    axes[2].plot(native["time_s"], native["estimatedSlope"], label="Causal slope estimate", color="#0369a1")
    axes[2].set_ylabel("Surface slope"); axes[2].set_xlabel("Native recording time (s)"); axes[2].legend(loc="upper right", fontsize=9)
    axes[0].set_title("Actual robot delay120 ms versus identified80 ms, heavy tool seed7\nPeak occurs on the flat after the bump; mean/rate force gate remains ready", loc="left", pad=12)
    fig.savefig(ROOT / f"{STEM}_robot_delay.svg"); fig.savefig(ROOT / f"{STEM}_robot_delay.png", dpi=160); plt.close(fig)
    table = "\n".join(f"<tr><td>{html.escape(g['scenario'])}</td><td>{g['executed']}</td><td>{g['passed']}</td>"
                      f"<td>{g['worstNormalError_N']:.9f}</td></tr>" for g in groups)
    page = f"""<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lower-series native sensitivity evidence</title><style>body{{font:16px/1.5 system-ui;background:#f7fafc;color:#172b3a;max-width:1120px;margin:32px auto;padding:0 20px}}img{{width:100%;height:auto;background:white}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:8px;border-bottom:1px solid #ccd8e2;text-align:left}}code{{overflow-wrap:anywhere}}.limit{{border-left:4px solid #b91c1c;padding:12px;background:#fff1f2}}</style>
<h1>Native lower-series sensitivity evidence</h1><p><strong>112 executed histories: 107 passes and 5 force failures.</strong> Two additional endpoint configurations have six paired software proof recordings with bitwise uninstrumented/zero-instrument and Normal/Accelerator equality.</p>
<p class="limit">These are separate one-factor sensitivities on the legacy sensing source. They establish neither a combined sensor budget nor a full-range uncertainty qualification. Final actual-AA joint cases are pending their own recordings.</p>
<p>Every force score uses the complete native normal-force history, including startup. The five failures vary actual robot transport while keeping the issued-command predictor fixed. All histories complete without contact loss or travel stops; the large added acceleration-noise points briefly clip drive effort or voltage.</p>
<img src="{STEM}.svg" alt="Separate one-factor peak normal-force error"><h2>Robot-delay failure</h2><img src="{STEM}_robot_delay.svg" alt="Native actual force, estimated force, actual robot and predicted speed, and causal surface slope around the retained peak">
<p>At 2.1263 s, the workpiece experiences 50.777479 N while the controller estimates 50.019716 N. Actual robot speed is +2.138 mm/s while its issued-command predictor gives −2.199 mm/s. The slope estimate is −0.0453 on a physically flat surface. No voltage clipping occurs in this failure. Queued motion and an incorrect robot predictor can therefore defeat the mean/rate force-stability gate.</p>
<h2>Recorded groups</h2><table><thead><tr><th>Scenario</th><th>Executed</th><th>Passed</th><th>Worst normal error (N)</th></tr></thead><tbody>{table}</tbody></table>
<p>The configured 33.6 N common preload passes every history. The shared feasible interval is 30.449–36.786 N; combined free/installed spring lengths are 37.18/30.374 mm. This verifies the spring-only packing constraint for these traces.</p>
<p>Frozen model SHA256:<code>{summary['sourceModelSha256']}</code><br>Candidate SHA256:<code>{summary['candidateSha256']}</code>. Exact source/helper/recipe hashes, actual and identified parameters, failed full histories and native balance audits are linked in <a href="{STEM}.json">the machine-readable evidence</a>.</p></html>"""
    (ROOT / f"{STEM}.html").write_text(page)
    print(json.dumps({"executed": len(rows), "passed": summary["passed"], "failed": len(failures),
                      "summary": str(ROOT / f"{STEM}.json")}, indent=2))


if __name__ == "__main__":
    main()
