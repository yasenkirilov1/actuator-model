"""Assemble retained native timing results without rerunning a simulation.

The terminated checkpoint and its external-termination record remain
unchanged. This joins its configurations with the eight-case private recovery;
the final checkpoint contained34 cases, so two recovery cases are benchmarks.
Run after the recovery completes; scoring is inherited from full native logs.
"""
from pathlib import Path
import hashlib
import html
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = Path(__file__).resolve().parent
STEM = "inertial_uncertainty_matched_acquisition_20260930"
FIRST = HERE / "inertial_uncertainty_v7_matched_acquisition" / f"{STEM}_results.json"
LAST = HERE / "inertial_uncertainty_v7_matched_acquisition_remainder" / (
    "inertial_uncertainty_matched_acquisition_remainder_20260930_results.json")
OUTPUT = HERE / "inertial_uncertainty_v7_timing_summary"


def read(path):
    return json.loads(path.read_text())


def main():
    first, last = read(FIRST), read(LAST)
    assert first["attempts"] == 34 and last["status"] == "complete"
    assert last["attempts"] == 8
    assert first["sourceModelSha256"] == last["sourceModelSha256"]
    assert first["candidateSha256"] == last["candidateSha256"]
    assert first["helperSources"] == last["helperSources"]
    rows = [dict(row, originalCaseId=row["id"]) for row in first["rows"]]
    recovery = [dict(row, originalCaseId=row["id"] + 32) for row in last["rows"]]
    repeated = [row for row in recovery if row["originalCaseId"] <= 34]
    rows += [row for row in recovery if row["originalCaseId"] > 34]
    assert [r["originalCaseId"] for r in rows] == list(range(1, 41))
    signatures = [(r["forceFilterTime_s"], r["forceDelay_s"],
                   r["accelerationFFfilter_s"], r["massS_actual_kg"],
                   r["reference_N"], r["angle_deg"], r["seed"],
                   r["forceNoiseRms_N"]) for r in rows]
    assert len(set(signatures)) == 40
    groups = []
    for timing in range(1, 5):
        for shaped in [False, True]:
            cohort = [r for r in rows if r["value"] == timing and
                      (r["accelerationFFfilter_s"] > 0) == shaped]
            assert len(cohort) == 5
            heavy = [r for r in cohort if r["massS_actual_kg"] == 2]
            light = [r for r in cohort if r["massS_actual_kg"] == .1]
            worst = max(cohort, key=lambda r: r["peakNormalForceError_N"])
            noise = next(r["accelNoiseRms_m_s2"] for r in heavy
                         if r["forceNoiseRms_N"] > 0)
            tau = cohort[0]["forceFilterTime_s"]
            exact_noise = .0123 * np.sqrt(np.tanh(.0001/(2*tau)) /
                                        np.tanh(.0001/(2*.0001)))
            groups.append(dict(
                timingIndex=timing, shaped=shaped,
                forceAndAccelFilterTime_s=tau,
                matchedTransport_s=cohort[0]["forceDelay_s"],
                measurementDcGroupDelay_s=cohort[0]["measurementTotalDcDelay_s"],
                accelerationPrediction_s=cohort[0]["accelerationPrediction_s"],
                declaredAccelOutputRms_m_s2=noise,
                exactDiscreteEnbwEquivalentOutputRms_m_s2=float(exact_noise),
                executed=len(cohort), passed=sum(r["passed"] for r in cohort),
                worstNormalError_N=worst["peakNormalForceError_N"],
                heavyWorstNormalError_N=max(r["peakNormalForceError_N"] for r in heavy),
                lightWorstNormalError_N=light[0]["peakNormalForceError_N"],
                margin_N=.5-worst["peakNormalForceError_N"],
                worstOriginalCaseId=worst["originalCaseId"],
                worstNativeTrace=worst["nativeTrace"],
                worstPeakTime_s=worst["peakNormalForceErrorTime_s"],
                originalCaseIds=[r["originalCaseId"] for r in cohort]))
    summary = dict(
        scope="40 unique native histories, each 10s repeated bump, every native normal-force sample including startup. Fixed PI .06/20, feedback LP1000Hz/damping1.5, guide400us, current100us, measured drag ratio5ms, no notch and no variation gate.",
        assembledStatus="complete40_unique_configurations_across34_partial_and8_recovery_executions",
        constituentResults=[str(FIRST), str(LAST)],
        constituentResultHashes=[hashlib.sha256(p.read_bytes()).hexdigest() for p in [FIRST, LAST]],
        preservedFirstCheckpointStatus=first["status"],
        initialPartialNativeExecutions=first["executed"],
        recoveryNativeExecutions=last["executed"],
        totalNativeExecutions=first["executed"]+last["executed"],
        repeatedBenchmarkExecutions=len(repeated),
        repeatedBenchmarkRows=repeated,
        duplicateBenchmarkEvidence=str(HERE / "inertial_uncertainty_v7_recovery_duplicate_equivalence.json"),
        sourceModelSha256=first["sourceModelSha256"],
        candidateSha256=first["candidateSha256"],
        plannedOriginalCases=40, validatedNativeHistories=sum(r["executed"] for r in rows),
        passed=sum(r["passed"] for r in rows),
        failedNative=sum(r["executed"] and not r["passed"] for r in rows),
        rejectedBeforeMetrics=sum(not r["executed"] for r in rows),
        voltageOrEffortClippingCases=sum(r["voltageClipFraction"] > 0 or
                                       r["effortClipFraction"] > 0 for r in rows),
        contactOrTravelFailures=sum(r["lostContact"] or r["strokeStop"] for r in rows),
        noiseConvention="The immutable primary screen uses continuous first-order ENBW scaling .0123*sqrt(100us/tau), with .02N force-output RMS held fixed. The exact native discrete EMA equivalent is 2.2-3.0% higher at 150-200us and is listed for selected-target confirmation. This bounded screening convention is not a calibrated hardware accuracy claim.",
        hardwareAcceptance="Zero transport removes additional whole-sample buffering at synchronous 10kHz acquisition. The identified filter still has analog/group phase. No exact ATI or mounted ADXL part transfer is asserted; bench identification is required.",
        limitations="Four heavy clean/noisy records and one light 90deg noisy endpoint per timing/strategy; no interior mass/force/orientation mesh, calibration tolerances or whole prototype qualification. Fixed guide400us intentionally isolates acquisition change and may require retuning.",
        groups=groups, rows=rows)
    OUTPUT.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(9.6, 4.1), layout="constrained")
    for shaped, offset, color, label in [(False, -.16, "#176e96", "Direct acceleration FF"),
                                         (True, .16, "#bd641e", "1ms FF filter + causal lead")]:
        gs = [g for g in groups if g["shaped"] == shaped]
        ax.bar(np.arange(4)+offset, [g["worstNormalError_N"] for g in gs], .29,
               color=color, alpha=.82, label=label)
        for x, g in zip(np.arange(4)+offset, gs):
            ax.text(x, g["worstNormalError_N"]+.013,
                    f'{g["passed"]}/5', ha="center", fontsize=9)
    ax.axhline(.5, color="#b1232b", linewidth=1.4, linestyle="--", label="±0.5N limit")
    ax.set_xticks(np.arange(4), ["100µs filter\n100µs transport", "100µs filter\n0 transport",
                                "150µs filter\n0 transport", "200µs filter\n0 transport"])
    ax.set_ylabel("Worst absolute normal-force error (N)")
    ax.set_ylim(0, max(g["worstNormalError_N"] for g in groups)*1.16)
    ax.grid(axis="y", alpha=.16)
    ax.set_axisbelow(True)
    ax.legend(loc="upper left", ncols=1, fontsize=9)
    fig.savefig(OUTPUT.with_suffix(".svg"))
    fig.savefig(OUTPUT.with_suffix(".png"), dpi=150)
    plt.close(fig)

    table_rows = "".join(
        f'<tr><td>{g["forceAndAccelFilterTime_s"]*1e6:.0f}/{g["matchedTransport_s"]*1e6:.0f}µs</td>'
        f'<td>{"Filtered + lead" if g["shaped"] else "Direct"}</td>'
        f'<td>{g["heavyWorstNormalError_N"]:.4f}N</td>'
        f'<td>{g["lightWorstNormalError_N"]:.4f}N</td>'
        f'<td>{g["passed"]}/5</td><td>{g["margin_N"]:.4f}N</td></tr>' for g in groups)
    page = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Native contact-force acquisition timing screen</title>
<style>body{{max-width:1000px;margin:32px auto;padding:0 20px;font:16px/1.5 system-ui;color:#18252e;background:#fafbfc}}h1{{font-size:28px}}img{{width:100%;background:white}}table{{border-collapse:collapse;width:100%;background:white}}th,td{{padding:8px 12px;text-align:left;border-bottom:1px solid #d7dfe3}}small{{color:#425563}}a{{color:#176e96}}</style>
<h1>Acquisition timing and the finite load-cell mode</h1>
<p>Forty unique native Simulink histories compare four synchronized timing targets and two acceleration-feedforward choices. Every 10s history contains seven sharp bumps. Bars show the largest error across every native sample; labels show the passing cases out of five.</p>
<img src="{OUTPUT.name}.svg" alt="Worst force error by filter and transport target, with the 0.5N limit">
<table><thead><tr><th>Filter/transport</th><th>Acceleration FF</th><th>Heavy worst</th><th>Light90°</th><th>Passed</th><th>Worst margin</th></tr></thead><tbody>{table_rows}</tbody></table>
<p>{html.escape(summary["hardwareAcceptance"])}</p>
<p>{html.escape(summary["noiseConvention"])}</p>
<p>{html.escape(summary["limitations"])}</p>
<p>The first batch stopped externally after 34 saved results. Its last printed progress was32, so the eight-case recovery also repeated cases33–34. Those histories remain preserved as duplicate benchmarks: there are42 executions and40 unique configurations. <a href="{OUTPUT.name}.json">Inspect the assembled data and evidence links</a>.</p>
<small>Model SHA256 {first["sourceModelSha256"]}. All histories use the same frozen input candidate and helpers.</small></html>'''
    OUTPUT.with_suffix(".html").write_text(page)
    print(json.dumps({"passed": summary["passed"], "groups": groups}, indent=2))


if __name__ == "__main__":
    main()
