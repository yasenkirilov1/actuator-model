"""Force-axis identifiability diagnostics; does not simulate or retune control.

Run with an existing numpy/scipy/matplotlib environment. All inputs are frozen
current-controller native records or retained qualification result JSONs.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "acf_simulink/validation/report_readability_20261006/loadcell"
QUAL = ROOT / "acf_simulink/validation/upper_no_damping_20261005/qualification_v8"
MAT = QUAL / "paired_voltage_matlab/usable_32V_native_recording.mat"
H, W, SPEED, MU = .001, .01, .1, .3
BLUE, ORANGE, OLIVE, GREY = "#245c96", "#c56a23", "#5d6e2b", "#4f5964"


def sha(p: Path):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def rel(p: Path):
    return str(p.relative_to(ROOT))


def csv_write(p, rows):
    with p.open("w", newline="") as f:
        writer = csv.DictWriter(f, list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def shape(q):
    h = 64*H*q**3*(1-q)**3
    slope = 192*H/W*q**2*(1-q)**2*(1-2*q)
    acc = 384*H/(W/SPEED)**2*q*(1-q)*(1-5*q+5*q*q)
    return h, slope, acc


def decorate(ax):
    ax.spines[["right", "top"]].set_visible(False)
    ax.grid(axis="y", color="#e7e9ed", linewidth=.8)
    ax.set_axisbelow(True)


def peak_signed(a):
    i = int(np.argmax(np.abs(a)))
    return i, float(a[i])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                         "axes.labelsize": 11, "svg.fonttype": "none"})
    qpeak = (1-1/np.sqrt(5))/2
    _, smax_a, _ = shape(qpeak)
    smax = float(smax_a)
    q = np.linspace(0, 1, 30001)
    h, slope, acc = shape(q)
    geometry = []
    for target in [5., 20., 50.]:
        normalizer = np.sqrt(1+slope*slope)
        axial = target*(1-MU*slope)/normalizer
        methods = {
            "axial_equals_normal": axial,
            "known_slope_ignores_drag": axial*normalizer,
            "known_slope_known_mu_oracle": axial*normalizer/(1-MU*slope),
            "known_slope_mu_0p2": axial*normalizer/(1-.2*slope),
            "known_slope_mu_0p25": axial*normalizer/(1-.25*slope),
            "known_slope_mu_0p4": axial*normalizer/(1-.4*slope),
        }
        for name, estimate in methods.items():
            i, err = peak_signed(estimate-target)
            geometry.append({"target_N": target, "method": name,
                             "maximumAbsoluteError_N": abs(err),
                             "signedErrorAtPeak_N": err,
                             "bumpProgressAtPeak": float(q[i]),
                             "slopeAtPeak": float(slope[i]),
                             "trueMu": MU,
                             "perfectSlopeAndAxialInertiaCorrection": True})
    csv_write(OUT / "analytical_axis_error.csv", geometry)

    # Same axial observation, different possible normal forces. These are a
    # sensitivity scenario, not measured friction bounds or confidence limits.
    Rz_nom = 50*(1-MU*slope)/np.sqrt(1+slope*slope)
    ambiguity_low = Rz_nom*np.sqrt(1+slope*slope)
    ambiguity_high = Rz_nom*np.sqrt(1+slope*slope)/(1-.6*slope)
    nmin, nmax = np.minimum(ambiguity_low, ambiguity_high), np.maximum(ambiguity_low, ambiguity_high)
    ambiguity_csv = [{"progress": float(q[i]), "height_mm": float(h[i]*1000),
                      "slope": float(slope[i]), "axialObservation_N": float(Rz_nom[i]),
                      "nominalNormal_N": 50., "muScenarioMin": 0., "muScenarioMax": .6,
                      "admissibleNormalMin_N": float(nmin[i]),
                      "admissibleNormalMax_N": float(nmax[i])}
                     for i in range(0, len(q), 30)]
    csv_write(OUT / "same_axial_reading_ambiguity.csv", ambiguity_csv)

    # Explicit native column contract: output order is N + 25 plant signals
    # + slope; lower sensing-cell force is truth[:,24], not actuator force.
    d = loadmat(MAT, simplify_cells=True)
    p, t, truth = d["p"], d["nativeTime"], d["truth"]
    N, Rz, Lz, s = truth[:,0], truth[:,1], truth[:,24], truth[:,26]
    target, mass = float(p["force"]["reference"]), float(p["mechanics"]["mS"])
    duration = float(p["surface"]["width"] / p["surface"]["scanSpeed"])
    qq = (t - p["surface"]["startTime"]) / duration
    _, analytic_s, az = shape(np.clip(qq, 0, 1))
    az[(qq<0)|(qq>1)] = 0
    analytic_s[(qq<0)|(qq>1)] = 0
    mu = float(p["contact"]["mu"])
    norm = np.sqrt(1+s*s)
    Rx = -N*(s+mu)/norm
    N2 = (Rz-s*Rx)/norm
    methods_native = {
        "axial_equals_normal": Rz,
        "known_slope_ignores_drag": Rz*norm,
        "known_slope_known_mu_oracle": Rz*norm/(1-mu*s),
        "known_slope_mu_0p2": Rz*norm/(1-.2*s),
        "known_slope_mu_0p4": Rz*norm/(1-.4*s),
        "two_force_axes_perfect_measurements": N2,
    }
    native_metrics = []
    for name, estimate in methods_native.items():
        i, err = peak_signed(estimate-N)
        native_metrics.append({"method": name,
                               "peakReconstructionError_N": abs(err),
                               "signedReconstructionErrorAtPeak_N": err,
                               "peakTime_s": float(t[i]),
                               "peakErrorAgainstTarget_N": float(np.max(np.abs(estimate-target))),
                               "scope": "Offline perfect-sensor/truth-geometry diagnostic on unchanged native controller record"})
    csv_write(OUT / "native_axis_postprocessing_metrics.csv", native_metrics)
    ot, observed = d["observationTime"], d["observation"]
    normal_at_observed_times = np.interp(ot, t, N)
    observed_norm = np.sqrt(1+observed[:,8]**2)
    acquired_methods = {
        "executing_acquired_normal_estimate": observed[:,0],
        "acquired_axial_known_estimated_slope_ignores_drag": observed[:,1]*observed_norm,
        "acquired_axial_known_mu_oracle": observed[:,1]*observed_norm/(1-mu*observed[:,8]),
        "acquired_two_force_axes_projection": (observed[:,1]-observed[:,8]*observed[:,2])/observed_norm,
    }
    acquired_metrics = []
    for name, estimate in acquired_methods.items():
        error = estimate-normal_at_observed_times
        i, err = peak_signed(error)
        acquired_metrics.append({"method":name,"acquiredObservationPoints":len(ot),
                                 "maximumDifferenceFromPresentTrueNormal_N":abs(err),
                                 "signedDifferenceAtPeak_N":err,"peakTime_s":float(ot[i]),
                                 "rmsDifferenceFromPresentTrueNormal_N":float(np.sqrt(np.mean(error**2))),
                                 "scope":"Recorded acquired signals compared with present-time truth at exact output timestamps; includes modeled noise, response, delay and slope-estimation error. This is not a calibrated bias, individual ADC error or closed-loop force metric."})
    csv_write(OUT / "acquired_axis_postprocessing_metrics.csv", acquired_metrics)
    guards = {
        "nativePoints": len(t),
        "surfaceSlopeAgainstAnalyticalMaximumDifference": float(np.max(np.abs(s-analytic_s))),
        "axialGeometryMaximumResidual_N": float(np.max(np.abs(Rz-N*(1-mu*s)/norm))),
        "cellForceBalanceMaximumResidual_N": float(np.max(np.abs(Rz-(Lz+mass*(p['d']['g']+az))))),
        "twoAxesProjectionMaximumResidual_N": float(np.max(np.abs(N2-N))),
        "trueNormalForcePeakError_N": float(np.max(np.abs(N-target))),
        "minimumRawAxialCell_N": float(np.min(Lz)),
        "maximumRawAxialCell_N": float(np.max(Lz)),
        "peakToolInertiaCorrection_N": float(np.max(np.abs(mass*az))),
        "executingEstimatorVersusAcquiredProjectionMaximumDifference_N":float(np.max(np.abs(observed[:,0]-acquired_methods['acquired_two_force_axes_projection']))),
    }
    assert guards["cellForceBalanceMaximumResidual_N"] < 1e-10
    assert guards["axialGeometryMaximumResidual_N"] < 1e-10
    assert guards["surfaceSlopeAgainstAnalyticalMaximumDifference"] < 1e-12

    # Chart reduction preserves minima and maxima, while metrics above score
    # every native point. Visible range includes the complete bump and ringing.
    use = np.flatnonzero((t >= .985) & (t <= 1.115))
    bins = np.array_split(use, 1500)
    keys = ["known_slope_ignores_drag", "known_slope_mu_0p2"]
    chosen = set([int(use[0]), int(use[-1])])
    for ids in bins:
        if not len(ids): continue
        for ar in [N, Lz, *[methods_native[k] for k in keys]]:
            chosen.add(int(ids[np.argmin(ar[ids])]))
            chosen.add(int(ids[np.argmax(ar[ids])]))
    ix = np.array(sorted(chosen))
    native_rows = [{"nativeIndex": int(i), "timeSinceBumpStart_ms": float((t[i]-1)*1000),
                    "trueNormal_N": float(N[i]), "rawAxialCell_N": float(Lz[i]),
                    "axialContactAfterPerfectInertia_N": float(Rz[i]),
                    "knownSlopeIgnoringDrag_N": float(methods_native[keys[0]][i]),
                    "knownSlopeWrongMu0p2_N": float(methods_native[keys[1]][i]),
                    "perfectTwoAxisReconstruction_N": float(N2[i]),
                    "trueSlope": float(s[i])} for i in ix]
    csv_write(OUT / "native_axis_figure_rows.csv", native_rows)

    capacity, input_hashes = [], {rel(MAT): sha(MAT)}
    for cohort in ["nominal700", "fresh32", "long4"]:
        src = QUAL / "evaluation" / cohort / "results.json"
        result = json.loads(src.read_text())
        input_hashes[rel(src)] = sha(src)
        rows = result["rows"]
        imin = min(rows, key=lambda r: r["minLowerLinkForce_N"])
        imax = max(rows, key=lambda r: r["maxLowerLinkForce_N"])
        capacity.append({"cohort": cohort, "recordings": len(rows),
                         "minimumRawAxialCell_N": imin["minLowerLinkForce_N"],
                         "minimumScenario": imin["scenario"],
                         "maximumRawAxialCell_N": imax["maxLowerLinkForce_N"],
                         "maximumScenario": imax["scenario"],
                         "scope": "Actual native cell loads, pre-tare; stops truncate physical histories"})
    csv_write(OUT / "locked_recording_capacity.csv", capacity)
    for src in [ROOT/"acf_simulink/generated/fixed_upper_20261005/acf_force_control_fixed_upper_20261005.slx",
                ROOT/"acf_simulink/experiments/build_normal_contact_trial.m",
                ROOT/"acf_simulink/experiments/build_normal_contact_trial_v3.m",
                QUAL/"selected_native_contracts_v1.json", QUAL/"candidate_lock.json"]:
        input_hashes[rel(src)] = sha(src)

    fig, axes = plt.subplots(2, 1, figsize=(10.4, 6.8), gridspec_kw={"height_ratios": [2.1, 1]})
    ax = axes[0]
    ax.axhspan(49.5, 50.5, color="#eef2f6", label="±0.5 N around target")
    ax.plot((t[ix]-1)*1000, N[ix], color=BLUE, lw=2.1, label="True normal force / ideal two-axis reconstruction")
    ax.plot((t[ix]-1)*1000, methods_native[keys[0]][ix], color=ORANGE, lw=1.8, label="Single axis: known slope, ignore drag")
    ax.plot((t[ix]-1)*1000, methods_native[keys[1]][ix], color=OLIVE, lw=1.8, ls="--", label="Single axis: assume μ = 0.2; actual μ = 0.3")
    ax.axhline(50, color=GREY, lw=1, ls=":")
    ax.set_ylabel("Reconstructed normal force (N)")
    ax.set_ylim(43.4, 56.6)
    ax.legend(loc="upper left", fontsize=9.5, frameon=False)
    decorate(ax)
    axes[1].plot((t[ix]-1)*1000, s[ix], color=GREY, lw=1.8)
    axes[1].axhline(0, color=GREY, lw=.8)
    axes[1].set_ylabel("Surface slope dh/dx")
    axes[1].set_xlabel("Time since bump starts (ms)")
    axes[1].set_ylim(-.40,.40)
    decorate(axes[1])
    for ax in axes: ax.set_xlim(-15,115)
    fig.suptitle("Does one force axis tell us the pressing force?", x=.055, ha="left", fontsize=17, fontweight="bold")
    fig.tight_layout(rect=[0,.13,1,.96])
    fig.text(.055, .025, "Unchanged native development case: S = 2 kg, N target = 50 N. Offline diagnostic with perfect inertia and slope.\nNo one-axis controller was simulated; friction-oracle and two-axis truth reconstruction overlap the blue curve.", fontsize=9.5)
    fig.savefig(OUT / "single_axis_native_diagnostic.svg", bbox_inches="tight", facecolor="white")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.4, 5.1))
    targets = np.array([5,20,50]); xx = np.arange(3); width=.31
    method_names = ["known_slope_ignores_drag", "known_slope_mu_0p2"]
    labels = ["Ignore sanding drag", "Assume μ = 0.2; actual μ = 0.3"]
    for j,(name,label,color) in enumerate(zip(method_names,labels,[ORANGE,OLIVE])):
        vals=[next(r["maximumAbsoluteError_N"] for r in geometry if r["method"]==name and r["target_N"]==v) for v in targets]
        bars=ax.bar(xx+(j-.5)*width,vals,width,color=color,label=label)
        ax.bar_label(bars,labels=[f"{v:.3f} N"for v in vals],padding=5,fontsize=10)
    ax.axhline(.5,color=GREY,lw=1.5,ls="--",label="Whole-system error limit: 0.5 N")
    ax.set_xticks(xx,["5 N","20 N","50 N"]);ax.set_xlabel("True normal pressing force")
    ax.set_ylabel("Maximum absolute reconstruction error (N)");ax.set_ylim(0,6.0)
    ax.legend(frameon=False,loc="upper left",fontsize=10)
    decorate(ax)
    ax.set_title("Even perfect axial sensing leaves a drag-dependent error",loc="left",fontsize=16,fontweight="bold",pad=18)
    fig.tight_layout(rect=[0,.16,1,1])
    fig.text(.06,.025,"Analytical geometry diagnostic: 1 mm / 10 mm C² bump, maximum slope 0.34346 (18.96°).\nKnown surface direction and perfect gravity/inertia correction assumed; this is not closed-loop qualification.",fontsize=9.5)
    fig.savefig(OUT / "single_axis_error_by_force.svg",bbox_inches="tight",facecolor="white");plt.close(fig)

    fig, ax = plt.subplots(figsize=(10.4,4.8))
    geo_ix = np.arange(0, len(q), 30)
    ax.fill_between(q[geo_ix]*10,nmin[geo_ix],nmax[geo_ix],color="#ead7c3",alpha=.9,label="Same axial reading, μ anywhere from 0 to 0.6")
    ax.plot(q[geo_ix]*10,np.full_like(q[geo_ix],50),color=BLUE,lw=2,label="Native nominal drag scenario: μ = 0.3, N = 50 N")
    ax.axhspan(49.5,50.5,facecolor="none",edgecolor=GREY,hatch="///",linewidth=0,label="±0.5 N around target")
    ax.set_xlabel("Position through bump (mm)");ax.set_ylabel("Possible normal force (N)")
    ax.set_xlim(0,10);ax.set_ylim(38,68);ax.legend(frameon=False,fontsize=10,loc="upper right");decorate(ax)
    ax.set_title("A single axial reading admits several normal forces",loc="left",fontsize=16,fontweight="bold",pad=18)
    fig.tight_layout(rect=[0,.17,1,1])
    fig.text(.06,.025,"Analytical sensitivity scenario, not a measured friction interval. All curves produce exactly the same Rz at each position.\nAn axial accelerometer removes inertia; it does not provide the missing tangential contact force.",fontsize=9.5)
    fig.savefig(OUT / "single_axis_force_ambiguity.svg",bbox_inches="tight",facecolor="white");plt.close(fig)

    # Relative normal-direction error rotates normal into tangential force.
    delta = np.deg2rad(.05)
    angular_error = 50*(1-np.cos(delta)+MU*np.sin(delta))
    summary = {
        "status": "complete", "kind": "load_cell_axis_identifiability_and_native_postprocessing",
        "generatedAtUtc": datetime.now(timezone.utc).isoformat(),
        "controllerRetuned": False, "newClosedLoopEpisodes": 0,
        "geometry": {"height_m":H,"width_m":W,"scanSpeed_m_s":SPEED,"bumpDuration_s":W/SPEED,
                     "polynomial":"h(q)=64 H q^3 (1-q)^3",
                     "maximumAbsSlope":smax,"maximumSlopeAngle_deg":float(np.rad2deg(np.arctan(smax))),
                     "slopeMaximumProgress":float(qpeak),"actualNominalMu":MU,
                     "gravityOrientationIsSeparateFromLocalBumpSlope":True},
        "equations": {
            "surfaceNormal":"n=(-s,1)/sqrt(1+s^2)", "scanTangent":"t=(1,s)/sqrt(1+s^2)",
            "contactForceOnTool":"R=N*n-T*t; T=mu*N in the retained plant",
            "singleAxisObservation":"Rz=(N-s*T)/sqrt(1+s^2)",
            "perfectKnownFrictionOracle":"N=Rz*sqrt(1+s^2)/(1-mu*s)",
            "twoForceAxisReconstruction":"N=(Rz-s*Rx)/sqrt(1+s^2)",
            "loadCellSignConvention":"Lraw is minus support force on S: Lraw=R+mS*gVector-mS*aS",
            "gravityTaredReconstruction":"Ltared=Lraw-mS*gVector; R=Ltared+mS*aS",
            "nativeAxialConvention":"Lraw_z=Rz-mS*(gPositiveAlongFlange+aS_z), where gPositiveAlongFlange=9.81*cos(angle)",
            "knownFrictionMismatch":"Nhat/N=(1-mu*s)/(1-muhat*s)",
        },
        "analyticalRows":geometry,"nativePostprocessingRows":native_metrics,
        "acquiredPostprocessingRows":acquired_metrics,
        "nativeGuards":guards,"nativeScenario":{"scenario":172,"massA_kg":1.5,"massB_kg":.6,"massS_kg":mass,"force_N":target,"orientation_rad":float(p['mechanics']['angle']),"seed":int(p['sensor']['seed']),"scope":"Frozen 32V unchanged-controller development voltage replay"},
        "nativeCapacityRows":capacity,
        "capacityTargets": {
            "axialMeasurementRangeDevelopmentTarget_N":100,
            "existingAdcBipolarHalfRange_N":160,
            "transverseDevelopmentTarget_N":100,
            "planarConservativeRawSideForceBoundAtN50Mu0p3Mass2_N":float(50*(smax+.3)/np.sqrt(1+smax*smax)+2*9.81),
            "rawSideBoundIncludesPossibleGravityAdditionNotQualificationMaximum":True,
            "momentExampleForce_N":50,"momentExampleLeverArm_m":.1,"momentExample_Nm":5,
            "qualificationIsNotCollisionSizing":True,
            "mechanicalCombinedLoadEnvelopeRequired":True,
        },
        "calibrationTargets":{
            "residualForceZeroPerAxis_N":.01,"residualGainFraction":.0005,
            "residualCrossAxisFraction":.0005,"relativeAlignment_deg":.05,
            "residualAccelerationBias_m_s2":.005,"relativeTimingResidual_s":2e-6,
            "approximateNormalBiasFromTwoForceAxesAtMaximumSlope_N":float(.01*(1+smax)/np.sqrt(1+smax*smax)),
            "normalErrorAt50N_mu0p3_directionError0p05deg_N":float(angular_error),
            "wholeRequirementIsNotSensorErrorBudget":True,
            "targetsAreNotCatalogGuaranteesOrNewQualification":True,
        },
        "sourceHashes":input_hashes,
        "sourceNotes":"Current saved graph and builder sources verify equation/sign/log contracts. Main figure uses all original native samples for metrics and a min/max-preserving display subset. The friction oracle and true normal are diagnostic only and never controller inputs.",
        "figures":[{"file":rel(OUT/name),"sha256":sha(OUT/name)} for name in ['force_axes_and_inertia.svg','single_axis_native_diagnostic.svg','single_axis_error_by_force.svg','single_axis_force_ambiguity.svg']],
    }
    (OUT / "loadcell_evidence.json").write_text(json.dumps(summary,indent=2)+"\n")
    print(json.dumps({"guards":guards,"capacity":capacity,"geometry":summary['geometry'],"files":len(list(OUT.iterdir()))},indent=2))


if __name__ == "__main__": main()
