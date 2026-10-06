"""Aggregate pointwise audits; never simulate or load a full native-history MAT."""

from pathlib import Path
import csv
import hashlib
import json
import math

from scipy.io import loadmat


HERE = Path(__file__).resolve().parent
PREFIX = "normal_voltage_calibrated_static_headroom_20260930"


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


audits = []
for family, count in [("long", 4), ("mixed", 3)]:
    for index in range(1, count + 1):
        path = HERE / f"{PREFIX}_{family}_case_{index:04d}.json"
        data = json.loads(path.read_text())
        assert data["family"] == family and data["nativeCaseId"] == index
        assert data["noNewNativeSimulation"] and data["loadedStartupIncluded"]
        assert data["genericCompleted"] and data["genericNormalPeakError_N"] <= 0.5
        assert data["genericVoltageClippingSamples"] == 0
        assert data["genericEffortClippingSamples"] == 0
        assert data["genericRequestedVsAppliedVoltageResidual_V"] < 1e-8
        assert data["mappedHotRequestedVsDirectRegulatorResidual_V"] < 1e-10
        assert data["mappedCurrentCommandWithin3p5A"]
        assert data["calibrationPreservation"]["identifiedToPhysicalKfRatioUnchanged"]
        assert data["mapping"]["currentResponse_s"] == 0.0001
        assert data["timeRange_s"][0] == 0
        assert data["nativeSamples"] == (3_000_001 if family == "long" else 400_001)
        audits.append((path, data))

# Stream and hash source files, including the seven MATs, without loading arrays.
sources = {}
for _, data in audits:
    for source in data["sources"]:
        prior = sources.get(source["file"])
        assert prior is None or prior == source
        sources[source["file"]] = source
for source in sources.values():
    path = Path(source["file"])
    assert path.stat().st_size == source["sizeBytes"]
    assert sha(path) == source["sha256"], f"Source changed after audit: {path}"

long_report_path = next(
    Path(s["file"]) for s in sources.values()
    if s["file"].endswith("inertial_uncertainty_calibrated_static_long_20260930_results.json")
)
mixed_report_path = HERE / "inertial_mechanics_calibrated_static_mixed_20260930_results.json"
long_report = json.loads(long_report_path.read_text())
mixed_report = json.loads(mixed_report_path.read_text())
assert long_report["sourceStillMatches"] and long_report["privateSourceUnmodified"]
assert long_report["sourceHelpersStillMatch"]
assert mixed_report["baseV8NativeModelSHA256"] == long_report["sourceModelSha256"]
assert long_report["helperSources"] == mixed_report["helperSources"]

mixed_manifest_path = HERE / "inertial_mechanics_calibrated_static_mixed_20260930_invocation_manifest.json"
mixed_manifest = json.loads(mixed_manifest_path.read_text())
mixed_recipe = Path(next(
    x["file"] for x in mixed_manifest["inputs"]
    if x["sha256"] == mixed_report["recipeSHA256"]
))
assert sha(mixed_recipe) == mixed_report["recipeSHA256"]
sources[str(mixed_recipe)] = {
    "file": str(mixed_recipe), "sha256": sha(mixed_recipe),
    "sizeBytes": mixed_recipe.stat().st_size,
}
sources[str(mixed_manifest_path)] = {
    "file": str(mixed_manifest_path), "sha256": sha(mixed_manifest_path),
    "sizeBytes": mixed_manifest_path.stat().st_size,
}
mixed_configs = loadmat(
    mixed_report["preparedConfigurationFile"],
    variable_names=["configs"], simplify_cells=True,
)["configs"]
assert len(mixed_configs) == 3
for helper in long_report["helperSources"]:
    path = Path(long_report["sourceModel"]).parent / "helpers" / helper["file"]
    assert sha(path) == helper["sha256"]
    sources[str(path)] = {
        "file": str(path), "sha256": helper["sha256"], "sizeBytes": path.stat().st_size,
    }

rows = []
range_fields = [
    ("genericVoltageRange_V", "generic_applied_voltage", "V"),
    ("genericRequestedVoltageRange_V", "generic_requested_voltage", "V"),
    ("genericPhysicalCurrentRange_A", "generic_current", "A"),
    ("genericCurrentCommandRange_A", "generic_current_command", "A"),
    ("genericAppliedCurrentCommandRange_A", "generic_limited_current_command", "A"),
    ("genericCurrentTrackingErrorRange_A", "generic_current_tracking_error", "A"),
    ("gapVelocityRange_m_s", "gap_velocity", "m_s"),
    ("mappedHotRequestedVoltageRange_V", "mapped_hot_requested_voltage", "V"),
    ("mappedRealPhysicalCurrentRange_A", "mapped_real_current", "A"),
    ("mappedRealCurrentCommandRange_A", "mapped_real_current_command", "A"),
]
for path, data in audits:
    native = data["nativeSourceRow"]
    calibration = data["calibrationPreservation"]
    peak = data["mappedHotRequestedVoltageRange_V"]["peakAbsolute"]
    if data["family"] == "mixed":
        config = mixed_configs[data["nativeCaseId"] - 1]["p"]
        assert native["preparedParametersPreservedAfterRun"]
        assert config["sensor"]["seed"] == native["seed"]
        assert config["coil"]["forceConstant"] == 20
        assert config["coil"]["resistance"] == 4
        assert config["coil"]["inductance"] == 0.003
        assert config["coil"]["currentResponse"] == 0.0001
        assert config["coil"]["voltage"] == 24
        assert config["coil"]["peakCurrent"] == 3.5
        assert config["mechanics"]["upperRigid"]
        assert config["mechanics"]["mS"] == calibration["massSActual_kg"]
        assert config["mechanics"]["k1"] == calibration["lowerStiffnessActual_N_m"]
        assert config["mechanics"]["c1"] == calibration["lowerDampingActual_Ns_m"]
        reference = float(config["force"]["reference"])
        angle = math.degrees(float(config["mechanics"]["angle"]))
        mu = float(config["contact"]["mu"])
    else:
        reference = float(native["reference_N"])
        angle = float(native["angle_deg"])
        mu = float(native["mu"])
    row = {
        "family": data["family"], "native_case_id": data["nativeCaseId"],
        "seed": native["seed"], "massS_actual_kg": calibration["massSActual_kg"],
        "massS_identified_kg": calibration["massSIdentified_kg"],
        "massB_actual_kg": calibration["massBActual_kg"],
        "massB_identified_kg": calibration["massBIdentified_kg"],
        "reference_N": reference, "angle_deg": angle, "mu": mu,
        "duration_s": data["timeRange_s"][1], "samples": data["nativeSamples"],
        "lower_stiffness_actual_N_m": calibration["lowerStiffnessActual_N_m"],
        "lower_damping_actual_Ns_m": calibration["lowerDampingActual_Ns_m"],
        "lower_stiffness_identified_N_m": calibration["lowerStiffnessIdentified_N_m"],
        "lower_damping_identified_Ns_m": calibration["lowerDampingIdentified_Ns_m"],
        "generic_normal_peak_error_N": data["genericNormalPeakError_N"],
        "generic_voltage_clipping_samples": data["genericVoltageClippingSamples"],
        "generic_current_clipping_samples": data["genericEffortClippingSamples"],
        "native_voltage_equation_residual_V": data["genericRequestedVsAppliedVoltageResidual_V"],
        "mapped_voltage_identity_residual_V": data["mappedHotRequestedVsDirectRegulatorResidual_V"],
        "reserve_at_31V_V": 31 - peak, "reserve_at_32V_V": 32 - peak,
        "reserve_at_36V_V": 36 - peak,
        "predicted_31V_clipping_samples": data["predicted31VClippingSamples"],
        "predicted_31V_clipping_fraction": data["predicted31VClippingFraction"],
        "predicted_32V_clipping_samples": 0 if peak <= 32 else None,
        "peak_mapped_voltage_time_s": data["peakMappedRequestedVoltageTime_s"],
        "mapped_regulator_component_at_peak_V": data["mappedHotVoltageComponentsAtPeak_V"]["currentRegulator"],
        "mapped_resistance_component_at_peak_V": data["mappedHotVoltageComponentsAtPeak_V"]["resistance"],
        "mapped_back_emf_component_at_peak_V": data["mappedHotVoltageComponentsAtPeak_V"]["backEMF"],
        "physical_Kf_real_N_A": calibration["physicalKfReal_N_A"],
        "identified_Kf_real_N_A": calibration["identifiedKfReal_N_A"],
        "identified_to_physical_Kf_ratio": calibration["identifiedToPhysicalKfRatio"],
        "mapped_initial_current_A": calibration["initialActualRealCurrent_A"],
        "mapped_initial_current_command_A": calibration["initialRealCommand_A"],
        "initial_normal_error_N": calibration["initialNormalForceError_N"],
        "pointwise_audit_file": str(path), "pointwise_audit_sha256": sha(path),
        "native_trace_file": native.get("nativeTrace", native.get("nativeTraceFile")),
    }
    for field, label, unit in range_fields:
        for measure in ["minimum", "maximum", "peakAbsolute", "rms"]:
            row[f"{label}_{measure}_{unit}"] = data[field][measure]
    trace_source = sources[row["native_trace_file"]]
    row["native_trace_sha256"] = trace_source["sha256"]
    rows.append(row)

assert len(rows) == 7 and sum(r["samples"] for r in rows) == 13_200_007
assert sum(r["predicted_31V_clipping_samples"] for r in rows) == 1
assert all(r["predicted_32V_clipping_samples"] == 0 for r in rows)
worst = max(rows, key=lambda r: r["mapped_hot_requested_voltage_peakAbsolute_V"])
assert worst["family"] == "mixed" and worst["native_case_id"] == 1
assert abs(worst["reserve_at_32V_V"] - 0.8929314522810365) < 1e-12

groups = {}
for family in ["long", "mixed", "all"]:
    subset = rows if family == "all" else [r for r in rows if r["family"] == family]
    demand = max(r["mapped_hot_requested_voltage_peakAbsolute_V"] for r in subset)
    groups[family] = {
        "existingNativeRecords": len(subset), "newNativeCases": 0,
        "nativeSamplesRead": sum(r["samples"] for r in subset),
        "allGenericNormalForcePass": True,
        "allGenericVoltageAndCurrentUnclipped": True,
        "mappedHotPeakDemand_V": demand, "minimum31VReserve_V": 31 - demand,
        "minimum32VReserve_V": 32 - demand,
        "predicted31VClippingSamples": sum(r["predicted_31V_clipping_samples"] for r in subset),
        "predicted32VClippingSamples": 0,
        "mappedRealPeakCurrent_A": max(r["mapped_real_current_peakAbsolute_A"] for r in subset),
        "mappedRealPeakCurrentCommand_A": max(r["mapped_real_current_command_peakAbsolute_A"] for r in subset),
        "genericNormalWorstError_N": max(r["generic_normal_peak_error_N"] for r in subset),
    }

csv_path = HERE / f"{PREFIX}.csv"
assert not csv_path.exists(), "Preserve prior aggregate evidence."
with csv_path.open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)

summary = {
    "date": "2026-09-30", "status": "complete_conditional_pointwise_voltage_screen",
    "scope": "Seven retained generic native histories: four calibrated-static 30 s repeated-bump records and three calibrated-static 4 s mixed records with identified actual lower k/c. No MATLAB process or new native case was launched for this screen.",
    "groups": groups, "worstMappedVoltageRecord": worst,
    "mapping": audits[0][1]["mapping"],
    "uniformCurrentScale": 20 / 24.4,
    "physicalGeneric": {"Kf_N_A": 20, "R_ohm": 4, "L_H": 0.003, "bus_V": 24, "peakCurrent_A": 3.5},
    "conditionalRealHot": {"Kf_N_A": 24.4, "R_ohm": 8.896, "L_H": 0.004, "currentResponse_s": 0.0001, "peakCurrent_A": 3.5, "hotCopperRiseAssumed_degC": 100},
    "residualKfIdentificationPreserved": {
        "heavyGenericIdentified_N_A": 20.05, "heavyRealIdentified_N_A": 24.461,
        "lightGenericIdentified_N_A": 19.95, "lightRealIdentified_N_A": 24.339,
        "forceKpReal_A_N": 0.3 * 20 / 24.4,
        "forceKiReal_A_Ns": 20 * 20 / 24.4,
        "forceKdReal_A_s_N": 0,
        "allOtherSensorRobotGuideMechanicalCalibrationRetained": True,
        "initialCurrentAndAllActiveCurrentFeedforwardAndInverseOutputsMustScaleTogether": True,
    },
    "recommendation": {
        "nominalSupply_V": 36, "verifyMinimumUsableTerminalEnvelope_V": 32,
        "minimumRecordedReserveAt32V_V": groups["all"]["minimum32VReserve_V"],
        "reason": "A verified 32 V terminal envelope exceeds all seven exact mapped demands. A nominal 36 V supply is a candidate only if the chosen amplifier supplies at least this loaded envelope with the required 100 us current response; a 4 V drop allowance is a design target, not a guaranteed amplifier specification.",
        "31VException": "Mixed seed 7 requires 31.1070685477 V at t=3.8701 s. Exactly one recorded 10 us sample exceeds 31 V; this prevents exact unclipped-history equivalence at 31 V but does not establish that force would fail at 31 V.",
        "higherBusMeaning": "Higher supply voltage restores a missing delivered-voltage envelope. At a sufficiently supplied, matched 100 us regulator it does not change the configured bandwidth or force trajectory.",
    },
    "maximumGenericVoltageEquationResidual_V": max(r["native_voltage_equation_residual_V"] for r in rows),
    "maximumPointwiseMappingResidual_V": max(r["mapped_voltage_identity_residual_V"] for r in rows),
    "existingNativeTrajectoriesOnly": True, "noNewNativeSimulation": True,
    "startupScoredForEveryRecord": True,
    "nativeSampleStep_s": 1e-5,
    "assumptions": audits[0][1]["assumptions"] + [
        "The recorded generic normal-force errors remain source results. Equivalent real-hot force trajectories follow conditionally only if the entire current-force transformation is implemented and the mapped current/terminal voltage is delivered without additional dynamics or clipping.",
        "The 31 V sample count is a sampled voltage-envelope check; it does not characterize the continuous-time pulse duration or the force error after hypothetical saturation.",
        "The 32 V reserve applies to these seven retained histories. It is not an exhaustive continuous uncertainty guarantee, a larger-bump qualification, a winding thermal proof, or a measured hardware margin.",
    ],
    "sourceContracts": {
        "baseV8ModelSha256": long_report["sourceModelSha256"],
        "finalCandidateSha256": long_report["candidateSha256"],
        "longInstrumentedModelSha256": long_report["privateModelSha256"],
        "longRecipeSha256": long_report["experimentSha256"],
        "mixedInstrumentedModelSha256": mixed_report["sourceModelSHA256"],
        "mixedRecipeSha256": mixed_report["recipeSHA256"],
        "helperSources": long_report["helperSources"],
    },
    "sources": list(sources.values()),
    "pointwiseAudits": [{"file": str(path), "sha256": sha(path)} for path, _ in audits],
    "readerRecipe": {"file": str(HERE / f"{PREFIX}_reader.py"), "sha256": sha(HERE / f"{PREFIX}_reader.py")},
    "aggregationRecipe": {"file": str(Path(__file__).resolve()), "sha256": sha(__file__)},
    "csv": {"file": str(csv_path), "sha256": sha(csv_path)},
    "rows": rows,
    "memoryScope": "Seven separate short-lived reader processes previously loaded one native-history MAT at a time. This aggregation reads small JSON files, the 25 kB prepared-configuration MAT and streamed byte hashes; no native-history arrays are loaded.",
}
json_path = HERE / f"{PREFIX}.json"
assert not json_path.exists(), "Preserve prior aggregate evidence."
json_path.write_text(json.dumps(summary, indent=2) + "\n")

markdown = [
    "# Calibrated-static hot-coil voltage headroom",
    "",
    "The seven retained histories support a **verified 32 V usable terminal-voltage envelope** for the conditional hot-coil transformation. The maximum exact demand is **31.107069 V**, leaving **0.892931 V** reserve at 32 V. A nominal **36 V supply** is a reasonable starting point only if the amplifier delivers at least this loaded output envelope and the required 100 µs current response.",
    "",
    "This is a pointwise algebraic screen of existing native histories. It adds **zero native simulations**. All startup samples were retained. Four 30 s records and three 4 s records contain **13,200,007 samples** in total at 10 µs spacing; each generic 24 V history is complete and has zero native voltage or current clipping.",
    "",
    "| Retained family | Seed | S (kg) | Reference (N) | Original native normal error peak (N) | Mapped hot voltage peak (V) | Reserve at 31 V (V) | Reserve at 32 V (V) | Samples above 31 V |",
    "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
]
for r in rows:
    markdown.append(
        f"| {r['family']} {r['duration_s']:g} s | {r['seed']} | {r['massS_actual_kg']:g} | {r['reference_N']:g} | "
        f"{r['generic_normal_peak_error_N']:.6f} | {r['mapped_hot_requested_voltage_peakAbsolute_V']:.6f} | "
        f"{r['reserve_at_31V_V']:.6f} | {r['reserve_at_32V_V']:.6f} | {r['predicted_31V_clipping_samples']} |"
    )
markdown += [
    "",
    "All four improved 30 s records fit 31 V; their maximum demand is 29.591533 V. Mixed seed 7 reaches 31.107069 V at 3.8701 s, after the programmed bump. Exactly one recorded 10 µs sample exceeds 31 V. This small excursion prevents a claim of exact unclipped equivalence at 31 V; it does **not** establish a normal-force failure under a hypothetical 31 V drive. No clipped counterfactual was simulated.",
    "",
    "The mixed records use actual and identified lower-link stiffness 23,695.2 N/m and damping 48 N·s/m. The long records use 19,746 N/m and 60 N·s/m. The 16 static residual calibration errors are halved in both families; the original AA response, transport delays, noise, robot delay/response residuals, guide and other mechanical parameters remain fixed. These records complement the separately audited nominal 3,500-case grid and the separately executed winding cases; their case counts must not be combined as additional real-coil simulations.",
    "",
    "## Exact current and winding transformation",
    "",
    "The generic physical winding is Kf = 20 N/A, R = 4 Ω and L = 3 mH. The conditional real hot winding is Kf = 24.4 N/A, R = 8.896 Ω and L = 4 mH, with the same matched 100 µs current regulator. The resistance corresponds to 6.4 Ω × 1.39, an assumed 100 °C copper temperature rise rather than a qualified safe winding temperature.",
    "",
    "Let `s = 20/24.4`, `i` be native generic physical current, `u` the same limited current command used by the native regulator, and `v` the coil-gap velocity. Native requested voltage is `Vg = (0.003/0.0001)(u−i) + 4i + 20v`. Both generic clipping flags are zero, so it equals the recorded applied voltage. The exact transformed demand is:",
    "",
    "```text",
    "ireal = s i;  ureal = s u",
    "Vhot = (0.004/0.0001)s(u−i) + 8.896s i + 24.4v",
    "     = 1.092896174863388 Vg + 2.920218579234974 i + 2.542076502732243 v",
    "```",
    "",
    "Each full native history passed both the native voltage-equation check and the direct-vs-transformed winding calculation. Maximum residuals are 3.55 × 10⁻¹⁵ V and 1.07 × 10⁻¹⁴ V respectively.",
    "",
    "Every active current-unit PI gain, holding/acceleration/guide feedforward term, proper inverse current output and current initial condition must scale by `s`. The transformed PI gains are Kp = 0.245901639 A/N and Ki = 16.393442623 A/(N·s); Kd remains zero. The identified Kf residual is preserved: heavy records retain physical/identified 20/20.05 N/A as 24.4/24.461 N/A, and the light record retains 20/19.95 as 24.4/24.339 N/A. The inverse uses that residual identified Kf, rather than silently correcting it to the physical value.",
    "",
    f"Across all seven mapped histories, peak physical current is {groups['all']['mappedRealPeakCurrent_A']:.6f} A and peak command is {groups['all']['mappedRealPeakCurrentCommand_A']:.6f} A, below the retained 3.5 A limit. Signed minimum/maximum, peak, RMS, initial states and native command/current ranges are retained in the [CSV](" + csv_path.name + ") and [JSON](" + json_path.name + ").",
    "",
    "## Scope of the recommendation",
    "",
    "Verify the chosen 36 V amplifier on the actual cold and hot winding: at least a 32 V bipolar terminal envelope at the relevant current, with a 100 µs load-specific response and acceptable phase/ripple. A 4 V amplifier-drop allowance is an engineering target, not a manufacturer guarantee. If the driver cannot provide that envelope at 36 V, a higher bus can restore delivered-voltage headroom; it does not improve the configured current bandwidth or force response once clipping is absent.",
    "",
    "Conditional trajectory equivalence assumes the whole force/current transformation, matched resistance/inductance/back-EMF compensation, identical upper-rigid robot kinematics and sensing/mechanics calibration, and an amplifier that delivers the calculated voltage/current without extra phase, ripple or saturation. No Kf-versus-stroke/temperature, real amplifier dynamics, finite robot impedance or thermal safety is qualified by this algebra. The reported native normal-force errors belong to the original generic records, rather than to newly executed real-coil cases.",
    "",
    "## Frozen evidence",
    "",
    f"- Base V8 model: `{summary['sourceContracts']['baseV8ModelSha256']}`.",
    f"- Final candidate: `{summary['sourceContracts']['finalCandidateSha256']}`.",
    f"- Long instrumented model: `{summary['sourceContracts']['longInstrumentedModelSha256']}`; recipe `{summary['sourceContracts']['longRecipeSha256']}`.",
    f"- Mixed instrumented model: `{summary['sourceContracts']['mixedInstrumentedModelSha256']}`; recipe `{summary['sourceContracts']['mixedRecipeSha256']}`.",
    "- All seven native trace hashes, source reports/configuration hashes, 16 helper hashes, reader recipe and aggregate recipe are retained in the aggregate JSON. Every source byte hash was checked again while building this document; MAT files were streamed for hashes without loading arrays.",
    "",
]
md_path = HERE / f"{PREFIX}.md"
assert not md_path.exists(), "Preserve prior aggregate evidence."
md_path.write_text("\n".join(markdown))
print(json.dumps({"records": len(rows), "newNativeCases": 0,
                  "samples": groups['all']['nativeSamplesRead'],
                  "maximumDemand_V": groups['all']['mappedHotPeakDemand_V'],
                  "minimumReserveAt32V_V": groups['all']['minimum32VReserve_V'],
                  "predicted31VClippingSamples": 1,
                  "json": str(json_path), "jsonSha256": sha(json_path),
                  "csv": str(csv_path), "csvSha256": sha(csv_path),
                  "markdown": str(md_path), "markdownSha256": sha(md_path)}))
