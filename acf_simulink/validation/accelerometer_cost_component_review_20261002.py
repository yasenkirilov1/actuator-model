#!/usr/bin/env python3
"""Reproduce manufacturer-to-inertia budgets; no physical/native simulations.

The source ledger intentionally leaves undocumented filter transfers unknown.
Noise-band integration is an explicit flat-white analytical inference, not a
claim that a named sensor has that output noise in an assembled sanding head.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE / "accelerometer_cost_component_review_20261002"


def write_json(path: Path, content: dict) -> None:
    path.write_text(json.dumps(content, indent=2, ensure_ascii=False) + "\n")


def main() -> None:
    ledger = json.loads(BASE.with_suffix(".json").read_text())
    g = ledger["gravity_m_s2"]
    sigma_target = ledger["conditionedAccelerationReferenceRMS_m_s2"]
    source = {c["part"]: c for c in ledger["candidates"]}
    rows = []
    limits = []
    for part, entry in source.items():
        for variant, density_ug in entry["noiseDensity_ug_per_sqrtHz"].items():
            density_si = density_ug * 1e-6 * g
            limits.append({
                "part": part,
                "densityVariant": variant,
                "density_ug_per_sqrtHz": density_ug,
                "flatWhiteENBWForReferenceRMS_Hz": (sigma_target / density_si) ** 2,
                "meaning": "Density-only flat-white estimate; ENBW is not -3 dB cutoff or sample rate, and extra ADC/front-end noise uses part of the budget.",
            })
            for enbw in ledger["inferenceInputs"]["equivalentNoiseBandwidths_Hz"]:
                sigma_a = density_si * math.sqrt(enbw)
                rows.append({
                    "part": part,
                    "densityVariant": variant,
                    "density_ug_per_sqrtHz": density_ug,
                    "assumedEquivalentNoiseBandwidth_Hz": enbw,
                    "inferredAccelerationRMS_m_s2": sigma_a,
                    "inferredInertiaRMS_atS0p1kg_N": 0.1 * sigma_a,
                    "inferredInertiaRMS_atS2kg_N": 2 * sigma_a,
                    "ratioToReference0p014RMS": sigma_a / sigma_target,
                    "calculationStatus": "analytical_flat_white_band_inference_not_experiment",
                })

    # No output PSD/phase information is invented from a bandwidth headline.
    icm = source["ICM-42688-P"]
    icm_rows = []
    for fast in icm["response"]["uiFastRows"]:
        for axis, density_ug in icm["noiseDensity_ug_per_sqrtHz"].items():
            icm_rows.append({
                **fast,
                "axis": axis,
                "assumedWhiteDensity_ug_per_sqrtHz": density_ug,
                "inferredRMS_usingDocumentedUI_NBW_m_s2": density_ug * 1e-6 * g * math.sqrt(fast["noiseBandwidth_Hz"]),
                "meaning": "Uses the UI table noise bandwidth and flat extrapolation of typical 10-Hz density. Includes neither an enabled AAF's extra filtering nor its phase; not a measured high-rate noise guarantee.",
            })

    adxl = source["ADXL354CEZ"]
    sensitivity = adxl["sensitivity_V_per_g"]["8"]
    gain = adxl["conditioningExample"]["gain"]
    adc = ledger["inferenceInputs"]["ADS8588S"]
    quantum_v = adc["span_V"] / 2 ** adc["bits"]
    quantum_a = quantum_v * g / (sensitivity * gain)
    sar_budgets = []
    for quality, snr in [("typical", adc["snrTypical_dB"]),
                         ("minimum_SNR", adc["snrMinimum_dB"])]:
        sigma_v = adc["span_V"] / (2 * math.sqrt(2)) * 10 ** (adc["sineLevel_dBFS"] / 20) / 10 ** (snr / 20)
        sigma_adc_a = sigma_v * g / (sensitivity * gain)
        sar_budgets.append({
            "adc": "ADS8588S",
            "sensor": "ADXL354C +/-8 g",
            "quality": quality,
            "conditioningGain": gain,
            "sensorSensitivity_V_per_g": sensitivity,
            "adcTotalRMS_V": sigma_v,
            "adcOnlyAccelerationRMS_m_s2": sigma_adc_a,
            "accelerationQuantum_m_s2": quantum_a,
            "adcRangeAfterConditioning_g": adc["span_V"] / (2 * gain * sensitivity),
            "analyticalSensorPlusADC_at1000HzENBW_RMS_m_s2": math.hypot(adxl["noiseDensity_ug_per_sqrtHz"]["8"] * 1e-6 * g * math.sqrt(1000), sigma_adc_a),
            "meaning": "SNR contains quantization; the noise and quantum are not two independent contributions. Actual sampling/filter correlation and front-end noise need separate treatment.",
        })
    m08 = ledger["inferenceInputs"]["ADS131M08_gain1_OSR128"]
    m08_gain = m08["analogConditioningGain"]
    m08_budget = {
        "adc": "ADS131M08, PGA1, OSR128",
        "sensor": "ADXL354C +/-8 g",
        "conditioningGain": m08_gain,
        "accelerationQuantum_m_s2": m08["span_V"] / 2 ** m08["bits"] * g / (sensitivity * m08_gain),
        "adcOnlyAccelerationRMS_m_s2": m08["noiseTypicalRMS_uV"] * 1e-6 * g / (sensitivity * m08_gain),
        "adcRangeAfterConditioning_g": m08["span_V"] / (2 * sensitivity * m08_gain),
        "meaning": "The ADC noise is its already Sinc3-filtered typical input RMS. Add the actual analog sensor response and Sinc3 transfer, not another generic AA replacement.",
    }

    quantization = [
        {"part": "ICM-42688-P", "mode": "16-bit +/-16 g", "accelerationQuantum_m_s2": g / 2048},
        {"part": "ICM-42688-P", "mode": "18 actual-bit high-resolution FIFO, +/-16 g", "accelerationQuantum_m_s2": g / icm["highResolutionFIFO"]["effectiveSensitivity_counts_per_g"]},
        {"part": "IIS3DWB", "mode": "16-bit +/-16 g, ideal span", "accelerationQuantum_m_s2": 32 * g / 2 ** 16},
        {"part": "IIS3DWB", "mode": "16-bit +/-8 g, ideal span", "accelerationQuantum_m_s2": 16 * g / 2 ** 16},
        {"part": "ADXL355", "mode": "+/-8 g, manufacturer typical sensitivity", "accelerationQuantum_m_s2": g / source["ADXL355BEZ"]["sensitivity_counts_per_g"]["8"]},
    ]

    timestamp_budgets = []
    jerk = ledger["inferenceInputs"]["sharpBumpExampleMaximumJerk_m_s3"]
    for dt in ledger["inferenceInputs"]["exampleTimestampErrors_s"]:
        timestamp_budgets.append({
            "timestampMismatch_s": dt,
            "exampleSurfaceJerk_m_s3": jerk,
            "firstOrderInertiaError_atS2kg_N": 2 * jerk * dt,
            "meaning": "Local small-time m*j*dt estimate on the analytical imposed profile, not actual tool-head jerk or a control pass/fail prediction. Native aligned histories decide performance.",
        })

    readouts = []
    for part in ["IIS3DWBTR", "ICM-42688-P", "ADXL355BEZ"]:
        timing = source[part]["interfaceTiming"]
        row = {"part": part,
               "directReadAtMaximumSPI_s": timing["directReadBits"] / timing["maximumSPIClock_Hz"],
               "meaning": "Wire-time only, excluding sensor group delay, data age/hold, chip-select guards, firmware and scheduling."}
        if part == "ICM-42688-P":
            row["timestamped16BytePacketRead_s"] = timing["timestamped16ByteFIFOPacketReadBits"] / timing["maximumSPIClock_Hz"]
            row["highResolution20BytePacketRead_s"] = timing["highResolution20ByteFIFOPacketReadBits"] / timing["maximumSPIClock_Hz"]
        readouts.append(row)

    baseline_price = source["ADXL1002BCPZ-RL7"]["price"]["quantity1"]
    prices = []
    for part, entry in source.items():
        count2d = 2 if entry["axesPerPart"] == 1 else 1
        count3d = 3 if entry["axesPerPart"] == 1 else 1
        price = entry["price"]["quantity1"]
        prices.append({
            "part": part,
            "quantity1UnitPrice_exVAT_EUR": price,
            "planarSensorOnlyCost_exVAT_EUR": count2d * price,
            "spatialSensorOnlyCost_exVAT_EUR": count3d * price,
            "planarSensorOnlySavingVersusReference_fraction": 1 - count2d * price / (2 * baseline_price),
            "spatialSensorOnlySavingVersusReference_fraction": 1 - count3d * price / (3 * baseline_price),
            "meaning": "Sensor silicon/packaging price only. Analog conditioning, PCB, clock/interface, testing, calibration, stock and full-system acquisition cost are separate.",
        })

    derived = {
        "schema": "acf.accelerometer_component_derived.v1",
        "status": "reproducible_analytical_component_budgets_no_native_experiments",
        "densityNoiseRows": rows,
        "densityOnlyENBWLimits": limits,
        "icmFastUIWhiteNoiseInferences": icm_rows,
        "adxl354WithSarBudgets": sar_budgets,
        "adxl354WithM08Budget": m08_budget,
        "digitalQuantumExamples": quantization,
        "timestampMismatchIllustrations": timestamp_budgets,
        "minimumSPIWireTimes": readouts,
        "sensorOnlyPriceComparisons": prices,
        "icm40kClockScreen": {
            "clock_Hz": 40000,
            "outputRate_Hz": 40000,
            "rawFramesPer10kController": 4,
            "inferredUIBandwidth_Hz": 8400 * 1.25,
            "inferredUI_NBW_Hz": 8831.7 * 1.25,
            "inferredUI_DCGroupDelay_s": 0.0001 / 1.25,
            "inferredMaximumAAFMinus3dB_Hz": 3979 * 1.25,
            "status": "Output-rate scaling is explicit; filter-frequency and inverse delay scaling are engineering inferences requiring physical verification. No complete AAF/common-decimator transfer is invented.",
        },
        "ism330DocumentedOISEquivalentPhaseDelayAt20Hz_s": 4.2 / 360 / 20,
    }
    ledger["derived"] = derived
    write_json(BASE.with_suffix(".json"), ledger)
    with BASE.with_suffix(".csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    # Mechanical checks for our research artifacts; these do not claim a
    # scientific model validation or datasheet certification.
    assert len(rows) == 55
    assert len({(r["part"], r["densityVariant"], r["assumedEquivalentNoiseBandwidth_Hz"]) for r in rows}) == len(rows)
    assert math.isclose(quantum_a, 0.0029927520751953124, rel_tol=1e-12)
    assert math.isclose(m08_budget["adcOnlyAccelerationRMS_m_s2"], 0.002089797115, rel_tol=1e-12)
    checks = {
        "status": "component_arithmetic_and_owned_file_consistency_checked",
        "nativeExperimentsPerformedInThisLane": 0,
        "densityNoiseRows": len(rows),
        "densityVariants": len(limits),
        "componentRecords": len(source),
        "rowIdentitiesUnique": True,
        "ADCQuantizationScalingChecked": True,
        "CSVSource": "derived.densityNoiseRows",
        "files": {},
    }
    for suffix in [".json", ".csv", ".py", ".md"]:
        path = BASE.with_suffix(suffix)
        if path.exists():
            checks["files"][path.name] = {
                "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
    write_json(BASE.with_suffix(".checks.json"), checks)
    print(json.dumps({"componentRecords": len(source), "noiseRows": len(rows), "status": checks["status"]}))


if __name__ == "__main__":
    main()
