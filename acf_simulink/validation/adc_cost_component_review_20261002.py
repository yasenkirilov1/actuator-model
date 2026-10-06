#!/usr/bin/env python3
"""Reproduce the ADC electrical-to-mechanical budgets; no simulation runs.

Manufacturer electrical values are in the JSON source ledger. This script
derives ideal quantization, SNR-equivalent input noise, and Sinc3 timing.
It deliberately does not label any component or acquisition rate qualified.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE = HERE / "adc_cost_component_review_20261002"
G = 9.80665
ADXL_V_PER_G = 0.04


def sar_noise(span_v: float, snr_db: float, sine_dbfs: float = 0.0) -> float:
    # A bipolar or unipolar full-scale sinusoid has amplitude span/2.
    sine_rms_v = span_v / (2 * math.sqrt(2)) * 10 ** (sine_dbfs / 20)
    return sine_rms_v / 10 ** (snr_db / 20)


def budget(name: str, span_v: float, bits: int, gain: float,
           sigma_v: float, force_range_n: float = 160) -> dict:
    quantum_v = span_v / 2 ** bits
    accel_v_per_m_s2 = ADXL_V_PER_G * gain / G
    force_v_per_n = span_v / (2 * force_range_n)
    return {
        "name": name,
        "adcSpan_V": span_v,
        "bits": bits,
        "accelerationConditioningGain": gain,
        "acceleration_ADC_range_g": span_v / (2 * ADXL_V_PER_G * gain),
        "accelerationRangeMeaning": "ADC half-span only; centered conditioning and sensor range determine available signed acceleration headroom.",
        "voltageQuantum_V": quantum_v,
        "accelerationQuantum_m_s2": quantum_v / accel_v_per_m_s2,
        "idealAccelerationQuantizationRMS_m_s2": quantum_v / accel_v_per_m_s2 / math.sqrt(12),
        "adcTotalNoiseRMS_V": sigma_v,
        "adcAccelerationNoiseRMS_m_s2": sigma_v / accel_v_per_m_s2,
        "adcNoiseExcludingIdealQuantizationRMS_V": math.sqrt(max(0, sigma_v ** 2 - quantum_v ** 2 / 12)),
        "illustrativeForceRange_N": force_range_n,
        "illustrativeForceGain_V_per_N": force_v_per_n,
        "illustrativeForceQuantum_N": quantum_v / force_v_per_n,
        "illustrativeForceADCNoiseRMS_N": sigma_v / force_v_per_n,
        "meaning": "Electrical budget, not a force simulation or a six-bridge calibration-matrix guarantee.",
    }


def main() -> None:
    ledger = json.loads(BASE.with_suffix('.json').read_text())
    components = {entry['part']: entry for entry in ledger['candidates']}
    rows = []
    # ADI's table does not explicitly specify the sine level. Using a full-scale
    # sine rather than -0.5 dBFS is the more conservative inference.
    ti_sar = components['ADS8588SIPMR']
    for gain in (1, 2, 4):
        for suffix, snr in [('typical', ti_sar['snrTypical_dB']),
                            ('minimum_SNR', ti_sar['snrMinimum_dB'])]:
            rows.append(budget(f'ADS8588S_gain{gain}_{suffix}', 10, 16, gain,
                               sar_noise(10, snr, -0.5)))
    c18 = components['AD7606C-18BSTZ']
    for suffix, snr in [('typical', c18['snrTypical_dB']),
                        ('minimum_SNR', c18['snrMinimum_dB'])]:
        rows.append(budget(f'AD7606C18_unipolar_{suffix}', 5, 18, 1,
                           sar_noise(5, snr)))
    c16 = components['AD7606C-16BSTZ']
    for suffix, snr in [('typical', c16['snrTypical_dB']),
                        ('minimum_SNR', c16['snrMinimum_dB'])]:
        rows.append(budget(f'AD7606C16_unipolar_{suffix}', 5, 16, 1,
                           sar_noise(5, snr)))
    for part, source in [('AD7606B', components['AD7606BBSTZ']),
                         ('AD7606', components['AD7606BSTZ'])]:
        for gain in (2, 4):
            for suffix, snr in [('typical', source['snrTypical_dB']),
                                ('minimum_SNR', source['snrMinimum_dB'])]:
                rows.append(budget(f'{part}_gain{gain}_{suffix}', 10, 16, gain,
                                   sar_noise(10, snr)))
    m08 = components['ADS131M08IPBSR']
    for osr_string, sigma_uv in m08['inputNoiseRMS_uV_byOSR_gain1'].items():
        osr = int(osr_string)
        row = budget(f'ADS131M08_gain1_OSR{osr}', 2.4, 24, 0.5, sigma_uv * 1e-6)
        # Gain1 amplifier mapping reserves ±60 g ADC range for a ±50 g sensor.
        for clock_hz in (m08['nominalMasterClock_Hz'],
                         m08['proposedSynchronousMasterClock_Hz']):
            fmod = clock_hz / m08['modulatorClockDivisor']
            fdata = fmod / osr
            group_s = 3 * (osr - 1) / (2 * fmod)
            low, high = 0.0, 0.5 * fdata
            for _ in range(80):
                f = (low + high) / 2
                h = abs(math.sin(math.pi * osr * f / fmod) /
                        (osr * math.sin(math.pi * f / fmod))) ** 3
                if h > 1 / math.sqrt(2):
                    low = f
                else:
                    high = f
            row.setdefault('clockVariants', []).append({
                'masterClock_Hz': clock_hz,
                'modulatorClock_Hz': fmod,
                'dataRate_Hz': fdata,
                'sinc3_FIR_length_modulator_samples': 3 * (osr - 1) + 1,
                'sinc3_groupDelay_s': group_s,
                'sinc3_minus3dB_Hz': (low + high) / 2,
                'fullFrame240bitRead_s_at_2xMasterClock': 240 / (2 * clock_hz),
                'filterGroupPlusFrameRead_s': group_s + 240 / (2 * clock_hz),
                'completeADCPhaseMustBeMeasured': True,
            })
        rows.append(row)
    ledger['derivedBudgets'] = rows
    ledger['derivedBudgetRecipe'] = str(Path(__file__).relative_to(HERE.parent.parent))
    ledger['noNewSimulations'] = True
    BASE.with_suffix('.json').write_text(json.dumps(ledger, indent=2) + '\n')
    fields = [k for k in rows[0] if k != 'meaning']
    with BASE.with_suffix('.csv').open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    print(f'Wrote {len(rows)} electrical budgets; no simulations executed.')


if __name__ == '__main__':
    main()
