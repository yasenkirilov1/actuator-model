"""Compose the user's roughly 2,000-word prototype/design-defence report.

Detailed prior reports and native experiment records remain linked references.
Final outcome prose must come from the reviewed fixed-mechanics evidence.
"""
from pathlib import Path
import argparse
import json
import re

ROOT = Path(__file__).resolve().parents[3]
TARGET = ROOT / 'reports/active-contact-flange-report-upper-spring-20261005.md'

TEXT = r'''# Active contact flange for sanding

## Intro

Our goal is to design a sanding flange that keeps the **normal pressing force experienced by the workpiece within ±0.5 N**, including a **1 mm high, 10 mm wide bump crossed at 100 mm/s**. It must work at **5–50 N**, fixed orientations **0–90°**, and tool mass **S = 0.1–2 kg**, with a **0.6 kg holder B**.

The upper connection is prescribed: **20 N/mm, zero external damping, unlimited spring length/travel**. The lower connection is **rigid**, with no external spring or damper. Actuator travel remains limited. Control may use force, tool acceleration, S–A position and robot commands already issued; it receives no surface preview.

## Question

Can the fixed mechanics meet **±0.5 N**, and which actuator, affordable sensors and controller should the prototype use?

## Results

__OUTCOME__

Score the largest absolute error from **every native sample**, including startup. Contact loss or a travel stop fails; **±7.5 mm about midstroke** is acceptable.

![Prescribed mechanics and measurement locations](figures/fixed-upper-rigid-lower-20261005.svg)

*A is the actuator body, B the moving holder, and S the complete tool downstream of the load cell. “Rigid lower” retains the sensing element's finite stiffness.*

### Mechanics: load path, springs and guides

#### Intro

The actuator moves B relative to A; the upper spring lets A move relative to the robot.

#### Question

Which mechanical effects govern force accuracy when neither external damper is available?

#### Results

Native Simulink blocks solve the mass force balances with gravity, contact geometry, sanding drag, coil dynamics, guide friction and physical limits. Maintained contact constrains S to follow the surface.

The **64 MN/m** sensing cell produces a mode near **1.64 kHz** under maintained contact. The upper spring introduces slower body motion; guide-friction reversals can excite the cell mode. Internal friction remains **0.2 N Coulomb plus 2 N·s/m viscous** despite zero external damping. The bump is a smooth C² polynomial, with continuous height, velocity and acceleration.

#### Recommendation

Keep the prescribed mechanics. Measure assembled cell/frame/guide modes and friction before final controller tuning. The upper spring has no packing restriction in this study; the **25 mm actuator stroke still does**.

### Actuator and electrical drive

#### Intro

The actuator needs fast bidirectional effort, modest moving mass and sustainable heat dissipation.

#### Question

Is a voice coil necessary, and would higher voltage solve the present force peaks?

#### Results

The studied hot-coil model uses **Kf = 24.4 N/A, R = 8.896 Ω, L = 4 mH**, a **3.5 A bipolar limit**, **32 V usable output**, and **50 µs current response**. The corresponding cold resistance is 6.4 Ω.

__VOLTAGE_RESULT__

Earlier tests found that the RE25 leadscrew met the transient but exceeded its continuous-current rating. They used different mechanics. Other motor/screw combinations are plausible, but require sizing for rotary inertia, friction and backlash.

#### Recommendation

Start with a voice coil: **24.4–25 N/A**, **55–60 N continuous hot-force target**, **85 N peak**, **25 mm stroke**, and an adjustable **48 V-rated bipolar driver**. Target **≥36 V usable output** for headroom; the tested candidate used 32 V. Include stationary coil/mounting mass: **A = 1.5 kg** in the full test grid. A flat-surface 50 N coil hold dissipates about **37 W**; verify cooling and the separate fast current-sensing loop.

### Force and position measurement

#### Intro

The B–S cell measures tool-support force, which includes downstream inertia.

#### Question

How do we recover the force actually pressing on a sloped workpiece?

#### Results

With consistent gravity tare, reconstruct the contact-force vector as **R̂ = Lₜₐᵣₑ + mS aS**, then calculate **N̂ = n̂ · R̂**. Multi-axis sensing distinguishes pressing force from sanding drag. Known tool weight removes static gravity; it does not remove dynamic inertia.

Measure **S relative to A**. Together with S acceleration and predicted robot motion, this constrains tool velocity and the local surface normal without receiving the bump profile. Force, acceleration and the estimated normal must describe the same effective instant.

#### Recommendation

Use a multi-axis cell in the entire B–S load path and a protected S–A encoder. Allow raw loads before tare: approximately **±160 N axial**, sufficient transverse capacity for gravity/drag, and checked moment ratings. Target **±0.01 N residual force zero**, **±0.05% gain/cross-axis residual**, **±0.05° frame alignment**, and approximately **1 µm conditioned position noise**. These are calibration targets, not catalog guarantees.

### Acceleration measurement

#### Intro

Acceleration measured on S supplies the tool-inertia correction directly.

#### Question

Which affordable sensor has enough noise and timing performance?

#### Results

The conditional choice is the analog **ADXL354C at ±8 g**, sampled alongside force. Its published bandwidth is **1.9 kHz**, with a **2.4 kHz mechanical resonance**. The simulation's matched response envelope requires hardware measurement; bandwidth alone does not establish phase. [ADI datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

Earlier calibration-stress comparisons gave **3/7 passes** for a cheaper digital-sensor response envelope, versus **7/7** for an ADXL354-like envelope with faster current control. They used earlier mechanics and support only a conditional sensor choice.

ADC code spacing is **0.000014 m/s²**, versus **0.00209 m/s² RMS** ADC noise and larger sensor noise. Code spacing is not usable acceleration accuracy. A **0.005 m/s² residual bias** contributes 0.01 N at a 2 kg tool.

#### Recommendation

Mount one ADXL354C rigidly on S near its centre of mass. Target **±0.005 m/s² calibrated bias**, **±0.25% scale residual**, and verified force/acceleration response matching. Measure clipping, thermal drift and vibration-induced bias with the actual sander powered before accepting the cheaper sensor.

### Acquisition electronics: amplifier, ADC and timing

#### Intro

Amplifier gain sets the useful measurement range; filter delay matters as much as converter resolution.

#### Question

Why use 30 ksample/s rather than a slower, cheaper acquisition setting?

#### Results

Use an **ADS131M08**, eight simultaneous 24-bit channels, at **30 ksample/s per channel**, **7.68 MHz clock and OSR128**. Six bridge channels plus two acceleration axes fit; the encoder has its own interface. The modeled chain uses **matched two-pole 1.9 kHz responses**, **49.6 µs sinc³ group delay**, a **50 µs EMA at the ADC output clock**, and **20 µs pipeline including SPI**, before fresh **10 kHz controller latching**. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

![Native experiments supporting the ADC rate](figures/adc-rate-evidence-compact-20261005.svg)

*Earlier compliant-link reference: four 30 s recordings per setting. This comparison supports the acquisition rate; it does not qualify the present fixed mechanics.*

At unchanged 8 kHz response, halving ADC rate from 30 to 15 ksample/s increased worst error from **0.439 to 0.494 N**. Earlier 500 Hz response screens failed at approximately **0.67 N** for the M08 chain. The current cell ringing supplies another reason to preserve kilohertz response. The bump's 0.1 s transit time alone is an inadequate bandwidth criterion.

#### Recommendation

Keep **30 ksample/s acquisition and 10 kHz control**. Match complete channel responses using the adjustable instrumentation filters. One ADXL354C plus one M08 cost approximately **€86.10 including VAT** at the recorded 5 October prices, excluding other electronics. For spatial sensing, two synchronized M08s accommodate three acceleration axes for approximately **€97.54**; the planar model still needs extending. [Recorded price/specification review](/home/tommie/Projects/actuator-model/acf_simulink/validation/rl_budget3h_20261005/budget_parts_review_20261005.json).

### Fast force controller

#### Intro

Fast current control handles the bump; the robot provides slow recentering.

#### Question

Which controller is supported by the current experiments?

#### Results

__CONTROL_RESULT__

The architecture combines force PI, measured-inertia feedforward, guide-friction compensation, a **four-state body-motion observer** and phase-shaped cell feedback. PI gains are **0.0246 A/N and 16.4 A/(N·s)**, with **1 kHz feedback filtering**. Cell correction is limited to **±0.05 A**. Inputs are measured signals and issued commands.

![Matched native force traces before and after cell-mode control](figures/fixed-mechanics-force-limit-20261005.svg)

*Complete-recording metrics use all native points. The displayed trace preserves extrema; the spectrum uses unreduced post-bump samples. This is a development case.*

#### Recommendation

__CONTROL_RECOMMENDATION__

### Robot commands and recentering

#### Intro

A delayed robot command can arrive after the force condition that triggered it has disappeared.

#### Question

How should recentering avoid destabilizing the fast force loop?

#### Results

Bounded **100 Hz** commands require stable force. A **0.08 N RMS variation gate** resets a **100 ms dwell**, alongside the slower stability condition. The predictor retains approximately **80 ms delay and 60 ms robot response**; closing the gate does not cancel queued commands. Identify both on the real robot.

#### Recommendation

Use the stability gate, conservative delay-aware correction and the accepted **±7.5 mm travel band**. Keep the voice coil responsible for the sharp bump. Validate queued motion under repeated bumps and sustained drift.

### Learned control policy

#### Intro

One recurrent policy was trained to request both coil current and robot correction from measurement history.

#### Question

Should the prototype use the learned controller?

#### Results

The retained **2×32 minGRU** passed 700 nominal cases on its earlier mechanical configuration, but only **30/32 fresh cases** and **3/4 long cases**. Worst fresh error was **0.679 N**; one long recording reached a travel stop. The deterministic baseline passed those 736 cases, but its **0.492 N** long-case worst left little reserve. These results do not transfer to the newly prescribed mechanics.

#### Recommendation

Keep learning as a research comparison. Use an inspectable deterministic controller for the prototype; retrain and independently test any learned candidate against the fixed current plant before considering deployment.

### Calibration, model qualification and prototype validation

#### Intro

Simulation tests a stated model, not every physical sanding disturbance.

#### Question

What must be measured before claiming ±0.5 N performance?

#### Results

Sensor-noise inputs are normalized to a fixed **3,177 Hz reference noise bandwidth**: **0.02 N RMS per force axis**, **0.0187 m/s² RMS acceleration**, and **1 µm RMS position**. They are reference-conditioned targets, not guaranteed output RMS of today's filters. ADC noise/quantization, calibrated offsets/scales, response mismatch and timing are separate. Force noise is bounded uniform; acceleration noise is Gaussian. Sanding-vibration tolerance remains unmeasured.

The model assumes fixed orientation, planar translation, preloaded smooth contact and known tool mass. Pad compliance, spindle rotation, angular inertia, frame flexibility, temperature drift and vibration rectification require assembled tests. Contact loss and subsequent impacts are outside this stopped-run model.

Independent MATLAB reproduces the nominal/stress failures within **31 µN force-waveform difference**; stricter sample-parity guards fail. The [study guide](/home/tommie/Projects/actuator-model/acf_simulink/experiments/upper_no_damping_20261005/README.md) preserves those checks.

#### Recommendation

Identify masses, stiffness/modes, friction, current response and channel timing. Calibrate the full load-cell matrix and acceleration frames; target **≤2 µs relative timing residual**. Then measure normal force independently through repeated bumps, 30 s drift tests and a powered sanding run. Require **≤0.45 N measured peak error** as a design target, preserving 0.05 N reserve to the requirement.

## Recommendation

__FINAL_RECOMMENDATION__

Use the **prescribed 20 N/mm undamped upper spring and rigid lower attachment**, a **25 mm voice coil with an adjustable 48 V-rated bipolar driver**, **multi-axis B–S load sensing**, an **S–A encoder**, and **ADXL354C + ADS131M08** on a custom PCB. Acquire at **30 ksample/s**, control at **10 kHz**, and gate **100 Hz recentering** on stable force. The sensor/ADC chip subtotal fits €100.

__FINAL_ACCEPTANCE__

Open the new model using the [fixed-mechanics guide](/home/tommie/Projects/actuator-model/acf_simulink/experiments/upper_no_damping_20261005/qualification_v8/GETTING_STARTED_FIXED.md). The [study guide](/home/tommie/Projects/actuator-model/acf_simulink/experiments/upper_no_damping_20261005/README.md) links full recordings and reproduction recipes. Earlier comparisons remain in the [reference report](/home/tommie/Projects/actuator-model/reports/active-contact-flange-report-rl-3h-20261005.md).
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--outcomes', type=Path, required=True)
    args = parser.parse_args()
    outcomes = json.loads(args.outcomes.read_text())
    text = TEXT
    for key in ('OUTCOME', 'VOLTAGE_RESULT', 'CONTROL_RESULT', 'CONTROL_RECOMMENDATION',
                'FINAL_RECOMMENDATION', 'FINAL_ACCEPTANCE'):
        text = text.replace('__' + key + '__', outcomes['narrative'][key])
    assert '__' not in text
    words = len(re.findall(r'\b[\wµ±]+\b', text))
    assert 1700 <= words <= 2300, f'Report length: {words} words'
    TARGET.write_text(text.strip() + '\n')
    print(f'{TARGET}: {words} words')


if __name__ == '__main__':
    main()
