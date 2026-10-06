"""Reorganize the current report without discarding its recorded evidence."""
from pathlib import Path
import re
import json

WORK = Path(__file__).resolve().parent
ROOT = Path('/home/tommie/Documents/Notes Remote/S7 Project')
source = (WORK / 'report-with-svgs.md').read_text()
parts = re.split(r'^## (.+)\s*$', source, flags=re.M)
chapters = {title: body.strip() for title, body in zip(parts[1::2], parts[2::2])}
used = []


def split_at(text, marker):
    assert text.count(marker) == 1, marker
    start = text.index(marker)
    return text[:start].strip(), text[start:].strip()


def take(title):
    assert title not in used, title
    used.append(title)
    return chapters[title]


def piece(title, body):
    return f'##### {title}\n\n{body.strip()}'


def component(title, intro, question, results, recommendation):
    return '\n\n'.join([
        f'### {title}', '#### Intro', intro, '#### Question', question,
        '#### Results', results, '#### Recommendation', recommendation,
    ])


goal = take('Our goal').split('\n\n')
assert len(goal) == 4
fast, robot = split_at(take('Control and robot recentering'), 'The robot loop recenters actuator travel slowly.')
comparison, stiffness = split_at(take('Experiments and results'), 'The finite load-cell study changes the structural problem substantially.')
stiffness, spring_screen = split_at(stiffness, 'The lower-compliance search contains')
spring_screen, short_endpoints = split_at(spring_screen, 'The selected core candidate then passes')
short_endpoints, packing = split_at(short_endpoints, 'The repeated cohort admits a common preload interval')
packing, identified_tolerance = split_at(packing, 'An additional **18 native recordings**')
grid, physical_tolerance = split_at(take('Final acquisition qualification and mechanical tolerance'), 'Physical spring and damper tolerance receives a separate test.')
hardware, raw_capacity = split_at(take('Practical force, position and drive components'), 'The load cell must accommodate **raw force before gravity tare**.')
raw_capacity, structure = split_at(raw_capacity, 'Rigidly fastening the sensor does not make it infinitely stiff.')
structure, force_accuracy = split_at(structure, 'Published force resolution is not calibrated accuracy.')
force_accuracy, driver = split_at(force_accuracy, 'A linear current amplifier is useful for the first precise-force prototype')
driver, guide = split_at(driver, 'A compact crossed-roller guide is a plausible low-friction starting point.')
guide, coil_part = split_at(guide, 'The **Moticont LVCM-070-051-01** is a useful real voice-coil reference:')

components = []
components.append(component(
    'Mechanics: load path, springs and guides',
    'The mechanical assembly must preserve a measurable load path while allowing the actuator to correct tool motion quickly. Spring packing, guide friction, structural modes and the partition of moving mass all affect the force controller.',
    'Which attachment arrangement and approximate spring/damper values can handle the specified bump across the tool-mass range, fit the spring-only 100 mm limits and remain practical to build?',
    '\n\n'.join([
        piece('Mechanical model and sensor placement', take('How the mechanism works')),
        piece('Contact model and disturbance severity', take('Contact, surface force and the sharp bump')),
        piece('Finite stiffness, joining mass and guides', structure + '\n\n' + guide),
        piece('Spring and damper search', spring_screen),
        piece('Preload and early working-envelope evidence', packing),
        piece('Identified mechanical coefficients and numerical refinement', identified_tolerance),
        piece('Actual mechanical tolerance and combined calibration cases', physical_tolerance),
    ]),
    'Use an **upper rigid attachment** and a **lower guided opposed spring pair** with **19.746 N/mm net stiffness**, approximately **9.873 N/mm per spring**, **33.6 N common preload per spring**, and **60 N·s/m parallel damping**. Start around **20 mm free length per spring**, allowing the stated ±0.49 mm tolerance; the conservative combined free length is **40.98 mm**, and the evaluated maximum combined installed length is **35.31 mm**. Provide the modeled individual force and cyclic-travel envelope.\n\n'
    'Keep the complete lower axial load through the cell, budget **B = 0.6 kg** separately from **S = 0.1–2 kg**, and identify assembled stiffness/damping for the inverse. Measure guide stiction, cable drag and joining-frame modes. The finite tests support this starting design; the spring-only packing limit does not qualify the whole flange size.',
))
components.append(component(
    'Actuator and electrical drive',
    'The actuator supplies both fast correction and continuous pressing effort. Current response, usable voltage, moving inertia, continuous force over stroke and heat must be assessed together.',
    'Should the prototype use a voice coil or leadscrew, and what force constant, stroke, current, voltage and thermal capacity must the actuator and driver provide?',
    '\n\n'.join([
        piece('Representative real coil and current-drive hardware', coil_part + '\n\n' + driver),
        piece('Native actuator comparisons and transmission sizing', take('Voice coil, leadscrew and other actuator choices')),
        piece('Voltage headroom, current limits and continuous heat', take('Voltage, current capacity and heat')),
    ]),
    'Use a **guided voice coil** for the first prototype: approximately **20–25 N/A**, **25 mm travel**, and preferably **65–70 N or more continuous force over the used stroke and operating temperature**. The evaluated winding range is roughly **4–6.4 Ω cold resistance** and **3–4 mH inductance**. Provide bipolar current, at least the modeled **3.5 A peak provision**, and a verified **100 µs current-response target**.\n\n'
    'Start with a **36 V supply only if the driver provides at least 32 V usable bipolar output under hot load**. The latest seven M08 hot-coil tests request up to 31.738466 V, so actual dropout needs measurement. Size cooling for continuous holding effort, not just the bump. A suitably larger low-backlash screw remains an alternative, but the tested RE25 is thermally undersized; its transient passes do not qualify continuous 50 N sanding.',
))
components.append(component(
    'Force and position measurement',
    'The B–S cell carries the tool’s downstream mass, so its reading contains support and inertia as well as contact loading. Relative position helps estimate motion and surface direction, while multi-axis force distinguishes pressing from sanding drag.',
    'Where should the sensors be mounted, and how can their readings reconstruct the force the workpiece experiences rather than only the force inside the flange?',
    '\n\n'.join([
        piece('Reaction-vector reconstruction and the surface normal', take('Why the load cell and accelerometer work together')),
        piece('Component references, range and calibrated accuracy', hardware + '\n\n' + raw_capacity + '\n\n' + force_accuracy),
    ]),
    'Use a **multi-axis load cell in the complete B–S load path**, with all S mass beyond it. Measure **S relative to A** with a protected position sensor and synchronized timestamp. A load cell needs capacity for raw gravity, sanding drag and moments before tare.\n\n'
    'Reconstruct the reaction vector from the consistently tared load and measured S inertia, then project onto the estimated surface normal at the same effective time. Target **±0.01 N residual force zero per used axis**, **±0.05% force gain/cross-axis residuals**, **±0.05° frame alignment**, approximately **1 µm reference-conditioned position noise**, and **±0.05% position-scale residual**. These are assembled post-calibration targets; catalog resolution alone does not establish them.',
))
components.append(component(
    'Acceleration measurement',
    'Acceleration measured directly on S supplies the dynamic-inertia correction. Lower sensor cost is useful only if noise, complete response, bias and synchronization retain the required force margin.',
    'Can a cheaper accelerometer replace the fast reference sensor without materially reducing force margin, and what resolution, noise, bias and timing must the complete acceleration chain achieve?',
    '\n\n'.join([
        piece('Reference sensor, specific acceleration and acquisition budgets', take('Realistic acceleration sensing')),
        piece('Executed lower-cost response and calibration experiments', take('Cheaper accelerometers are limited mainly by response')),
    ]),
    'Retain **two ADXL1002-class analog axes on S** for the planar prototype; use three appropriately acquired axes for a spatial system. The tested cheaper accelerometer envelopes do not retain the preferred 0.05 N reserve across their stressed cases.\n\n'
    'Target **±0.005 m/s² residual acceleration bias** and **±0.25% scale residual**, with response and timestamps matched to force. The original reference-conditioned noise budget is **0.014 m/s² RMS** before final quantization; bandwidth changes use fixed input noise density. At S = 2 kg, the bias allowance alone represents **0.01 N** of inertia error. Measure powered-tool vibration, clipping, rectified bias, lever-arm effects and warm drift before treating this budget as achieved.',
))
components.append(component(
    'Acquisition electronics: amplifier, ADC and timing',
    'The instrumentation amplifier sets the useful range before conversion. ADC noise, filter phase, clocking, readout and controller latching determine how accurately and promptly force and acceleration can be combined.',
    'What is the lowest-cost acquisition chain supported by native experiments, and why are its bandwidth and sample rate necessary rather than simply choosing a large bit count?',
    '\n\n'.join([
        piece('Converters, input scaling and signal path', take('Why a cheaper ADC can be enough')),
        piece('Bandwidth, rate, timing, noise and hot-coil experiments', take('Native experiments justify the bandwidth and sample rate')),
    ]),
    'Use an **ADS131M08 at 30 ksample/s per simultaneous channel** as the prototype starting point. Engineer and measure a **matched approximately 8 kHz complete force/acceleration response**, retain the **50 µs EMA setting**, and verify the **20 µs total post-conversion pipeline including SPI**. Feed fresh samples to the **10 kHz force controller** and align the position encoder and estimated normal to the same measurement frame.\n\n'
    + take('Acceptance of the lower-cost sensing chain'),
))
components.append(component(
    'Fast force controller',
    'The fast controller combines moderate force-error feedback with compensation for gravity, observed tool inertia, drag and the compliant lower mechanism. Its inputs remain causal; surface truth belongs to the physical plant and scoring.',
    'Does PI, PID or a more advanced causal strategy solve the sharp bump with useful margin, and which dynamics actually limit further improvement?',
    '\n\n'.join([
        piece('Controller architecture and causal inputs', fast),
        piece('The lower-link inverse', take('Why the compliant candidate needs an additional feedforward term')),
        piece('Equal-cohort PI/PID/feedforward comparison', comparison),
        piece('Finite-cell structural sensitivity', stiffness),
        piece('Selected core controller and repeated bump test', short_endpoints),
        piece('Observed limits and faster-drive tradeoffs', take('What limits performance')),
    ]),
    'Use **measured-inertia Feedforward PI** with the causal lower-link inverse. The generic 20 N/A coil setting is **Kp = 0.3 A/N**, **Ki = 20 A/(N·s)** and **Kd = 0**, with a **1 kHz feedback-only filter** and **1.5 kHz inverse roll-off**. Identify assembled k/c, downstream mass and Kf; rescale current-domain gains for another force constant.\n\n'
    'Retain physical current dynamics, saturation and antiwindup. Raising voltage helps only when clipping limits slew; a deliberately faster current loop needs its own sensing/noise and thermal qualification. Assess the learned residual separately below before changing this prototype baseline.',
))
components.append(component(
    'Robot commands and recentering',
    'The robot provides slow gross motion and recenters the short-stroke actuator. Its delay makes unqualified force chasing risky, and commands already transmitted continue to affect motion after new commands are blocked.',
    'When is force stable enough to issue a recentering command, and how should queued motion and loaded robot delay constrain that command?',
    piece('Stability gate, delay-aware gain and command prediction', robot),
    'Keep the robot update at **100 Hz**, with the slow force-error/trend gate and the **0.08 N RMS variation detector** evaluated at fast force samples. Require the **100 ms stable dwell** before new recentering corrections. Use the loaded-delay/response gain cap, track issued commands explicitly and preserve their queued effect when the gate closes.\n\n'
    'The evaluated reference is **80 ms delay / 60 ms response**, with **±5 ms identification residuals**. Measure the actual loaded robot and begin with a low recenter gain. A learned robot policy must obey the same hard gate and command envelope, rather than learning whether to bypass them.',
))
ml_path = WORK / 'ml-component.md'
if ml_path.exists():
    components.append(ml_path.read_text().strip())
else:
    components.append(component(
        'Learned control policy',
        'Two small learned policies are being investigated: a bounded fast current correction and a gated robot-recentering command policy.',
        'Can an actually trained small model improve or reliably reproduce the current controller on independent native trajectories, without additional sensing or surface preview?',
        '**Study in progress.** Training, selection and untouched native test results will be inserted here before delivery.',
        '**Pending executed evidence.** The deterministic controller remains the prototype baseline during the experiment.',
    ))

noise = '''The nominal 700-case lower-cost grid includes sensor noise, converter noise and explicit quantization, with zero residual static calibration errors. The selected long/mixed stress cases add the tighter fixed biases, gains, alignment and identified-parameter errors. These selected corners do not exhaust the complete error box.

| Measurement | Original reference-conditioned sensor noise, RMS | Fixed residual zero in selected stress cases |
| --- | ---: | ---: |
| Force, each used axis | 0.02 N | ±0.01 N |
| S acceleration, each used axis | 0.014 m/s² | ±0.005 m/s² |
| S–A position | 1 µm | No independent position-zero error stressed |

Force and position noise start as bounded uniform noise; acceleration noise is Gaussian. Input noise density is fixed when changing conditioning or sample rate, so the listed conditioned RMS values are not constant at every bandwidth. Converter noise is added separately before smoothing and explicit quantization; the selected M08 reference-acceleration chain uses approximately **0.00284 N force noise** and **0.01045 m/s² acceleration noise**, RMS at that injection point. Position uses a separate encoder. These contributions cannot be added directly without their transfer functions.

The fixed residual biases remain constant during each recording. The studies do not provide measured powered-sander vibration, vibration rectification, evolving thermal drift, random sampling jitter or switching-current ripple. Those need assembled measurements.'''
components.append(component(
    'Calibration, model qualification and prototype validation',
    'Native simulation supports a design choice only within its recorded model, metric, inputs and test identities. Calibration residuals, mechanical identification and physical validation consume reserve beyond nominal noise.',
    'What has actually passed, how much margin remains under mismatch, and what must be measured before claiming ±0.5 N on the powered physical prototype?',
    '\n\n'.join([
        piece('Full scientific-reference grid and numerical qualification', grid),
        piece('Random noise, fixed offsets and what is not injected', noise),
        piece('Executed calibration and timing mismatch', take('Required sensing, calibration and timing')),
        piece('Model limitations and disturbance scope', take('What the model does and does not establish')),
        piece('Running, changing and reproducing the native model', take('Running, changing and reproducing the model')),
    ]),
    'Use the **tighter post-calibration residuals**, identify each assembled tool/mechanism, and check force balance, response/phase, hot driver headroom and structural modes. The nominal 3,500-case reference grid and 700-case lower-cost grid have different frozen acquisition sources and noise cohorts; neither proves all combinations of mismatch or real sanding vibration. Keep the learned-policy test cohort distinct from those established baselines.\n\n'
    'Validate with an independent workpiece-force reference while the sander is powered and at operating temperature. The proposed physical criterion is **maximum absolute reference-force error plus applicable measurement uncertainty ≤0.45 N**, preferably with about **0.02 N or better reference uncertainty**. This is an acceptance target, not an observed hardware pass.',
))

final_recommendation = take('Concrete recommendation')
final_recommendation = final_recommendation.replace(
    '| Robot | Identify loaded delay/response, target ±5 ms residuals, retain queued commands and gate new recentering commands |',
    '| Robot | Identify loaded delay/response, target ±5 ms residuals, retain queued commands and gate new recentering commands |\n'
    '| Learned control | Retain deterministic force control and robot recentering; keep the tested small networks as optional research alternatives |',
)
final_recommendation = final_recommendation.replace(
    'A Moticont LVCM-070-051-01-class coil demonstrates',
    '**Learning update, 5 October:** the two trained policies have been tested separately and together in **54 held-out native recordings**, all passing ±0.5 N. The force network nevertheless raises its held-out worst error from **0.447468 N to 0.468650 N**, and the robot network reproduces the deterministic recentering law. Keep the transparent deterministic controllers. A new heavy calibration case with an initial recentering offset reaches **0.452692 N** with the baseline, leaving **0.047308 N reserve**; the preferred 0.05 N reserve is therefore not established for every active-recentering initial condition. This strengthens the need for the assembled operating-envelope test below.\n\n'
    'A Moticont LVCM-070-051-01-class coil demonstrates',
)
assert set(used) == set(chapters), sorted(set(chapters) - set(used))
report = '\n\n'.join([
    '# Active contact flange for sanding', '## Intro', '\n\n'.join(goal[:2]),
    '## Question',
    '**How should we design a compact active sanding flange that meets the pressing-force requirement with useful margin, reasonable mechanics and affordable sensing—and does a small learned controller improve that system?**',
    'The component questions below cover mechanics, actuator/drive, force/position sensing, acceleration sensing, acquisition, fast force control, robot commands, learned control and prototype qualification. Each uses the same **Intro → Question → Results → Recommendation** structure. The learned policies have the same causal input restrictions and physical limits as the existing controller.',
    '## Results', '\n\n'.join(goal[2:]),
    '**Learning update, 5 October:** two small neural policies were actually trained and tested in the native model. Across **54 held-out recordings**, every run passes ±0.5 N, but the learned force correction worsens 10 of 12 paired peaks and the robot network only reproduces the existing recentering rule. Four of these pairs activate both policies and show three regressions and one small improvement. The prototype recommendation therefore remains **deterministic measured-inertia Feedforward PI plus gated, delay-aware robot recentering**. The component results below separate these finite tests from the earlier full qualification grids.',
    '\n\n'.join(components),
    '## Recommendation', final_recommendation,
]) + '\n'

report += '\n**Build the first prototype with the guided voice coil, selected lower spring/damper pair, synchronized M08 and fast analog acceleration sensing, and deterministic force/robot controllers. Accept the assembled system only after independent powered-sanding force tests meet the stated margin.**\n'

# Relocated notes must point back to actual repository evidence.
report = report.replace('](../acf_simulink/', '](/home/tommie/Projects/actuator-model/acf_simulink/')
(WORK / 'report-nested-draft.md').write_text(report)
(WORK / 'restructure-audit.json').write_text(json.dumps({
    'sourceChapters': list(chapters), 'usedExactlyOnce': used,
    'componentCount': len(components), 'mlFinalized': ml_path.exists(),
    'figures': len(re.findall(r'!\[', report)),
}, indent=2))
print(WORK / 'report-nested-draft.md')
print(len(components), 'components;', len(re.findall(r'!\[', report)), 'figures; ML finalized:', ml_path.exists())
