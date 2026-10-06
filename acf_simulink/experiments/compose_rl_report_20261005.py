"""Replace the previous learning study with the reviewed unified-policy study.

Uses the preserved report as a source, leaving historical reports/evidence intact.
Final-result prose is supplied separately after the locked tests have completed.
"""
from pathlib import Path
import argparse
import json
import re

ROOT = Path(__file__).resolve().parents[2]
STUDY = ROOT / 'acf_simulink/validation/rl_budget3h_20261005'
PREVIOUS = ROOT / 'acf_simulink/validation/rl_report_20261005/previous_delivery/00_active-contact-flange-report.md'

ACTUATOR_UPDATE = '''##### New current-response and voltage experiments

The unified-controller study adds a matched drive experiment using one fixed, earlier recurrent checkpoint. At the development worst case, **32, 36 and 48 V usable output give the same 0.418728 N learned peak and 0.428618 N baseline peak with the 100 µs current-response tuning**. Raising voltage removes a few late clipping samples but does not change the worst bump-entry error. Current-loop response and supply voltage are separate design choices.

Changing the modeled and identified response from **100 to 50 µs** gives a clearer benefit. Across the same **28 development conditions**, both controllers pass at both response settings: **112 complete native recordings and 111,600,112 native samples**. The baseline worst peak improves from **0.428618 to 0.397116 N**; the earlier learned checkpoint improves from **0.418728 to 0.373499 N**. Its peak improves in 26 of 28 pairs; the largest of two regressions is 0.003485 N. Brief voltage-clipping samples increase from 104 to 27,176 across its 28 recordings at 32 V, while no current-limit sample occurs. Every recording still passes the force requirement.

![Measured development benefit of faster current response](figures/current_response_28cases.svg)

*Matched development cases and fixed earlier checkpoint, with every native sample scored. These are additional response tests, not final-winner qualification at 50 µs. The primary learned-policy results below use 100 µs and 32 V.*

![Voltage and current-loop response in the same development case](figures/actuator_limit_peaks.svg)

*Twelve complete recordings separate voltage-only changes from faster current tuning. More voltage removes clipping; changing current response reduces the entry transient. The faster 32 V learned waveform was independently replayed in MATLAB.*

This supports testing a **50 µs current-response option** on the actual hot coil and driver. It does not support buying a higher-voltage supply as a substitute for measuring loop phase and bandwidth. Confirm stability, ripple, usable voltage and heat before promoting the faster tuning; the tested current-response envelope is an assumption about a regulator, not a demonstrated amplifier specification.

'''

ROBOT_UPDATE = '''##### Accepted travel band and sustained motion

The current objective accepts **±7.5 mm about the 12.5 mm stroke midpoint without a centering penalty**. The 0–25 mm physical stops remain unchanged, leaving 5 mm between either accepted-band edge and the corresponding stop. The learned controller may leave the robot stationary anywhere inside that band. Excursion outside it adds an increasing soft cost; contact loss and a physical stop still fail the run.

The raw learned correction can request up to ±10 mm/s, then the existing delay-aware sign/amplitude envelope, force gate and robot limits constrain the actual command. The native envelope permits only about 5.15 mm/s at a 7.5 mm offset. With a sustained 5 mm/s outward programmed motion, the steady balance is therefore near a 7.3 mm offset, leaving little band margin. Allowing a broad band does not justify waiting until its edge before compensating a known delayed command.

Development includes bounded programs that reverse, constant nominal motion from the beginning of the recording, and initial offsets outside the accepted band but inside the physical stops. Full histories report maximum and final band excess and time inside; an intentional initial excursion is not mislabeled as a failed physical stop. Queued nominal and correction commands remain active after the gate closes. Fresh long tests separately assess whether the selected policy maintains travel under continued motion.

'''


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--prose', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    prose = json.loads(a.prose.read_text())
    text = PREVIOUS.read_text()
    if prose.get('goal_addition'):
        text = text.replace('\n## Question\n', '\n'+prose['goal_addition'].strip()+'\n\n## Question\n', 1)
    draft = (STUDY / 'report_draft/unified_policy_methods.md').read_text()
    draft = re.sub(r'^## Unified recurrent controller\n\n>[^\n]*\n\n', '### Learned control policy\n\n', draft)
    draft = re.sub(r'^> \*\*Final sensing[^\n]*\n\n', '', draft, flags=re.M)
    # The draft's phase/subtopic headings sit one level above their final placement.
    draft = re.sub(r'^(#{3,4}) (?!Learned control policy)(.+)$', r'\1# \2', draft, flags=re.M)
    draft = draft.replace('Center ±7.5 mm is accepted without a substantial centering penalty.',
                          'Travel within ±7.5 mm of center has zero centering cost.')
    draft = draft.replace('The direct path carries fast force measurements around the encoder\'s saturating nonlinearity.',
                          'The direct path preserves fast force response alongside the recurrent history.\n\n![One shared recurrent controller and its causal signal paths](figures/unified_recurrent_control_band10_v3.svg)\n\n*Two outputs share one history. The hard robot gate, issued-command queue, current regulation and physical limits remain explicit.*')
    draft = draft.replace('The retained figure and complete records bind this interpretation:\n\n`acf_simulink/validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5/memory_reliance_v2.svg`',
                          'The experiment isolates the effect of retaining this network\'s state.\n\n![Effect of carrying or resetting recurrent state](figures/memory_reliance_v2.svg)\n\n*Four development pairs using the same earlier gen5 weights. History helps stroke management; force-peak changes are small and mixed. This is not a newly optimized memoryless baseline.*')
    draft = draft.replace('Evolution search evaluates candidates',
                          'Evolutionary reward search evaluates candidates')
    draft = draft.replace('The winner\'s actual ancestry must accompany its weights.',
                          'The selected checkpoint and its ancestry are recorded with the results below. [PPO method](https://arxiv.org/abs/1707.06347), [evolutionary reward optimization](https://arxiv.org/abs/1703.03864), [minGRU architecture](https://arxiv.org/abs/2410.01201).')
    placeholder = '**[FINAL RESULTS PLACEHOLDER — replace only after weights and inputs are locked.]**'
    start = draft.index(placeholder)
    end = draft.index('\n#### Recommendation', start)
    draft = draft[:start] + prose['learning_results'].rstrip() + '\n' + draft[end:]
    rec = draft.index('\n#### Recommendation')
    draft = draft[:rec] + '\n#### Recommendation\n\n' + prose['learning_recommendation'].rstrip() + '\n\n'
    begin = text.index('### Learned control policy')
    end = text.index('### Calibration, model qualification and prototype validation', begin)
    text = text[:begin] + draft + text[end:]
    if prose.get('learning_intro_addition'):
        intro = text.index('#### Intro', text.index('### Learned control policy'))
        question = text.index('#### Question', intro)
        text = text[:question]+prose['learning_intro_addition'].strip()+'\n\n'+text[question:]
    text = re.sub(r'\*\*Learning update, 5 October:\*\*[^\n]+', prose['top_summary'], text, count=1)
    force_section = text.index('### Force and position measurement')
    recommendation = text.rindex('#### Recommendation', 0, force_section)
    text = text[:recommendation] + ACTUATOR_UPDATE + text[recommendation:]
    robot = text.index('### Robot commands and recentering')
    robot_rec = text.index('#### Recommendation', robot)
    text = text[:robot_rec] + ROBOT_UPDATE + text[robot_rec:]
    if prose.get('acceleration_update'):
        begin=text.index('### Acceleration measurement')
        end=text.index('### Acquisition electronics:',begin)
        position=text.index('#### Recommendation',begin,end)
        text=text[:position]+prose['acceleration_update'].rstrip()+'\n\n'+text[position:]
    if prose.get('acceleration_recommendation'):
        begin=text.index('### Acceleration measurement')
        end=text.index('### Acquisition electronics:',begin)
        position=text.index('#### Recommendation',begin,end)
        text=text[:position]+'#### Recommendation\n\n'+prose['acceleration_recommendation'].rstrip()+'\n\n'+text[end:]
    text = text.replace('A learned robot policy must obey the same hard gate and command envelope, rather than learning whether to bypass them.',
                        'Keep ±7.5 mm as an acceptable travel region, anticipate already-issued motion and retain the same hard gate and command envelope for learned control.')
    text = text.replace('The component questions below cover mechanics,',
                        'Robot recentering accepts ±7.5 mm about mid-stroke without a centering penalty; force tracking and physical stops retain their limits.\n\nThe component questions below cover mechanics,')
    text = text.replace('| Learned control | Retain deterministic force control and robot recentering; keep the tested small networks as optional research alternatives |',
                        '| Learned control | '+prose['recommendation_table']+' |')
    final = text.index('\n## Recommendation')
    text = text[:final] + re.sub(r'\*\*Learning update, 5 October:\*\*[^\n]+', prose['final_learning_summary'], text[final:], count=1)
    for old,new in prose.get('exact_replacements',{}).items():
        assert old in text, old
        text=text.replace(old,new)
    text = text.replace('The approximately €9 converter reduces ADC cost;', 'The previously reviewed approximately €9 converter reduces ADC cost;')
    if prose.get('system_recommendation'):
        pos = text.rindex('**Build the first prototype')
        text = text[:pos] + prose['system_recommendation'].rstrip() + '\n'
    assert text.count('```text') == PREVIOUS.read_text().count('```text')
    assert '[FINAL RESULTS PLACEHOLDER' not in text and '[FINAL SYSTEM RECOMMENDATION PLACEHOLDER' not in text
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(text)
    print(json.dumps({'output':str(a.output),'words':len(text.split()),'figures':text.count('![')}))


if __name__ == '__main__':
    main()
