"""Prepare a versioned report around the user's fixed mechanical arrangement.

The completed prior report remains unchanged. Scored new-study sections and
final tuning/qualification must be integrated before this draft is delivered.
"""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
PREVIOUS = ROOT / 'reports/active-contact-flange-report-rl-3h-20261005.md'
OUTPUT = ROOT / 'reports/active-contact-flange-report-upper-spring-20261005.md'


def replace_recommendation(document, component, text):
    start=document.index('### '+component+'\n')
    end=document.find('\n### ',start+1)
    if end<0:end=document.index('\n## Recommendation',start)
    section=document[start:end]
    rec=section.index('#### Recommendation\n')
    section=section[:rec]+'#### Recommendation\n\n'+text.strip()+'\n'
    return document[:start]+section+document[end:]


def main():
    assert not OUTPUT.exists(), 'Preserve the existing versioned draft'
    original=PREVIOUS.read_text()
    contact=original[original.index('##### Contact model and disturbance severity'):original.index('##### Finite stiffness, joining mass and guides')]
    text=original
    intro_end=text.index('\n## Question')
    intro='''# Active contact flange for sanding

## Intro

We want an active flange that keeps the **normal pressing force experienced by the workpiece within ±0.5 N of a 5–50 N target**, including a **1 mm high, 10 mm wide bump scanned at 100 mm/s**. The required fixed orientations span **0–90°**, and the interchangeable tool mass is **S = 0.1–2 kg**. Holder B is assumed to be 0.6 kg, so the moving tool-side mass B+S is **0.7–2.6 kg**.

The mechanics are prescribed: **upper R–A spring = 20 N/mm with zero damping and ideal unlimited length/travel; lower B–S connection = rigid**. The rigid attachment still contains a finite-stiffness load cell, represented here by 64 MN/m. This report optimizes control and sensing around that arrangement. A is a dynamic mass on the upper spring: the development cases include 0.4 kg and a 1.5 kg body allowance representative of the real coil example. The actuator retains its 25 mm stroke and physical failure stops.

The controller may use a multi-axis load cell between B and S, tool position relative to A, an accelerometer on S, known tool weight or a gravity tare, and predictions from **robot commands already issued**. It receives no known bump profile, surface preview or future robot movement. We want understandable native Simulink blocks, measurable performance margin, and a concrete prototype recommendation. There is no external lower spring to pack; the prescribed upper spring is exempt from the earlier spring-length restriction.

For the first custom PCB, **the accelerometer and ADC chips together should cost less than €100**. Load cell, instrumentation amplifiers, clock, PCB manufacture and assembly are outside that subtotal. Earlier studies used an upper rigid attachment and lower spring/damper pair; those studies remain separately identified reference evidence.

**Study in progress.** The fixed-mechanics candidate is being developed and tested. The historical force passes below do not qualify this new arrangement.
'''
    text=intro+text[intro_end:]
    text=text.replace('**How should we design a compact active sanding flange that meets the pressing-force requirement with useful margin, reasonable mechanics and affordable sensing—and does a small learned controller improve that system?**','**How should we control and instrument the prescribed undamped upper spring and rigid lower attachment so the sanding flange meets the pressing-force requirement with useful margin and affordable sensing?**')
    text=text.replace('## Results\n\n','## Results\n\nThe current mechanical arrangement is the prescribed upper 20 N/mm spring and rigid lower connection. New native development results are being integrated below. All three completed large-grid studies summarized next used the previous upper-rigid/lower-spring-damper reference; they are retained comparisons.\n\n',1)
    text=text.replace('Use that baseline for the affordable bench prototype; neither a robust hardware margin nor a deployment-ready neural controller is established.','That baseline was the best affordable controller on its recorded reference mechanics; it is not qualified for the now-prescribed mechanics. Neither a robust hardware margin nor a deployment-ready neural controller is established.')
    mech='''### Mechanics: load path, springs and guides

#### Intro

The upper attachment is fixed by the user at 20 N/mm, without external damping or a spring-length/travel limit. The lower attachment is rigid. Control changes must handle the resulting A-body motion and finite sensing-cell mode rather than changing those attachments.

#### Question

Can the prescribed arrangement cross the sharp bump with ±0.5 N workpiece-normal force error, and which measurements and controller settings make it practical?

#### Results

##### Mechanical model and sensor placement

Robot boundary R supplies gross motion. The upper spring connects R to body A. The voice coil acts between A and holder B, with its equal-and-opposite reaction moving A. The complete B–S load passes through the multi-axis cell to tool S. The cell retains finite stiffness even though the external mounting is rigid. Position measures S relative to A; acceleration is measured on S beyond the cell.

![User-prescribed upper spring and rigid lower load path](figures/fixed-upper-rigid-lower-20261005.svg)

*Current mechanical contract and sensor locations. This is a functional diagram, not a dimensional drawing or an additional spring recommendation.*

Positions are positive away from the workpiece. The actuator gap is `u = xA − xB`; travel is its configured midpoint plus u. Loaded deviations obey:

```text
Q1 = Q10 + 64,000,000 (xS − xB)
Q2 = Q20 + 20,000 (xA − xR)
mA aA = Fa − Q2 − mA g_axis
mB aB = Q1 − Fa − mB g_axis
mS aS = Rz − Q1 − mS g_axis
```

Fa is net transmitted actuator force after physical guide friction; electromagnetic force is Kf times actual current. Gravity along the flange axis is `g_axis = g cos(theta)`. Both external damping coefficients are zero. The internal guide retains 0.2 N Coulomb friction and 2 N·s/m viscous friction. S remains constrained by maintained contact with the prescribed surface. Contact loss or an actuator travel stop terminates the recording and fails it.

Each recording begins at its own loaded equilibrium: `Q10 = Fref − mS g_axis`, `Q20 = Fref − (mA+mB+mS) g_axis`, and `Fa0 = Fref − (mB+mS) g_axis`. This is an already-contacting loaded start. It does not establish initial surface acquisition, a sudden target change, or a rotating loaded flange.

'''+contact+'''
##### Finite stiffness, joining mass and guides

With prescribed S motion, the 0.6 kg B stage and 64 MN/m sensing cell have a calculated mode near **1.64 kHz**. The undamped upper spring has a calculated A-body frequency near **35.6 Hz at 0.4 kg** or **18.4 Hz at 1.5 kg**. These frequencies follow from the native mass/stiffness equations; closed-loop motion also depends on current dynamics, sensing, control and guide friction. No external damper is added to suppress them.

An actual frame, sensing cell, guide and coil mounting have additional modes. The representative cell stiffness and ideal linear upper spring are model inputs; measure their assembled transfer. An unlimited ideal upper spring does not remove the actuator's stroke restriction. Sensing-cell force must remain in the complete S load path, without a parallel guide or support bypass.

##### Undamped mechanics with the current budget sensing chain

The new fixed-mechanics study uses the existing native physical model with a hot 24.4 N/A coil, 8.896 Ω winding, 4 mH inductance, 32 V usable bipolar drive, 3.5 A limit and 50 µs current-response target. The sensor contract remains the declared matched 1.9 kHz budget envelope, 30 ksample/s ADC, fresh 10 kHz control and the retained force, acceleration and 1 µm position noise.

Development uses the eight corners of S=0.1/2 kg, force=5/50 N and orientation=0/90°, plus three interior conditions, at both A masses. Every 2 s recording includes startup, the bump at 1 s and the endpoint. Every native 300 kHz physical sample contributes to the force/physical verdict. Algorithmic observer weights, filtered friction compensation and causal acceleration compensation may change; the physical noise and prescribed connections stay fixed. Prepared cases and software-only wiring checks are separate from executed recordings.

**Pending executed evidence.** Controller development and subsequent locked evaluation results will replace this paragraph before delivery.

##### Spring and damper search

Earlier searches selected an upper rigid attachment with a lower 19.746 N/mm spring pair and 60 N·s/m damping. Those recorded studies explain the prior reports and remain reproducible in the [previous report](active-contact-flange-report-rl-3h-20261005.md). Their nominal-grid, tolerance and spring-packing results apply to their recorded load path. They do not choose the prescribed upper spring or replace the current rigid lower attachment.

#### Recommendation

Keep the **prescribed 20 N/mm undamped upper spring** and **rigid B–S attachment through the finite-stiffness multi-axis cell**. Use a guided voice coil, S-mounted synchronized acceleration sensing and S–A position sensing. Identify A/B/S masses and the assembled cell/guide response for the controller. The following sections distinguish the new fixed-mechanics candidate from the preserved reference-controller experiments.
'''
    start=text.index('### Mechanics: load path, springs and guides\n');end=text.index('### Actuator and electrical drive\n',start)
    text=text[:start]+mech+'\n'+text[end:]
    # Each retained results block explicitly owns its recorded configuration.
    for component in ['Actuator and electrical drive','Acceleration measurement','Acquisition electronics: amplifier, ADC and timing','Calibration, model qualification and prototype validation']:
        start=text.index('### '+component+'\n');at=text.index('#### Results\n\n',start)+len('#### Results\n\n')
        note='The completed experiments in this retained section used their recorded upper-rigid/lower-spring-damper reference. Component specifications and signal definitions remain useful; their force passes do not qualify the current fixed mechanics. New-study outcomes are identified in the mechanics and controller sections.\n\n'
        text=text[:at]+note+text[at:]
    architecture='The fast loop regulates reconstructed normal force. The current fixed-mechanics candidate uses a four-state motion observer: A minus R position/velocity and A minus S position/velocity. Measured S–A position corrects the estimate; matched measured S acceleration, transmitted-force estimation and already-issued robot commands propagate it. Estimated tool velocity is `vS_hat = vR_hat + (vA−vR)_hat − (vA−vS)_hat`. Thus upper spring motion is accounted for before estimating the workpiece normal. Sensor and normal estimates share one acquisition frame.\n\nThe measured-force version estimates net transmitted force from the existing cell and holder inertia: `deltaFa_hat = deltaL − mB_hat aS_hat + (mB_hat/kCell_hat) H2(s) deltaL`, where `H2(s)=s²/(1+s/(2πfc))²` is a proper bounded derivative. The alternative uses the already-issued current request, an identified current lag and guide-friction prediction in the same measurement frame. Both are causal; neither uses actual plant current, A motion, surface slope or future commands. Observer/process weights and regularization are engineering settings tested on development recordings.\n\nThe remaining core-controller comparisons below describe the previous reference mechanics. Its lower-link command inverse is disabled for the current externally rigid lower connection; the small sensing-cell correction above serves the observer, rather than a soft-link command inverse.'
    old='The fast loop regulates reconstructed normal force. A PI/PID correction combines with gravity compensation, measured-acceleration feedforward and a measured correction for surface slope and sanding drag. The two-state motion observer estimates S–A separation and velocity from relative position, measured S acceleration and the causal robot-motion predictor. With upper A rigidly attached to the prescribed robot boundary, these give an estimate of the tool\'s axial velocity and therefore local surface slope. An upper compliant attachment needs an additional estimate of A\'s motion before this reconstruction can be qualified.'
    assert old in text;text=text.replace(old,architecture,1)
    text=text.replace('| Selected core setting | Value | Meaning |','| Historical reference core setting | Value | Meaning |')
    text=replace_recommendation(text,'Fast force controller','Use measured-inertia PI/feedforward with a four-state upper-motion observer, measured reaction-vector/drag reconstruction, finite-cell mode filtering and guarded robot recentering. Keep the lower soft-link command inverse disabled. Select and qualify the new observer and compensation coefficients on the prescribed mechanics; the historical P=0.3/I=20 setting is not a current rigid-lower recommendation.')
    text=replace_recommendation(text,'Learned control policy','Keep the unified minGRU checkpoint as research evidence. Its completed training and locked tests used the previous mechanical arrangement. It is not qualified for the prescribed upper spring and rigid lower connection, and the previous fresh-force and long-travel failures already prevent deployment. Use the deterministic candidate and independently validate the assembled prototype before considering a new learned-policy study.')
    rec=text.index('\n## Recommendation\n')
    text=text[:rec]+'''
## Recommendation

Build the bench prototype around the **prescribed 20 N/mm upper spring with zero external damping, rigid B–S mounting through the multi-axis load cell, and a guided voice coil**. Measure S acceleration and S relative to A; reconstruct workpiece-normal pressing force after tool-inertia compensation. Use the new four-state upper-motion estimate with measured-inertia PI/feedforward and gated delayed robot recentering. The physical actuator keeps its travel stops; the no-penalty recentering band is ±7.5 mm about stroke midpoint.

| Subsystem | Concrete prototype starting point |
| --- | --- |
| Upper attachment | User-prescribed 20 N/mm net stiffness, zero external damping; ideal unlimited spring length/travel in this model |
| Lower attachment | Rigid complete load path through the multi-axis cell; representative 64 MN/m finite stiffness, zero added damping |
| Masses | B=0.6 kg, known interchangeable S=0.1–2 kg; identify total A mass including coil body and attachments |
| Actuator | Guided bipolar voice coil, approximately 20–25 N/A, 25 mm travel; continuous force/current sized at hot operating temperature over used stroke |
| Driver | 36 V nominal supply only if it provides at least 32 V usable bipolar output; 3.5 A peak model limit, measured 50 µs current-response target |
| Acquisition | One ADS131M08 at 30 ksample/s per simultaneous channel; measure the complete installed transfer and timing |
| Acceleration | One ADXL354C, ±8 g, two analog axes for the planar prototype; one ADC plus accelerometer chip subtotal about €86 including VAT in the retained price review |
| Position | S relative to A; 1 µm conditioned output noise assumed in the development bank; synchronize its frame with force/acceleration |
| Force controller | 10 kHz measured-inertia PI/feedforward, four-state upper observer, finite-cell filtering; final coefficients require completed new-study selection |
| Robot | Identify loaded command delay/response; retain queued commands when the force-stability gate closes; manage the ±7.5 mm band separately from force and hard stops |
| Learned policy | Retained research checkpoint; unqualified on the prescribed mechanics |

**Pending executed evidence.** The final selected coefficients, exact performance, remaining limitations and prototype acceptance conditions will replace this paragraph before delivery.
'''
    OUTPUT.write_text(text)
    print(OUTPUT)


if __name__=='__main__':
    main()
