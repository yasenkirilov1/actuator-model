# Actuator comparison: bounded physical provisions

These are input assumptions and manufacturer reality checks for the new common-sensor native comparison. They are not measured prototype performance. Native experiment counts and force results belong to the separately hash-bound `inertial_actuators_*.json` recordings.

The illustrative screw motor is the [maxon RE25 302005](https://www.maxongroup.com/maxon/view/product/302005): a 130 g, 24 V DC winding with 23.4 mNm/A torque constant, 2.32 Ω resistance, 0.238 mH inductance and 10.8 g cm² rotor inertia (1.08×10⁻⁶ kg m²). Its catalog maximum continuous current is 1.16 A and torque 26.3 mNm. This product is marked NRND and is a winding reference, not a purchasing recommendation. Stall current is not a qualified continuous or pulse rating.

The native comparison adds **0.5×10⁻⁶ kg m²** screw/coupler inertia as an explicit engineering assumption. For perspective, a solid 8 mm diameter, 100 mm long steel shaft at an assumed density of 7,850 kg/m³ contributes approximately 0.316×10⁻⁶ kg m² before coupling inertia. A screw drawing and actual rotational inertia are required before prototype qualification. Separate thrust bearings must carry the axial tool load; a motor shaft bearing rating does not establish an assembled screw thrust rating.

For ideal reversible pitch conversion, `ell = lead/(2π)`, axial force per current is `Kt/ell`, and the added axial inertia is `Jrot/ell²`. The retained winding and assumed total rotating inertia give:

| Lead | Axial force constant | Reflected rotating mass | Current for 50 N | Cold copper loss at 50 N | 90% sizing current |
|---|---:|---:|---:|---:|---:|
| 3 mm/rev | 49.01 N/A | 6.931 kg | 1.020 A | 2.415 W | 1.134 A |
| 5 mm/rev | 29.41 N/A | 2.495 kg | 1.700 A | 6.708 W | 1.889 A |
| 10 mm/rev | 14.70 N/A | 0.624 kg | 3.401 A | 26.831 W | 3.779 A |

The 90% column is a conservative motoring sizing allowance, not a claim that the native motor transformation implements constant efficiency. The native model retains reversible pitch conversion plus explicit smooth Coulomb and viscous loss torque. Scaling torque by a constant efficiency while leaving back EMF unchanged would give an inconsistent regenerative model. The default screw loss torque (1 mNm Coulomb, 1 µNm·s/rad viscous, transition speed 1 rad/s) and 3.5 A peak/24 V four-quadrant drive are engineering assumptions requiring assembled measurement. Current-response tuning of 100 µs is an active regulation target; it is distinct from passive winding L/R ≈103 µs.

At 50 N and 90°, the 5/10 mm examples already exceed catalog continuous current before friction. A transient normal-force pass would therefore not establish continuous sanding capability. The 3 mm example reduces holding losses but makes the rotating inertia large relative to the 0.6 kg attachment. Other winding sizes, larger continuous torque, pitch, direct current control and acceleration feedforward may change this tradeoff. The tested motor/controller cannot establish that all screw actuators fail.

The 3 mm pitch also has little torque margin: 26.3 mNm catalog continuous torque converts to **49.6 N** after the separate 90% motoring sizing allowance, before additional guide friction. Its current criterion alone therefore does not establish a 50 N continuous design.

A larger motor can remove this static limitation. The [maxon EC-i 30 539489](https://www.maxongroup.com/maxon/view/product/539489) is a 242 g, 48 V, three-phase reference with **103 mNm continuous torque**, 1.82 A continuous current, 55.4 mNm/A torque constant and 1.55×10⁻⁶ kg m² rotor inertia. A 10 mm pitch converts its catalog torque to **64.7 N ideal**, or **58.2 N** with the illustrative 90% sizing allowance. With the same assumed screw/coupler inertia it reflects **0.809 kg**. Thus a compact screw system is physically plausible on continuous-force and inertia grounds; it is not ruled out by the smaller RE25 results.

This brushless example was **not simulated or qualified**. It needs a synchronized commutated current drive, characterized ripple/cogging, bearings, screw losses/backlash, thermal validation and an appropriately identified current/force model. Catalog terminal resistance/inductance and bus, phase and torque-producing current conventions must be resolved before using it as a scalar winding substitute. The native experiments below retain the explicitly stated RE25 winding; they do not claim to test the EC-i.

## Voice-coil guide friction

THK publishes a rolling friction coefficient of **0.001–0.0025** for its VR/VRU/VRT crossed-roller guides and tables, with drag affected by preload, bearing load and lubricant. Its [selection criteria](https://tech.thk.com/es/products/pdf/en_b00_015.pdf) also warns that moments and preload increase resistance. A [VR2-60×11Z dimensional example](https://tech.thk.com/en/products/pdf/en_a07_007.pdf) has 60 mm rails and 30 mm maximum stroke; the 12 mm combined dimension and 6 mm rail height demonstrate a compact guide class. Four rails and two cages make one set. This is a geometry illustration rather than assembled flange sizing. The [THK VR product description](https://www.thk.com/us/en/products/other_linear_motion_guides/cross_roller_guide_ball_guide/vr/) describes two parallel guide units for loads in four directions and adjustable preload.

With an **assumed** equivalent rolling load/preload of 50–100 N, `µP` gives approximately 0.05–0.25 N rolling drag. Thus **0.2 N** is a plausible guide-friction *design target*. Sanding side loads and moments, selected preload, mounting alignment, seals, dust protection and flexible power cables can increase it. The catalog coefficient does not prove total assembled drag, the model's **2 N·s/m** viscous term, or its **0.2 mm/s** smooth reversal transition. Preserve these terms in nominal experiments and test realistic larger losses separately; measure friction in both directions across speed, stroke, temperature, side load and contamination before assigning an uncertainty allowance.

The representative 25.4 mm/64.7 N coil and its mass/thermal/force-versus-travel limitations are recorded separately in [realistic_voice_coil_examples.md](realistic_voice_coil_examples.md). Its stationary body exceeds the earlier A=0.4 kg allocation; a real motor comparison must include that body and mounting mass explicitly.
