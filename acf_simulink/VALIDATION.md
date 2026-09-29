# Validation record

Authored 2026-09-24. These are software checks with illustrative parameters, not hardware validation.

## Executed checks

- MISS_HIT parsed and linted every MATLAB source file successfully.
- 150 independent algebraic cases checked Newton force balance for unequal A/B masses, with the three actuator variants. Screw rotor torque balance was also checked.
- Initial loaded equilibrium produced zero acceleration/current derivative to numerical precision for each actuator.
- An independent Python translation ran the complete sampled control/robot/plant simulation for the cases below.
- A separated, closing tool was projected to rigid contact and produced a positive inelastic impulse. A severe bump produced loss of contact and one re-impact.
- An ideal actuator limited to 13 N and a requested acceleration limit of 0.05 m/s^2 saturated at 13 N and failed force tolerance. External loading drove acceleration above the requested cap, as expected.

## Example results

All use a 20 N setpoint and the final gains in acf_parameters.m. Numerical step is 0.2 ms. These are Python reference results, not an executed Simulink run.

| Actuator | Scenario | Peak force error (N) | Peak gap acceleration (m/s²) | Peak current (A) | ±0.5 N result |
|---|---|---:|---:|---:|---|
| leadscrew | flat | 0.037335 | 0.025469 | 0.291251 | Pass |
| leadscrew | bump | 0.324626 | 0.161702 | 0.362483 | Pass |
| voice_coil | bump | 0.245836 | 0.417172 | 0.642689 | Pass |
| ideal_position | bump | 0.361528 | 0.089602 | 0.000000 | Pass |
| leadscrew | sharp_bump | 1.288979 | 2.587971 | 1.594103 | Fail |
| leadscrew | ramp | 0.096645 | 0.025469 | 0.291251 | Pass |

The standard bump is 1 mm high and 60 mm wide at 100 mm/s. The sharp bump is 1 mm high and 10 mm wide. The ramp rises 3 mm across 200 mm. Robot delay is 80 ms. These actuator/controller examples are not a fair hardware comparison or proof that one technology is superior.

## Timestep check

For the standard leadscrew bump, reducing the numerical step from 0.2 ms to 0.1 ms changed peak force error from 0.3246255796 N to 0.3246257781 N: a difference of 1.98e-07 N.

The sharpened stress case (10 mm height, 5 mm width) produced one impact. It was classified as a failure. The numerical impulse is resolved only to the integration step; no finite impact-force accuracy is claimed.

## Checks still requiring MATLAB/Simulink

No MATLAB or Simulink runtime was available in the authoring environment. The native diagram has therefore not been built, compiled, rendered or simulated here. Its source uses documented MathWorks APIs. acf_build compiles it before saving the SLX on your machine.

After acf_start, run:

```matlab
acf_selftest(acfModel, acfParameters)
```

This runs the MATLAB physics/contact/delay checks and compares a short Simulink run with acf_reference, which uses the same MATLAB component functions. The simulator comparison and the MATLAB self-tests are supplied but were not executed here.

Before a hardware decision, enter measured mechanical/motor/sensor parameters, repeat the timestep check on the hardest case, and evaluate a stated surface/speed/robot-delay envelope. Controller tuning and omitted mechanics can change the requirements substantially.
