# Final native actuator comparison with the selected acquisition pipeline

The voice coil and the modeled leadscrew both meet the ±0.5 N normal-force
requirement in the four matched final checks below. The selected small
leadscrew motor does **not** satisfy continuous holding duty. This is a
bounded, nominal model comparison; the leadscrew has not been qualified over
the complete mass, force and orientation mesh or as a physical prototype.

## Executed protocol and provenance

Nine native recordings were executed in one isolated MATLAB batch process:
four voice-coil cases, the same four leadscrew cases, and one leadscrew solver
refinement. Each recording lasts **4 s**, includes startup and one sharp bump,
and uses the 1 mm high, 10 mm wide bump at 100 mm/s. This was the prepared
final protocol. The earlier lower-series-v1 matched comparison used 10 s
recordings and is separate evidence.

- Immutable combined donor:
  `/home/tommie/.local/state/actuator-model/inertial-delivery-frozen-actual-acquisition-actuator-donor-v8-20260930/`.
- Native public model SHA256:
  `7fa60a844f8d7d0b0b1d15d4afcdc901c6d7cbe26a463e41cb2e0b78e8d8e0f8`.
- Prepared candidate SHA256:
  `65b97e2e4703b69f2a47a8d8d819fc22f82c94daf115e24d75f1dd84156b305f`.
- Executed recipe `experiments/qualify_inertial_actuators_final.m` SHA256:
  `3e6471e63e00f792b5a000f6fdc7ff5403596b9c7ed8a1f9ef70492d12e66c34`.
- Results JSON SHA256:
  `1b4a3bcbdd84cc954da6653dcccf7fbd5a40435e32a486bcc5416e1b84e7a569`.
- Plot replay JSON SHA256:
  `7ffcbe20f23ec24264e6cb99eff167f1dd817b44c4a5e401d6c83ed0d4eeb246`.
- Private worker:
  `/home/tommie/.local/state/actuator-model/tp7f84836f_a1e2_40da_ad8f_19ed50fa47ed/`.

The model, all matching helpers and the candidate were copied privately.
The model graph and controller settings were used directly: no grafting or
tuning occurred in this final comparison. Source and private model bytes,
and the frozen candidate bytes, were checked unchanged after the recordings.
The batch exited with code 0. It used two computation threads, private
preferences/cache/code/SDI storage on disk, cleared its own SDI between
recordings and restored SDI preferences on exit. The desktop model was not
used.

All eight base recordings retain 400,001 native samples at a 10 µs solver
step. The refinement retains 800,001 samples at 5 µs. **Every full native
history is saved**, including the full parameter structure and all sensing,
control and physical channels. No failed cases or preparation rejections
occurred. Three active-branch compilations were checked for algebraic loops;
none were found.

The public `r.data(:,1)` is native workpiece normal force. The separately
retained `r.axialContact.force_N` is used for axial force-balance checks.
Scoring is `max(abs(true normal force − target))` over every native sample,
including startup, with completion, contact, travel, current, force-balance
and spring-packing checks. Plot reduction is independent of these scores.

## Common mechanical, sensing and control conditions

- Robot-side A = 0.4 kg; moving holder B = 0.6 kg. S is the complete known
  downstream tool mass, as listed in the table.
- The upper robot attachment is rigid. The lower connection has effective
  stiffness 19,746 N/m and damping 60 N·s/m. Its opposed pair has two nominal
  9,873 N/m springs, common preload 33.6 N, 20 mm free length each and a
  ±0.49 mm length tolerance provision.
- The multiaxis cell is rigidly mounted between the joining frame and S in
  the load path. The entire S mass is downstream. Its 64 MN/m stiffness is
  catalog-reference metadata in this soft-link, massless-series
  approximation; there is no separate intermediate-frame/cell mode.
- Measurement uses the B–S force vector, an accelerometer on S and S relative
  to A position. Contact-force reconstruction compensates downstream S
  inertia before projecting onto the estimated surface normal.
- Friction coefficient µ = 0.3 in these four matched cases. The controller
  uses a measured drag-ratio estimate with a 5 ms filter, not the plant's
  true µ or the known bump profile.
- Raw acquisition is 100 kHz, with the recorded 5.1 kHz analog antialias
  provision, 50 µs raw EMA and 20 µs pipeline delay. Force/control sampling
  is 10 kHz, with the selected matched 100 µs force and acceleration
  filtering provisions. Force, position and acceleration are synchronized.
- Noisy cases use 0.02 N force noise per modeled axis, 1 µm position noise
  and 0.014 m/s² conditioned acceleration noise. The model's 0.00468 m/s²
  acceleration quantization and range guard remain enabled in the noise-off
  case. These are simulation provisions, not measured assembled hardware
  accuracy.
- Robot force-stability gating and the fast variation gate remain enabled.
  The latter uses the selected 0.08 N RMS threshold, 20 ms variation window,
  100 ms mean/filter/dwell provisions. Existing queued commands remain in
  the native delayed robot path.
- Both drive current-response provisions are 100 µs. Both controllers use
  force-normalized proportional/integral gains of 6 N/N and
  400 N/(N·s), a 1 kHz feedback filter, causal measured-motion feedforward
  and the proper 1.5 kHz lower-link inverse. Acceleration prediction and its
  additional command filter are bypassed. No total-current notch is active.

These matched model conditions do not establish catalog sensor bias,
crosstalk, scale, filter phase, temperature drift, current bandwidth or real
robot tracking. The retained force-cell and position-sensor research notes
explicitly distinguish catalog resolution from the required assembled
calibration residual.

## Matched results

The reported mass is **S alone**; the 0.6 kg moving holder is additional.
Noise-off uses actual seed 29 with zero stochastic noise, retaining
quantization. Noisy runs use seed 1729.

| S | Target | Orientation | Noise | Coil peak error | Screw peak error |
|---:|---:|---:|---|---:|---:|
| 2 kg | 50 N | 90° | off | 0.293260 N | 0.301007 N |
| 2 kg | 50 N | 90° | seed 1729 | 0.283522 N | 0.292191 N |
| 2 kg | 5 N | 0° | seed 1729 | 0.274796 N | 0.282736 N |
| 0.1 kg | 50 N | 90° | seed 1729 | 0.100459 N | 0.219436 N |

All eight matched cases pass. Their largest coil error leaves 0.206740 N to
the force limit; the largest screw error leaves 0.198993 N. A single noisy
seed giving a smaller peak than the noise-off case does not demonstrate that
noise improves the controller or establish a statistical tolerance bound.

Across the four coil cases, peak current is 2.899233 A against the 3.5 A
provision and peak voltage is 18.444880 V against 24 V. Across the four screw
cases, peak current is 4.155383 A against 5 A and peak voltage is 10.383110 V
against 24 V. Neither current effort nor voltage clips. These are modeled
drive envelopes; the 5 A screw peak has no catalog-qualified duration here.

Normal-force geometry, downstream S force balance and combined B/S axial
force balance all have worst residual below 2.85×10⁻¹⁴ N. The additional
leadscrew rotor force-balance residual is below 2.27×10⁻¹⁴ N. All traces stay
in contact, avoid travel stops and pass the selected spring working-range
and packing checks.

The combined conservative free spring length is **40.98 mm**, and the maximum
combined installed length is **34.173558 mm**. The smallest individual spring
compression in these base checks is 0.572987 mm; the largest individual spring
force is 61.542900 N. These are model/formula checks of the spring provision,
not a purchased spring's fatigue certificate or a whole-flange packaging
qualification.

## Leadscrew model and thermal meaning

The screw is a conditional 10 mm/rev axis using the RE25 reference motor:
Kt = 0.0234 N·m/A, R = 2.32 Ω, L = 0.238 mH, 24 V and 5 A peak. Motor rotor
inertia is 1.08×10⁻⁶ kg·m²; assumed screw/coupler inertia adds
0.5×10⁻⁶ kg·m². The model uses ideal torque-to-axial conversion plus its stated
smooth Coulomb and viscous losses. It does not model efficiency variation,
backlash, commutation ripple or a thermal state.

With ℓ = pitch/(2π), the force constant is Kt/ℓ = 14.702654 N/A and the
reflected inertia is J/ℓ² = 0.623759 kg. Its lower-link inverse uses
B+J/ℓ² = 1.223759 kg, with a separate causal issued-robot-acceleration term.
The mapped Coulomb force is 0.001 N·m/ℓ = 0.628319 N. The independent current
PI gains are therefore 0.408090 A/N and 27.205973 A/(N·s); the coil gains are
0.3 A/N and 20 A/(N·s). The common external mechanics and sensing are held
fixed while these drive-specific quantities remain explicit.

At 50 N and 90°, the screw's static holding current is **3.400747 A** and its
static copper loss is **26.830980 W**. The reference RE25 catalog provision
is 1.16 A continuous. At the 2 kg, 5 N, 0° case, gravity compensation still
requires −1.394714 A and 4.512928 W copper loss, also above that continuous
current rating. Thus **all four selected screw load cases fail the continuous
catalog check**, even though their transient force traces pass.

The nominal coil provision holds 50 N at 90° with 2.5 A and 25 W copper loss.
It has no thermal model or catalog continuous-current qualification in this
comparison. A real coil must meet the separately specified continuous force
and thermal envelope at its operating travel and temperature.

The modeled screw demonstrates that a screw can track these selected sharp
tests with the common sensor reconstruction and a suitable native current
controller. The small reference motor is unsuitable for continuous duty at
these selected loads. A larger continuous-torque motor or another pitch can
be mechanically viable; neither is dynamically qualified by this final
four-case comparison. At 10 mm pitch, 50 N alone requires approximately
79.6 mN·m before efficiency and friction allowances. Larger-motor catalog
examples and the assumed guide-friction provision are documented separately
in `inertial_actuators_provision.md`.

The controller uses measured S–A velocity as an approximation for guide
velocity when the lower link is compliant. No B velocity sensor or shaft
encoder was added to the selected current-force loop. An encoder, improved
velocity estimate, motor ripple model or changed screw pitch would require
its own recorded comparison.

## Solver refinement and retained figures

The heavy noisy screw base case has peak error **0.292190818868 N** at 10 µs.
The otherwise identical 4 s case has **0.292190833737 N** at 5 µs, an increase
of **1.4869343×10⁻⁸ N**. Both pass, with the same non-clipping result and valid
force balances. This checks the selected screw and this endpoint; it is not
a refinement study for every alternative actuator or parameter combination.

`inertial_actuators_final_actual_acquisition_v8_20260930_replay.json` contains
all nine histories reduced to approximately 1 ms plus native global extrema.
Its force-peak assertions passed for every recording. It binds the native
trace hashes, source/candidate/recipe hashes and full-history peak/RMS values.
Qualification always uses the full histories, not this plotting reduction.

The shared S=2 kg, 50 N, 90°, µ=0.3, seed-1729 condition is also global case
3500 in the frozen V8 coil mesh. Its independently scored peak and RMS force
error, peak current, peak voltage and minimum/maximum actuator travel match
final donor case 2 exactly. `..._mesh_bridge.json` records those six scalar
comparisons. This complements the integrator's full-history source-parity
checks; it is not an additional full-history equality claim.

The reviewed PNG/SVG figures are:

- `inertial_actuators_final_actual_acquisition_v8_20260930_peak_errors`:
  the four matched full-record peak errors against the ±0.5 N criterion.
- `inertial_actuators_final_actual_acquisition_v8_20260930_heavy_bump`:
  the heavy noisy bump's true force error, current, voltage and actuator
  travel. Actuator travel is distinct from the measured S–A position.

Reproduction uses `validation/inertial_actuators_launch_final_v8.m`, the
immutable donor above and a fresh result tag. The read-only export is
`validation/inertial_actuators_export_final_v8.m`; figure generation is
`validation/inertial_actuators_plot_final_v8.py`. The full 3500-case nominal
coil mesh is a separate study using the frozen V8 coil source, with its
source-to-delivery parity chain recorded by the integrator.

The recommendation remains a guided voice-coil axis meeting the approximate
continuous-force/thermal envelope, paired with the selected soft lower link
and synchronized force, acceleration and relative-position sensing. This
comparison supports that prototype direction while preserving the screw as
a viable modeling alternative requiring a larger continuous provision and
its own physical and full-range qualification.
