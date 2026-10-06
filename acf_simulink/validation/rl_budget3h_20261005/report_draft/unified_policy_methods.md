## Unified recurrent controller

> **Internal methods draft.** The final checkpoint, fresh qualification results and system recommendation have not been inserted. Development results below identify their specific checkpoint and cases. Do not publish the placeholders as completed experiments.

> **Final sensing model locked; fresh qualification results pending.** Selected checkpoint `ca74a1be34ff2cb801f2a094aeae2e30968f52b3aaec6aacea216ca2b11ddaef` uses ADXL354C plus ADS131M08, a declared 1.9 kHz matched response envelope and 50 µs current response. These are the locked model settings, not completed qualification results. Sensor, acquisition and driver statements are bound to `selected_controller/cohort_manifest.json` and its hashed budget bank/native metadata. Earlier gen5 memory and arithmetic records below retain their own sensing/current-response inputs.

### Intro

Our goal is one compact controller choosing voice-coil current and robot correction from causal measurements and known tool parameters. The workpiece's normal force must stay within ±0.5 N through the 1 mm × 10 mm bump scanned at 100 mm/s, across 5–50 N, S = 0.1–2 kg and fixed orientations of 0–90°. B is 0.6 kg. Center ±7.5 mm is accepted without a substantial centering penalty.

Both actions use one shared history: current updates every 100 µs and new robot corrections are permitted at 100 Hz. Native Simulink mechanics, contact, sensing, current regulation, command delays and limits remain the environment.

### Question

Can the policy learn fast force control and delayed robot correction using measurable inputs and the existing stability constraints? A good result needs complete force recordings and useful robot actions; low force error alone can hide a robot that receives no commands.

### Results

#### Design and causal inputs

The network has a 48-to-32 encoder, two genuine Puffer minGRU layers with 32 states each, and two action means. Recurrent, direct measurement and physical-product readouts combine within the same actor. The direct path carries fast force measurements around the encoder's saturating nonlinearity.

The observation contains 40 values; eight products are derived inside the policy. Fixed centering/scaling comes from engineering ranges and is stored with the weights.

| Input group | External values | What the policy receives |
|---|---:|---|
| Force reconstruction | 3 | Normal, axial and lateral force from synchronized load-cell and downstream S-inertia measurements. |
| Tool motion and geometry | 6 | Measured S acceleration in two modeled directions, S–A position, observed relative velocity, estimated actuator travel and estimated local slope. |
| Commands, guard and recenter history | 9 | Present/aligned predictions from issued robot commands; issued current, total robot velocity and correction; gate readiness; recenter error. |
| Known or identified context | 21 | Target, masses, orientation, spring/damper and coil parameters, electrical/travel limits, equilibrium current and acquisition/robot timing. |
| Robot clock | 1 | The current 100 Hz decision phase. Absolute episode time and bump timing are absent. |
| Derived features inside the policy | 8 additional | Mass-conditioned acceleration, S–B interactions, force geometry, issued robot acceleration, damping × velocity and stiffness × travel error. |

Inference uses the causal observation port and identified context. True force and hidden states feed scoring/rewards. Surface profile and future motion stay outside inference; prediction uses commands already issued, including queued commands.

The minGRU state is carried between decisions and across rollout/chunk boundaries. It resets at a new physical episode. Closing the robot gate preserves both the neural history and commands already in flight. The initial paired filters span 0.1, 0.4, 3 and 20 ms; their weights and update gates can change during learning.

The two latent means are mapped to full requests as follows:

- Current request: known static equilibrium current plus `2 A × tanh(current mean)`.
- Robot correction request: `10 mm/s × tanh(robot mean)` at eligible 100 Hz events.

The experimental current replaces the dynamic force PI/feedforward command. Static equilibrium uses known weight/orientation and target; native current regulation and physical limits still apply. Robot force-stability/variation gates, dwell, delay-aware sign/amplitude envelope and velocity limits remain hard constraints. An ineligible update sends no new correction and cannot withdraw an issued command.

Travel excursion beyond center ±7.5 mm is penalized. Some development cases intentionally start outside: report entry, time inside and maximum/final excess. Physical 0–25 mm stops remain failures. A stationary robot within the band is acceptable.

#### Initialization and learning

Initialization uses a 0.1 A/N direct force gain and signed filter pairs cancelling their common startup component. Later probes initialized trainable recurrent/auxiliary connections with an approximate known spring/damper inverse. This disclosed physics prior remains within the actor's coefficients.

PPO samples continuous latent actions and updates the actor from rewards and a value head. Discount/advantage constants use physical seconds. Contact/travel failures terminate; time limits bootstrap the final causal observation with cloned state. Robot losses use eligible 100 Hz actions. The stabilized critic detaches its gradient from the shared trunk to limit actor interference.

Evolution search evaluates candidates in the native closed loop using reward, antithetic directions and covariance adaptation. There are no teacher actions or supervised labels. The band/history stage searched 55 coefficient directions; inverse refinement searches 37. The architecture stores 8,101 trainable parameters. These searches optimize constrained subspaces; PPO can update the full actor. The winner's actual ancestry must accompany its weights.

Whole trajectories separate training, selection and final evaluation. Weights, source, native library, inputs, seeds and identities lock before fresh outcomes. Metrics include every native sample, startup and final sample. Failures/rejections remain retained. Assembled sanding performance requires measurement.

#### Completed development evidence

One fixed band-search checkpoint, `48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4`, was tested with its 64 learned states either carried normally or reset at every 100 µs decision. Both variants used identical weights, sensors, observers, plant, seeds and robot queues. TRAIN cases 0, 40, 96 and 139 produced eight complete recordings and 7,800,008 scored native samples. All eight passed the force requirement and avoided physical failure.

The force-peak changes were small and mixed. Carried/reset peaks were 0.10811/0.10832 N, 0.40415/0.40447 N, 0.15539/0.15087 N and 0.23457/0.23877 N. Travel showed a clearer history effect: TRAIN 96 spent 90.28%/51.59% of its recording inside the accepted band, and TRAIN 139 spent 82.71%/0%. The latter ended inside the band with carried state and 1.406 mm beyond it with resets. This is evidence that the learned history affects robot correction in these development cases. The reset variant retains the external causal observers and is not an independently optimized memoryless controller.

That checkpoint has 30 nonzero recurrent current-readout weights and 10 nonzero robot-readout weights, with L2 norms 0.784 and 4.285. Its band-search stage evolved 55 directions. The retained figure and complete records bind this interpretation:

`acf_simulink/validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5/memory_reliance_v2.svg`

#### Native inference and numerical precision

C inference carries 64 float32 states and emits action means/value. Its 8,181-float buffer holds parameters plus 80 normalization values, about 32.7 kB. CPU collection avoids per-tick GPU synchronization; the GPU performs PPO learning. Native recordings independently score force, electrical effort, travel and commands.

An inverse initialization with readout coefficients up to 662.1 amplified C/PyTorch state differences and cancellation error. Its first rollout showed a maximum current-request difference of 0.3155 mA and log-probability differences of 0.0356 on CUDA/0.0394 on CPU. Replay guards stopped that PPO continuation before an update. Both readout rounding and recurrent arithmetic contributed.

Fixed C closed-loop results retain their implementation/hashes. PPO needs verified behavior replay. Export acceptance must use the actual selected weights and arithmetic; compiler options and datatype are part of that contract.

The replay measurements are retained in `validation/rl_budget3h_20261005/runner_checks/inverse_replay_v1/results.json`; the four-case memory test is retained in `validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5/results.json`, both under `acf_simulink/`.

**[FINAL RESULTS PLACEHOLDER — replace only after weights and inputs are locked.]** Insert the selected checkpoint and training ancestry, paired fresh 32-case results, paired 700-point nominal-grid results if executed, retained long-run results if executed, full native sample counts, failures, robot/travel behavior and the exact comparison to the verified baseline. Planned grids and prepared binaries are not completed experiments.

### Recommendation

Select using force peaks, failures and travel behavior together, then evaluate the locked winner on fresh native cases. Keep force strict and travel-band metrics separate. Retain weights, normalization, action scale, native inputs and compiler/export hashes.

For reuse, `policy_export.py` emits C source/header, flat weights, layout/features and checkpoint. Supply 40 causal observations, carry one 64-value state per flange and apply the recorded action mapping/guards at the qualified cadence. Deployment uses deterministic means; exploration/value support training. Hardware needs measured deadlines, acquisition alignment, current response and gate/queue behavior.

The public `acf_force_control_simple.slx` continues to expose the verified deterministic controller and its established controls. Experiments use private models and exports so current users can reproduce the delivered baseline while the learned candidate is reviewed. **[FINAL SYSTEM RECOMMENDATION PLACEHOLDER.]** Promote a learned branch only after the final source/history, native replay, failure-stop, solver/save/reopen and diagram checks are complete. The final report should state which branch the prototype should start with and why its actual evidence supports that choice.
