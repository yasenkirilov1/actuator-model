### Learned control policy

#### Intro

We trained two small neural policies and tested them in the native physical model: a fast coil-current correction and a slow robot-recentering policy. The experiments use the lower-cost M08 acquisition candidate, the matched 8 kHz response assumption, a hot 24.4 N/A winding with 8.896 Ω resistance and 4 mH inductance, and 32 V usable bipolar output. The physical plant, sensor delays, noise, current dynamics, stops and robot queue remain in the simulation.

Both policies use **supervised learning from native trajectories**. The force network imitates an approximate sensor-lag correction; the robot network imitates the existing safe recentering rule. They are separately trained, then tested together with their weights fixed. This is an executed small-model experiment, rather than a claim of end-to-end reinforcement learning or an optimal learned controller.

#### Question

**Can a small learned model improve force margin or robot recentering enough to justify its additional complexity, using the same causal measurements and no surface preview?** A useful improvement must survive independent closed-loop recordings, including startup, physical limits, calibration mismatch and active robot commands. Fitting a training label accurately is only an intermediate check.

#### Results

##### Learned fast current correction

The force network has **10 inputs, 8 ReLU hidden units and 1 output: 97 trained weights and biases**. It updates at 10 kHz. Its ten inputs are reconstructed normal-force deviation, acceleration-feedforward current, observed S–A velocity, estimated surface slope, the four corresponding causal 100 µs increments, identified S mass and force target. Native Gain, Bias, Sum, Saturation and Unit Delay blocks perform inference. The output adds a correction bounded to **±0.05 A** before the existing current notch and current limits. Antiwindup sees the combined actuator command.

Eight separate 4 s native training trajectories provide **320,008 fresh controller frames**. We train on **36,528 frames**, retaining all startup and bump-window frames and subsampling quiet periods. Four separate trajectories provide 160,004 selection frames, with 18,264 balanced frames used for selecting weights. Entire trajectories are assigned to their split; adjacent samples from one trajectory are not randomly shared between training and test. NumPy Adam trains for 180 epochs with seed 20261005. The selected network has **4.784 mA RMS** error against its approximate teacher on the balanced selection frames.

Plant truth is permitted only to create training labels and score results. The label is the bounded sum of a proportional-feedback estimation correction and a total-moving-mass inertia correction:

`clip(Kp × (N_est − N_true) − (B_id + S_id)/Kf_id × (aS_true − aS_FF), ±0.05 A)`.

This label **does not account for the changed response of the compliant lower-link inverse**. It is an approximate imitation target, not an optimal actuator action. No true force, true acceleration, future surface, bump timing or future robot command enters network inference. A saved graph audit verifies that distinction.

Twelve native selection runs compare predeclared residual gains 0.25, 0.5 and 1. The minimax selection rule chooses **gain 1**, which changes the selection-set worst peak from **0.332238 N to 0.330610 N**. This small improvement does not establish a general benefit; its mean peak is worse. The gain and weights are then frozen before the final tests.

![Training and independent selection of the force network](figures/ml_force_training.svg)

*Learning reduces teacher-fit error, but this supervised objective is different from maximum closed-loop workpiece-force error. Selection trajectories are separate from training and from the final tests.*

The untouched test consists of **12 trajectory pairs**: eight nominal conditions, two calibrated 30 s recordings and two mixed-mechanics/calibration 4 s recordings per controller. The noise seeds are independent of training and selection; some mass/force/orientation endpoints deliberately overlap selection. Both baseline and learned control pass all 12 recordings, but **10 paired peaks worsen and 2 improve**. All eight nominal paired peaks worsen.

| Force-network test metric | Deterministic PI + feedforward | With learned correction |
| --- | ---: | ---: |
| Passed held-out trajectories | 12/12 | 12/12 |
| Worst full-recording force error | 0.447468 N | 0.468650 N |
| Reserve against ±0.5 N | 0.052532 N | 0.031350 N |
| Mean of the 12 trajectory peaks | 0.271991 N | 0.277879 N |
| Heavy 2 kg tool, 5 N, 0° | 0.315947 N | 0.348814 N |

The largest paired regression is **0.032867 N**. The learned controller's maximum recorded current is 2.443944 A, voltage is 30.995909 V and bounded residual is 0.042414 A. There is no voltage or ADC clipping, contact loss, travel stop, current-limit failure or spring-packing failure in these tests. Consequently, higher supply voltage at unchanged current-loop tuning would not address the observed learned-control regression. The largest absolute travel coordinate is 16.152 mm within the modeled **0–25 mm travel interval**, centered at 12.5 mm.

![Held-out force peaks with and without the learned current correction](figures/ml_force_holdout_peaks.svg)

*Peaks use every native force sample, including startup. N1–N8 are nominal trajectory pairs; S1, S3, S5 and S7 are the four selected calibration/mechanics pairs. This is a finite native simulation test, not physical prototype evidence.*

![Force and current during a bump in the worst learned test recording](figures/ml_force_worst_bump.svg)

*The plotted bump is an excerpt of the worst learned trajectory. Qualification and the 0.468650 N result use its complete native history, including later repeated bumps.*

The main force study executes **48 native recordings and scores 88,800,048 native force samples** across collection, gain selection and held-out testing. Two further solver-refinement recordings halve the physical step to 1.667 µs: the worst nominal learned peak changes by only **0.86 µN**. Two short software recordings prove zero algebraic loops and identical force, feature and action histories after save/reopen. Independent numerical inference reproduces all logged network actions within 1e−12 A. These checks verify the implemented experiment; they do not turn its small test set into the earlier 700- or 3,500-case qualification grid.

##### Learned robot commands

The robot policy has **one input, eight ReLU hidden units and one output**, with odd symmetry. Eight scalar parameters are learned and expanded into 25 stored affine coefficients. It learns the distance outside the recentering deadband from the **causal estimated distance to mid-stroke**, normalized by 2 mm. The existing delay-aware recentering gain then converts this distance into a velocity correction at **100 Hz**.

The hard constraints stay outside the network: the original slow force-error/trend gate, fast variation detector and stable dwell, the delay-bounded sign/amplitude envelope, and robot speed/acceleration limits. The deterministic teacher remains selectable. A learned policy can issue neither an opposite-sign correction nor a larger correction than that envelope permits. Closing the gate stops **new recentering corrections**; the programmed nominal path and commands already transmitted remain in effect.

Eight native training trajectories provide **4,808 robot-rate frames**, including **4,682 nonzero issued corrections**. Four separate selection trajectories provide 2,404 frames. Four initializations are fitted; seed 29 is selected without using final tests. The raw network itself matches the selection teacher's commands to about **2.17e−19 m/s**, before its safety envelope. This is a learned reproduction of a simple deadband law, rather than a new robot-control strategy.

![The learned robot policy and its logged teacher](figures/ml_robot_learned_policy.svg)

*The learned transfer reproduces the deterministic recentering deadband and slope over its collected range. The hard gate and delay-aware gain remain separate native blocks.*

There are **11 untouched trajectory pairs**: eight nominal delay/mass/force/orientation cases and three selected 8 s calibration/mechanics cases. The test includes active initial recentering demands and loaded delay/response mismatch of ±5 ms. All **22 native recordings pass**, scoring **43,200,022 force samples**. Both controllers have a worst peak of **0.417868 N**; the largest paired peak difference is below **1e−11 N**. There are zero gate-closed command violations, direction/gain-envelope violations or safety-clipped raw network frames. Final travel errors and nonzero-command counts also match. The native training/selection collection is another 12 recordings, separate from those 22 tests.

![Robot-only held-out peaks](figures/ml_robot_paired_results.svg)

*The near-identical results show successful imitation, with no demonstrated force or recentering advantage. The three stressed 8 s pairs are distinct from the earlier 30 s acquisition tests.*

![Active recentering with the force gate and queued robot requests](figures/ml_robot_active_trajectory.svg)

*This selected calibration test shows overlapping baseline/learned force and travel histories. New recentering corrections become zero when the gate closes. The total issued request also contains the programmed nominal path; the delayed total request persists because transmitted commands cannot be withdrawn by closing the gate.*

##### Both learned actions in the same closed loop

We next run both policies together, with their independently selected weights and force gain 1 fixed. **Four new predeclared identities**, using seeds 3101–3104, are each simulated for 8 s with a deterministic baseline and the combined learned controller. Initial travel offsets deliberately exercise recentering. Both networks are active: the learned system issues 3,009 nonzero robot-correction frames and 320,004 nonzero current-correction frames across the four recordings.

| New paired test | Deterministic peak | Both learned policies | Change in peak |
| --- | ---: | ---: | ---: |
| S = 0.1 kg, 5 N, 0°, µ = 0, −1.5 mm initial offset | 0.026882 N | 0.043450 N | +0.016568 N |
| S = 2 kg, 50 N, 90°, µ = 0.3, +1.5 mm initial offset | 0.348118 N | 0.355113 N | +0.006995 N |
| S = 1 kg, 25 N, 45°, actual delay 85 ms / response 55 ms, +1 mm offset | 0.175371 N | 0.185500 N | +0.010129 N |
| Heavy retained calibration corner with +1.5 mm initial offset | 0.452692 N | 0.449781 N | −0.002912 N |

All **8 native recordings pass ±0.5 N**, with **19,200,008 force samples** scored. There is no ADC clipping or gate/sign/amplitude violation. Three peaks worsen; one improves by 2.912 mN. The combined tests request at most **2.38713 A** and **28.72696 V**, with zero voltage clipping. Raising voltage at the same current-loop tuning therefore does not improve the modeled drive in these tests. In the heavy nominal case with zero programmed axial path, one gate closure leaves **23,999 native samples (about 80 ms)** of delayed recentering request after new corrections have become zero; this isolates the queued-command effect from the nominal path. The learned worst reserve is 0.050219 N, only **0.000219 N beyond the preferred 0.05 N simulation reserve** in this small cohort. That narrow result does not justify promoting the learned controller.

![The two fixed learned policies tested together](figures/ml_combined_paired_results.svg)

*Both the ±0.5 N requirement and the preferred 0.45 N peak-error screen are shown. The more demanding heavy calibration/initial-offset baseline reaches 0.452692 N; it passes the force requirement but has only 0.047308 N reserve. This new trajectory has a different initial recentering demand from the earlier 0.438821 N hot-coil test.*

![Combined-policy force, stroke and command histories](figures/ml_combined_worst_trajectory.svg)

*The displayed trajectory retains actual robot delay, queued requests, both learned actions and all physical guards. Display excerpts and downsampling are not used to calculate qualification peaks.*

##### Interpretation and reproducibility

The force network's small selection improvement does not survive the broader held-out comparisons. Its teacher objective omits the lower inverse's changed response, and its calibration-stress cases were not used for training. Those are limitations of this attempt; these tests do not identify one exclusive cause of the regressions. The robot network learns a rule the existing native blocks already express clearly. The combined screen confirms that activating both policies produces no consistent benefit.

Across the three principal held-out comparisons there are **54 executed native recordings**: 24 force-only, 22 robot-only and 8 combined. Every recording passes the ±0.5 N requirement and its physical/policy guards. Training, selection, solver checks, short save/reopen checks and rejected software launches are separately retained. The learned policies have **not** undergone a new full 700- or 3,500-condition qualification, a complete joint uncertainty sweep, embedded timing measurements or powered-sander testing.

Weights, exact recipes, frozen source hashes, paired scores and selected histories remain available. See the [force-learning recipe](/home/tommie/Projects/actuator-model/acf_simulink/experiments/ml_force_20261005_README.md), [robot recipe](/home/tommie/Projects/actuator-model/acf_simulink/experiments/ml_robot_20261005_README.md), [combined recipe](/home/tommie/Projects/actuator-model/acf_simulink/experiments/ml_combined_20261005_README.md), [force test results](/home/tommie/Projects/actuator-model/acf_simulink/validation/ml_force_20261005/summary.json), [robot test results](/home/tommie/Projects/actuator-model/acf_simulink/validation/ml_robot_20261005_figures_final/summary.json) and [combined test results](/home/tommie/Projects/actuator-model/acf_simulink/validation/ml_combined_20261005_figures/summary.json). Both builders add ordinary native inference blocks to caller-owned private models; the public physical model is unchanged.

#### Recommendation

**Use the deterministic measured-inertia Feedforward PI and causal lower-link inverse for force control, with deterministic gated, delay-aware robot recentering.** Keep the trained networks as evaluated alternatives. The force network loses held-out margin, the robot network reproduces an existing rule, and their combined finite screen offers no consistent improvement.

For the prototype, spend the available effort on calibrated force/acceleration response, assembled mechanical identification, hot drive headroom and powered-tool validation. The new 0.452692 N deterministic recentering case also shows that the preferred 0.05 N reserve is not established for every active-recentering initial condition. Retain the independent-force bench criterion and verify that operating envelope before claiming the requested physical performance.
