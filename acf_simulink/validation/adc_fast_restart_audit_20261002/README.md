# Native ADC Fast Restart audit — 2026-10-02

This is a software execution audit of the frozen private acquisition fork, not additional hardware qualification. Nine native recordings were executed in an isolated MATLAB batch process using private preferences, model, helpers, cache/code and owned RAM-backed SDI storage. Two earlier recipe-construction attempts were rejected before any native simulation and are preserved in `software_rejected_attempt_0001/` and `software_rejected_attempt_0002/`.

All three requested warm/cold comparisons match exactly, with no tolerance:

| Change made with Fast Restart enabled | OFF cold reference | Native physical history | ADC conversion history |
|---|---|---:|---:|
| Heavy seed 7 → seed 29 | Heavy seed 29, 0.25 s | 75,001 × 19 values, exact | 7,501 × 8 values, exact |
| Heavy → light, seed 7 | S=0.1 kg, 5 N, 0°, 4 s | 1,200,001 × 19 values, exact | 120,001 × 8 values, exact |
| AA cutoff 500 → 5,100 Hz, seed 7 | Heavy 5,100 Hz, 4 s | 1,200,001 × 19 values, exact | 120,001 × 8 values, exact |

Heavy means S=2 kg, B=0.6 kg, 50 N and 90°. All runs use µ=0.3, SINC3 output at 30 ksample/s, the fixed 300 kHz internal noise/filter clock, a 300 kHz native solver and the unchanged 10 kHz controller. Actual and identified AA cutoffs are changed together. All physical histories include startup, and each 4 s history includes the sharp bump. A separate ON first-compile versus OFF cold heavy 4 s repeat also matches exactly.

The seed contrast is real: seed 7 versus cold seed 29 has a maximum true-normal-force history difference of 0.110496179 N. The cutoff contrast is also real: the heavy 500 Hz history peaks at 0.671830320 N error, while its 5,100 Hz counterpart peaks at 0.348237978 N; their maximum force-history difference is 0.422721373 N. These contrasts show that the comparisons are not simply identical because all inputs were ignored.

All nine native noise blocks expose `Seed` as `run-to-run-tunable`. The warm-run transcripts still warn that variable `p` changed in a nontunable block context. In the tested comparisons, clocks remain identical and the warm histories demonstrate that seeds, physical parameters, startup states, continuous AA coefficients and corresponding identified response changes are applied correctly. The last warning ID is `Simulink:Engine:NonTunableVarChangedMaxWarnings`. The broad warning alone does not establish that a changed seed or physical parameter was ignored.

The audit does not qualify arbitrary changes to clocks, robot delay-buffer lengths, filter state dimensions, ADC response order/length, or untested timing/calibration combinations. Fast Restart OFF is a guaranteed reference. Safe reuse must recompile whenever any actual block sample time, delay-buffer length, solver clock or state/filter dimension changes; including seed conservatively is permissible despite its demonstrated tunability. `clock_delay_parameter_inventory.json` records source expressions for review, not proof of each parameter's tunability.

`results.json` records all run and comparison metrics. Every full compact and ADC trace is retained in `*_native.mat`; warning transcripts are separate `*_transcript.txt` files. The 19 compact columns preserve native true normal force, projected vector force and axial/lower-link force-balance checks. All balance residuals remain below 1e-8 N. Source and all 18 helper hashes were checked before and after execution, and SDI storage/recording preferences were restored. `audit_seal.json` binds the numerical evidence and recipes. Private preferences and generated cache/code are excluded from the evidence seal and should not be distributed.
