# Viewer validation

## Completed

- MISS_HIT 0.9.44: `acf_visualize.m`, `acf_start_visualizer.m`,
  `acf_viewer_selftest.m` and the updated `acf_plot.m` pass syntax/static checks.
- Node.js syntax check of the JavaScript shipped in the HTML template.
- **31 replay-logic checks passed**, running that exact script in a minimal
  in-memory DOM. See `replay_logic_results.json` and `replay_logic_test.cjs`.
- The mechanism SVG produced by the replay script was rendered and inspected.
- Four independent Python-reference recordings supply the demo. All their
  logged samples are included. No browser-side physics solver is substituted.

| Example recording | Samples | Peak absolute logged force error | Contact lost | Rigid impacts |
|---|---:|---:|---|---:|
| 1 mm bump, 60 mm wide, 100 mm/s | 16,001 | 0.245836 N | No | 0 |
| 1 mm sharp bump, 10 mm wide, 100 mm/s | 16,001 | 1.470077 N | No | 0 |
| 3 mm ramp, 200 mm wide, 100 mm/s | 21,001 | 0.109037 N | No | 0 |
| Stress: 3 mm bump, 5 mm wide, 400 mm/s | 13,001 | 69.933339 N* | Yes | 2 |

*This is the largest finite logged force error. The rigid impacts have impulses
and **no finite modeled force peak**. This example fails the requirement.

The stress case is illustrative and is not a validated operating envelope or
a recommendation for hardware. The defaults describe example parameters.

The logic checks cover correct signal mapping to R/A/B/S; force/stroke values;
play, pause, stepping and restart; true versus measured traces; whole-run versus
current-frame status; contact loss and one-sample impacts; event navigation;
drive/stroke flags; malformed input rejection; and all three actuator displays.
Synthetic UI fixtures are used only for otherwise-uncovered display states;
they are not included among the physical demo recordings.

## Not executed here

- MATLAB/Simulink is unavailable. The exporter, launcher and MATLAB self-test
  have not run in MATLAB. Syntax checks are not a substitute for execution.
- The cloud browser rejected local `file:` URLs under its security policy.
  Full browser rendering, the HTML file chooser, responsive/mobile layout and
  real browser timing are unverified. The DOM runner is a logic test only.
- `browser_test.cjs` is a provided Playwright test, not a completed test report.

## Reproduce

From the model folder in MATLAB, run `acf_viewer_selftest`, then
`acf_start_visualizer`. The latter is the full Simulink-to-browser check.

From the unpacked complete package's parent directory:

```sh
python3 acf_simulink/viewer/validation/build_demo.py
node acf_simulink/viewer/validation/replay_logic_test.cjs
```

The Python reference requires NumPy. To run the additional browser test locally,
install Playwright and its Chromium browser through the official Playwright
instructions, then run `browser_test.cjs`. The test writes screenshot files in
the sibling `acf_visualizer` folder; create that folder if it does not exist.

The replay uses the complete exported sample record for status and event
checks. Plot drawing retains the minima and maxima in each horizontal bin.
The plotted error is **actual minus target**, opposite in sign to the PI input
error, with identical absolute tolerance.
