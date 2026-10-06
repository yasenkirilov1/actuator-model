# Measured tool acceleration: hardware assumptions

This is a sourced design note for the candidate experiments, not an executed
performance qualification. The accelerometer is rigidly mounted on S, with
its axes calibrated into the load-cell frame. S is the complete downstream
mass, including attachments that move with the tool.

The load-cell sign convention is positive workpiece reaction. For specific
acceleration **f = a − g**, Newton's law gives
**R = Lraw + mS f**. With consistent static gravity tare on both channels,
**R = Ltared + mS adynamic**. Project this reconstructed force vector onto
the causal estimated surface normal. This measurement does not require the
coil's force constant.

## Representative acquisition design

| Item | Sourced characteristic | Candidate treatment |
| --- | --- | --- |
| ADXL354C, analog three-axis | Selectable ±2/±8 g; 22.5 µg/√Hz in ±2 g; ±8 g noise estimated 50% higher | Start in ±8 g with density 33.75 µg/√Hz, finite range and calibration residuals |
| Internal filtering | Approximately 1.5 kHz sinc-shaped antialias response; overall −3 dB bandwidth 1.9 kHz | Use an explicitly identified response envelope; test its phase rather than assigning zero delay |
| Output filtering | 32 kΩ series output resistance | External capacitor and ADC loading add to the measured transfer function |
| Gain and axes | Factory sensitivity 92–108 mV/g in ±8 g; cross-axis sensitivity 1% | Calibrate scale and the full axis matrix; gravity tare alone is insufficient |
| AD7606C-18 | Eight simultaneous 18-bit channels; sub-microsecond nominal conversion without oversampling | Common acquisition trigger; measure complete latency and budget computation/transport separately |

Sources: [ADXL354/355 Rev. D](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf),
[AD7606C-18 Rev. B](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf).

The proposed first trial uses a matched first-order response of 200 µs plus
100 µs configured acquisition/control delay on acceleration and force.
This is a **transfer envelope to identify and stress**, not the manufacturer's
exact filter model or a guaranteed analog latency. Stress common response
at 200–400 µs and residual channel mismatch at ±50/100 µs. Retain the actual
sensor transfer and identified controller transfer separately.

For an output-referred white noise density `na` and first-order time constant
`tau`, equivalent noise bandwidth is `1/(4*tau)` and filtered RMS is
`na*sqrt(1/(4*tau))`. The proposed ±8 g density is 0.000331 m/s²/√Hz,
giving **0.01170 m/s² RMS** at 200 µs. At S=2 kg that contributes
**0.02340 N RMS** to reconstructed force, before other noise sources.
Discrete noise generators must reproduce this output RMS or apply the
proper spectral density before filtering, rather than silently filtering
the specified RMS a second time.

At 0–5 V ADC range, the 18-bit quantum is 19.07 µV. With 100 mV/g
acceleration sensitivity, this corresponds to **0.001870 m/s²** per code.
Its ideal quantization RMS is 0.000540 m/s². ADC accuracy, analog-reference
ratios, buffering, drift and channel calibration remain separate effects.

The sharp bump's peak jerk is approximately 384 m/s³. At S=2 kg,
50 µs differential cell/accelerometer delay can introduce roughly 0.0384 N
inertia error; 100 µs gives 0.0768 N. These are first-order error-budget
calculations, not simulated pass results. Match complete phase responses,
not only timestamped sample rates.

## Alternative and prototype checks

The analog single-axis
[ADXL1002](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl1001-1002.pdf)
offers ±50 g and 25 µg/√Hz noise density with a much higher mechanical
bandwidth. It needs external antialias filtering and orthogonal units for
a force vector. Its temperature-dependent scale/offset need calibration.
Its 11 kHz response specification is not an 11 kHz first-order low-pass.

The digital ADXL355's 4 kHz output has 0.63 ms **digital-filter** delay,
before other latency. This is a useful slower comparison, with its actual
filter delay included; output rate cannot substitute for a delay model.

Before choosing a part or final range, measure the operating sander's
vibration with the tool attached. Avoid saturation and test vibration
rectification, temperature drift, mounting resonances and the accelerometer's
distance from S's centre of mass. Rotation requires lever-arm compensation;
measuring acceleration on a flexible PCB does not establish rigid-tool
acceleration. The current translational bump model does not establish these
properties of a running sanding head.
