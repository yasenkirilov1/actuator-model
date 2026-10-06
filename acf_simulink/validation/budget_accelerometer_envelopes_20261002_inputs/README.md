# Cheaper accelerometer feasibility envelopes

`configs.mat` contains 36 **prepared, unexecuted** native-model cases. The
root ADC qualification runner executes these inputs and scores every native
surface-normal force sample, including startup. `manifest.json` records the
case identities, input hash, unchanged acquisition source and preparation
guards. Preparation is not force qualification.

The unchanged acquisition source is the private native ADC model with SHA256
`b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5`.
All cases contain its required native calibration-default fields and use the
same physics, PI gains, guides and robot stability gates. The exact Sinc3
clock is 3.84 MHz with OSR128: **30 kSPS**, not 32 kSPS. Its fine integration
and sensor-noise clock is 300 kHz. Actual and identified complete responses
and common extra latency are explicitly matched in every case.

| Cases | Noise/range anchor | Complete matched response | Common extra latency | Drive |
|---|---|---|---|---|
| 12 primary | ADXL354C: 33.75 µg/√Hz, ±8 g | Assumed two-pole 1.9 kHz | 0/50/100 µs | Original 100 µs current response, 24 V |
| 8 driver tradeoff | Same ADXL354C anchor | Same assumed 1.9 kHz | 0/50 µs | Separately labeled 50 µs current response, 36 V |
| 8 counterfactual bandwidth | Same ADXL354C noise/range only | 3 or 5.1 kHz counterfactual | 0 µs | Original drive |
| 8 digital acceptance envelopes | 65/70 µg/√Hz, ±16 g | Assumed matched 5.1 kHz | 80/120 µs | Original drive |

ADXL354 cases use two endpoints: S=2 kg/F=50 N/90° and
S=0.1 kg/F=5 N/0°, friction 0.3 and seeds 7/29. The digital envelopes use the
heavy endpoint and both seeds. Those are finite screens; no interpolation,
full calibration box or physical sanding qualification follows from a pass.

The ADXL354 noise anchor maps to **0.0186557069 m/s²** at the immutable
3177.12656 Hz reference noise bandwidth. This only sets the underlying
one-sided PSD. Actual output RMS changes with the selected analog response,
Sinc3 filter and EMA. Its 100 mV/g sensitivity gives approximately
0.0020898 m/s² converter output noise and 0.00001403 m/s² code spacing for
the assumed ADS131M08 gain/offset. Explicit quantization is separate from
the additive ADC noise. Every part still needs assembled gain, zero, frame
and temperature calibration; those nominal screens do not exercise such errors.

The two-pole 1.9 kHz response is an **engineered complete-response assumption**
anchored to the ADXL354 bandwidth. It is not an exact published or measured
part transfer. Extra latency probes expose that uncertainty. The faster
3/5.1 kHz cases cannot establish ADXL354 feasibility. The digital cases do not
represent an ICM clock, register filter, internal conversion chain or external
ADC connection; they ask what a cheaper digital sensor would need to deliver
after timestamp/response matching. A positive result is an acceptance target,
not a purchasing recommendation.

`optional_adxl355_configs.mat` contains four separately prepared optimistic
latency comparators: the two endpoints and seeds 7/29, 33.75 µg/√Hz/±8 g,
1.9 kHz matched response and 630 µs common extra latency. It retains the
common 30 kHz frame and does **not** implement the real ADXL355 4 kHz output
or internal FIR. It is optional, not included in the 36-case main count.

The faster-current tradeoff changes both current response and supply voltage
on the **scientific reference coil**. Its results must remain separate from
the original drive and from hot real-winding/driver thermal qualification.

Load `configs` verbatim through the root runner's `customCasesFile` support;
do not reset it through nominal case constructors or replace its hardware
fields. The complete prepared parameters carry `p.budgetAccelerometer`
metadata and explicit fresh `p.adcStudy` response/timing fields. Preserve this
input file and version a new file when parameters change.
