# Lower-cost accelerometer component review — 2 October 2026

The goal is to reduce sensing cost while preserving the active sanding flange's **±0.5 N surface-normal force requirement**, including the sharp bump and tools up to 2 kg. The accelerometer sits on S, downstream of the B–S load cell. Its measured acceleration supplies the inertia correction used to recover the workpiece's contact force.

**The strongest cost candidates are ADXL354C for a simpler analog conversion and ICM-42688-P or IIS3DWB for a larger digital cost reduction.** Their extra noise, internal response and timestamps must enter the native experiments. This review supplies component contracts and reproducible calculations; it performs no native simulations and does not qualify a replacement.

## Comparable indicative prices

These are sensor IC prices in euros, excluding VAT and shipping, retrieved on **2 October 2026**. Regional distributor pages are sometimes indexed caches, and some simultaneously show stock and availability warnings. The observations are useful cost comparisons, not order promises. One three-axis part replaces two planar or three spatial single-axis sensors; this does not imply a corresponding whole-system saving.

| Exact part | Interface and package | Unit, quantity 1 / 10 | Sensor-only planar / spatial cost at quantity-1 price | Observed stock |
|---|---|---:|---:|---:|
| [ADXL1002BCPZ-RL7](https://www.digikey.nl/nl/products/detail/analog-devices-inc/ADXL1002BCPZ-RL7/13180293) | Analog, single-axis LFCSP, cut tape | €64.88 / €58.903 | €129.76 / €194.64 | 0 |
| [ADXL354CEZ](https://www.mouser.de/de/ProductDetail/Analog-Devices/ADXL354CEZ?qs=sGAEpiMZZMu5vlrqIFXt5fRj789CQj8hZTjIPfetPjCmoM2z2noYCA%3D%3D) | Analog, three-axis LCC, tray | €60.44 / €53.45 | €60.44 / €60.44 | 23 |
| [IIS3DWBTR](https://www.mouser.fr/fr/ProductDetail/STMicroelectronics/IIS3DWBTR?qs=%252B6g0mu59x7IIEe51lWewNQ%3D%3D) | Digital, three-axis LGA, cut tape | €19.90 / €17.38 | €19.90 / €19.90 | 19,760 |
| [ICM-42688-P](https://eu.mouser.com/en/ProductDetail/TDK-InvenSense/ICM-42688-P?qs=OlC7AqGiEDk8QrNAV728zw%3D%3D) | Digital, three-axis LGA, cut tape | €5.33 / €4.17 | €5.33 / €5.33 | 0 |
| [ADXL355BEZ](https://eu.mouser.com/en/ProductDetail/Analog-Devices/ADXL355BEZ?qs=vcbl%252BK4rRld73tJrFMMu%2Fg%3D%3D) | Digital, three-axis LCC, tray | €70.08 / €62.22 | €70.08 / €70.08 | 4,707; restricted/last-stock warning |

Earlier ADXL354 EU caches showed €55.38, and earlier IIS3DWB Netherlands caches €17.55; the table uses the later regional observations. Relative to two reference ADXL1002s, sensor-only reductions are about **53% analog, 85% IIS3DWB, or 96% ICM**. Digital acceleration needs only the six load-cell bridge channels on the external ADC. Three analog acceleration axes need nine external channels in a general spatial system; the planar native model currently uses eight.

Analog replacements still need buffered, centered gain and an appropriate reference/filter. Digital replacements add SPI/DMA, clock/timestamp handling and calibration effort. The six bridge instrumentation amplifiers remain common to both arrangements. Approximate front-end context is retained in the [ADC review](adc_cost_component_review_20261002.md); it is not a purchasing list.

## Component contracts

**Reference ADXL1002:** one analog axis, ±50 g, 40 mV/g at 5 V, typical 25 µg/√Hz noise. Its 21 kHz mechanical resonance produces rising gain: the 4.7 kHz 5% and 11 kHz 3 dB points are not first-order low-pass corners. An external engineered antialias filter is separate. Sensitivity and midpoint track supply. Temperature sensitivity variation is ±5%; the 5 g zero-offset max–min excursion across temperature is not a linear drift specification. Multi-axis scale, orientation and warm tare calibration remain necessary. [ADI datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl1001-1002.pdf).

**ADXL354C:** three analog axes, ±2/8 g. At ±8 g, sensitivity is 100 mV/g and density is approximately 33.75 µg/√Hz, estimated by design. The sinc-shaped internal analog filter is approximately 1.5 kHz; the full response reaches −3 dB at 1.9 kHz with a 2.4 kHz mechanical mode. The output includes 32 kΩ; added capacitance gives a known extra RC. Complete internal phase is unpublished. Initial scale tolerance is ±8%; temperature scale is ±0.01%/°C and bias limits ±0.15 mg/°C. Its range needs powered-sander clipping checks. **ADXL355** shares the analog chain but adds digital filtering: maximum 4 kSPS/1 kHz, 0.63 ms digital delay, or 0.88 ms with synchronization interpolation. These delays exclude preceding analog response. Its low noise and 20-bit output do not remove that latency. [ADI Rev D datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

**IIS3DWB:** three digital axes, ±2/4/8/16 g, 16-bit, fixed 26.667 kSPS. With all axes enabled, typical noise is 75 µg/√Hz on X/Y and 110 on Z. Characterized upper figures are 110/190, not production guarantees. Bypassing LPF2 leaves the normal chain and a characterized 5–6.3 kHz bandwidth. Neither the datasheet nor application note discloses its complete transfer or full delay bound. SPI supports 10 MHz: a direct six-byte read plus command takes 5.6 µs on the wire. Internal timestamps have nominal 12.5 µs resolution, corrected by frequency trim. There is no external sampling-clock pin; DRDY capture and clock mapping must align force and acceleration. Typical bias drift is ±1 mg/°C. Scale is ±2% initially, with characterized ±2% temperature variation over the range. [ST datasheet](https://www.st.com/resource/en/datasheet/iis3dwb.pdf), [ST AN5444](https://www.st.com/resource/en/application_note/an5444-iis3dwb-ultrawide-bandwidth-lownoise-3axis-digital-vibration-sensor-stmicroelectronics.pdf).

**ICM-42688-P:** three digital acceleration axes, ±2/4/8/16 g; typical X/Y/Z noise 65/65/70 µg/√Hz. Nominal maximum output is 32 kSPS. The common-decimation/UI table gives 8.4 kHz cutoff, 8,831.7 Hz noise bandwidth and 100 µs DC group delay at that rate. Its separate second-order AAF supports 42–3,979 Hz or bypass. The whole chain is not the selected UI filter order; full coefficients/phase are unpublished. External CLKIN supports 31–50 kHz, and documented rate scaling allows a 40 kHz clock/output, four frames per 10 kHz control tick. Inverse-delay scaling is an inference requiring measurement. SPI supports 24 MHz; direct read is 2.33 µs and a high-resolution packet 7 µs. A 20-bit packet carries only 18 actual acceleration bits at ±16 g. Initial bias/scale figures are ±20 mg/±0.5%; characterized drift is ±0.15 mg/°C and ±0.005%/°C. [Manufacturer product page](https://www.invensense.tdk.com/en-us/products/6-axis/icm-42688-p), [current manufacturer-linked Rev 1.9 datasheet](https://d17t6iyxenbwp1.cloudfront.net/s3fs-public/2026-06/DS-000347%20ICM-42688-P%20v1.9.pdf?VersionId=XL060vOv88u6zsKx9dkBVJEyV452m0eq).

ISM330DHCX was also checked: around €7.61, up to 6.667 kSPS and 60 µg/√Hz typical. Its documented fastest OIS response is 631 Hz, with −4.2° phase at 20 Hz, equivalent to 0.583 ms there. The high-rate UI transfer is incompletely specified; the ODR/2 cutoff rule is limited to rates through 833 Hz. It is therefore not the preferred fast replacement. [ST datasheet](https://www.st.com/resource/en/datasheet/ism330dhcx.pdf), [indicative price](https://eu.mouser.com/fr/ProductDetail/STMicroelectronics/ISM330DHCXTR?qs=%252B6g0mu59x7Jv2Xps2dM6Xw%3D%3D).

## What the numerical trade means

For an explicitly assumed flat-white density, output RMS is `density × g × sqrt(ENBW)`. **Equivalent noise bandwidth is neither cutoff frequency nor sample rate.** It accounts for the full filter magnitude response. Higher sample rate alone does not eliminate physical sensor noise; filtering trades noise against phase and usable disturbance information.

| Part and range/axis | Density, µg/√Hz | RMS at assumed 1,000 Hz ENBW, m/s² | ENBW allowing 0.014 m/s² before extra front-end noise |
|---|---:|---:|---:|
| ADXL1002 | 25 | 0.00775 | 3,261 Hz |
| ADXL354, ±8 g | 33.75 | 0.01047 | 1,789 Hz |
| ICM, X/Y | 65 | 0.02016 | 482 Hz |
| ICM, Z | 70 | 0.02171 | 416 Hz |
| IIS3DWB, X/Y | 75 | 0.02326 | 362 Hz |
| IIS3DWB, Z | 110 | 0.03411 | 168 Hz |

These are calculations, not sensor measurements. The ICM table's 8,831.7 Hz noise bandwidth and a white extrapolation of its typical density would imply **0.05990/0.06451 m/s² X/Y/Z RMS** before an enabled AAF or later filtering. High-rate density, colored noise and actual full response need measurement. The reference model's conditioned 0.014 m/s² is a stated acquisition assumption, not something guaranteed by an IC's part number. The native study must retain each cheaper sensor's realistic increased noise unless its actual conditioning establishes a reduction.

For **ADXL354 ±8 g**, center its approximately 0.9 V output and apply gain 5. Nominal ±8 g becomes ±4 V at an ADS8588S set to ±5 V. Derived ADC steps are **0.00299275 m/s²**, with **0.00160696 m/s² typical / 0.00197700 minimum-SNR-equivalent ADC RMS**. Thus 16 bits are sufficient for fine acceleration steps in this conditioning example. The ADC-only error is smaller than this sensor's density contribution. At an assumed 1,000 Hz ENBW, combined sensor-plus-ADC RMS is about **0.01059 m/s²** before amplifier/reference contributions. Into ADS131M08, gain 1 and ±1.2 V ADC range give approximately 0.00001403 m/s² steps and 0.00208980 m/s² typical ADC RMS at OSR128. Its real Sinc³ filter also belongs in the model. [TI ADS8588S datasheet](https://www.ti.com/lit/ds/symlink/ads8588s.pdf), [ADC calculations](adc_cost_component_review_20261002.json).

The ICM ±16 g steps are 0.00478840 m/s² in 16-bit mode or **0.00119710 m/s² with 18 actual FIFO bits**. Digital quoted noise already includes its internal conversion chain; treating the full density noise and an independent quantization RMS as separate additions can double-count. Quantization resolution is also not bias, scale accuracy, temperature stability or vibration immunity.

At S = 2 kg, an acceleration residual of 0.005 m/s² becomes 0.01 N of inertia error. Warm tare removes static bias, not scale errors during motion or vibration rectification. A 0.15 mg/°C sensor bias coefficient corresponds to 0.001471 m/s²/°C; 1 mg/°C corresponds to 0.009807 m/s²/°C. Temperature tracking, calibrated axis/scale and mounting are therefore material even for inexpensive devices. A powered sander may impose much larger vibration than the bump itself; none of these chip specifications proves sufficient range or lack of rectification on that assembly.

Timing matters separately. The analytic C² compact bump gives a one-sided peak imposed-profile jerk of 384 m/s³. A local first-order `m × jerk × timestamp_error` illustration at 2 kg is 0.01536 N for 20 µs, 0.0768 N for 100 µs and 0.48384 N for 630 µs. These are **not native control results**: actual S jerk, matched measurement frames, observer behavior and the causal controller decide the outcome. They explain why a precise slow accelerometer can be worse than a noisier fast one and why delaying force/normal estimates to the acceleration frame must be tested explicitly.

## Native experiment and prototype contracts

1. Screen ADXL354, ICM and IIS3DWB in isolated native model forks with their range, quantization, noise, clock and acquisition age. Use exact disclosed filter portions; label undisclosed full response as a screen assumption. Do not relabel the reference 5.1 kHz response with a cheaper part name.
2. Retain the true force histories, all startup samples, failed cases, matched timestamp reconstruction and causal observer inputs. Repeat the heavy-tool sharp bump, high-force/orientation and joint calibration/mechanics corners. Cost selection should use remaining force margin as well as nominal passes.
3. Measure the assembled acceleration-to-interface complex response and axis matching on a fixture, then compare it to the screened envelope. Capture DRDY, ADC conversion edges and MCU timestamps. Digital output rate or wire-time is not physical group delay.
4. Run the powered tool on the intended mounting: measure PSD, peaks/clipping, bias rectification and temperature drift. Refit scale/axis/bias, and rerun the native traces with measured transfer/noise before claiming the same prototype margin.

**Recommendation for the research sequence:** first test **ADXL354C + centered gain 5** as the simplest substantial analog saving, and **ICM-42688-P at an externally clocked 40 kSPS** as the largest digital saving. Keep **IIS3DWB** as the industrial, presently stocked digital alternative. Select a cheaper candidate only after its native experiment preserves margin and its assembled transfer is measured; retain the ADXL1002 reference if those conditions do not hold.

The [source ledger and derived values](accelerometer_cost_component_review_20261002.json), [55-row noise CSV](accelerometer_cost_component_review_20261002.csv), [calculation recipe](accelerometer_cost_component_review_20261002.py) and [checks/hash record](accelerometer_cost_component_review_20261002.checks.json) reproduce the component budgets. Run the recipe from any working directory; it uses only the Python standard library and performs no simulation.
