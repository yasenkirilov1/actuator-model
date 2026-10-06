# Lower-cost ADC component review — 2 October 2026

The goal remains **±0.5 N actual surface-normal pressing force throughout the sharp bump**, with S from 0.1 to 2 kg. This review identifies credible cheaper measurement hardware and converts its electrical specifications into experimental inputs. It executes **no force simulations**: a component becomes a recommendation only after the accompanying native bandwidth/rate/noise/timing experiments pass with a useful margin.

The strongest candidates are **ADS8588S**, which retains simultaneous SAR acquisition at the existing 100 ksample/s rate for about €19 per chip, and **ADS131M08**, about €9 per chip, which trades SAR acquisition for a genuine Sinc³ decimator at up to 32 ksample/s. Reducing sample rate is not necessary merely to buy a cheaper ADC. The SAR option already provides a large chip-cost reduction.

## Comparable price observations

These are indicative EU distributor observations retrieved on 2 October 2026. Prices exclude VAT, shipping and a MouseReel service fee; tray and ordinary cut tape both permit single-unit purchases. Web retrieval sometimes returns cached regional pages. Direct HTTP verification returned distributor access-challenge pages, and baseline availability differs between cached regional pages. Therefore these are documented price observations, not a verified cart or stock promise. Mouser's [published terms](https://www.mouser.com/en/saleterms/) specify that prices exclude VAT and other charges.

| Part / package | Acquisition architecture | Maximum rate | EUR each, quantity 1 | EUR each, quantity 10 | Observed availability and price source |
|---|---|---:|---:|---:|---|
| AD7606C-18BSTZ, LQFP-64 tray | 8 simultaneous, 18-bit SAR | 1 MSPS/channel | 63.86 | 55.11 | Regional availability conflict; [Mouser EU](https://eu.mouser.com/en/ProductDetail/Analog-Devices/AD7606C-18BSTZ?qs=T94vaHKWudSLf6RPLoNi0g%3D%3D) |
| ADS8588SIPMR, LQFP-64 cut tape | 8 simultaneous, 16-bit SAR | 200 kSPS/channel | 19.13 | 15.16 | 992 in opened page; [Mouser EU](https://eu.mouser.com/en/ProductDetail/Texas-Instruments/ADS8588SIPMR?qs=34RfhUjJmKc7AQrQTm6kKA%3D%3D) |
| ADS131M08IPBSR, TQFP-32 cut tape | 8 simultaneous, 24-bit delta-sigma | 32 kSPS/channel | 9.25 | 7.23 | 4,674; [Mouser Ireland](https://www.mouser.ie/en/ProductDetail/Texas-Instruments/ADS131M08IPBSR?qs=sGAEpiMZZMtTlwiyc7n6Sr3rdcuL3armPSnBReZ0DVg%3D) |
| AD7606BBSTZ, LQFP-64 tray | 8 simultaneous, 16-bit SAR | 800 kSPS/channel | 46.04 | 36.15 | 850; [Mouser EU](https://eu.mouser.com/de/ProductDetail/Analog-Devices/AD7606BBSTZ?qs=%252B6g0mu59x7K4V6DmnEpfIQ%3D%3D) |
| AD7606BSTZ, LQFP-64 tray | 8 simultaneous, 16-bit SAR | 200 kSPS/channel | 44.73 | 36.56 | 1,710; distributor says no further stocking after current inventory; [Mouser EU](https://eu.mouser.com/en/ProductDetail/Analog-Devices/AD7606BSTZ?qs=%2FtpEQrCGXCxOGkaD7urMag%3D%3D) |
| AD7606C-16BSTZ, LQFP-64 tray | 8 simultaneous, 16-bit SAR | 1 MSPS/channel | 53.47 | 42.61 | 417; [Mouser Spain](https://www.mouser.es/ProductDetail/Analog-Devices/AD7606C-16BSTZ?qs=pUKx8fyJudAIi5elYfPn7w%3D%3D) |
| ADS8688AIDBTR, TSSOP-38 cut tape | 8 multiplexed, 16-bit SAR | 500 kSPS **aggregate** | 19.21 | 15.36 | 1,957; [Mouser Germany](https://www.mouser.de/en/ProductDetail/Texas-Instruments/ADS8688AIDBTR?qs=sGAEpiMZZMtgJDuTUz7Xu0ezfY%2FuekT414JQLTV%252BXonNpT7ohYobeQ%3D%3D) |

An independent baseline observation is **€61.35 excluding VAT, zero stock** at [DigiKey Netherlands](https://www.digikey.nl/nl/products/detail/analog-devices-inc/AD7606C-18BSTZ/13547778). A Netherlands M08 page gives €9.28 / €7.24 rather than €9.25 / €7.23. These small differences do not change the architecture comparison. Against the €63.86 baseline observation, the ADS8588S chip saving is approximately **70%** and the M08 saving approximately **86%**. Those percentages do not describe the entire measurement assembly.

## The channel count is eight, even for the planar controller

The raw multi-axis load cell contributes **six bridge channels**. Two orthogonal analog ADXL1002 units provide the two acceleration axes in the current scanning plane. That is eight analog channels, plus the separately timestamped position encoder. A three-axis acceleration arrangement needs nine channels and a second synchronized converter or another separately qualified acquisition arrangement. A two-channel ADC cannot replace the complete acquisition chain simply because the controller regulates one force scalar.

The user permits a proper instrumentation amplifier with a chosen analog cutoff. Accordingly, front-end gain, offset, input common mode and external filtering are design variables. Neither SAR candidate accepts a raw strain bridge as a complete precision front end, and using the M08's high PGA gain is optional.

## SAR timing and bandwidth that actually enter the experiment

The ADS8588S contains a **third-order Butterworth analog filter**, specified at **16 kHz in ±5 V mode**, with approximately **19 µs group delay**. Its no-oversampling conversion takes at most 3.9 µs; 128 data bits at a proposed 16 MHz SPI clock need 8 µs. A 20 µs conversion/readout/computation budget therefore has approximately 8.1 µs left for the host. The filter delay is separate from that pipeline budget. Oversampling changes both bandwidth and latency and must not be silently enabled. [TI datasheet, sections 7.5, 7.10 and 8.3.5](https://www.ti.com/lit/ds/symlink/ads8588s.pdf).

The analog transfer used for a nominal hardware model is:

```text
H_ADC(s) = ω³ / (s³ + 2ωs² + 2ω²s + ω³),   ω = 2π·16000
```

Its calculated DC group delay is 19.895 µs, consistent with the rounded manufacturer value. The selected external instrumentation filter precedes this filter and sample/hold. Inject ADC intrinsic noise after the external filter, not before it. All channels should use the same ADC range so that a range-dependent filter does not introduce force/acceleration mismatch.

At 50 ksample/s the full 11.9 µs conversion/read fits within a 20 µs sample period. At 100 ksample/s a single-output read can overlap the next conversion: 11.9 µs is still before the next output-register update at approximately 13.9 µs. The manufacturer's reading-during-conversion timing must be observed. This is a proposed bus schedule, not a measured MCU timing result. Two serial output lines or parallel readout provide other implementations.

AD7606B provides only **±10, ±5 and ±2.5 V** ranges. Its software mode does **not** add 0–5 V unipolar acquisition. At ±5 V its input response is 13.5 kHz with 12 µs phase delay; conversion is at most 0.85 µs. The plain AD7606 has a 15 kHz / 15 µs ±5 V response. The C-16 supports genuine 0–5 V and a 25 kHz / 6.7 µs low-bandwidth response, but saves substantially less money. [AD7606B datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606b.pdf), [plain AD7606 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606_7606-6_7606-4.pdf), [C-16 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-16.pdf).

## Resolution, actual noise and acceleration conditioning

An ideal 18-bit quantum is insufficient to budget the existing ADC. AD7606C-18 in 0–5 V low-bandwidth mode specifies **86.5 dB typical / 84.5 dB minimum SNR at 1 MSPS**. This is far from ideal 18-bit SNR. Its typical input noise inferred from a full-scale sine is approximately **83.64 µV RMS**; its ideal quantum is only 19.07 µV. [ADI datasheet, table 2](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf).

The cheaper ADS8588S ±5 V specifications, using the stated −0.5 dBFS test amplitude, imply **81.93 µV typical / 100.80 µV at the minimum SNR bound**. This nearly matches the expensive converter's voltage noise despite the lower bit count. Its quantization is coarser, so the chosen acceleration conditioning matters. The raw ADXL1002 sensitivity is 40 mV/g at a 5 V supply. [ADXL1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl1001-1002.pdf).

The following rows are reproduced by the accompanying Python recipe. ADC noise is the integrated electrical ADC budget before any *additional* host filtering; it is not the complete sensor noise.

| ADC and acceleration conditioning | ADC acceleration range | Acceleration quantum | ADC acceleration noise, typical | Minimum-SNR equivalent noise |
|---|---:|---:|---:|---:|
| C-18, 0–5 V, direct 40 mV/g | 125 g total span | 0.004676 m/s² | 0.020506 m/s² | 0.025816 m/s² |
| ADS8588S, ±5 V, gain 1 | 250 g total ADC span | 0.037409 m/s² | 0.020087 m/s² | 0.024712 m/s² |
| ADS8588S, ±5 V, centered gain 2 | ±62.5 g | **0.018705 m/s²** | **0.010044 m/s²** | **0.012356 m/s²** |
| ADS8588S, ±5 V, centered gain 4 | ±31.25 g | **0.009352 m/s²** | **0.005022 m/s²** | **0.006178 m/s²** |
| M08, ±1.2 V differential, PGA1, conditioning gain 0.5, OSR128 | ±60 g | **0.0000701 m/s²** | **0.010449 m/s²** | Typical table only; no minimum-noise guarantee |

For centered gain 2, `V_ADC = 2·(V_ADXL−2.5 V)` maps the sensor's ±50 g to ±4 V, preserving its range. It requires a conditioning stage able to produce bipolar output; include the associated supply and reference. Gain 4 lowers ADC noise and quantization but clips above approximately ±31.25 g. The smooth bump does not establish vibration headroom for a powered sander. A direct single-supply connection is simpler but uses only part of a bipolar ADC's codes.

For the M08, a proposed single-supply level shift is `V+ = 1.65 + 0.5·(V_ADXL−2.5)`, `V− = 1.65 V`, on a suitable 3.3 V input supply. This gives differential ±1 V over the sensor's ±50 g, within ±1.2 V. Proper amplification into PGA1 avoids imposing the high-gain PGA's tighter input common-mode limits on the bridge design.

SNR already contains quantization noise. A model that injects the full SNR-equivalent noise **and** a quantizer overbudgets that contribution; keep that as an explicit conservative case or subtract the ideal quantization variance when separating analog noise and quantization. Conversely, an ADC's internally filtered noise may be correlated between samples. Reducing its whole quoted RMS as independent white samples through an EMA can be optimistic. Use the actual internal response for its analog front-end noise, a separate quantizer, or a conservative case that preserves quoted noise after conditioning. Published SNR at 200 kSPS or 1 MSPS does not constitute a noise guarantee at an arbitrarily chosen lower rate.

For force, an illustrative effective ±160 N axis mapped to ±5 V has 31.25 mV/N sensitivity, **0.004883 N per 16-bit code**, and **0.002622 N RMS** ADC noise at the typical ADS8588S specification. Six raw bridge voltages do not independently equal six force components. Actual propagation is:

```text
Σ_force = C_force · Σ_ADC · C_forceᵀ
σ_force_axis = σ_voltage · ||C_force_row||₂    (equal independent bridge noise)
```

The calibrated matrix and chosen gains must meet the force-axis noise contract and prevent bridge or amplifier clipping. The illustrative N/code conversion is not a claim that an unspecified raw load cell already has this gain.

## The low-cost delta-sigma option needs its genuine filter

ADS131M08 at PGA1 has typical input noise of **21.31, 10.68, 7.56 and 5.35 µV RMS** at OSR128, 256, 512 and 1024 respectively. These values already include its Sinc³ decimation response. At OSR128 the nominal 8.192 MHz master clock gives 32 ksample/s, while a supported **7.68 MHz external clock gives 30 ksample/s** and three raw frames per 10 kHz controller tick. A 240-bit frame at synchronous 15.36 MHz SPI requires 15.625 µs. Keep global chopping disabled for the stated continuous-mode transfer. [TI datasheet, tables 7-1 and 8-2; sections 8.3.7 and 8.5.1.7](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

| Proposed M08 clock 7.68 MHz | OSR | Native output rate | Calculated Sinc³ −3 dB point | FIR group delay | Group delay + 240-bit read |
|---|---:|---:|---:|---:|---:|
| Fastest synchronous configuration | 128 | 30 ksample/s | 7.858 kHz | 49.609 µs | 65.234 µs |
| Lower-rate configuration | 256 | 15 ksample/s | 3.929 kHz | 99.609 µs | 115.234 µs |
| Lower-rate configuration | 512 | 7.5 ksample/s | 1.964 kHz | 199.609 µs | 215.234 µs |

These are calculations from the manufacturer's exact transfer, not measured device delay. External analog filtering, accelerometer phase, MCU processing, controller holding and any additional internal phase must be included in the installed chain. Lower output rate changes the internal filter's delay and response, not just the frequency of a sample/hold. At the normal 8.192 MHz clock, OSR128 filter group delay is **46.509 µs** and its cutoff **8.382 kHz**.

The exact modulator-clock transfer is `[(1−z^(−OSR))/(OSR·(1−z^(−1)))]³`, followed by decimation. The FIR has `3·(OSR−1)+1` taps and group delay `3·(OSR−1)/(2·f_MOD)`. It cannot be replaced with a first-order RC at the ADC output rate. Startup or resynchronization introduces separate settling behavior; establish settled sensing before the already-loaded-contact controller begins.

## Cost and calibration consequences

The front end does not vanish with a cheap converter. As cost context, six INA821-class bridge amplifiers at observed **€6.44 each** cost approximately **€38.64**, before filter amplifiers, reference/excitation, precision passives, protection, regulators, PCB or test. Several dual filter/conditioning op amps add roughly €10–15 at these quantities. Much of that is common to both the expensive and cheap ADC architectures. These are approximate cost bounds, not a part list. [INA821 price](https://eu.mouser.com/en/ProductDetail/Texas-Instruments/INA821IDGKT?qs=LDGDZb5k%2F%252B8diOW5CeiHfQ%3D%3D), [dual filter-amplifier price example](https://www.mouser.be/c/semiconductors/amplifier-ics/operational-amplifiers-op-amps/?series=OPA1656).

An INA821 has 290 kHz bandwidth at gain 100, but only 30 kHz at gain 1000. The actual chosen gain matters when budgeting front-end phase; a nominally fast instrumentation amplifier does not stay equally fast at every gain. Its typical 7 nV/√Hz input noise and the load-cell calibration matrix determine the bridge-amplifier contribution. [INA821 datasheet](https://www.ti.com/lit/ds/symlink/ina821.pdf).

Static tare and temperature compensation remain necessary. For example, the ADS8588S maximum offset drift of 3 ppm/°C, interpreted against its 10 V span, is 30 µV/°C. At centered acceleration gain 2 this is approximately **0.003677 m/s²/°C**, consuming a ±0.005 m/s² residual-bias target after about **1.36°C** of uncompensated change at the maximum bound. That is an ADC-only analytical bound; the sensor and conditioning circuit also drift. The M08's 215 nV/°C typical offset drift maps to approximately **0.000105 m/s²/°C** with gain-0.5 conditioning, but a typical value is not a guaranteed whole-chain temperature result. Compare calibrated residual accuracy separately from sample RMS noise.

Gain, reference drift, nonlinearity and channel-response matching require the same attention. ADS8588S specifies an internal-reference gain drift of approximately ±10 ppm/°C typical and input INL up to ±1.5 codes; M08 specifies up to 20 ppm/°C gain drift including its internal reference at PGA1. The ADS8588S 16 kHz bandwidth and 19 µs delay are typical/design values; its electrical table does not provide a maximum channel-to-channel analog group-delay mismatch. Shared simultaneous conversion therefore does not establish perfectly matched installed force and acceleration paths. Identify and calibrate those paths on the assembled system before applying a tight residual phase contract.

ADS8688A is a credible multiplexed converter, but offers no chip-cost advantage over ADS8588S. At maximum aggregate throughput, eight-channel scanning gives 62.5 ksample/s per channel and 14 µs between first and last channel. With a six-bridge calibration matrix this timing must be reconstructed rather than assuming an instantaneous force vector. Its 500 kSPS headline is an aggregate rate. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads8688a.pdf).

An MCU-integrated ADC can save the separate converter cost, but would require a separate hardware noise and timestamp qualification. For example, ST reports approximately 12.8 single-ended or 13.7 differential effective bits in its STM32H7 application note; three ADC engines with channel scanning are not automatically eight simultaneous converters. The primary MCU datasheet and board-specific behavior govern, rather than the 16-bit register size. [ST AN5354](https://www.st.com/resource/en/application_note/an5354-getting-started-with-16bit-adc-on-stm32h7-mcus-stmicroelectronics.pdf), [STM32H743 datasheet](https://www.st.com/resource/en/datasheet/stm32h743ag.pdf).

## Decision to carry into native qualification

Test **ADS8588S with centered acceleration gain 2** first, at the existing raw rate and then the reduced rates supported by the experiment schedule. Its cost advantage is large without changing the fundamental simultaneous SAR architecture. Include its real analog filter, converter noise, quantization and pipeline, not only a lower bit count.

Also test **ADS131M08 at synchronous 30 ksample/s** with PGA1 and level-shifted analog conditioning. Its extra cost saving is worth checking, but the Sinc³ phase, noise placement and reduced-rate behavior must be present in the native experiment. Select the least expensive arrangement that retains an experimentally demonstrated margin in the worst repeated, calibrated and mechanical conditions; nominal bandwidth theory alone does not select a minimum rate.

The source ledger is [JSON](adc_cost_component_review_20261002.json), derived budgets are [CSV](adc_cost_component_review_20261002.csv), and the [calculation recipe](adc_cost_component_review_20261002.py) regenerates 22 electrical budget rows without running a simulation.
