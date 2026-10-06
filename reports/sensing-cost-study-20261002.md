# Lower-cost sensing for the active sanding flange

## Goal and decision

Keep the workpiece's normal pressing force within **±0.5 N throughout the recording**, including the specified 1 mm × 10 mm bump at 100 mm/s, with S = 0.1–2 kg, B = 0.6 kg, 5–50 N targets and fixed 0–90° orientations. Reduce acquisition and accelerometer cost while retaining useful force margin. The controller still has force, S–A position, S acceleration and already-issued robot commands; it has no surface preview.

**Decision:** reduce ADC cost first. The ADS131M08 at 30 ksample/s with an engineered matched 8 kHz response passes all 700 new nominal conditions and the retained calibration/mechanics cases. Keep the reference acceleration sensors for the first prototype. The tested cheaper acceleration envelopes either fail the joint-error cases or consume most of the remaining force margin. The original 3,500-case result belongs to the scientific reference acquisition; these replacement chains have their own recorded evidence.

The cost study contains **999 executed native recordings**: 993 bump recordings and six quiet tone probes. Nine compilation-parity recordings are counted separately. Deliberate paired repeats are included; this does not mean 999 unique parameter identities. Prepared-only sensor cases and software preparation rejections are excluded. [Execution ledger](../acf_simulink/validation/budget_sensing_execution_ledger_20261002.json).

## Why a cheaper ADC can be enough

The ADC's bit count does not determine its useful force resolution by itself. An instrumentation amplifier maps the load cell's useful range into the converter's voltage range. The important quantities are input-referred noise, quantization after that gain, matched channel response, and the age of the sample when the controller uses it.

Indicative single-IC prices retrieved on 2 October 2026 are **€63.86 for AD7606C-18**, **€19.13 for ADS8588S**, and **€9.25 for ADS131M08**, excluding VAT and shipping. These are documented regional price observations; they are not checked shopping carts. The ADC-only savings are approximately 70% and 86%. Instrumentation amplifiers, references, filtering and calibration remain part of the electronics. [Price/specification review](../acf_simulink/validation/adc_cost_component_review_20261002.md).

| Converter | Tested practical setting | Force quantum at effective ±160 N | ADC acceleration quantum with reference analog conditioning | Intrinsic ADC response |
| --- | --- | ---: | ---: | --- |
| ADS8588S, 16-bit simultaneous SAR | 50 ksample/s per channel, ±5 V | 0.004883 N | 0.018705 m/s², ADXL1002 centered gain 2 | Third-order 16 kHz analog filter, about 19.9 µs DC delay |
| ADS131M08, 24-bit simultaneous delta-sigma | 30 ksample/s per channel, PGA1 | 0.0000191 N | 0.0000701 m/s², ADXL1002 gain 0.5 | Sinc³, OSR128 at 3.84 MHz modulator, 49.6 µs group delay |

Acceleration resolution is the step between adjacent codes; it is not the RMS noise or calibration accuracy. The SNR-equivalent ADC acceleration noise in these two configurations is approximately **0.01236 m/s²** for the SAR's datasheet-minimum-SNR-derived model noise and **0.01045 m/s²** for the delta-sigma typical OSR128 noise. The SAR minimum is specified at 200 ksample/s; it is a modeling provision rather than a verified noise bound at the tested 50 ksample/s. The M08 figure is typical, rather than a guaranteed maximum at the selected clock. The experiment injects ADC noise separately from physical sensor noise and avoids counting ideal quantization variance twice. Separate M08 probes double its typical noise and add 200 µs output-noise correlation. Measure both converters' assembled noise at the chosen rate. [TI SAR datasheet](https://www.ti.com/lit/ds/symlink/ads8588s.pdf), [TI delta-sigma datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

The M08 uses a **7.68 MHz master clock**, a 3.84 MHz modulator and OSR128 to produce 30 ksample/s. Reading status, eight channels and CRC requires 240 SPI bits: **15.625 µs at 15.36 MHz SPI**. The modeled **20 µs total post-conversion pipeline includes that readout**, leaving about 4.375 µs for the MCU path; it is not another 20 µs added to readout. The SAR's worst conversion plus a 128-bit read at 16 MHz is about 11.9 µs within the same total allowance. Verify those budgets on the chosen MCU.

With an analog accelerometer, **yes, the external ADC measures acceleration as well as force**. Two acceleration axes plus six raw bridge channels fit an eight-channel ADC in the current planar arrangement. A digital accelerometer uses its own internal ADC and a timestamped SPI interface; it does not consume an external ADC channel. A spatial arrangement with six bridges and three analog acceleration axes needs nine channels. Position has its own encoder interface. The bridge calibration matrix must establish the effective force-axis noise and range; raw channel specifications alone do not establish them.

## Native experiments justify the bandwidth and sample rate

The new private native Simulink model includes a continuous instrumentation response **before sampling**, the SAR's analog response or a genuine Sinc³ decimator, converter noise, range/quantization, an EMA with a 50 µs time-constant setting at the converter-output clock, the readout pipeline and fresh 10 kHz controller latching. Physical sensor noise density stays fixed when sample rate changes. Qualification uses every physical normal-force sample, including startup; plots are reduced only afterward. The force and acceleration response envelopes are matched, and position has a separate encoder with an aligned measurement frame.

The screen executes **168 recordings**, with two heavy/light physical endpoints and clean/seed-7 noise variants at each setting. The instrumented physical plant, estimator and controller are frozen. A separate nine-recording audit checks warm/cold compilation parity across seed, mass/force/orientation and response changes; all physical and ADC samples in the tested pairs match exactly.

![Native ADC bandwidth and rate sweep](figures/budget_adc_bandwidth_rate.png)

*The numbers are worst complete-recording errors, in N, across four recordings per cell. Values above 0.500 N fail. The 0.450 N design threshold reserves 0.050 N for the physical requirement. This is a finite endpoint screen, not a continuous bandwidth proof.*

The low-bandwidth failures are substantial. With a 500 Hz matched response, the best rate still gives roughly 0.608 N error for the SAR and 0.672 N for the delta-sigma chain. A 1 kHz response is only a nominal pass at some settings. The familiar 10 Hz bump transit does not capture the controller's need to recover downstream inertia and react at bump entry/exit; the executed waveform resolves the faster dynamics.

Rate matters even with the same analog cutoff. At 8 kHz response, the delta-sigma group delay increases from about **49.6 µs at 30 ksample/s**, to **99.6 µs at 15 ksample/s**, to **199.6 µs at 7.5 ksample/s**. Lower rates also age the samples more before each controller latch. The controller rate remains 10 kHz in all these comparisons.

The four retained **30 s repeated-bump calibration cases** preserve the selected tighter static residuals, channel-response mismatches and robot timing errors. They expose the trade more sharply:

| Converter and setting | Completed / passed | Worst normal-force error | Remaining reserve to 0.5 N |
| --- | ---: | ---: | ---: |
| ADS131M08, 15 ksample/s, 8 kHz matched response | 4 / 4 | 0.494316 N | 0.005684 N |
| ADS131M08, 30 ksample/s, 5.1 kHz matched response | 4 / 4 | 0.451330 N | 0.048670 N |
| ADS131M08, 30 ksample/s, 8 kHz matched response | 4 / 4 | 0.438821 N | 0.061179 N |
| ADS8588S, 50 ksample/s, 5.1 kHz matched response | 4 / 4 | 0.404214 N | 0.095786 N |

Thus 15 ksample/s is a numerical pass in these four cases, but does not retain a useful design margin. Keeping **30 ksample/s** has an experimental justification. The selected **8 kHz** response retains over 0.05 N reserve in these cases; 5.1 kHz narrowly misses that preferred reserve. This establishes a tested choice, rather than proving that 8 kHz is the minimum possible cutoff. The value describes a complete engineered response target: include the accelerometer's own amplitude/phase rather than assigning it an arbitrary filter corner.

For orientation, a nominal two-pole 8 kHz response contributes **28.1 µs low-frequency delay**, Sinc³ adds **49.6 µs**, and the sampled EMA contributes **35.2 µs** at 30 ksample/s. Including the nominal pipeline gives about **132.9 µs before sample-age and controller-latch effects**. These are component delay calculations, not a single frequency-independent phase model. Native execution retains the full filters and clocked behavior. At 15 ksample/s, the decimator alone doubles its delay despite the unchanged 8 kHz analog response; the measured force-error increase above verifies that timing matters.

![Native long-test force waveforms](figures/budget_adc_long_waveforms.png)

*Each curve shows the bump containing the full-recording worst sample in that setting's worst 30 s recording. The cases retain calibration error and seeded noise. Dashed lines are the force requirement; dotted lines reserve 0.05 N.*

The selected M08 chain also completes a **700-case nominal grid**: five S masses, ten force targets, seven orientations, two physical drag ratios and **one noise seed, 29, per condition**. All 700 unique cases pass; worst complete-history error is **0.292803986 N**, leaving **0.207196014 N** reserve. Every one of the 1,200,001 native force samples in each four-second recording is scored. This is 700 new noise realizations, not the historical five-seed 3,500-case qualification. Preparation hashes, condition identities and disjoint partition coverage are recorded in [the grid audit](../acf_simulink/validation/budget_adc_range_grid_20261002.json).

![ADC noise and timing robustness probes](figures/budget_sensing_response_noise.png)

*Two heavy-tool recordings per probe, seeds 7/29. Each changes one acquisition property. These nominal probes are separate from the simultaneous calibration-error cases.*

Doubling the ADC's typical noise gives **0.339688 N** worst error; introducing a 200 µs correlation time gives **0.332057 N**. An extra acceleration-only skew of 20 µs gives **0.358505 N**, and 50 µs gives **0.381665 N**. The tested nominal cases pass, but timing consumes appreciably more margin than doubling converter noise. Keep the specified common clock and tested relative timestamp residuals rather than treating those probes as permission for arbitrary skew. Halving the physical solver step with unchanged acquisition clocks changes the paired peak by only **0.000000304 N**. Six additional quiet sensor-tone probes at 19, 49 and 99 kHz are retained separately; they do not constitute a measured sanding-vibration spectrum.

The selected **30 ksample/s / 8 kHz chain is also executed with the hot real-winding parameters**: Kf=24.4 N/A, R=8.896 Ω, L=4 mH, 32 V usable bipolar output and the unchanged **100 µs** current target. Current-domain gains are rescaled by 20/24.4 while their identified Kf residuals remain. All seven retained long/mixed cases pass, worst **0.438821 N**, with **zero voltage clipping**. Maximum requested voltage is **31.738466 V**, leaving only **0.261534 V** below the modeled output limit; peak current is **2.455697 A**. This is a new native electrical check, not reuse of the older pointwise voltage screen. At least 32 V usable output remains supported on these cases, but driver dropout and hot winding tolerances need an assembled headroom measurement. The fixed hot resistance does not simulate evolving temperature. [Selected hot-coil evidence](../acf_simulink/validation/budget_adc_hot_coil_summary_20261002.json).

## Cheaper accelerometers are limited mainly by response

Two reference single-axis ADXL1002s cost approximately **€129.76** at the observed quantity-one prices. One three-axis **ADXL354C is about €60.44**, while an **ICM-42688-P is about €5.33** and **IIS3DWB about €19.90**. Those are sensor IC comparisons, not complete board costs. The analog ADXL354 still uses the external ADC; the digital candidates have their own conversion. [Component and price review](../acf_simulink/validation/accelerometer_cost_component_review_20261002.md).

ADXL354C's ±8 g setting has an approximately 33.75 µg/√Hz noise-density target and 100 mV/g sensitivity. A centered buffered input into the M08 gives roughly **0.0000140 m/s² per ADC code** and **0.00209 m/s² typical ADC noise**. Its higher sensor noise is larger than the ADC contribution. That resolution is sufficient for investigating the force objective; it does not establish bias, scale, temperature drift or vibration rectification.

The **published 1.9 kHz overall 3 dB bandwidth is not a complete transfer function**. The ADXL354 has an internal sinc-shaped response and a 2.4 kHz mechanical mode. Current manufacturer documentation does not supply the full coefficients or phase. The following simulations therefore test clearly identified **required matched response envelopes** with the part's noise and range; they do not claim that the part implements a two-pole Butterworth filter. [ADI datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf), [response review](../acf_simulink/validation/budget_accelerometer_response_review_20261002.md).

![Cheaper accelerometer response and noise experiments](figures/budget_accelerometer_envelopes.png)

*36 completed native nominal recordings. Analog cases cover both physical endpoints and seeds 7/29; the digital envelope cases cover the heavy endpoint. Counterfactual 3/5.1 kHz ADXL354 bandwidths are retained in the source but excluded from this practical comparison. Extra common delay is known to the estimator and matched across measurements.*

At the existing 100 µs current target, the ADXL354-like 1.9 kHz envelope gives **0.405691 N** worst error with no additional common latency. An extra 50 µs raises it to **0.457041 N**; 100 µs raises it to **0.484946 N**. Noise alone is therefore not the deciding obstacle. These delay increments are sensitivity cases; published bandwidth cannot prove that they bracket the real part's phase.

A separately tested **50 µs current target at ideal 36 V** improves the same envelope to **0.378082 N**, or **0.429852 N** with the extra 50 µs. This changes current-loop bandwidth as well as drive provision. It is not evidence that increasing voltage alone improves an unclipped 100 µs loop. The joint calibration and real-hot-winding tests are needed before choosing that trade.

The low-cost digital envelope at 70 µg/√Hz and an additional common 80 µs gives **0.434807 N** in its nominal heavy endpoint. With 120 µs it reaches **0.477704 N**. The ICM's own documented decimation, selectable filters, clock and DRDY/readout timestamps still need to establish the required complete response. Its published delay combines configured filters and clocks; it cannot be replaced by an arbitrary RC pole. The native digital cases use a labeled common response/latency envelope, not an implementation of the ICM's undocumented complete transfer. Calling its output rate its sensing bandwidth would be incorrect.

The stock ADXL354 evaluation board also includes 0.1 µF output capacitance. With the documented 32 kΩ output impedance, that adds a **3.2 ms RC**, about 49.7 Hz. Such a configuration is far slower than the studied envelope. A fast prototype needs intentionally selected output capacitance and buffering. [ADI evaluation guide](https://www.analog.com/media/en/technical-documentation/user-guides/eval-adxl354-355-ug-1030.pdf).

The decisive followup preserves **four 30 s calibration-error recordings and three four-second known-k/c mixed-mechanics recordings** for each of seven sensing/drive combinations. The controller uses the same mechanics and causal measurements. Actual and identified sensor/mass/robot residuals stay separate.

| Required acceleration-response envelope and drive | Passed / executed | Worst error | Reserve to 0.5 N |
| --- | ---: | ---: | ---: |
| Reference density, matched 8 kHz, 100 µs current, scientific 24 V coil | 7 / 7 | 0.438821 N | 0.061179 N |
| ADXL354-like 1.9 kHz, no extra delay, 100 µs current, scientific 24 V coil | 5 / 7 | 0.516806 N | −0.016806 N |
| ADXL354-like 1.9 kHz, no extra delay, 50 µs current, ideal 36 V coil | 7 / 7 | 0.482489 N | 0.017511 N |
| ADXL354-like 1.9 kHz, extra 50 µs, 50 µs current, ideal 36 V coil | 4 / 7 | 0.530965 N | −0.030965 N |
| Digital 70 µg/√Hz, matched 5.1 kHz + 80 µs, 100 µs current | 3 / 7 | 0.581260 N | −0.081260 N |
| Digital 70 µg/√Hz, matched 5.1 kHz + 80 µs, 50 µs current | 3 / 7 | 0.559963 N | −0.059963 N |
| ADXL354-like 1.9 kHz, no extra delay, 50 µs current, hot real-coil 32 V output | 7 / 7 | 0.482489 N | 0.017511 N |

![Cheaper sensing under calibration and drive stress](figures/budget_sensor_calibration_stress.png)

*49 executed native recordings; failures are retained. Each row is a declared sensor response envelope, not a supplier-certified accelerometer transfer. The 0.45 N design threshold retains 0.05 N reserve.*

The faster ADXL354-like case passes these seven inputs, but its remaining reserve is less than one third of the reference envelope's. Its hot-coil variant uses Kf=24.4 N/A, fixed R=8.896 Ω, L=4 mH and 32 V usable terminal output; brief voltage clipping occurs, and the recorded worst error stays essentially the same. This is a real-winding native simulation with a **deliberately faster 50 µs current target**, not a claim that voltage alone fixes sensing delay. Adding 50 µs common sensing latency still produces failures. The digital response envelopes fail four of seven cases even with the faster current loop.

A final **seven-case tighter dynamic-matching experiment** reduces analog-response and EMA residuals from ±10% to ±1% and relative channel offsets from ±10 µs to ±2 µs, while preserving static calibration, common pipeline uncertainty, robot timing, mechanics, noise and drive. Common and per-channel response factors both remain, so their combined maximum is approximately 2.01%. The no-extra-delay ADXL354-like envelope passes at **0.489946 N worst**, only **0.010054 N reserve**. Tighter matching did not recover the desired margin; the peak need not improve monotonically because it combines deterministic error and seeded noise. Seven extra-delay matching inputs were prepared but **not executed**, and are excluded from the experiment count.

Therefore a cheap accelerometer is **not yet supported as a low-loss replacement**. ADXL354C is an attractive analog cost target for a later measured-transfer study, but the executed envelope results do not justify recommending it for this force specification. The digital part's large chip saving does not compensate for the observed failures. Keep two ADXL1002-class analog axes in this planar prototype; use three axes and sufficient simultaneous ADC channels for spatial reconstruction. Their own resonance and assembled phase still require measurement.

## Prototype acceptance and recommendation

Use these experiments to choose acquisition hardware and a measured response target. Characterize the assembled acceleration and force chains together: complex response, effective timestamp, axis matching, ADC/MCU timing, hot scale/bias and powered-tool vibration. Match the force, acceleration and estimated-normal frames before reconstructing workpiece force. The tighter calibration targets remain **±0.01 N force zero, ±0.005 m/s² acceleration bias**, and the other residuals in the main report. At S=2 kg, 0.005 m/s² acceleration bias already corresponds to 0.01 N of inertia error.

The required physical acceptance remains **maximum independent reference-force error plus its applicable uncertainty ≤0.45 N** over the full loaded bump recording, including startup and powered sanding. The bandwidth/cost study is a finite simulation result; it does not provide powered-tool vibration or prototype accuracy measurements.

The native acquisition variant, its helper/parameter hashes, all executed cohort receipts and selected full histories are retained under `acf_simulink/validation/`. The public desktop model still contains the scientific reference chain, so its older results remain reproducible. Use [the lower-cost study recipe](../acf_simulink/experiments/README_budget_sensing_20261002.md) to run the frozen variant in a private batch process. Replacing hardware response assumptions requires a new recorded comparison.

**Concrete recommendation:** build the guided voice-coil/measured-inertia PI prototype with the selected lower spring pair and 60 N·s/m damping, a synchronized multi-axis B–S cell, S–A position measurement and the reference S-mounted acceleration sensors. Reduce acquisition cost with **ADS131M08 at 30 ksample/s per channel**, a measured matched **8 kHz complete response**, 50 µs EMA setting, **20 µs total readout/MCU pipeline** and fresh **10 kHz control**. Retain the **100 µs current target and at least 32 V usable bipolar output** from a nominal 36 V supply. Keep ADS8588S at 50 ksample/s/5.1 kHz as the approximately €19 converter alternative if its stronger observed four-case margin or simpler response is preferred. Choose a cheaper accelerometer only after its measured transfer and full loaded tests recover at least 0.05 N reserve; the current experiments do not show that.
