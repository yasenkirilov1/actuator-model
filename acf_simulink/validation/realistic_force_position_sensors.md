# Manufacturer evidence for force and position sensing

Reviewed 2026-09-30. This is a bounded component-feasibility review, not a purchasing list or a hardware qualification. No new simulation was run for this review. The task is true normal-force control at 5–50 N, S = 0.1–2 kg, fixed flange orientation 0–90°, with a rigidly mounted B–S multi-axis load cell, S–A position and synchronized S acceleration. The 0.02 N RMS force noise, 1 µm RMS position noise and 100 µs channel delays remain proposed system assumptions until an assembled acquisition path is characterized.

**Recommendation:** use a compact, stiff six-axis cell in the ATI Mini27 Titanium SI-80-4 class, conditioned as six analog gauge channels and acquired with the S accelerometer on simultaneous, synchronized ADCs. Use a protected TONiC/RELM optical position encoder, with a hardware counter latched at the same acquisition epoch. This is a realistic architecture to prototype. None of the reviewed force-cell factory specifications establish a 0.05–0.1 N calibrated residual; that is an acceptance target requiring actual assembled calibration and validation. Retain the Mini40 SI-80-4 as an alternative if finer published force resolution outweighs its lower stiffness.

## Force range, resolution and mechanical fit

The sensor range applies to **raw** load, before gravity tare or inertial compensation. At S = 2 kg, gravity contributes up to 19.62 N in a transverse channel. Current campaign estimates supplied by the model integrator put a transverse peak near 50 N for sanding friction µ = 0.3, and near 64 N for µ = 0.6, including gravity. These are provisional modeled loads, not a manufacturer rating or an independently qualified envelope. An ±80 N transverse range is therefore plausible; moments, combined loading and all raw recorded axes must still be checked. A software tare does not restore hardware range.

| Representative sensor/calibration | Raw force ranges Fx/Fy; Fz | Published effective resolution Fx/Fy; Fz | Package / relevant constraint | Assessment |
| --- | --- | --- | --- | --- |
| [ATI Mini27 Titanium SI-80-4](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Mini27+Titanium) | ±80 N; ±160 N | 0.03 N; 0.06 N | Ø27 × 18.2 mm, 33.4 g; Tx/Ty ±4 Nm, Tz ±2 Nm | Compact, relatively stiff candidate; resolution is not RMS noise or accuracy. |
| [ATI Mini40 SI-80-4](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Mini40) | ±80 N; ±240 N | 0.02 N; 0.04 N | Ø40 × 12.2 mm, 49.9 g; torques ±4 Nm | Finer published resolution; lower stiffness. SI-40-2 only allows ±40 N transverse, insufficient for the provisional high-load cases. |
| [ATI Nano25 SI-125-3](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Nano25) | ±125 N; ±500 N | 0.02083 N; 0.0625 N | Ø25 × 21.6 mm, 63.4 g; torques ±3 Nm | More transverse headroom and high stiffness, but broad axial range costs resolution. |
| [ATI Nano43](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Nano43) | Largest listed calibration: ±36 N in all force axes | 0.0078125 N for SI-36-0.5 | Ø43 × 11.5 mm, 38.7 g | Reject for the high-force raw envelope. No manufacturer Nano43 SI-70 calibration was found. |
| [ME K3D60a 100 N](https://www.me-systeme.de/de/pdf/2521/K3D60a_100N) | ±100 N in each force axis | No qualified 10 kHz RMS-noise value in the reviewed data | 60 × 60 × 25 mm; 0.1 mm nominal deflection | Range fits, but compliance and calibration are material limitations. |
| [FUTEK MTA400](https://media.futek.com/content/futek/files/pdf/productdrawings/mta400.pdf) | ±1112.06 N; ±2224.11 N | Not established for this task | 907 g; 0.25% rated-output linearity/hysteresis, ≤3% crosstalk | Reject as the reference: large range and mass, with rated linearity equivalent to 2.78/5.56 N. |

ATI's listed resolution is an effective value after discarding four noise counts for the stated interfaces, not a broadband RMS-noise specification. Filtering can improve it while adding phase delay. The [ATI FAQ](https://www.ati-ia.com/zh-CN/library/documents/FT_FAQ.pdf) explains that measurement uncertainty includes nonlinearity, hysteresis and crosstalk under steady temperature, and is specific to each axis and calibration certificate. Sample RMS noise does not cover those errors.

The [current Novanta/ATI Mini27 page](https://ati.novanta.com/product/mini27-titanium-force-torque-sensor/) states axis-dependent accuracy of 0.75–1.25% full scale. For the stated ranges, that corresponds to 0.6–1.0 N transverse or 1.2–2.0 N axial, depending on the actual axis certificate. This fails to establish the desired residual accuracy. The page's separate calibration table conflicts with its main specifications and the legacy manufacturer's SI-80-4 table; this review uses the consistent main ranges and legacy table. Do not substitute the conflicting table or interpret factory accuracy as an already corrected residual.

The September 2026 ME datasheet gives 0.2%FS linearity, compensated crosstalk below 1%FS and zero-temperature sensitivity of 0.02%FS/K: respectively 0.2 N, below 1 N and 0.02 N/K for this range. Its 0.5 mV/V sensitivity at 5 V excitation gives 25 µV/N, so a 0.02 N input-equivalent RMS-noise target requires approximately 0.5 µV RMS before decoupling. This is an electronics design target, not a verified sensor result. The [current K3D family page](https://www.me-systeme.de/de/k3d-3-achsen-kraftsensor) limits K3D40 variants to 2–50 N; older catalogs listing a K3D40 100 N variant should not drive the current recommendation.

## Loaded stiffness and bandwidth

For illustration only, a single spring with attached mass S has f = sqrt(k/S)/(2π). Manufacturer calculated stiffness gives these estimates at S = 2 kg:

| Sensor | Transverse / axial stiffness | Approximate attached-mass modes |
| --- | --- | --- |
| Mini27 Titanium | 31 / 64 MN/m | 627 / 900 Hz |
| Mini40 | 11 / 20 MN/m | 373 / 503 Hz |
| Nano25 | 53 / 110 MN/m | 819 / 1180 Hz |
| K3D60a 100 N | About 1 MN/m inferred from rated load / nominal deflection; axis dependence not specified by that deflection entry | About 113 Hz |

These estimates omit the sensor's effective moving mass, tooling flexibility, rotations, damping and distributed modes. They are **not measured sensor bandwidths**. The manufacturer's unloaded resonance cannot be applied to a mounted 2 kg tool unchanged. ATI explicitly notes that tooling mass and configuration determine application frequency response in its [interface documentation](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx). Finite B–S stiffness must be included or shown negligible over the closed-loop bandwidth, even though the mounting is rigid and there are no intentionally added B–S springs. Otherwise the native rigid-connection model remains conditional.

## Acquisition and force calibration

The ATI analog DAQ/TWE path is a plausible way to achieve deterministic synchronized acquisition. The [interface table](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx) lists DAQ conditioning with a second-order 5.1 kHz anti-alias filter; analog output latency is a few microseconds, while digitization rate and delay depend on the DAQ. TWE requires a designed anti-alias filter. The table's Net RDT path is 7 kHz / 288 µs before user-network delay; its EtherCAT path is 1 kHz / up to 1 ms. These particular interfaces are not substitutes for the assumed 10 kHz / 100 µs path. Newer interfaces require their own exact manual and qualification.

The [ATI TWE manual](https://www.ati-ia.com/app_content/documents/9620-05-TWE.pdf), revision 17, requires six amplified raw gauge channels, characterized gains/excitation and the transducer's calibration matrix. Six gauges plus S accelerometer XYZ require **nine analog channels**. Use two common-trigger/common-clock simultaneous ADCs, or a genuinely qualified interface that outputs decoupled forces. A single eight-channel ADC cannot acquire all nine. Host USB/network delivery is not the acquisition epoch.

Practical acceptance targets for the assembled force path:

- Measure ≤0.02 N RMS **per reconstructed force axis** at the actual sample rate and implemented filter; include decoupling, motor PWM, moving cables and sanding vibration. A higher ADC bit count alone is not evidence.
- Aim for a maximum residual error of 0.05–0.1 N after offset, gain, multiaxis, temperature and mounting calibration over the operating loads. This is about 0.05–0.1%FS for a 100 N range. Calibrate combined forces and moments, loading/unloading, orientations and coil temperatures; retain independent check loads. It is not established that these cells can achieve it over that entire domain.
- Identify force, acceleration and position transfer functions and relative delays. A common trigger synchronizes ADC conversions; it does not cancel unequal analog filter phase or mechanical response. Include those responses and measured uncertainty in native qualification.
- Align sensor coordinates with the tool and surface-normal estimate. Reconstruct contact force using downstream S inertia and weight in a consistent frame, then project onto the estimated normal. Record raw gauges, corrected force vector, S acceleration, normal estimate and final force so cancellation errors are inspectable.

## S–A position measurement

A [Renishaw TONiC T101x readhead with RELM20 scale](https://www.renishaw.com/media/pdf/en/fa9b5d5f3056478c8031f7c3e8091102.pdf) is a realistic metrology reference. RELM scale is certified to ±1 µm at 20°C for lengths up to 1 m. The published TONiC jitter can be down to 0.5 nm RMS and typical subdivision error about ±30 nm. These are encoder-specific figures, not mounted flange accuracy. A 40 mm scale offers 30 mm measuring length without limit magnets; a 50 mm scale offers 25 mm with the documented 10 mm limits. Allow mounting clearance beyond the required travel.

Choose a 0.1 µm digital interface rather than unnecessary nanometre interpolation. Its quantization RMS would be about 0.029 µm if phase is uniformly distributed; real noise must be measured. Latch the quadrature counter at the force/acceleration ADC trigger and maintain a common timestamp. The [TONiC datasheet](https://www.renishaw.com/media/pdf/en/2a02c0b218ec491084e6cf95f3965146.pdf) supports 0.1 µm interfaces and provides a speed/clock table, but a complete measurement-to-controller delay bound was not found in the reviewed documents. Identify interface/counter/computation delay rather than assigning zero or 100 µs without evidence.

The [January 2026 installation guide](https://www.renishaw.com/media/pdf/en/a15686d384634a2f95b819e44cd7a8e9.pdf) lists a 10 g readhead, IP40 sealing and an IP20 Ti/TD interface. Protect the optics and scale from sanding dust; place interface electronics on A where practical. Reference/homing is needed because this is incremental sensing. The datasheet warns that external magnetic fields above 6 mT can falsely activate reference-selection/limit sensors. Verify location near the voice-coil magnet, use the appropriate all-reference-marks readhead and independent travel-stop protection; optical incremental interpolation alone does not remove that magnetic caveat.

Calibrate the assembled S–A brackets and scale over the full 25 mm travel, all orientations and hot/cold operation. Verify ≤1 µm RMS noise with motor switching active, along with offset, scale error, bracket deflection, tilt sensitivity and lost-count detection. Measuring magnet position or B–A is equivalent to S–A only after the B–S deformation has been shown negligible or compensated.

The architecture above supports a realistic sensor-informed prototype. The next evidence gate is measured raw range, combined-load residual calibration and synchronized dynamic response with the actual tool installed; catalog resolution cannot qualify the final ±0.5 N normal-force requirement.
