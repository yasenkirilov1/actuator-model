# Fixed-controller ADC comparison

This is compact diagnostic evidence for the readable report revision, not a
new controller or physical-prototype qualification. The report authoring step
reuses these completed experiments; it launches no simulation or training.

## Question and controlled comparison

Can cheaper code spacing or lower converter rates retain the locked
fixed-mechanics controller's force performance? Seven known development cases
are repeated with the prescribed 20,000 N/m undamped upper spring, rigid lower
attachment, finite cell stiffness and the recorded multi-axis reconstruction.
The controller and its identified phase compensation are unchanged.

At 30 ksample/s, changing 24-bit codes to 16, 14 or 12 bits changes **only code
spacing**. The analogue response, modeled ADS131M08 noise and acquisition
timing remain unchanged. These are not complete models of particular cheaper
chips. Rate comparisons re-export actual output timing, sinc3 converter delay
and the manufacturer noise envelope. All native samples, including startup,
are scored; contact loss or a travel stop fails the recording.

| Samples/s/channel | Code bits | Passing cases | Worst force error | Physical failures |
|---:|---:|---:|---:|---:|
| 30,000 | 24 | 7/7 | 0.495250 N | 0 |
| 30,000 | 16 | 7/7 | 0.491806 N | 0 |
| 30,000 | 14 | 6/7 | 0.501971 N | 0 |
| 30,000 | 12 | 4/7 | 0.553362 N | 0 |
| 15,000 | 24 | 6/7 | 0.538603 N | 0 |
| 7,500 | 24 | 0/7 | 52.217653 N | 7 |

There are **42 completed diagnostic recordings and 21,291,567 scored native
samples**, with zero new qualification recordings. The seven 7.5 ksample/s
runs stop during startup before reaching the bump; they cannot establish
complete-recording waveform parity. The 30 ksample/s reference force
waveforms are bitwise identical to the selected original development cases.
The slightly lower group maximum at 16 bits does not establish a general
benefit from quantization: the largest paired peak-error increase is 0.002311 N,
and paired waveforms differ by up to 0.082473 N.

An independent MATLAB rate replay was attempted but **failed during MATLAB
startup before simulation execution**. It supplies no independent completed validation. The other
reference MATLAB/C parity outcomes remain as originally recorded.

## What this means for component choice

The converter group delay increases from 49.609 to 99.609 or 199.609 µs as
the output rate falls from 30 to 15 or 7.5 ksample/s. Lower-rate noise improves,
but the unchanged vibration controller loses timing margin. The comparison
supports retaining 30 ksample/s and the matched 1.9 kHz analogue-response
reference initially. It does not establish a universal minimum rate, an
optimal amplifier cutoff, or a separately retuned slow-converter design.

The reviewed [ADS131M04](https://www.ti.com/lit/ds/symlink/ads131m04.pdf)
OSR128 filter topology and gain-one noise specification match the corresponding
[M08](https://www.ti.com/lit/ds/symlink/ads131m08.pdf) setting. Recorded
quantity-one prices on 6 October 2026, including 21% VAT, were €7.1511 for
M04 and €11.4345 for M08. M04 can acquire one analogue force channel and
three ADXL354C acceleration outputs simultaneously. This channel allocation
is a first-prototype proposal, not an executed single-axis qualification.

The €2.9524 [MCP3208](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/21298e.pdf)
has one multiplexed 12-bit converter. Its 100 ksample/s maximum at 5 V is
aggregate, giving at most 25 ksample/s each across four sequential channels.
Input settling, nonlinearity, offsets, reference accuracy and channel skew
need separate characterization. A microcontroller ADC can have no incremental
converter-chip cost, but oversampling and filtering introduce their own noise,
delay and synchronization trade-offs. Neither option inherits this study's
ideal code-spacing results.

## Evidence and preservation

- [Executed compact results](../../../validation/report_readability_20261006/adc/executed_results_compact.json)
  bind all case identities and measured outcomes. Source history paths are
  historical references; bulk arrays were removed under the storage audit.
- [Reviewed manufacturer/price records](../../../validation/report_readability_20261006/adc/parts_review_20261006.json)
  retain datasheet revisions, distributor links, scaling and timing assumptions.
- [Reference parity record](../../../validation/report_readability_20261006/adc/reference_native_parity.json)
  records the original seven 27-channel comparisons;
  [the unsuccessful replay log](../../../validation/report_readability_20261006/adc/matlab_replay.log)
  preserves the independent MATLAB startup failure.
- `build_adc_diagnostics.m` and `run_adc_diagnostics.py` describe the original
  private preparation/execution recipe. Rerunning requires recreating removed
  private inputs under the shared storage budget; the report build does not do it.
- [Storage audit](../../../validation/storage_cleanup_20261006/README.md)
  records which bulk histories were removed. Frozen controller/model inputs,
  compact metrics and selected essential recordings are preserved.
