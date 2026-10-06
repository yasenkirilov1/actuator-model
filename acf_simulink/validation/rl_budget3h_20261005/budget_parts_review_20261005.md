# Accelerometer and ADC chip budget — 5 October 2026

**ADXL354C plus one ADS131M08 fits the €100 chip budget at €86.10 including Dutch 21% VAT. Two ADS131M08s also fit, at €97.54.** These are quantity-one component prices for the user's own PCB; amplifiers, buffers, clock, power supplies, PCB, assembly and shipping are additional. No purchase was made.

## Price and availability evidence

The following product pages were opened directly on **5 October 2026, 15:12–15:17 UTC**. Their current content supersedes older search snippets. Stock is the distributor's displayed count, not reserved stock or a delivery commitment.

| Component | Packaging, quantity | Excluding VAT | Including 21% VAT | Displayed stock | Primary listing |
|---|---|---:|---:|---:|---|
| ADXL354CEZ | Tray, 1 | €61.71 | €74.67 | 64 | [DigiKey Netherlands](https://www.digikey.nl/en/products/detail/analog-devices-inc/ADXL354CEZ/6557313) |
| ADS131M08IPBSR | Cut tape, 1 | €9.45 | €11.43 | 4 | [DigiKey Netherlands](https://www.digikey.nl/en/products/detail/texas-instruments/ADS131M08IPBSR/11502308) |

| Channel allocation | Chip subtotal excluding VAT | Subtotal including VAT | Margin below €100 |
|---|---:|---:|---:|
| One accelerometer + one ADC: six bridge channels + two acceleration axes | €71.16 | **€86.10** | €13.90 |
| One accelerometer + two ADCs: six bridge channels + three acceleration axes | €80.61 | **€97.54** | €2.46 |

Totals apply VAT to the unrounded subtotal. The listings explicitly show prices with and without VAT; [DigiKey's terms](https://www.digikey.nl/en/terms-and-conditions) state the tax exclusion, and the [Dutch Tax Administration](https://www.belastingdienst.nl/wps/wcm/connect/bldcontenten/belastingdienst/business/vat/vat_in_the_netherlands/calculating_vat/vat_tariffs) confirms the standard 21% rate. DigiKey lists €25 shipping below its €75 order threshold; the component calculation is not an all-in delivered quote.

One ADC has no spare analog channel in the eight-channel planar allocation: S–A position must be digital/separate. Two ADCs provide sixteen channels, so there is room for three acceleration axes and analog position. Use a common clock and synchronized conversion reset; calibrate inter-device timing.

## Manufacturer constraints used for retargeting

**Accelerometer.** The ADXL354C is a three-axis analog sensor with selectable ±2g/±8g range. At ±8g its typical sensitivity is 100 mV/g. ADI specifies 22.5 µg/√Hz noise at ±2g and estimates 50% more at ±8g: **33.75 µg/√Hz**, rather than the ±2g headline value. Overall −3 dB bandwidth is **1.9 kHz**, including mechanics and filters; the internal filter alone is approximately 1.5 kHz. A 2.4 kHz mechanical resonance means a simple 1.9 kHz model envelope does not reproduce the exact phase response. The output includes 32 kΩ series resistance: buffer it and include any added RC response. Offset, sensitivity and cross-axis calibration remain necessary; tare does not remove dynamic errors. [ADI datasheet, Rev. D, Table 1 and Applications Information](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

**ADC.** ADS131M08 provides eight simultaneous differential channels, 24-bit output and up to 32 ksample/s/channel. At gain 1 and OSR 128, TI reports **21.31 µV RMS input noise** and 16.8 effective bits. With ADXL354C at 100 mV/g, that contributes **0.002090 m/s² RMS**, versus **0.00001403 m/s² per nominal code step**. The regular filter is sinc³: with a 7.68 MHz master clock, OSR 128 gives **30 ksample/s**, and the calculated filter group delay is **49.609 µs**, before transport, latching and other filters. This delay follows (3(OSR-1)/(2f_\mathrm{MOD})), with (f_\mathrm{MOD}=f_\mathrm{CLK}/2). Startup settling is a separate constraint. Keep DC blocking off for static force measurement. [TI datasheet, Rev. B, Tables 7-1/7-2 and §8.3.7](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

## Alternatives and recommendation

The [ADXL354B chip](https://www.digikey.nl/en/products/detail/analog-devices-inc/ADXL354BEZ/6557312) currently has the same €61.71 price and only ±2g/±4g ranges, so it offers no saving here. ADI's [ADXL354C evaluation board](https://www.digikey.nl/en/products/detail/analog-devices-inc/EVAL-ADXL354CZ/6557294) is cheaper (€45.95 excluding VAT, 210 displayed in stock), but is a board-level prototyping alternative to the user's selected own-PCB scope. These are limited comparisons, not a market-wide cheapest-part claim.

**Retain ADXL354C at ±8g and ADS131M08 at 30 ksample/s as the budget candidate.** Use one ADC for the planar test configuration; use two synchronized ADCs for all three acceleration axes. The chip budget is verified. Control performance must come from the new native recordings with this acquisition response, range, noise and calibration—not from price or nominal bit depth. The modeled 1.9 kHz envelope still requires assembled sensor-response and sanding-vibration measurements before prototype qualification.
