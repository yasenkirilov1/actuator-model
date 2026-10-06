The final V8 solver check uses the nominal grid’s worst case: S = 2 kg, 50 N, 75°, μ = 0.3, seed 97, and the sharp 1 mm high / 10 mm wide bump at 100 mm/s. Every fine force sample is scored.

| Integration step | Native force samples | Peak normal-force error | Margin to 0.5 N |
|---|---:|---:|---:|
| 10 µs | 400,001 | 0.315810929066 N | 0.184189070934 N |
| 5 µs | 800,001 | 0.315822976214 N | 0.184177023786 N |
| 2.5 µs | 1,600,001 | 0.315825399325 N | 0.184174600675 N |

The 2.5 µs peak is 14.47 µN above the 10 µs result and 2.42 µN above the 5 µs result. At the original 10 µs timestamps, all normal-contact channels differ by at most 53 nN. The higher fine peak primarily resolves a time between coarser samples. Both fine runs pass force, stop checks, and fixed 33.6 N spring packing with no current or voltage clipping. The 68 physical and fast sensor/control channels use the full integration grid. Four robot-gate channels retain their 401-point, 10 ms grid; ready and stable decisions remain identical.

Only the integration step and three corresponding tick counts change. Actual 100 kHz acquisition, the 10 kHz controller, sensor noise, the 24 V / 100 µs current drive, mechanics, preload, and controller remain fixed. This provides numerical convergence for one worst case. Hardware and uncertain mechanical parameters require separate validation.

Three new native returns yield two qualified fine histories. The first 5 µs audit omitted four gate channels and had no durable trace. A second launcher rejected before simulation. The retained 5 µs return was recovered offline after correcting the gate-grid audit; no extra simulation was needed. The 2.5 µs run completed directly. Original logs and source snapshots are preserved, and every original 3,500-case grid artifact remains unchanged.
