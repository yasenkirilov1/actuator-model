# Actuator answers for the readable report

These results support a **voice-coil development prototype**, with the user's
20 N/mm undamped upper spring and rigid lower connection. They do not establish
an accepted ±0.5 N system. No controller was tuned, model changed or new
simulation run for this evidence package.

## What does the bump require?

The bump takes **100 ms** to cross. The exact smooth profile makes the tool
move 1 mm, reaching **34.35 mm/s** and **2.4 m/s²**. For a 2 kg tool, that
acceleration contributes **4.8 N** to the load-cell reading. The controller
must distinguish it from the force on the workpiece. These values describe
the tool following the surface; the required actuator travel also includes
the body moving on the upper spring and robot recentering.

Use `sharp-bump-motion-demand.svg`. Its three aligned panels show why "only a
1 mm bump" still creates a meaningful dynamic force.

## Why prefer a voice coil to the studied leadscrew?

A voice coil produces linear force directly: changing current changes force,
and reversing current reverses force. It has no screw thread or gearbox to
accelerate or reverse. That suits a controller correcting pressing force
while the tool moves up and down. It still needs good guidance: bearing drag,
cables and the load cell can introduce friction and vibration.

A screw trades mechanical advantage against inertia. A fine lead gives more
force per motor ampere, but the shaft must turn farther for each millimetre of
tool motion. The small RE25 motor plus assumed screw/coupler has only
**1.58×10⁻⁶ kg·m²** of rotary inertia. With a **3 mm/revolution** screw, its
acceleration nevertheless requires the same extra axial force as accelerating
**6.93 kg**. At **10 mm/revolution**, that equivalent mass falls to **0.624 kg**,
but holding 50 N requires **3.40 A** before losses. The motor is rated for only
**1.16 A continuous**. This "extra mass" is not physical weight on the flange:
it expresses the force needed to speed up and slow down the shaft.

Use `leadscrew-pitch-inertia-current.svg` to show the tradeoff. The displayed
holding currents are ideal; a separate 90% efficiency sizing assumption raises
them by 11%. At 3 mm lead, the catalog continuous torque gives only 49.57 N
under that efficiency assumption, before guide losses.

The experiments support a qualified preference rather than a prohibition.
Earlier matched native tests found that **both actuators passed all four sharp
bump conditions**, with nine accepted recordings including one screw refinement.
The screw violated its continuous holding-current rating in all four conditions.
Those tests had a **rigid upper attachment and a compliant, damped lower
connection**; they do not qualify today's fixed mechanics. A larger motor or a
different screw can be viable. A primary-source 103 mN·m motor example would
provide about 58.25 N at 10 mm lead and 90% assumed efficiency; this is static
sizing, not a simulation or complete screw recommendation.

Use `historical-voicecoil-leadscrew-errors.svg` if the report has room for the
experimental comparison; retain its prominent mechanics qualification.

## Which voice-coil parameters should we seek?

The following are design targets, not a claim that one catalog part satisfies
everything:

| Parameter | Target and reason |
|---|---|
| Stroke | About **25 mm**. Keep routine travel within the central **±7.5 mm** band; upper-body movement and delayed recentering consume travel. |
| Continuous force | **At least 55–60 N when hot, throughout the used stroke**, with the actual cooling/mounting. A cold center-force label is insufficient. |
| Peak force | About **85 N with our 3.5 A driver**. A motor's much larger catalog peak is unavailable unless the driver supplies the required current. |
| Force constant | Around **24.4–25 N/A**, or another winding giving equivalent force, current and heat. At 24.4 N/A, 50 N takes **2.05 A**. |
| Resistance / efficiency | Low hot resistance relative to force constant. The relevant thermal measure is **Kf/√R**, not Kf alone. Prefer around **10 N/√W or better at operating temperature**. |
| Inductance | The reference is **4 mH**. Together with resistance, current slew and motion-generated voltage, this determines required driver headroom. Lower inductance can help but needs separate identification. |
| Moving mass | Prefer about **240 g**, included inside **B = 600 g**, leaving roughly 360 g for the holder and other moving hardware. |
| Body mass | Include the stationary actuator body and mount in **A**. The 1.24 kg reference body is why the current tests use **A = 1.5 kg**. |
| Driver | **Bipolar 3.5 A**, adjustable and rated for **48 V**; aim for **≥36 V usable output** during development, and measure its fast current response. |

The [Moticont drawing](https://www.moticont.com/pdf/lvcm-070-051-01.pdf) is a real
reference for these mass, winding and stroke values. Its **64.7 N** continuous
rating is a **25°C reference**, with the plotted continuous force declining
toward roughly **52 N** near the ends. That curve does not establish an exact
force-constant map or installed hot rating. The drawing's 45 W continuous-power
reference also does not prove the 55–60 N hot target.

At the modeled hot resistance **8.896 Ω**, holding 50 N generates **37.36 W** of
copper heat. Holding 60 N generates **53.79 W**. Cooling and sustained-force
rating need separate qualification. The hot resistance is an engineering
100°C-rise assumption; it is not a measured prototype temperature. Raising
the bus voltage does not reduce this heat at the same current.

The coil and driver have different ratings. The coil catalog quotes 204.3 N
intermittently, which would take **8.37 A** at its nominal force constant.
Our **3.5 A** limit provides **85.4 N**, even with a higher-voltage driver.
Likewise, a **50 µs current-response target** belongs to the controlled
driver/coil system. The coil's passive L/R time constant is **625 µs cold**
and **450 µs** with the assumed hot resistance; the driver uses voltage
headroom to change current more quickly.

## Does higher voltage help?

The fixed-model MATLAB comparison held the controller, sensors and 50 µs
current response unchanged. In an unclipped nominal case, **32 V and 48 V were
bitwise identical**: peak demand was 23.76 V and peak force error 0.459 N.
Extra voltage is useful only when the original driver reaches its output limit.

In one clipped stress case, increasing usable output from 32 V to 48 V removed
2,386 clipped native samples and improved peak force error from
**2.748 N to 0.650 N**. The higher-voltage run needed **33.02 V**, not the full
48 V. It still failed ±0.5 N. This supports adjustable development headroom,
not a newly qualified 36/48 V design. Use
`fixed-mechanics-voltage-headroom.svg`: it shows force, current and output
voltage from the same recording interval.

The records are in `actuator_evidence_reviewed.json`, with full-history source hashes,
CSV calculation/figure rows, and `primary_sources_review_20261006.json`.
