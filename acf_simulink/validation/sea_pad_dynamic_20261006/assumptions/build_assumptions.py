"""Bounded, source-bound sizing assumptions; does not run a simulation."""
import hashlib
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
sources = [
    ("re35", "RE35 273757", "https://www.maxongroup.com/maxon/view/product/motor/dcmotor/re/re35/273757"),
    ("ec_i30", "EC-i30 539489", "https://www.maxongroup.com/maxon/view/product/539489"),
    ("re25", "RE25 302005", "https://www.maxongroup.com/maxon/view/product/302005"),
    ("motor_simulation", "Motor data and simulation", "https://support.maxongroup.com/hc/en-us/articles/360013761160-Motor-data-and-simulation"),
    ("commutation", "Strength of BLDC EC motor with sinusoidal commutation", "https://support.maxongroup.com/hc/en-us/articles/360006288554-Strength-of-BLDC-EC-motor-with-sinusoidal-commutation"),
    ("ec_catalogue", "EC-i30 catalogue page305 March2025", "https://www.maxongroup.com/medias/sys_master/root/9402695286814/Cataloge-Page-EN-305.pdf"),
    ("mirka_5mm", "Interface77mm6holes Grip", "https://www.mirka.com/cs-CZ/p/Interface-77---6-GRIP/?variant=94118"),
    ("mirka_10mm", "Interface Pad3inch Grip", "https://www.mirka.com/en-ca/p/Interface-77---NONE-GRIP/"),
    ("mirka_catalogue", "Tools for Sanding and Polishing brochure", "https://www.mirka.com/SharePoint/Marketing/Communications/WebContent/MSF_Printed_material/Tools%20for%20Sanding%20and%20Polishing/Tools%20for%20Sanding%20and%20Polishing%20brochure%20English.pdf"),
    ("thk", "Ball-screw conversion efficiency FAQ", "https://www.thk.com/jp/en/products/faq/ball_screw/-/0089/modal.html"),
    ("poron_general", "PORON4701-30 Very Soft publication17-006, July2023", "https://www.rogerscorp.com/-/media/project/rogerscorp/documents/elastomeric-material-solutions/poron/english/data-sheets/17-006-poron-4701-30-very-soft.pdf"),
    ("poron_curve", "PORON4701-30 ideal CFD battery-pad curves publication17-388, July2023", "https://www.rogerscorp.com/-/media/project/rogerscorp/documents/elastomeric-material-solutions/poron/english/data-sheets/17-388-ideal-cfd-curves-for-battery-pad-applications-poron-4701-30-very-soft.pdf"),
]
source_records = [dict(id=i, title=t, url=u, readDate="2026-10-06", method="Official manufacturer source inspected with web tool") for i, t, u in sources]
pdf = HERE / "maxon-ec-i30-539489-catalogue-2025.pdf"
source_records[5].update(file=pdf.name, sha256=hashlib.sha256(pdf.read_bytes()).hexdigest())

motors = [
    dict(id="RE35_273757", sourceId="re35", architecture="brushed coreless DC", role="adopted primary diagnostic motor", catalogueStatus="NRND; motor-sized reference, not purchasing recommendation", nominalVoltage_V=48, Kt_Nm_A=.0758, Ke_V_s_rad=.0758, R25_ohm=4.68, L1kHz_H=.00129, Jrot_kg_m2=6.54e-6, mass_kg=.340, continuousTorque_Nm=.104, continuousCurrent_A=1.42, noLoadCurrent_A=.046, noLoadSpeed_rpm=5990, nominalSpeed_rpm=5160, maxSpeed_rpm=12000, maxWindingTemperature_C=155, RthHousingAmbient_K_W=6.2, RthWindingHousing_K_W=2, windingTimeConstant_s=27.2, motorTimeConstant_s=707, currentConvention="Actual two-terminal armature current"),
    dict(id="ECi30_539489", sourceId="ec_i30", architecture="three-phase iron-wound brushless, Hall sensors, four pole pairs", role="source-bound future low-inertia option; not executed adopted plant", nominalVoltage_V=48, Kt_Nm_A=.0554, equivalentKe_V_s_rad=.0554, R25_ohm=1.39, L1kHz_H=.00146, Jrot_kg_m2=1.55e-6, mass_kg=.242, continuousTorque_Nm=.103, continuousCurrent_A=1.82, noLoadCurrent_A=.119, noLoadSpeed_rpm=8210, nominalSpeed_rpm=7150, maxSpeed_rpm=10000, maxWindingTemperature_C=155, RthHousingAmbient_K_W=9.01, RthWindingHousing_K_W=2.46, windingTimeConstant_s=30.9, motorTimeConstant_s=1090, currentConvention="Catalogue block current in each commutation interval; Kt gives average block torque; R/L are phase-to-phase", linkedCatalogueDifferences=dict(R25_ohm=1.37, continuousTorque_Nm=.104, continuousCurrent_A=1.84, windingTimeConstant_s=31.3, sourceId="ec_catalogue", selection="Use live-page values consistently; do not mix revisions")),
    dict(id="RE25_302005", sourceId="re25", architecture="brushed coreless DC", role="historical smaller comparison only", catalogueStatus="NRND reference", nominalVoltage_V=24, Kt_Nm_A=.0234, Ke_V_s_rad=.0234, R25_ohm=2.32, L1kHz_H=.000238, Jrot_kg_m2=1.08e-6, mass_kg=.130, continuousTorque_Nm=.0263, continuousCurrent_A=1.16, noLoadCurrent_A=.0369, noLoadSpeed_rpm=9560, maxWindingTemperature_C=125),
]
sizing = []
for motor in motors:
    for lead in (.005, .010):
        ell = lead / (2 * math.pi)
        current = 50 * ell / motor["Kt_Nm_A"]
        sizing.append(dict(motorId=motor["id"], lead_mm_rev=1000*lead, ell_m_rad=ell, totalRotaryInertia_kg_m2=motor["Jrot_kg_m2"]+.5e-6, reflectedInertiaAtNut_kg=(motor["Jrot_kg_m2"]+.5e-6)/ell**2, ideal50NCurrent_A=current, holding50NCurrent90PercentSizing_A=current/.9, continuousAxialForce90PercentSizing_N=.9*motor["continuousTorque_Nm"]/ell, static50NCurrentPassBeforeExtraLoss=current/.9<=motor["continuousCurrent_A"], static50NTorquePassBeforeExtraLoss=50*ell/.9<=motor["continuousTorque_Nm"], cold50NCopper_W=(current/.9)**2*motor["R25_ohm"]))

pads = []
for stiffness in (20000, 50000, 100000):
    thickness = .010 if stiffness == 20000 else .005
    pads.append(dict(kEffective_N_m=stiffness, thickness_m=thickness, compressionAt5N_m=5/stiffness, compressionAt50N_m=50/stiffness, maximumWorkingCompression_m=.5*thickness, fractionAt50N=50/stiffness/thickness, flat50NWithinWorkingRange=50/stiffness<=.5*thickness, dampingSensitivity_N_s_m=[0, 40, 80, 150]))

spring_rows = []
for stiffness in (3500, 5000, 10000):
    each = stiffness/2
    spring_rows.append(dict(netRate_N_m=stiffness, individualRate_N_m=each, individualFreeLength_m=.050, nominalInstalledLength_m=.025, assumedMinInstalledLength_m=.008, assumedMaxInstalledLength_m=.042, combinedFreeLength_m=.100, individualNominalPreload_N=each*.025, signedTravelBookkeeping_m=.017, signedNetForceAtLimit_N=stiffness*.017, individualForceRangeAtLimit_N=[each*.008, each*.042], actualSpringGeometryAndStressQualified=False))

shaft_mass = 7850 * math.pi * (.008/2)**2 * .100
data = dict(
    schemaVersion=1, readDate="2026-10-06", status="source_bound_engineering_assumptions_no_simulation", nativeExperiments=0, physicalMeasurements=0,
    scope="First-prototype total axial pressing force from one-axis B-S cell plus S acceleration; sanding drag ignored; no local-pressure objective or results",
    sources=source_records, motors=motors, staticMotorSizing=sizing,
    recommendation="Use adequately sized RE35 terminal-DC motor for adopted SEA diagnostic. Keep explicit independent rotor/nut states. EC-i30 is an unexecuted future option; RE25 is an undersized historical comparator.",
    drive=dict(assumedPeakCurrentLimit_A=3.5, assumedUsableBipolarTerminalVoltage_V=48, assumedCurrentResponseTarget_s=.000050,
        peakStatus="3.5A is an assumed bounded driver limit, not motor continuous rating or established allowable peak duration",
        responseStatus="Servo target subject to computed L*di/dt voltage clipping and installed measurement; not passive L/R",
        hotResistanceIfUsed="Rhot=R25*(1+0.0039*100) is an assumed100C temperature rise, not a measured hot rating",
        heating="Report measured model RMS current and copper loss separately from short transient force passes; continuous torque/current ratings do not validate this mounting/cooling",
        inductance="Catalogue stationary1kHz measurement. Manufacturer says effective PWM inductance may be30–80percent of catalogue. Use catalogueL as a declared slew assumption, not identified driver phase response",
        ecCaveat="EC catalogue constants/current apply to average block commutation. Supply current and FOC sinusoidal phase amplitude differ; no ripple/cogging/commutation model is established by the equivalent DC equations"),
    screw=dict(transmission="Efficient reversible ball-screw reference with no gearbox; generic sliding/trapezoidal leadscrew needs a separately identified loss/self-locking model",
        leadSweep_mm_rev=[5, 10], efficiencyRole="90percent is static sizing only, supported by THK ball-screw FAQ. Native dynamics should use reversible pitch mapping and explicit passive torque/guide loss, not a sign-independent efficiency multiplier",
        assumedScrewCouplerInertia_kg_m2=.5e-6,
        geometrySanity=dict(shaftDiameter_m=.008, shaftLength_m=.100, steelDensityAssumed_kg_m3=7850, shaftMass_kg=shaft_mass, shaftPolarInertia_kg_m2=.5*shaft_mass*(.008/2)**2, status="Order-of-magnitude check, not purchased screw inertia; actual root/journals/length/coupling change it"),
        assumedAdditionalNutGuidedCarriageMass_kg=.075, carriageRange_kg=[.05, .10], carriageStatus="Explicit added translating mass, not measured. Do not hide it in B or add rotary inertia to B",
        bearings="Use dedicated axial screw-support/thrust bearings; small motor-shaft axial ratings are below50N",
        losses="No-load current at high catalog speed identifies combined loss only. Coulomb/viscous/stiction split at our lower speed remains an engineering assumption pending identification",
        bodyMass="Motor belongs in A. Matched A=1.5kg must include stated motor and remaining mount/ballast, not silently assume screw has same body mass as coil"),
    pad=dict(diameter_m=.077, geometryReferences=[dict(sourceId="mirka_5mm", part="8294700111", diameter_m=.077, thickness_m=.005, holes=6), dict(sourceId="mirka_10mm", part="1033", diameter_m=.077, thickness_m=.010, holes=0)], rows=pads,
        mechanicalPropertyStatus="Whole-pad equivalent k/c sweep is representative engineering sensitivity, NOT measured Mirka stiffness/damping. Reviewed manufacturer pages list geometry/use, no dynamic k/c",
        materialPlausibility=dict(sourceId="poron_general", material="PORON4701-30 Very Soft,240kg/m3 nominal density grade; not Mirka pad material", manufacturerStockThicknessRange_m=[.00478, .0127], compressionForceDeflectionRangeAt25Percent_kPa=[7, 35], testCrossheadSpeed_m_min=.0051, diskDiameterAssumption_m=.077, fullUnperforatedDiskArea_m2=math.pi*.077**2/4,
            conversion="k_secant = manufacturer bulk compression stress * assumed disk area / (compression fraction * assumed thickness)",
            secantKAt25Percent_N_mm=[dict(thickness_m=t, low=7000*(math.pi*.077**2/4)/(.25*t)/1000, high=35000*(math.pi*.077**2/4)/(.25*t)/1000) for t in [.005, .010]],
            limits="Quasistatic bulk-material plausibility only, not tangent/dynamic stiffness, manufacturer sanding-pad specification, local-pressure analysis or a guarantee across amplitude/frequency"),
        nonlinearityExample=dict(sourceId="poron_curve", density_kg_m3=240, stockThicknessRange_m=[.0013,.0030], compressionFractions=[.25,.50,.60,.70], bulkCFD_kPa=[21.6,42.1,68.3,149.0], status="Different thin battery-pad construction; illustrative material nonlinearity, not exact5/10mm sanding pad", limits="Datasheet typical values are not specification limits; no dynamic damping coefficient is identified"),
        damping="Every nonzero c remains unvalidated material sensitivity; c=0 is a lossless counterfactual. Fit installed frequency/amplitude-dependent response before prototype claims",
        compressionGuard="Stop/flag validity beyond50percent thickness. A positive force from extrapolated linear elasticity is not a valid pad beyond its declared working range",
        footprint="Sum only compressive local-contact contributions before reporting total axial force. Clipping an already averaged height is not equivalent in partial contact. Do not publish local-pressure scores",
        causality="Plant surface truth is unavailable to controller; controller inputs are acquired force, S acceleration, relative position and already-issued robot commands",
        omissions=["Nonlinear compression/hysteresis/creep/temperature/aging", "Holes, hook-and-loop, abrasive and local workpiece compliance", "Pad orbit/spin, distributed inertia and flexural modes", "Unknown sanding drag deferred by prototype scope"]),
    internalSEASprings=dict(purpose="Opposed pair free-length/force bookkeeping only; not a qualified coil-spring design", upperSpringChanged=False, externalB_SChanged=False, assumedDamping_N_s_m=0, rows=spring_rows,
        verification="Check actual solid height, wire/diameter/turn count, stress/fatigue, buckling guidance, seats and frame flexibility.50mm free and8mm minimum do not establish manufacturability at these loads",
        topology="Internal nut-to-B spring, not prescribed R-A upper spring or external B-S connection; rotor inertia acts at independent nut coordinate"),
    storage=dict(retainedBytesUpperBound=200000, temporaryBytesUpperBound=300000, newExperimentArrayBytes=0, sharedArrayBudget_GiB=5, freeSpaceChecked="df -h . showed362GiB available before writing on2026-10-06; root audits shared arrays", owner="sea_pad_assumptions")
)
destination = HERE / "source_bound_assumptions.json"
destination.write_text(json.dumps(data, indent=2)+"\n")
owned = sum(p.stat().st_size for p in HERE.iterdir() if p.is_file())
assert owned <= 200000, owned
assert all(row["flat50NWithinWorkingRange"] for row in pads)
assert all(abs(m["Ke_V_s_rad"]-m["Kt_Nm_A"])<1e-12 for m in motors if "Ke_V_s_rad" in m)
print(json.dumps(dict(status="complete", ownedBytes=owned, jsonBytes=destination.stat().st_size, simulationRuns=0, arraysCreatedBytes=0, re35Sizing=[r for r in sizing if r["motorId"]=="RE35_273757"])))
