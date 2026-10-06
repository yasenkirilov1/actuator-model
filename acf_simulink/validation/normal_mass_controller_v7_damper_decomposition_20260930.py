"""Read-only paired native analysis of the external actuator-damper screen."""
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.io import loadmat
from scipy.signal import lfilter


FOLDER = Path(__file__).resolve().parent
STEM = Path(__file__).with_suffix("")
SOURCES = [
    ("intrinsic_b2", FOLDER / "normal_mass_controller_inertial_repeated_bump_confirmation_20260930_v7_common_provisional_worst_native_trace.mat"),
    ("external_added118_total120", FOLDER / "normal_mass_controller_inertial_parallel_actuator_damper_20260930_v7_100us_transport_traces/controller_04_worst_native.mat"),
]
rows = []
for label, source in SOURCES:
    saved = loadmat(source, simplify_cells=True)
    r = saved["r"] if "r" in saved else saved["worst"]
    p = r["parameters"]
    t, d = r["time"], r["data"]
    n, imu = r["normalContact"]["data"], r["accelerometer"]["data"]
    dt = p["force"]["sampleTime"]
    stride = round(dt / p["sim"]["step"])
    take = np.arange(0, len(t), stride)
    held = lambda value: np.repeat(value, stride)[:len(t)]
    kf = p["coil"]["forceConstant"]
    mass = p["mechanics"]["mB"] + p["mechanics"]["mS"]
    assert not p["inertial"]["currentNotch"]["enabled"] and not np.any(d[:, 21] > .5)
    error_measured = n[take, 3] - p["force"]["reference"]
    integral = np.r_[0., np.cumsum(error_measured[:-1]) * dt]
    unfiltered_pi = -p["inertial"]["forceKp"] * error_measured - p["inertial"]["forceKi"] * integral
    feedback = p["d"]["feedbackFilter"]
    pi = held(lfilter(feedback["numerator"], feedback["denominator"], unfiltered_pi))
    a = np.exp(-dt / p["sensor"]["filterTime"])
    robot_a = lfilter([1-a], [1, -a], r["commandPrediction"]["data"][take, 2])
    robot_a = np.r_[0., robot_a[:-1]]
    predicted_gap_v = held(r["motionObserver"]["data"][take, 1] +
                           p["inertial"]["guideVelocityPrediction_s"] * (robot_a - imu[take, 0]))
    friction = lambda velocity: p["coil"]["viscousFriction"] * velocity + p["coil"]["coulombForce"] * np.tanh(velocity / p["coil"]["frictionSpeed"])
    ff_az = r["accelerationFeedforward"]["data"][:, 2]
    command = p["d"]["coilHoldCurrent"] + pi - mass * ff_az/kf + (n[:, 5]-p["force"]["reference"])/kf + friction(predicted_gap_v)/kf
    command_residual = float(np.max(np.abs(command-d[:, 32])))
    assert command_residual < 1e-11
    g = np.sqrt(1+n[:, 2]**2)/(1-p["contact"]["mu"]*n[:, 2])
    error = n[:, 0]-p["force"]["reference"]
    terms = {
        "currentTracking_N": g*kf*(d[:, 13]-d[:, 32]),
        "accelerationFeedforwardError_N": g*mass*(imu[:, 2]-ff_az),
        "normalProjectionFeedforward_N": g*(n[:, 5]-p["force"]["reference"]/g),
        "guideFeedforwardError_N": g*(friction(predicted_gap_v)-friction(d[:, 2])),
        "filteredPIcorrection_N": g*kf*pi,
        "finiteCellInertia_N": g*p["mechanics"]["mB"]*(d[:, 11]-imu[:, 2]),
    }
    residual = float(np.max(np.abs(sum(terms.values())-error)))
    assert residual < 1e-10
    peak = int(np.argmax(np.abs(error)))
    phase = (t-p["surface"]["startTime"]) % p["surface"]["repeatPeriod"]
    flat = ((t>.25)&(t<p["surface"]["startTime"]-.05)) | ((t>=p["surface"]["startTime"])&(phase>p["d"]["duration"]+.2)&(phase<p["surface"]["repeatPeriod"]-.05))
    window = np.abs(t-t[peak])<=.025
    columns = {"time_s":t,"normalError_N":error,**terms,
               "physicalBAcceleration_m_s2":d[:, 11],"physicalSAcceleration_m_s2":imu[:, 2],
               "measuredSAcceleration_m_s2":imu[:, 0],"finalFFAcceleration_m_s2":ff_az,
               "physicalCoilGapVelocity_m_s":d[:, 2],"predictedGuideVelocity_m_s":predicted_gap_v}
    excerpt = STEM.with_name(STEM.name+"_"+label).with_suffix(".csv")
    np.savetxt(excerpt,np.column_stack([v[window] for v in columns.values()]),delimiter=",",header=",".join(columns),comments="")
    rows.append({"label":label,"sourceTrace":str(source),"sourceTraceSha256":hashlib.sha256(source.read_bytes()).hexdigest(),
                 "massS_kg":p["mechanics"]["mS"],"seed":p["sensor"]["seed"],"totalViscous_Ns_m":p["coil"]["viscousFriction"],
                 "nativeSamples":len(t),"normalPeakError_N":float(error[peak]),"peakTime_s":float(t[peak]),
                 "termsAtPeak_N":{key:float(values[peak]) for key,values in terms.items()},
                 "settledFlatRmsError_N":float(np.sqrt(np.mean(error[flat]**2))),
                 "settledFlatRmsCellInertia_N":float(np.sqrt(np.mean(terms["finiteCellInertia_N"][flat]**2))),
                 "commandResidual_A":command_residual,"normalDecompositionResidual_N":residual,"excerpt":str(excerpt)})

report = {"status":"complete_read_only_analysis","analysisScript":str(Path(__file__).resolve()),
          "analysisSha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          "scope":"Two paired native ten-second histories, sameS2/F50/90deg/friction.3/seed1729. Physical truth is diagnostic only; zero new simulations.",
          "rows":rows,"interpretation":"Reasonable matched externalA-Bdamping reduces flat modal noise slightly but does not remove the submillisecond bump-entry acceleration/current phase error. This is distinct from seriesB-Sjoint damping or physically changing the measured acquisition response."}
assert rows[0]["seed"]==rows[1]["seed"] and rows[0]["massS_kg"]==rows[1]["massS_kg"]
STEM.with_suffix(".json").write_text(json.dumps(report,indent=2)+"\n")
print(json.dumps(rows,indent=2))
