from pathlib import Path
import argparse
import hashlib
import json
import math
import resource

import numpy as np
from scipy.io import loadmat

ap=argparse.ArgumentParser()
ap.add_argument('report')
ap.add_argument('case_id',type=int)
ap.add_argument('family')
args=ap.parse_args()
V=Path('/home/tommie/Projects/actuator-model/acf_simulink/validation')
report_path=Path(args.report)
report=json.loads(report_path.read_text())
row=next(x for x in report['rows'] if int(x.get('id',x.get('caseIndex')))==args.case_id)
trace_path=Path(row.get('nativeTrace',row.get('nativeTraceFile')))
out=V/f'normal_voltage_calibrated_static_headroom_20260930_{args.family}_case_{args.case_id:04d}.json'
assert not out.exists(),'Preserve existing pointwise-audit evidence.'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def number(x):return float(np.array(x).squeeze())
def range_of(x):return {'minimum':float(np.min(x)),'maximum':float(np.max(x)),
                       'peakAbsolute':float(np.max(abs(x))),'rms':float(np.sqrt(np.mean(x*x)))}

# Each invocation owns exactly one retained native history and exits afterward.
# This intentionally avoids simultaneous3-million-sample allocations.
data=loadmat(trace_path,variable_names=['r'],simplify_cells=True)['r']
p=data['parameters']
d=data['data'];t=np.asarray(data['time']).ravel();n=data['normalContact']['data']
assert d.shape==(len(t),35) and n.shape==(len(t),8)
if 'nativeSamples' in row:assert len(t)==int(row['nativeSamples'])
else:assert args.family=='mixed' and len(t)==400001
assert np.all(np.isfinite(d)) and np.all(np.isfinite(n))
assert np.array_equal(t,np.asarray(data['normalContact']['time']).ravel())
assert t[0]==0 and abs(t[-1]-number(p['sim']['stopTime']))<1e-9

Kactual=number(p['coil']['forceConstant'])
Rgeneric=number(p['coil']['resistance'])
Lgeneric=number(p['coil']['inductance'])
tau=number(p['coil']['currentResponse'])
Ilimit=number(p['coil']['peakCurrent'])
Vlimit=number(p['coil']['voltage'])
Kidentified=number(p['uncertainty']['forceConstant_model_N_A'])
Kinverse=number(p['inertial']['lowerLoadInverse']['identifiedForceConstant_N_A'])
assert Kactual==20 and Rgeneric==4 and Lgeneric==.003 and tau==.0001
assert Ilimit==3.5 and Vlimit==24 and number(p['mechanics']['upperRigid'])==1
assert abs(Kinverse-Kidentified)<1e-12

i=d[:,13];u=d[:,32];v=d[:,2];Vactual=d[:,14]
applied=np.clip(u,-Ilimit,Ilimit)
tracking=applied-i
Vrequested=(Lgeneric/tau)*tracking+Rgeneric*i+Kactual*v
generic_voltage_residual=float(np.max(abs(Vactual-Vrequested)))
effort_clipped=int(np.count_nonzero(d[:,21]))
voltage_clipped=int(np.count_nonzero(d[:,22]))
if 'voltageClipFraction' in row:
    assert float(row['voltageClipFraction'])==float(row['effortClipFraction'])==0
assert voltage_clipped==effort_clipped==0
assert generic_voltage_residual<1e-8
assert np.max(abs(u))<=Ilimit+1e-10
assert np.max(abs(Vrequested))<=Vlimit+1e-10

scale=20/24.4
Kreal=Kactual/scale
KrealIdentified=Kidentified/scale
Lreal=.004;Rreal=8.896
alpha=Lreal*scale/Lgeneric
beta=Rreal*scale-alpha*Rgeneric
gamma=Kreal-alpha*Kactual
Vhot=alpha*Vrequested+beta*i+gamma*v
Vhot_direct=(Lreal/tau)*(scale*tracking)+Rreal*(scale*i)+Kreal*v
map_residual=float(np.max(abs(Vhot-Vhot_direct)))
assert map_residual<1e-10
assert abs((KrealIdentified/Kreal)-(Kidentified/Kactual))<1e-14
assert number(p['inertial']['forceKp'])==.3 and number(p['inertial']['forceKi'])==20
assert not number(p['inertial']['currentNotch']['enabled'])

reserve=31-np.abs(Vhot)
peak_index=int(np.argmax(abs(Vhot)))
normal_error=n[:,0]-number(p['force']['reference'])
normal_peak=float(np.max(abs(normal_error)))
assert abs(normal_peak-float(row['peakNormalForceError_N']))<1e-10
assert normal_peak<=.5 and bool(row['passed']) and bool(row['completed'])
command_real=scale*u;current_real=scale*i
predicted_clip_count=int(np.count_nonzero(abs(Vhot)>31))
calibration={
    'physicalKfGeneric_N_A':Kactual,'identifiedKfGeneric_N_A':Kidentified,
    'identifiedInverseKfGeneric_N_A':Kinverse,'physicalKfReal_N_A':Kreal,
    'identifiedKfReal_N_A':KrealIdentified,'identifiedInverseKfReal_N_A':Kinverse/scale,
    'identifiedToPhysicalKfRatioUnchanged':True,
    'identifiedToPhysicalKfRatio':Kidentified/Kactual,
    'uniformCurrentScale':scale,
    'forceKpGeneric_A_N':number(p['inertial']['forceKp']),
    'forceKiGeneric_A_Ns':number(p['inertial']['forceKi']),
    'forceKpReal_A_N':scale*number(p['inertial']['forceKp']),
    'forceKiReal_A_Ns':scale*number(p['inertial']['forceKi']),
    'forceKdReal_A_s_N':scale*number(p['inertial']['forceKd']),
    'massSActual_kg':number(p['mechanics']['mS']),
    'massSIdentified_kg':number(p['sensor']['distalMass_model_kg']),
    'massBActual_kg':number(p['mechanics']['mB']),
    'massBIdentified_kg':number(p['uncertainty']['massB_model_kg']),
    'lowerStiffnessActual_N_m':number(p['mechanics']['k1']),
    'lowerDampingActual_Ns_m':number(p['mechanics']['c1']),
    'lowerStiffnessIdentified_N_m':number(p['inertial']['lowerLoadInverse']['identifiedStiffness_N_m']),
    'lowerDampingIdentified_Ns_m':number(p['inertial']['lowerLoadInverse']['identifiedDamping_Ns_m']),
    'allSensorRobotGuideAndMechanicalCalibrationResidualsRetained':True,
    'initialActualGenericCurrent_A':float(i[0]),'initialActualRealCurrent_A':float(scale*i[0]),
    'initialGenericCommand_A':float(u[0]),'initialRealCommand_A':float(scale*u[0]),
    'initialNormalForceError_N':float(normal_error[0]),
}
source_paths=[report_path,trace_path]
for k in ['sourceModel','candidateFile','experimentSnapshot','privateModel','preparedConfigurationsFile','preparedConfigurationFile']:
    if k in report and Path(report[k]).exists():source_paths.append(Path(report[k]))

result={
    'date':'2026-09-30','status':'complete_pointwise_algebraic_screen','family':args.family,
    'nativeCaseId':args.case_id,'nativeSourceRow':row,
    'noNewNativeSimulation':True,
    'counterfactual':'Real known physicalKf24.4N/A, hotR8.896ohm,L4mH,currentresponse100us. Uniform20/24.4 current scaling preserves the original residualKf identification; all other recorded sensor, robot, guide and mechanics calibration remains fixed.',
    'nativeSamples':len(t),'timeRange_s':[float(t[0]),float(t[-1])],
    'loadedStartupIncluded':True,'genericCompleted':True,'genericNormalPeakError_N':normal_peak,
    'genericVoltageRange_V':range_of(Vactual),'genericRequestedVoltageRange_V':range_of(Vrequested),
    'genericPhysicalCurrentRange_A':range_of(i),'genericCurrentCommandRange_A':range_of(u),
    'genericAppliedCurrentCommandRange_A':range_of(applied),'genericCurrentTrackingErrorRange_A':range_of(tracking),
    'gapVelocityRange_m_s':range_of(v),
    'genericVoltageClippingSamples':voltage_clipped,'genericEffortClippingSamples':effort_clipped,
    'genericRequestedVsAppliedVoltageResidual_V':generic_voltage_residual,
    'mappedHotRequestedVoltageRange_V':range_of(Vhot),
    'mappedRealPhysicalCurrentRange_A':range_of(current_real),
    'mappedRealCurrentCommandRange_A':range_of(command_real),
    'mappedHotRequestedVsDirectRegulatorResidual_V':map_residual,
    'minimumReserveAt31V_V':float(np.min(reserve)),
    'peakMappedRequestedVoltageTime_s':float(t[peak_index]),
    'mappedHotVoltageComponentsAtPeak_V':{'currentRegulator':float((Lreal/tau)*scale*tracking[peak_index]),
                                         'resistance':float(Rreal*scale*i[peak_index]),
                                         'backEMF':float(Kreal*v[peak_index])},
    'predicted31VClippingSamples':predicted_clip_count,
    'predicted31VClippingFraction':predicted_clip_count/len(t),
    'mappedCurrentCommandWithin3p5A':bool(np.max(abs(command_real))<=3.5),
    'conditional31VHeadroomSupported':predicted_clip_count==0,
    'calibrationPreservation':calibration,
    'mapping':{'equation':'Vhot=alpha*VgenericRequested+beta*Igeneric+gamma*vGap',
               'alpha':alpha,'beta_ohm':beta,'gamma_V_s_m':gamma,
               'currentResponse_s':tau,'realPhysicalR_ohm':Rreal,'realPhysicalL_H':Lreal,
               'residualIdentifiedKfScaledRatherThanCorrected':True},
    'assumptions':[
        'Pointwise requested-voltage algebra only, not a real-coil native simulation or hardware qualification.',
        'Generic requested voltage matches applied voltage because both native clipping flags are zero; the logged regulator uses the same limited current command.',
        'All active current-unit feedback gains, holding/acceleration/guide feedforward and proper inverse current outputs, initial current and identifiedKf must use the uniform force-current transformation. Residual identified-to-physicalKf error is preserved.',
        'The real winding has matched hotR/L/back-EMF compensation and the same100us current response. The modeled amplifier realizes the mapped terminal voltage/current without added phase, ripple or clipping.',
        'Same upper-rigid prescribed robot-side kinematics and all original sensor, robot, guide and mechanical calibration. Actual finite robot impedance is not qualified.',
        'HotRfactor1.39 corresponds to an assumed100degreeC copper rise. Temperature-dependentKf, Kf-versus-stroke, physical thermal response and safe supplier operating temperature remain unqualified.',
        '31V usable output is a terminal-voltage envelope, not a guaranteed output specification from a36V supply.'
    ],
    'sources':[{'file':str(x),'sha256':sha(x),'sizeBytes':x.stat().st_size} for x in dict.fromkeys(source_paths)],
    'sourceModelSha256':report.get('sourceModelSha256',report.get('sourceModelSHA256',report.get('modelSha256'))),
    'privateModelSha256':report.get('privateModelSha256'),
    'candidateSha256':report.get('candidateSha256'),
    'recipeSha256':report.get('experimentSha256',report.get('recipeSHA256')),
    'helperSources':report.get('helperSources',[]),
    'peakResidentMemory_kB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    'memoryScope':'One complete retained MAT history in this short-lived process; process exit releases every3-million-sample array before the next file is loaded.'
}
assert all(math.isfinite(result[k]) for k in ['minimumReserveAt31V_V','genericNormalPeakError_N'])
out.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'file':str(out),'case':args.case_id,'family':args.family,
                  'samples':len(t),'error_N':normal_peak,
                  'hotRequestedPeak_V':result['mappedHotRequestedVoltageRange_V']['peakAbsolute'],
                  'reserve31_V':result['minimumReserveAt31V_V'],
                  'predictedClipping':predicted_clip_count,'peakResidentGiB':result['peakResidentMemory_kB']/1024**2}))
