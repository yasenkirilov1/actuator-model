"""Read-only estimator replay over saved native-Simulink trajectories.

No plant integration is performed. This does not qualify a new closed loop.
Force samples and issued commands are recorded native signals. Tool-to-robot
position measurements are constructed from the recorded native positions with
an explicitly stated simulated measurement noise. No surface/robot truth is
fed to the replay observer other than through that physical sensor channel.
"""
from pathlib import Path
import json
import numpy as np
from scipy.io import loadmat
from scipy.linalg import expm, solve_discrete_are

ROOT = Path(__file__).resolve().parents[2]
BASE = Path('/home/tommie/.local/state/actuator-model/qualification-20260930/robot_predictor_worker/traces')
OUT = Path(__file__).with_suffix('.json')


def recover_recorded_gap_measurement(r):
    """Invert logged observer correction to recover its actual sensor samples."""
    p = r['parameters']; c = p['d']['coilControl']; d = r['data']
    tt = r['robotPrediction']['estimatedTime']
    ix = np.rint(tt / p['sim']['step']).astype(int)
    states = r['robotPrediction']['estimatedStates']
    ar = r['robotPrediction']['data'][ix, 2]
    x = np.zeros(c['A'].shape[0]); du = c['D'][:, 1]
    ys = []; errors = []
    for k, i in enumerate(ix):
        yf = d[i, 31] - p['force']['reference']
        rest = np.zeros(9); rest[8] = ar[k]
        rem = states[k] - c['C'] @ x - c['D'][:, 0] * yf - rest
        yu = du @ rem / (du @ du)
        ys.append(yu); errors.append(np.max(abs(rem - du * yu)))
        fr = p['coil']['coulombForce'] * np.tanh(states[k, 5] / p['coil']['frictionSpeed'])
        inputs = [yf, yu, d[i, 32] - p['d']['coilHoldCurrent'], fr, ar[k]]
        x = c['A'] @ x + c['B'] @ inputs
    return np.array(ys), ix, max(errors)


def synthesize(p, jerk_order, process_power, force_sigma, position_sigma, position_kind,
               correct_direct_friction=False):
    m = p['mechanics']; drive = p['coil']; w = drive['control']
    ma, mb, ms = m['mA'], m['mB'], m['mS']; mu = ma * mb / (ma + mb)
    kf = drive['forceConstant']; b = drive['viscousFriction']
    ts = p['force']['sampleTime']; tau = drive['currentResponse']
    npstates = 10 if jerk_order else 9
    A = np.zeros((npstates, npstates)); Bu = np.zeros(npstates); Bf = np.zeros(npstates)
    A[0, 1] = 1; A[2, 3] = 1; A[4, 5] = 1
    A[1, [0, 1, 5, 6, 7]] = [-m['k1']/mb, -m['c1']/mb, -b/mb, kf/mb, 1]
    A[3, [2, 3, 5, 6, 8]] = [-m['k2']/ma, -m['c2']/ma, -b/ma, kf/ma, -1]
    A[5, [0, 1, 2, 3, 5, 6]] = [-m['k1']/mb, -m['c1']/mb, -m['k2']/ma, -m['c2']/ma, -b/mu, kf/mu]
    A[6, 6] = -1/tau; Bu[6] = 1/tau
    Bf[[1, 3, 5]] = [-1/mb, -1/ma, -1/mu]
    Cf = np.zeros(npstates); Cf[[0, 1, 7]] = [m['k1'], m['c1'], ms]
    if m['lowerRigid']:
        A[:2, :] = 0; A[0, 0] = -1000; A[1, 1] = -1000; Bf[1] = 0
        A[5, :] = 0; A[5, [2, 3, 5, 6, 7]] = [-m['k2']/ma, -m['c2']/ma, -b/ma, kf/ma, -1]
        Bf[5] = -1/ma; Cf[:] = 0; Cf[[5, 6, 7]] = [-b, kf, mb+ms]
    if m['upperRigid']:
        A[2:4, :] = 0; A[2, 2] = -1000; A[3, 3] = -1000; Bf[3] = 0
        A[5, :] = 0; A[5, [0, 1, 5, 6, 8]] = [-m['k1']/mb, -m['c1']/mb, -b/mb, kf/mb, 1]
        Bf[5] = -1/mb
    if m['lowerRigid'] and m['upperRigid']:
        A[5, :] = 0; A[5, [7, 8]] = [-1, 1]; Bf[5] = 0
    Br = A[:, 8].copy(); Br[8] = 0
    A[8, 8] = -1/p['robotPrediction']['residualTime']
    if jerk_order:
        A[7, 9] = 1
    B = np.column_stack([Bu, Bf, Br])
    Z = expm(np.block([[A, B], [np.zeros((3, npstates+3))]]) * ts)
    Ad, Bd = Z[:npstates, :npstates], Z[:npstates, npstates:]
    Qc = w['frictionPower'] * np.outer(Bf, Bf)
    Qc[9 if jerk_order else 7, 9 if jerk_order else 7] += process_power
    Qc[8, 8] += p['robotPrediction']['residualJerkPower']
    V = expm(np.block([[A, Qc], [np.zeros_like(A), -A.T]]) * ts)
    Qd = V[:npstates, npstates:] @ V[:npstates, :npstates].T
    delay = p['d']['sensorDelaySamples']; nf = max(delay-1, 0)
    ng = p['d']['strokeDelaySamples']; N = npstates + 1 + nf + ng
    Aa = np.zeros((N, N)); Aa[:npstates, :npstates] = Ad
    Ba = np.zeros((N, 3)); Ba[:npstates] = Bd
    Qa = np.zeros((N, N)); Qa[:npstates, :npstates] = Qd
    Cf_measured = Cf.copy(); Cf_measured[7] -= 1.4
    a = np.exp(-ts / p['sensor']['filterTime']) if p['sensor']['filterTime'] else 0
    row = np.zeros(N); row[:npstates] = (1-a)*Cf_measured; row[npstates] = a
    Aa[npstates] = row
    Dm = np.zeros((2, 3))
    force_direct_friction = float(correct_direct_friction and m['lowerRigid'])
    if force_direct_friction:
        Ba[npstates, 1] = -(1-a)
        if delay == 0:
            Dm[0, 1] = -(1-a)
    C = np.zeros((2, N)); C[0] = row if delay == 0 else np.eye(N)[npstates+nf]
    for j in range(nf): Aa[npstates+1+j, npstates+j] = 1
    Cp = np.zeros(npstates)
    if position_kind == 'tool_robot': Cp[[0, 2, 4]] = [1, 1, -1]
    else: Cp[4] = 1
    first = npstates+1+nf
    if ng:
        Aa[first, :npstates] = Cp
        for j in range(1, ng): Aa[first+j, first+j-1] = 1
        C[1, first+ng-1] = 1
    else: C[1, :npstates] = Cp
    scales = [1e-3, .1, 1e-3, .1, 1e-3, .1, 1, 10, 10]
    if jerk_order: scales += [1e3]
    scales += [1]*(1+nf) + [1e-3]*ng
    scales = np.array(scales)
    As = Aa * scales[None, :] / scales[:, None]
    Bs = Ba / scales[:, None]; Cs = C * scales[None, :]
    Qs = Qa / scales[:, None] / scales[None, :]
    Qs = (Qs+Qs.T)/2 + np.eye(N)*1e-15
    R = np.diag([force_sigma**2, position_sigma**2])
    P = solve_discrete_are(As.T, Cs.T, Qs, R)
    L = np.linalg.solve(Cs@P@Cs.T+R, (P@Cs.T).T).T
    Ac = As @ (np.eye(N)-L@Cs)
    Bc = np.column_stack([As@L, Bs-As@L@Dm])
    Cc = scales[:npstates, None] * (np.eye(N)-L@Cs)[:npstates]
    Dc = scales[:npstates, None] * L[:npstates]
    Dknown = -Dc@Dm
    return Ac, Bc, Cc, Dc, Cf, np.max(abs(np.linalg.eigvals(Ac))), Dknown, force_direct_friction


def replay(r, recovered, ix, design, actual_position_sigma, position_kind):
    Ac, Bc, Cc, Dc, Cf, pole, Dknown, force_direct = design
    p = r['parameters']; d = r['data']
    tt = r['robotPrediction']['estimatedTime']; ar = r['robotPrediction']['data'][ix, 2]
    currentdev = d[ix, 32] - p['d']['coilHoldCurrent']
    force = d[ix, 31] - p['force']['reference']
    delay = p['d']['strokeDelaySamples']
    truegap = d[ix, 1] - p['mechanics']['midTravel'] - p['d']['u0']
    gapold = np.r_[np.zeros(delay), truegap][:len(ix)]
    noise = recovered - gapold
    original_sigma = p['sensor']['strokeNoiseRms']
    if original_sigma: noise *= actual_position_sigma / original_sigma
    if position_kind == 'tool_robot':
        raw = d[ix, 6] - d[ix, 25] + p['d']['u0']
        position = np.r_[np.zeros(delay), raw][:len(ix)] + noise
    else: position = gapold + noise
    x = np.zeros(Ac.shape[0]); out = []; frout = []; fr = 0
    for k in range(len(ix)):
        y = np.array([force[k], position[k]])
        prior_correction = Cc@x + Dc@y
        for _ in range(20):
            estimate = prior_correction + Dknown@np.r_[currentdev[k], fr, ar[k]]
            estimate[8] += ar[k]
            next_fr = p['coil']['coulombForce']*np.tanh(estimate[5]/p['coil']['frictionSpeed'])
            if abs(next_fr-fr) < 1e-12:
                fr = next_fr
                break
            fr = next_fr
        x = Ac@x + Bc@np.r_[y, currentdev[k], fr, ar[k]]
        out.append(estimate); frout.append(fr)
    out = np.array(out); use = (tt >= 2) & (tt <= 2.12)
    ae = out[:, 7] - d[ix, 29]
    fe = out@Cf-force_direct*np.array(frout) - (d[ix, 0]-p['force']['reference'])
    return dict(peakAccelerationError_m_s2=float(np.max(abs(ae[use]))),
                peakForceReconstructionError_N=float(np.max(abs(fe[use]))),
                rmsForceReconstructionError_N=float(np.sqrt(np.mean(fe[use]**2))),
                peakObserverPole=float(pole)), tt, out, fe


if __name__ == '__main__':
    rows = []; proof = {}
    for filename in ['smoke_004.mat', 'precision_matched_028.mat']:
        r = loadmat(BASE/filename, simplify_cells=True)['r']
        recovered, ix, roundtrip = recover_recorded_gap_measurement(r)
        w = r['parameters']['coil']['control']
        matched = synthesize(r['parameters'], False, w['surfaceJerkPower'],
                             w['forceNoise_N'], w['strokeNoise_m'], 'gap')
        _, _, estimated, _ = replay(r, recovered, ix, matched,
                                    r['parameters']['sensor']['strokeNoiseRms'], 'gap')
        proof[filename] = dict(recordedMeasurementRecoveryResidual=roundtrip,
                               maximumMatchedNativeObserverStateResidual=float(np.max(
                                   abs(estimated-r['robotPrediction']['estimatedStates']))),
                               nativeTrajectoryForceError_N=float(r['metrics']['peakAbsForceError_N']))
        designs = []
        for kind in ['gap', 'tool_robot']:
            for jerk in [False, True]:
                powers = [7, 70, 700] if not jerk else [1e5, 1e6, 1e7, 1e8]
                for power in powers:
                    for position_sigma in [1e-7, 1e-6]:
                        designs.append((kind, jerk, power, .02, position_sigma, False))
        if r['parameters']['mechanics']['lowerRigid']:
            for jerk, power in [(False,70), (False,1000), (True,1e6)]:
                for fsigma in [.02,.06]:
                    for psigma in [1e-7,1e-6]:
                        designs.append(('tool_robot',jerk,power,fsigma,psigma,True))
        for kind, jerk, power, fsigma, position_sigma, fix_friction in designs:
            design = synthesize(r['parameters'], jerk, power, fsigma, position_sigma,
                                kind, fix_friction)
            result, _, _, _ = replay(r, recovered, ix, design, position_sigma, kind)
            row = dict(nativeTrace=filename, measurement=kind,
                       surfaceJerkState=jerk, processPower=power,
                       correctedDirectCoulombTerm=fix_friction,
                       assumedForceNoise_N=fsigma,
                       recordedForceNoiseRms_N=r['parameters']['sensor']['noiseRms'],
                       assumedAndConstructedPositionNoise_m=position_sigma, **result)
            rows.append(row)
            print(json.dumps(row), flush=True)
            OUT.write_text(json.dumps(dict(
                scope='Estimator replay only; not a new closed-loop plant simulation or qualification.',
                measurementRecovery=proof, rows=rows), indent=2))
