"""Score private mechanics banks using their unchanged Simulink-generated C.

This is an experiment executor, not a Python plant. Native baseline mode runs
the PI/feedforward controller at its own 10 kHz clock; large C call chunks
only reduce Python overhead. Every major physical sample is independently
scored, including time zero, startup and the last sample. Failed histories and
each gain's worst are preserved in lossless FP64 NPZ files. No public SLX is
modified. Fresh banks must be prepared after development gain selection.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
REFERENCE_EXECUTOR = HERE.parent / 'rl_budget3h_20261005'
sys.path.insert(0, str(REFERENCE_EXECUTOR))
from native_api import NativePlant
from final_evaluate import atomic_json, atomic_npz, array_sha, sha

_PLANT = None
_OUTPUT = None


def init_worker(build, output):
    global _PLANT, _OUTPUT
    _PLANT = NativePlant(build, threads=1)
    _OUTPUT = Path(output)


def score(record, final, scenario, metadata):
    t, d = record['time'], record['truth']
    dt = float(metadata['step_s'])
    n = len(t)
    finite = bool(n and all(np.isfinite(a).all() for a in record.values()))
    row = dict(scenario)
    row.update(nativeSamples=n, fullNativeHistory=True, baselineController=True,
               nativeIntervalMetrics=final.metrics)
    if not finite:
        row.update(passed=False, completed=False, softwareRejected=True,
                   physicalFailure=bool(final.physical_failure))
        return row
    ticks = np.rint(t / dt).astype(np.int64)
    grid = bool(t[0] == 0 and n == ticks[-1] + 1 and np.all(np.diff(ticks) == 1)
                and np.max(np.abs(t - ticks * dt)) < max(1e-12, dt * 1e-6))
    error = d[:, 0] - scenario['force_N']
    abs_error = np.abs(error)
    contact = bool(final.contact_loss or np.any(d[:, 18] < .5))
    stop = bool(final.travel_stop or np.any(d[:, 21] > .5))
    completed = bool(t[-1] >= scenario['duration_s'] - dt / 2)
    band = float(scenario.get('recenterBandRadius_m', .0075))
    offset = np.abs(d[:, 2] - scenario['midTravel_m'])
    outside = offset > band + 1e-12
    ipeak = int(np.argmax(abs_error))
    after = t >= scenario.get('bumpStart_s', 1) + .1
    tail = t >= max(0., scenario['duration_s'] - .5)
    row.update(
        nativeGridComplete=grid, softwareRejected=not grid,
        nativeFirstTime_s=float(t[0]), nativeLastTime_s=float(t[-1]),
        completed=completed, endTime_s=float(t[-1]),
        peakNormalForceError_N=float(abs_error[ipeak]), peakErrorTime_s=float(t[ipeak]),
        initialNormalForceError_N=float(error[0]), finalNormalForceError_N=float(error[-1]),
        rmsNormalForceError_N=float(np.sqrt(np.mean(error ** 2))),
        postBumpPeakForceError_N=float(abs_error[after].max()) if np.any(after) else None,
        finalHalfSecondRmsError_N=float(np.sqrt(np.mean(error[tail] ** 2))),
        forceOutsideSamples=int(np.sum(abs_error > .5)),
        forceOutsideFraction=float(np.mean(abs_error > .5)),
        peakCurrent_A=float(np.abs(d[:, 14]).max()),
        rmsCurrent_A=float(np.sqrt(np.mean(d[:, 14] ** 2))),
        peakVoltage_V=float(np.abs(d[:, 15]).max()),
        voltageClipSamples=int(np.sum(d[:, 23] > .5)),
        voltageClipFraction=float(np.mean(d[:, 23] > .5)),
        currentLimitSamples=int(np.sum(d[:, 22] > .5)),
        currentLimitFraction=float(np.mean(d[:, 22] > .5)),
        overCurrentSamples=int(np.sum(np.abs(d[:, 14]) > scenario['peakCurrent_A'] * (1 + 1e-4))),
        overVoltageSamples=int(np.sum(np.abs(d[:, 15]) > scenario['usableVoltage_V'] + 1e-7)),
        minTravel_m=float(d[:, 2].min()), maxTravel_m=float(d[:, 2].max()),
        finalTravel_m=float(d[-1, 2]), maxAbsTravelError_m=float(offset.max()),
        maxTravelBandExcess_m=float(np.maximum(offset - band, 0).max()),
        travelOutsideBandSamples=int(outside.sum()),
        travelOutsideBandTime_s=float(outside[:-1].sum() * dt),
        travelBandPassed=bool(completed and not np.any(outside)),
        contactLoss=contact, travelStop=stop, physicalFailure=contact or stop,
        peakLowerLinkForce_N=float(np.abs(d[:, 24]).max()),
        peakUpperLinkForce_N=float(np.abs(d[:, 25]).max()),
        minLowerLinkForce_N=float(d[:, 24].min()), maxLowerLinkForce_N=float(d[:, 24].max()),
        minUpperLinkForce_N=float(d[:, 25].min()), maxUpperLinkForce_N=float(d[:, 25].max()),
        maxAbsLowerDeviation_m=float(np.abs(d[:, 7] - d[:, 6]).max()))
    if scenario.get('upperExternalSpringPresent'):
        q20 = scenario.get('knownStaticUpperForce_N')
        if q20 is not None and scenario['upperC_N_s_m'] == 0:
            row['maxAbsUpperDeviation_m'] = float(np.abs((d[:, 25] - q20) / scenario['upperK_N_m']).max())
    spring_rows = []
    for which in ('lower', 'upper'):
        pair = scenario.get(which + 'Pair', {})
        if not pair.get('present'):
            continue
        rate = pair['singleSpringRate_N_m']
        if which == 'lower':
            delta = d[:, 7] - d[:, 6]
            loaded = scenario['knownStaticLowerForce_N'] / scenario['actualK_N_m'] + delta
        else:
            delta = (d[:, 25] - scenario['knownStaticUpperForce_N']) / scenario['upperK_N_m']
            loaded = d[:, 25] / scenario['upperK_N_m']
        common = pair['preloadPerSpring_N'] / rate
        maximum = float(common + np.abs(loaded).max())
        minimum = float(common - np.abs(loaded).max())
        min_length = pair['freeLength_m'] - pair['freeLengthTolerance_m'] - maximum
        peak_force = rate * maximum
        cyclic = float(np.max(delta) - np.min(delta))
        spring_rows.append(dict(
            link=which, target=pair, minimumCompression_m=minimum, maximumCompression_m=maximum,
            minimumInstalledLength_m=min_length, maximumSpringForce_N=peak_force,
            cyclicStroke_m=cyclic, unseating=minimum < 0,
            workingLengthPassed=min_length >= pair['minWorkingLength_m'],
            cyclicTargetPassed=cyclic <= pair['maxCyclicStrokeTarget_m'],
            forceEnvelopeTargetPassed=peak_force <= pair['forceEnvelopeTarget_N'],
            maximumCombinedFreeLength_m=2 * (pair['freeLength_m'] + pair['freeLengthTolerance_m']),
            maximumCombinedInstalledLength_m=2 * (pair['freeLength_m'] + pair['freeLengthTolerance_m'] - common)))
    combined_free = sum(s['maximumCombinedFreeLength_m'] for s in spring_rows)
    combined_installed = sum(s['maximumCombinedInstalledLength_m'] for s in spring_rows)
    row.update(externalSpringScreen=spring_rows, combinedFreeSpringLength_m=combined_free,
               combinedInstalledSpringLength_m=combined_installed,
               externalSpringPackingPassed=bool(combined_free <= .1 and combined_installed <= .1),
               externalSpringOperationTargetsPassed=all(not s['unseating'] and s['workingLengthPassed']
                   and s['cyclicTargetPassed'] and s['forceEnvelopeTargetPassed'] for s in spring_rows))
    row['forcePassed'] = bool(completed and abs_error[ipeak] <= .5 and not (contact or stop))
    row['passed'] = bool(row['forcePassed'] and grid and row['overCurrentSamples'] == 0
                         and row['overVoltageSamples'] == 0)
    row['marginBelowLimit_N'] = .5 - row['peakNormalForceError_N']
    row['designMarginPassed'] = bool(row['passed'] and row['peakNormalForceError_N'] <= .45)
    row['forcePhysicalPassed'] = row['passed']
    row['forceAndSpringTargetsPassed'] = bool(row['passed'] and row['externalSpringPackingPassed']
                                              and row['externalSpringOperationTargetsPassed'])
    row['recordArrayHashes'] = {k: array_sha(a) for k, a in record.items()}
    return row


def execute(index, preserve=False, seed=None):
    scenario = dict(_PLANT.scenarios[index])
    if seed is not None:
        scenario['seed'] = seed
    started = time.monotonic()
    ctx = _PLANT.create(index, seed=seed, baseline=True, record=True)
    try:
        duration = scenario['duration_s']
        obs_records = [np.r_[0., ctx.result.obs]]
        # Baseline ignores these external action inputs. All internal clocks run.
        actions = np.zeros((1, 2), np.float64)
        while True:
            state = ctx.snapshot()
            if state.terminated or state.end_s >= duration - _PLANT.metadata['actionStep_s'] / 2:
                break
            remaining = max(1, int(round((duration - state.end_s) / _PLANT.metadata['actionStep_s'])))
            _PLANT.step_repeat_batch_arrays([ctx], actions, min(100, remaining))
            state = ctx.snapshot()
            obs_records.append(np.r_[state.end_s, state.obs])
        final = ctx.snapshot()
        record = ctx.take_record(copy=False)
        row = score(record, final, scenario, _PLANT.metadata)
        row['wallTime_s'] = time.monotonic() - started
        path = _OUTPUT / f"case_{index:04d}"
        path.mkdir(exist_ok=True)
        if preserve or not row['passed']:
            if shutil.disk_usage(_OUTPUT).free < 2 * 1024 ** 3:
                raise OSError('Under 2 GiB free: preserve existing evidence and stop before writing history')
            filename = 'native_history.npz' if not preserve else 'retained_native_history.npz'
            atomic_npz(path / filename, record)
            row['historyFile'] = str(path / filename)
            row['historySha256'] = sha(path / filename)
            # Diagnostic-only 100 Hz observer snapshots; scoring uses full truth.
            atomic_npz(path / 'observer_100Hz.npz', {'observations': np.asarray(obs_records)})
            row['observerDiagnosticFrameRate_Hz'] = 100
        atomic_json(path / ('result.json' if not preserve else 'retention_replay.json'), row)
        return row
    finally:
        ctx.close()


def group_rows(rows):
    groups = {}
    for r in rows:
        key = (r.get('gainID', 1), r.get('massA_kg', .4))
        groups.setdefault(key, []).append(r)
    out = []
    for (gain, mass), part in sorted(groups.items()):
        finite = [r for r in part if r.get('peakNormalForceError_N') is not None]
        worst = max(finite, key=lambda r: r['peakNormalForceError_N']) if finite else None
        out.append(dict(gainID=gain, massA_kg=mass, cases=len(part),
                        passed=sum(r['passed'] for r in part),
                        physicalFailures=sum(r['physicalFailure'] for r in part),
                        softwareRejected=sum(r['softwareRejected'] for r in part),
                        designMarginPassed=sum(r.get('designMarginPassed', False) for r in part),
                        springPackingPassed=sum(r.get('externalSpringPackingPassed', True) for r in part),
                        springOperationTargetsPassed=sum(r.get('externalSpringOperationTargetsPassed', True) for r in part),
                        forceAndSpringTargetsPassed=sum(r.get('forceAndSpringTargetsPassed', r['passed']) for r in part),
                        worstNormalForceError_N=worst['peakNormalForceError_N'] if worst else None,
                        worstScenario=worst['scenario'] if worst else None,
                        worstIdentity=worst['identity'] if worst else None))
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-build', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--indices', help='Comma-separated native bank indexes; default all')
    parser.add_argument('--split', default='development')
    args = parser.parse_args()
    build, output = args.native_build.resolve(), args.output.resolve()
    if output.exists():
        raise ValueError('Use a new output directory; previous evidence is immutable')
    plant = NativePlant(build, threads=1)
    indices = [int(x) for x in args.indices.split(',')] if args.indices else list(range(len(plant.scenarios)))
    if not indices or len(set(indices)) != len(indices):
        raise ValueError('Case IDs must be nonempty and unique')
    if any(plant.scenarios[i]['split'] != args.split for i in indices):
        raise ValueError('Bank split differs from declared experiment stage')
    metadata, bridge = plant.metadata, json.loads((build / 'bridge_build.json').read_text())
    assert bridge['native_computation_recovered_exactly']
    assert abs(metadata['actionStep_s'] - .0001) < 1e-15
    configs = Path(metadata['configsFile'])
    assert sha(configs) == metadata['configsSha256']
    seal = {'libacf_native.so': 'shared_library_sha256', 'native_parameter_bank.c': 'parameter_bank_sha256',
            'acf_rl_native_context.c': 'adapted_c_sha256'}
    for file, key in seal.items():
        assert sha(build / file) == bridge[key], file
    output.mkdir(parents=True)
    lock = dict(status='frozen_before_execution', split=args.split, indices=indices,
                createdUtc=datetime.now(timezone.utc).isoformat(), nativeBuild=str(build),
                configsFile=str(configs), inputHashes={str(build / f): sha(build / f) for f in
                    ('metadata.json', 'scenarios.json', 'bridge_build.json', 'libacf_native.so',
                     'native_parameter_bank.c', 'acf_rl_native_context.c')},
                configsSha256=sha(configs), sourceModelSha256=metadata['sourceModelSha256'],
                scenarios=[plant.scenarios[i] for i in indices], executorSha256=sha(__file__),
                importedNativeApiSha256=sha(REFERENCE_EXECUTOR / 'native_api.py'),
                importedScoringUtilitiesSha256=sha(REFERENCE_EXECUTOR / 'final_evaluate.py'),
                contract='Native PI/feedforward baseline; every physical major point scored, time zero and endpoint included. Development gains may be selected only on development outputs. No neural inference or retraining. Failed and gain-worst histories are retained; diagnostic observations are 100 Hz only.')
    atomic_json(output / 'input_lock.json', lock)
    frozen = output / 'executor'
    frozen.mkdir()
    for source in (Path(__file__), REFERENCE_EXECUTOR / 'native_api.py',
                   REFERENCE_EXECUTOR / 'native_bridge.c', REFERENCE_EXECUTOR / 'final_evaluate.py'):
        shutil.copyfile(source, frozen / source.name)
    rows = []
    started = time.monotonic()
    context = multiprocessing.get_context('spawn')
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=context,
                             initializer=init_worker, initargs=(str(build), str(output))) as pool:
        futures = {pool.submit(execute, i): i for i in indices}
        for future in as_completed(futures):
            r = future.result()
            rows.append(r)
            print(f"CASE {r['identity']} peak={r.get('peakNormalForceError_N')} pass={r['passed']} physical={r['physicalFailure']}", flush=True)
            atomic_json(output / 'progress.json', dict(status='running', completed=len(rows), requested=len(indices),
                                                      rows=sorted(rows, key=lambda r: r['scenario'])))
        groups = group_rows(rows)
        retain = sorted({g['worstScenario'] for g in groups if g['worstScenario'] is not None
                         and not next(r for r in rows if r['scenario'] == g['worstScenario']).get('historyFile')})
        for future in as_completed({pool.submit(execute, i, True): i for i in retain}):
            replay = future.result()
            original = next(r for r in rows if r['scenario'] == replay['scenario'])
            assert replay['recordArrayHashes'] == original['recordArrayHashes'], 'Retention replay differs'
            original.update(historyFile=replay['historyFile'], historySha256=replay['historySha256'],
                            retentionReplayHashesExact=True)
    unchanged = all(sha(file) == digest for file, digest in lock['inputHashes'].items()) and sha(configs) == lock['configsSha256']
    unchanged = unchanged and sha(__file__) == lock['executorSha256']
    unchanged = unchanged and sha(REFERENCE_EXECUTOR / 'native_api.py') == lock['importedNativeApiSha256']
    unchanged = unchanged and sha(REFERENCE_EXECUTOR / 'final_evaluate.py') == lock['importedScoringUtilitiesSha256']
    assert unchanged
    results = dict(status='complete', split=args.split, inputLockSha256=sha(output / 'input_lock.json'),
                   lockedInputsUnchanged=unchanged, cases=len(rows), passed=sum(r['passed'] for r in rows),
                   physicalFailures=sum(r['physicalFailure'] for r in rows),
                   nativeSamples=sum(r['nativeSamples'] for r in rows),
                   retentionReplayCount=len(retain), replaySamplesExcludedFromUniqueTotals=True,
                   wallTime_s=time.monotonic() - started, groups=groups,
                   rows=sorted(rows, key=lambda r: r['scenario']))
    atomic_json(output / 'results.json', results)
    print(json.dumps({k: v for k, v in results.items() if k != 'rows'}, indent=2), flush=True)


if __name__ == '__main__':
    main()
