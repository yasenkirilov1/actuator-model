"""Retain 10 kHz diagnostics from an exact replay of a scored native case.

The C plant/controller is unchanged. This recording supplies no controller
input and does not add an independent qualification case.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'rl_budget3h_20261005'))
from native_api import NativePlant
from final_evaluate import atomic_json, atomic_npz, array_sha, sha


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--native-build',type=Path,required=True)
    parser.add_argument('--result',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    assert not args.output.exists()
    args.output.mkdir(parents=True)
    scored=json.loads(args.result.read_text())
    plant=NativePlant(args.native_build.resolve(),threads=1)
    ctx=plant.create(scored['scenario'],baseline=True,record=True)
    samples=[np.r_[0.,ctx.result.obs]]
    action=np.zeros((1,2),np.float64)
    try:
        while True:
            state=ctx.snapshot()
            if state.terminated or state.end_s >= scored['duration_s']-plant.metadata['actionStep_s']/2:
                break
            plant.step_repeat_batch_arrays([ctx],action,1)
            state=ctx.snapshot()
            samples.append(np.r_[state.end_s,state.obs])
        native=ctx.take_record(copy=False)
        hashes={k:array_sha(v) for k,v in native.items()}
        assert hashes==scored['recordArrayHashes'], 'Diagnostic replay changed native history'
        snapshots=np.asarray(samples)
        flat=(snapshots[:,0]>=.2)&(snapshots[:,0]<.95)
        est_v=snapshots[:,9]*scored['scanSpeed_m_s']
        observation_file=args.output/'observer_10kHz.npz'
        atomic_npz(observation_file,{'observations':snapshots})
        report=dict(status='exact_diagnostic_replay_complete',scenario=scored['scenario'],
                    resultFile=str(args.result.resolve()),resultSha256=sha(args.result),
                    nativeBuild=str(args.native_build.resolve()),
                    nativeLibrarySha256=sha(args.native_build/'libacf_native.so'),
                    nativeArrayHashesExact=True, qualificationCasesAdded=0,
                    diagnosticRate_Hz=1/plant.metadata['actionStep_s'],samples=len(samples),
                    observationFile=str(observation_file.resolve()),observationSha256=sha(observation_file),
                    flatWindow_s=[.2,.95],
                    flatEstimatedNormalErrorRms_N=float(np.sqrt(np.mean((snapshots[flat,1]-scored['force_N'])**2))),
                    flatEstimatedSlopeRms=float(np.sqrt(np.mean(snapshots[flat,9]**2))),
                    flatEstimatedSVelocityRms_m_s=float(np.sqrt(np.mean(est_v[flat]**2))),
                    scope='Full native force histories are identical to original scored case. Diagnostic estimates sampled each controller tick. No truth supplied to controller.')
        atomic_json(args.output/'diagnostic_replay.json',report)
        print(json.dumps(report,indent=2))
    finally:
        ctx.close()


if __name__=='__main__':
    main()
