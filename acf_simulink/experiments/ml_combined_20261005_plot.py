#!/usr/bin/env python3
"""Audit four predeclared native combined-policy pairs and render SVGs."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument("input",type=Path);parser.add_argument("output",type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    report=json.loads((args.input/'results.json').read_text());assert report['status']=='complete'
    rows=report['rows'];assert len(rows)==8 and all(r['executed'] for r in rows)
    groups={}
    for r in rows:groups.setdefault(r['identity'],{})[r['policy']]=r
    assert len(groups)==4 and all(set(g)=={'baseline','combined'} for g in groups.values())
    pairs=[]
    for identity,g in groups.items():
        a,b=g['baseline'],g['combined']
        for key in ['massS_kg','force_N','angle_deg','mu','seed','offset_m','actualRobotDelay_s','actualRobotResponse_s']:
            assert a[key]==b[key]
        pairs.append({'identity':identity,'baseline_peak_N':a['peakNormalForceError_N'],'combined_peak_N':b['peakNormalForceError_N'],
                      'change_N':b['peakNormalForceError_N']-a['peakNormalForceError_N'],
                      'baseline_rms_N':a['rmsNormalForceError_N'],'combined_rms_N':b['rmsNormalForceError_N'],
                      'baseline_passed':a['passed'],'combined_passed':b['passed'],
                      'baseline_final_travel_error_mm':a['finalTravelError_m']*1000,
                      'combined_final_travel_error_mm':b['finalTravelError_m']*1000,
                      'combined_peak_learned_current_A':b['peakLearnedCurrent_A'],
                      'combined_nonzero_current_frames':b['learnedCurrentNonzeroFrames'],
                      'combined_nonzero_robot_frames':b['nonzeroCorrectionFrames'],
                      'combined_gate_closures':b['gateClosures'],'combined_pending_request_samples':b['queuedCommandAfterGateCloseSamples']})
    plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':11,'text.color':'#26384c',
                         'axes.labelcolor':'#26384c','axes.edgecolor':'#c9d3df','axes.spines.top':False,'axes.spines.right':False,'grid.color':'#e5ebf2'})
    blue,orange,red='#3363a6','#d18436','#cf4a4a'
    x=np.arange(4);fig,ax=plt.subplots(figsize=(11,5.7))
    ax.bar(x-.18,[p['baseline_peak_N'] for p in pairs],.34,color=blue,label='Deterministic force + recenter control')
    ax.bar(x+.18,[p['combined_peak_N'] for p in pairs],.34,color=orange,label='Both trained policies, unchanged safety limits')
    ax.axhline(.5,color=red,lw=1.5,label='Required limit: 0.5 N');ax.axhline(.45,color='#9f7c39',lw=1,ls='--',label='Design margin screen: 0.45 N')
    ax.set_ylim(0,max(.65,max(p['combined_peak_N'] for p in pairs)*1.25));ax.set_ylabel('Maximum normal-force error (N)')
    labels=['0.1 kg · 5 N · 0°\nμ = 0, −1.5 mm offset','2 kg · 50 N · 90°\nμ = 0.3, +1.5 mm offset',
            '1 kg · 25 N · 45°\nRobot timing mismatch','2 kg · 50 N · 90°\nJoint calibration stress']
    ax.set_xticks(x,labels);ax.grid(axis='y',alpha=.6);ax.set_axisbelow(True)
    ax.legend(ncol=2,frameon=False,loc='upper left')
    for i,p in enumerate(pairs):
        ax.text(i,max(p['baseline_peak_N'],p['combined_peak_N'])+.015,f"Δ {p['change_N']*1000:+.1f} mN",ha='center',fontsize=10,color='#6b543f')
    ax.set_title('Two small learned policies in the same native closed loop\nFour new, predeclared eight-second trajectory pairs',loc='left',fontweight='bold',pad=18)
    fig.text(.08,.015,'Weights and force-residual gain were fixed before these tests. Peaks use every native sample, including startup.',fontsize=9)
    fig.tight_layout(rect=(0,.035,1,1));fig.savefig(args.output/'ml_combined_paired_results.svg');plt.close(fig)

    worst=max(groups.values(),key=lambda g:g['combined']['peakNormalForceError_N'])
    a=loadmat(args.input/f"recording_{worst['baseline']['recording']:03d}_compact.mat",simplify_cells=True)['compact']
    b=loadmat(args.input/f"recording_{worst['combined']['recording']:03d}_compact.mat",simplify_cells=True)['compact']
    fig,axes=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for c,label,color in [(a,'Deterministic',blue),(b,'Combined learned',orange)]:
        t=c['time'];d=c['data'];p=c['parameters']
        axes[0].plot(t,d[:,14]-p['force']['reference'],label=label,color=color,lw=1)
        axes[1].plot(t,(d[:,1]-p['mechanics']['midTravel'])*1000,label=label,color=color,lw=1.5)
    ft=b['forceFeatureTime'];fd=b['forceFeatures'];axes[2].plot(ft,fd[:,10]*1000,color=orange,lw=1,label='Learned extra current')
    for ax in axes:ax.grid(alpha=.55);ax.set_axisbelow(True)
    axes[0].axhline(.5,color=red,lw=1,ls='--');axes[0].axhline(-.5,color=red,lw=1,ls='--')
    axes[0].set_ylabel('Normal-force error (N)');axes[1].set_ylabel('Travel from centre (mm)');axes[2].set_ylabel('Learned current addition (mA)')
    axes[2].set_xlabel('Time (s)');axes[0].legend(frameon=False);axes[2].legend(frameon=False)
    fig.suptitle('Worst combined-policy trajectory\nThe learned robot policy remains gated; the force residual changes current causally',x=.075,ha='left',fontweight='bold',fontsize=13)
    fig.tight_layout(rect=(0,0,1,.93));fig.savefig(args.output/'ml_combined_worst_trajectory.svg');plt.close(fig)
    summary={'date':'2026-10-05','status':'complete','study':'fixed combined-policy screen',
             'identities':4,'recordings':8,'native_samples':report['nativeSamples'],'duration_per_recording_s':8,
             'source_model_sha256':report['sourceModelSha256'],'private_model_sha256':report['privateModelSha256'],
             'robot_weights_sha256':report['robotWeightsSha256'],'force_weights_sha256':report['forceWeightsSha256'],
             'fixed_force_selection_gain':report['fixedForceSelectionGain'],'retuning_from_test':False,
             'baseline_passed':sum(p['baseline_passed'] for p in pairs),'combined_passed':sum(p['combined_passed'] for p in pairs),
             'baseline_worst_N':max(p['baseline_peak_N'] for p in pairs),'combined_worst_N':max(p['combined_peak_N'] for p in pairs),
             'improved_pairs':sum(p['change_N'] < -1e-8 for p in pairs),'worsened_pairs':sum(p['change_N'] > 1e-8 for p in pairs),
             'gate_closed_command_violations':sum(r['gateClosedCommandViolations'] for r in rows),
             'direction_or_gain_envelope_violations':sum(r['directionEnvelopeViolations'] for r in rows),
             'maximum_robot_network_replay_error_m':max(r['networkInferenceResidual_m'] for r in rows),
             'maximum_force_network_replay_error_A':max(r['forceNetworkInferenceResidual_A'] for r in rows),
             'combined_nonzero_robot_frames':sum(p['combined_nonzero_robot_frames'] for p in pairs),
             'combined_nonzero_current_frames':sum(p['combined_nonzero_current_frames'] for p in pairs),
             'any_adc_clipping':any(r['anyAdcClipping'] for r in rows),'rows':pairs,
             'recipe_sha256':sha(Path(__file__)),'native_results_sha256':sha(args.input/'results.json'),
             'figures':[{'file':f.name,'sha256':sha(f)} for f in sorted(args.output.glob('*.svg'))],
             'recommendation':'Retain deterministic PI/feedforward and deterministic gated recentering; this fixed small learned system offers no robust performance advantage.',
             'qualification_scope':'Four predeclared new identities with fixed weights and gain. Finite native test, not full range/residual-box or physical hardware qualification.'}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['status','native_samples','baseline_passed','combined_passed','baseline_worst_N','combined_worst_N','improved_pairs','worsened_pairs']},indent=2))


if __name__=='__main__':main()
