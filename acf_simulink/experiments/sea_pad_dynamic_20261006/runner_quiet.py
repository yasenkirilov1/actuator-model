"""Corrected-encoder, quiet SEA controller development and fresh evaluation."""
from __future__ import annotations

import itertools
import numpy as np

import runner_campaign as campaign
from runner_study import default_config


def quiet_candidates(actuator:int) -> list[dict]:
    assert actuator==1
    combinations=list(itertools.product([0,.5,2,5,10],[0,100,400],[0,.5,.8,1],[50,100],
                                         [60,100,160],[100,300],[0,100,300],[1,2]))
    rng=np.random.default_rng(26061001)
    selection=sorted(rng.choice(len(combinations),size=144,replace=False))
    families=[]
    for ix in selection:
        k,d,ff,icut,servo,target,fcut,mode=combinations[ix]
        families.append(dict(force_kp=k,force_ki=0,velocity_gain=d,inertia_gain=ff,
                             inertia_cutoff=icut,servo_hz=servo,servo_zeta=1,
                             servo_target_cutoff=target,force_feedback_cutoff=fcut,
                             servo_velocity_feedforward=mode))
    for k,ff in itertools.product([2,5,10],[0,.5,.8]):
        families.append(dict(force_kp=k,force_ki=0,velocity_gain=50,inertia_gain=ff,
                             inertia_cutoff=100,servo_hz=160,servo_zeta=1,
                             servo_target_cutoff=300,force_feedback_cutoff=300,
                             servo_velocity_feedforward=2))
    return families


def main():
    campaign.candidates=quiet_candidates
    for damping in [0,80]:
        name=f"sea_v2_point20k_c{damping}_5k_l10"
        base=default_config(1,pad_c=damping,lead=.01)
        result=campaign.develop_config(name,base,top_count=12)
        campaign.evaluated_config(result["candidate_lock"],trace=True,seeds=[260611,260612])


if __name__=="__main__":main()
