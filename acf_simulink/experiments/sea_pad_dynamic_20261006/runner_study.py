"""Bounded, reproducible development and locked diagnostic evaluation runner."""
from __future__ import annotations

import argparse
import ctypes as ct
from datetime import datetime, timezone
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np

from engine_api import FIELDS, HERE, OUT, ROOT, Parameters, TRACE_COLUMNS, compile_engine, make_parameters, simulate, write_trace

VALIDATION = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006"


def contract() -> dict:
    path = HERE / "study_contract.json"
    forecast = VALIDATION / "storage_forecast.json"
    if not path.exists() or not forecast.exists():
        raise RuntimeError("Root-owned study contract and storage forecast are required before execution.")
    return json.loads(path.read_text())


def default_config(actuator: int = 0, pad_k: float = 20000, pad_c: float = 0,
                   diameter: float = 0, sea_k: float = 5000, lead: float = .005) -> dict:
    spec = contract()
    mech, coil, motor = spec["mechanics"], spec["coil"], spec["screw_motor"]
    sensor, surface, numerics = spec["acquisition"], spec["surface"], spec["numerics"]
    p = dict.fromkeys(FIELDS, 0.)
    p.update(actuator=actuator, pad_points=spec["pad"]["strips"], dt=numerics["base_step_s"],
             duration=surface["duration_s"], force_target=50, mass_a=mech["mA_kg"],
             mass_b=mech["mB_kg"], mass_s=2, gravity=mech["gravity_m_s2"], angle_deg=90,
             upper_k=mech["upper_stiffness_N_m"], cell_k=mech["cell_stiffness_N_m"],
             pad_k=pad_k, pad_c=pad_c, pad_diameter=diameter,
             pad_thickness=.01 if pad_k==20000 else .005 if pad_k<1000000 else 0,
             pad_compression_limit=.005 if pad_k==20000 else .0025 if pad_k<1000000 else 0,
             bump_height=surface["height_m"], bump_width=surface["width_m"],
             scan_speed=surface["scan_speed_m_s"], bump_start=surface["point_bump_start_s"],
             nut_mass=mech["nut_translation_mass_kg"], lead=lead,
             rotor_inertia=motor["rotor_inertia_kg_m2"]+motor["assumed_screw_coupler_inertia_kg_m2"],
             sea_k=sea_k, sea_c=0, spring_deflection_limit=mech["sea_signed_deflection_limit_m"],
             nut_travel_limit=mech["nut_q_hard_limit_m"], tool_travel_limit=mech["flange_u_hard_limit_m"],
             force_constant=coil["Kf_N_A"] if actuator==0 else motor["Kt_Nm_A"],
             resistance=coil["R_hot_ohm"] if actuator==0 else motor["R_hot_ohm"],
             inductance=coil["L_H"] if actuator==0 else motor["L_H"],
             current_response=coil["current_response_s"], voltage_limit=coil["usable_voltage_V"],
             current_limit=coil["peak_current_A"], continuous_current=0 if actuator==0 else motor["catalog_continuous_current_A"],
             guide_coulomb=mech["guide_coulomb_N"], guide_viscous=mech["guide_viscous_N_s_m"],
             guide_eps=mech["guide_smoothing_m_s"], shaft_coulomb=motor["shaft_coulomb_Nm"],
             shaft_viscous=motor["shaft_viscous_Nm_s_rad"], shaft_eps=motor["shaft_smoothing_rad_s"],
             analog_cutoff=sensor["matched_two_pole_cutoff_Hz"], adc_rate=sensor["adc_rate_Hz"],
             control_rate=sensor["controller_rate_Hz"], ema_time=sensor["adc_output_ema_s"],
             pipeline_time=sensor["adc_group_delay_s"]+sensor["pipeline_s"],
             force_noise_rms=sensor["conditioned_noise_at_reference_ENBW"]["force_N_RMS"],
             accel_noise_rms=sensor["conditioned_noise_at_reference_ENBW"]["acceleration_m_s2_RMS"],
             position_noise_rms=sensor["conditioned_noise_at_reference_ENBW"]["position_m_RMS"],
             noise_reference_enbw=sensor["noise_reference_ENBW_Hz"],
             force_quantum=320/2**sensor["quantization_bits"], accel_quantum=235.3596/2**sensor["quantization_bits"],
             position_quantum=1e-7, force_scale=1, accel_scale=1, identified_mass_s=2,
             force_kp=10, force_ki=500, velocity_gain=400, inertia_gain=.5,
             inertia_cutoff=100, velocity_leak=1, servo_hz=80, servo_zeta=1,
             servo_target_cutoff=300, encoder_quantum=lead/16384, noise_enabled=1, seed=260600,
             servo_velocity_feedforward=1, force_feedback_cutoff=0,
             robot_soft_band=mech["flange_u_soft_band_m"])
    return p


def preparation_rejection(p: dict) -> str | None:
    g=p["gravity"]*np.cos(np.deg2rad(p["angle_deg"]))
    fa=p["force_target"]-(p["mass_b"]+p["mass_s"])*g
    zs=0 if p["pad_k"]==0 else -p["force_target"]/p["pad_k"]
    zb=zs-(p["force_target"]-p["mass_s"]*g)/p["cell_k"]
    za=(fa-(p["mass_a"]+(p["nut_mass"] if p["actuator"] else 0))*g)/p["upper_k"]
    if abs(za-zb)>p["tool_travel_limit"]:
        return "initial_flange_stroke"
    if p["pad_compression_limit"] and -zs>p["pad_compression_limit"]:
        return "initial_pad_compression"
    if p["actuator"]:
        q=za-zb+fa/p["sea_k"]
        if abs(q)>p["nut_travel_limit"]:return "initial_nut_stroke"
        if abs(fa/p["sea_k"])>p["spring_deflection_limit"]:return "initial_spring_deflection"
    else:q=0
    current=fa/p["force_constant"] if not p["actuator"] else (fa-p["nut_mass"]*g)*p["lead"]/(2*np.pi*p["force_constant"])
    if abs(current)>p["current_limit"]:return "initial_peak_current"
    if abs(current*p["resistance"])>p["voltage_limit"]:return "initial_voltage"
    return None


def execute(lib, p: dict, identity: str, trace_path: Path | None = None) -> dict:
    rejection=preparation_rejection(p)
    if rejection:return {"id":identity,"config":p,"status":"preparation_rejected","reason":rejection}
    m, trace=simulate(lib,p,trace_rate=1000 if trace_path else 0)
    if trace_path:write_trace(trace_path,trace)
    return {"id":identity,"config":p,"status":"executed","metrics":m,
            **({"trace":str(trace_path.relative_to(ROOT))} if trace_path else {})}


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n")


def equilibrium_checks(lib) -> dict:
    rows=[]
    for a,m,f,angle in itertools.product([0,1],[.1,2.],[5,50],[0,90]):
        p=default_config(a)
        p.update(mass_s=m,identified_mass_s=m,force_target=f,angle_deg=angle)
        cp=make_parameters(p)
        x=np.zeros(9); eq=np.zeros(2); dx=np.zeros(9); outputs=np.zeros(10)
        ptr=lambda v:v.ctypes.data_as(ct.POINTER(ct.c_double))
        lib.engine_initialize(ct.byref(cp),ptr(x),ptr(eq))
        lib.engine_rhs(ct.byref(cp),0,ptr(x),x[8],0,ptr(dx),ptr(outputs))
        residual=float(np.max(np.abs(dx)))
        rows.append(dict(actuator=a,mass=m,force=f,angle=angle,maximum_derivative_residual=residual))
        if residual>1e-6:raise AssertionError(rows[-1])
    return {"status":"passed","count":len(rows),"checks":rows}


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument("mode",choices=["pilot","tune","evaluate","refine","replay"])
    args=parser.parse_args()
    lib=compile_engine()
    dump(OUT/"equilibrium_checks.json",equilibrium_checks(lib))
    if args.mode=="pilot":
        records=[]
        for actuator in [0,1]:
            p=default_config(actuator)
            row=execute(lib,p,f"pilot_{actuator}",VALIDATION/"development"/f"pilot_{actuator}.csv")
            records.append(row)
            print(json.dumps({"id":row["id"],"metrics":row.get("metrics"),"status":row["status"]}),flush=True)
        dump(VALIDATION/"development/pilot.json",records)
    else:
        raise NotImplementedError("Further bounded study phases are added after pilot evidence.")


if __name__=="__main__":main()
