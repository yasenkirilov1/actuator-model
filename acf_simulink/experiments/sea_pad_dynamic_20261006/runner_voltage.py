"""Unchanged-controller32/48V performance-limit diagnostics."""
from __future__ import annotations

import hashlib
import json

from engine_api import HERE, compile_engine, simulate
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine();rows=[]
    groups=["coil_wide20k_c0_width_transfer","sea_v2_wide20k_c0_5k_l10_width_transfer","sea_v2_point20k_c80_5k_l10"]
    for name in groups:
        group=json.loads((VALIDATION/"evaluation"/f"{name}.json").read_text())
        original=max(group["records"],key=lambda r:r["metrics"]["force_peak_error"])
        p={**default_config(round(original["config"]["actuator"])),**original["config"]}
        m32,tr32=simulate(lib,{**p,"voltage_limit":32},trace_rate=1000)
        m48,tr48=simulate(lib,{**p,"voltage_limit":48},trace_rate=1000)
        same_shape=tr32.shape==tr48.shape
        rows.append({"group":name,"config":p,"status":"executed_unchanged_controller_voltage_pair",
                     "metrics_32V":m32,"metrics_48V":m48,
                     "recorded_1kHz_trace_bitwise_identical":bool(same_shape and (tr32==tr48).all()),
                     "recorded_1kHz_force_max_difference_N":float(abs(tr32[:,1]-tr48[:,1]).max()) if same_shape else None,
                     "native_scored_peak_difference_N":abs(m32["force_peak_error"]-m48["force_peak_error"]),
                     "note":"Allintegration endpoints scored in bothruns; identical1kHzsampledtrace is a diagnostic, notfull-history byteparity."})
    dump(VALIDATION/"evaluation/voltage_limit_pairs.json",
         {"status":"executed_diagnostic_not_retuned_qualification","records":rows,"execution_count":6,
          "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()})
    for r in rows:print("VOLTAGE",r["group"],r["metrics_32V"]["force_peak_error"],r["metrics_48V"]["force_peak_error"],r["recorded_1kHz_trace_bitwise_identical"],flush=True)


if __name__=="__main__":main()
