"""Bounded plant-only references for native replay (temporary histories only).

Input JSON is a list of {identity, config, commands_csv} entries. Commands CSV
contains exact 10 kHz time/current rows; the completed controller is elsewhere.
This script refuses more than three full 2s double-precision histories.
"""
from __future__ import annotations
import argparse
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = ROOT / 'acf_simulink/validation/sea_pad_dynamic_20261006/native'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('cases_json',type=Path);parser.add_argument('--manifest-name',default='replay_manifest.json');args=parser.parse_args()
    entries=json.loads(args.cases_json.read_text())
    if isinstance(entries,dict):entries=entries['cases']
    assert 1 <= len(entries) <= 3
    spec=importlib.util.spec_from_file_location('sea_engine_api',HERE.parent/'engine_api.py')
    api=importlib.util.module_from_spec(spec);spec.loader.exec_module(api)
    target=OUT/'owned_temporary';target.mkdir(exist_ok=True)
    (target/'owned_sea_pad_native').write_text('Owned bounded native preparation/replay\n')
    lib=api.compile_engine()
    total_forecast=sum((round(e['config']['duration']/e['config']['dt'])+1)*8+ (round(e['config']['duration']*e['config']['control_rate'])+1)*9*8 for e in entries)
    assert total_forecast < 300*1024**2
    rows=[]
    for e in entries:
        config=e['config'];command_source=Path(e['commands_csv'])
        command_file=command_source
        if command_source.suffix=='.gz':
            command_file=target/(e['identity']+'_commands.csv')
            command_file.write_bytes(gzip.decompress(command_source.read_bytes()))
        commands=np.loadtxt(command_file,delimiter=',',skiprows=1)
        expected_times=np.arange(len(commands))/config['control_rate']
        assert commands.shape[1]==2 and np.max(abs(commands[:,0]-expected_times)) < 1e-10
        cmd=np.ascontiguousarray(commands[:,1]);count=round(config['duration']/config['dt'])+1
        engine_output=api.physical_replay(lib,config,cmd)
        assert engine_output.shape==(count,10)
        path=target/(e['identity']+'_force.bin');np.ascontiguousarray(engine_output[:,0]).tofile(path)
        stride=round(1/(config['control_rate']*config['dt']))
        state_path=target/(e['identity']+'_states.bin');np.ascontiguousarray(engine_output[::stride,1:]).tofile(state_path)
        row=dict(identity=e['identity'],config=config,commands_csv=str(command_file.resolve()),reference_binary=str(path),reference_count=count,reference_width=1,reference_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),reference_states_binary=str(state_path),reference_state_stride=stride,reference_state_count=len(engine_output[::stride]),reference_states_sha256=hashlib.sha256(state_path.read_bytes()).hexdigest(),reference_force_peak_error_N=float(np.max(abs(engine_output[:,0]-config['force_target']))))
        if command_source.suffix=='.gz':
            row['retained_commands_archive']=str(command_source.resolve())
            row['retained_commands_archive_sha256']=hashlib.sha256(command_source.read_bytes()).hexdigest()
        if 'engine_original_commands_csv' in e:
            original_source=Path(e['engine_original_commands_csv'])
            original_file=original_source
            if original_source.suffix=='.gz':
                original_file=target/(e['identity']+'_original_split_event.csv')
                original_file.write_bytes(gzip.decompress(original_source.read_bytes()))
                row['retained_original_split_event_archive']=str(original_source.resolve())
                row['retained_original_split_event_archive_sha256']=hashlib.sha256(original_source.read_bytes()).hexdigest()
            original=np.loadtxt(original_file,delimiter=',',skiprows=1)
            assert np.max(abs(original[:,0]-expected_times))<1e-10
            row['split_event_vs_regular_grid_force_peak_difference_at_10k_N']=float(np.max(abs(original[:,3]-engine_output[::stride,0])))
            row['original_split_event_metrics']=e['engine_original_metrics']
            row['original_split_event_commands_csv']=str(original_file.resolve())
            row['original_split_event_commands_sha256']=hashlib.sha256(original_file.read_bytes()).hexdigest()
            assert row['original_split_event_commands_sha256']==e['engine_original_commands_sha256']
        rows.append(row)
        del engine_output,commands,cmd
    manifest=dict(status='temporary_regular_grid_open_loop_references_executed',controller_independently_verified=False,retained_binary_bytes=total_forecast,cases=rows,engine_source_sha256=hashlib.sha256((HERE.parent/'engine.c').read_bytes()).hexdigest(),native_force_guard_N=1e-5,footprint_guard_N=1e-4)
    (OUT/args.manifest_name).write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'status':manifest['status'],'cases':len(rows),'temporary_bytes':total_forecast}))

if __name__=='__main__':main()
