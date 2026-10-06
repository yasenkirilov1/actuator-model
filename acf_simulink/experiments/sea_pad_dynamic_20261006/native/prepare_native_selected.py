"""Convert the engine's retained completed command cases to native inputs."""
from pathlib import Path
import hashlib
import json
import numpy as np

HERE=Path(__file__).resolve().parent
OUT=HERE.parents[3]/'acf_simulink/validation/sea_pad_dynamic_20261006/native'
source=OUT.parent/'engine/replay/replay_cases.json'
record=json.loads(source.read_text());rows=[]
for case in record['cases']:
    config=json.loads(Path(case['config']).read_text())
    commands=np.loadtxt(case['commands'],delimiter=',',skiprows=1)
    assert commands.shape[1]==4 and np.max(abs(commands[:,2]))==0
    identity='selected_'+case['name'];target=OUT/(identity+'_commands.csv')
    np.savetxt(target,commands[:,:2],delimiter=',',header='time_s,current_command_A',comments='',fmt='%.16g')
    rows.append(dict(identity=identity,config=config,commands_csv=str(target),engine_original_commands_csv=case['commands'],engine_original_commands_sha256=hashlib.sha256(Path(case['commands']).read_bytes()).hexdigest(),engine_original_metrics=case['metrics']))
(OUT/'selected_cases.json').write_text(json.dumps(dict(status='retained_completed_engine_controller_commands_for_native_replay',controllerQualification=False,source_manifest_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=rows),indent=2)+'\n')
print(json.dumps(dict(selected_cases=len(rows),source_manifest=str(source))))
