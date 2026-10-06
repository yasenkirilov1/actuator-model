"""Fixed, declared open-loop excitation cases for native physics parity."""
from pathlib import Path
import importlib.util
import json
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from runner_study import default_config

OUT=HERE.parents[3]/'acf_simulink/validation/sea_pad_dynamic_20261006/native'
rows=[]
for identity,actuator,diameter in [('pilot_coil_point',0,0),('pilot_sea_point',1,0),('pilot_sea_77mm',1,.077)]:
    config=default_config(actuator,20000,80,diameter,5000,.01)
    config.update(force_target=20,mass_s=1.4,angle_deg=45,identified_mass_s=1.4,noise_enabled=0)
    times=np.arange(round(config['duration']*config['control_rate'])+1)/config['control_rate']
    g=config['gravity']*np.cos(np.deg2rad(config['angle_deg']))
    fa=config['force_target']-(config['mass_b']+config['mass_s'])*g
    current=fa/config['force_constant'] if not actuator else (fa-config['nut_mass']*g)*config['lead']/(2*np.pi*config['force_constant'])
    cmd=current+np.where(times>=.5,.002*np.sin(2*np.pi*10*(times-.5)),0)
    path=OUT/(identity+'_commands.csv')
    np.savetxt(path,np.column_stack([times,cmd]),delimiter=',',header='time_s,current_command_A',comments='',fmt='%.16g')
    rows.append(dict(identity=identity,config=config,commands_csv=str(path)))
(OUT/'pilot_cases.json').write_text(json.dumps(dict(status='declared_open_loop_native_physics_pilot',controllerQualification=False,excitation='Holding current plus0.002A10Hz sine from0.5s; same full sharp ridge and2s recording.',cases=rows),indent=2)+'\n')
print(json.dumps(dict(pilot_cases=3,controller_qualification=False)))
