"""Bind the two frozen 77 mm x 5 mm worst histories for native plant replay."""
from pathlib import Path
import gzip
import hashlib
import io
import json
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = ROOT / 'acf_simulink/validation/sea_pad_dynamic_20261006/native'
SOURCE = OUT.parent / 'engine/replay/replay_cases_pad5mm.json'


def main():
    record = json.loads(SOURCE.read_text())
    engine_hash = hashlib.sha256((HERE.parent / 'engine.c').read_bytes()).hexdigest()
    assert engine_hash == record['engine_source_sha256']
    assert len(record['cases']) == 2
    rows = []
    for case in record['cases']:
        config_path = Path(case['config'])
        config = json.loads(config_path.read_text())
        assert config['pad_diameter'] == .077 and config['pad_thickness'] == .005
        assert config['pad_k'] == 50000 and config['pad_c'] == 0
        assert config['pad_compression_limit'] == .0025
        assert case['source_sha256'] == engine_hash
        source_commands = Path(case['commands'])
        original_bytes = gzip.decompress(source_commands.read_bytes())
        commands = np.loadtxt(io.BytesIO(original_bytes), delimiter=',', skiprows=1)
        assert commands.shape[1] == 4 and np.max(abs(commands[:, 2])) == 0
        identity = case['name']
        text = io.StringIO()
        np.savetxt(text, commands[:, :2], delimiter=',', comments='', fmt='%.16g',
                   header='time_s,current_command_A')
        target = OUT / (identity + '_commands.csv.gz')
        target.write_bytes(gzip.compress(text.getvalue().encode(), mtime=0))
        rows.append(dict(identity=identity, config=config, commands_csv=str(target),
                         source_case_identity=case['id'],
                         source_config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
                         engine_original_commands_csv=str(source_commands),
                         engine_original_commands_sha256=hashlib.sha256(original_bytes).hexdigest(),
                         engine_original_commands_archive_sha256=hashlib.sha256(source_commands.read_bytes()).hexdigest(),
                         engine_original_metrics=case['metrics']))
    result = dict(status='two_retained_frozen_gain_5mm_engine_histories_for_native_replay',
                  controllerQualification=False,
                  source_manifest_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                  source_engine_sha256=engine_hash, cases=rows)
    (OUT / 'pad5mm_cases.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(dict(status=result['status'], native_cases=len(rows))))


if __name__ == '__main__':
    main()
