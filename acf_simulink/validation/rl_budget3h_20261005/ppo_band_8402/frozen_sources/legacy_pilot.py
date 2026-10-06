"""Bounded early PPO diagnostic while the improved shared actor is prepared.

Uses the retained native graph and original policy family. Changes are explicit:
zero value initialization and a smaller common learning rate. No teacher data.
"""
import json
from pathlib import Path
import sys
import torch

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / "rl_unified_20261005"
sys.path.insert(0, str(OLD))
import train

original_policy = train.UnifiedPolicy
original_config = train.PPOConfig

def policy(*args, **kwargs):
    result = original_policy(*args, **kwargs)
    torch.nn.init.zeros_(result.critic.weight)
    torch.nn.init.zeros_(result.critic.bias)
    return result

def config(*args, **kwargs):
    return original_config(*args, learning_rate=1e-5, **kwargs)

train.UnifiedPolicy = policy
train.PPOConfig = config
if __name__ == "__main__":
    output = Path(sys.argv[sys.argv.index("--output") + 1])
    try:
        train.main()
    finally:
        if output.exists():
            (output / "legacy_pilot.py").write_bytes(Path(__file__).read_bytes())
            (output / "pilot_overrides.json").write_text(json.dumps({
                "purpose": "Learning-rate/value-initialization diagnostic",
                "learning_rate": 1e-5,
                "critic_initial_weight_and_bias": 0,
                "teacher_data": False,
                "selection_only": True,
            }, indent=2) + "\n")
