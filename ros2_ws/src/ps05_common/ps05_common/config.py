"""Configuration loading for all PS05 nodes.

Every tunable parameter lives in ``configs/<name>.yaml`` at the repository root
(Rule 11). A scenario file (``simulation/scenarios/<id>.yaml``) may override any
key under ``overrides.<name>``. The active scenario is selected with the
``PS05_SCENARIO_FILE`` environment variable so the same launch files serve the
demo, repeated trials and robustness tests.
"""
import copy
import hashlib
import json
import os
from pathlib import Path

import yaml

CONFIG_NAMES = (
    'camera', 'lidar', 'fusion', 'mapping', 'planner', 'safety', 'mission',
    'landing', 'dashboard', 'localization', 'sensors',
)


def repo_root() -> Path:
    env = os.environ.get('PS05_ROOT')
    if env:
        return Path(env)
    # Fallback: walk up from this file (works for source checkouts / --symlink-install)
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'configs' / 'mission.yaml').exists():
            return parent
    return Path('/workspace')


def config_dir() -> Path:
    return Path(os.environ.get('PS05_CONFIG_DIR', repo_root() / 'configs'))


def deep_merge(base: dict, override: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_scenario() -> dict:
    path = os.environ.get('PS05_SCENARIO_FILE')
    if not path:
        return {}
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load(name: str, scenario: dict | None = None) -> dict:
    """Load configs/<name>.yaml merged with the active scenario's overrides."""
    path = config_dir() / f'{name}.yaml'
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f) or {}
    if not isinstance(cfg, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    scen = load_scenario() if scenario is None else scenario
    return deep_merge(cfg, (scen.get('overrides') or {}).get(name, {}))


def load_all(scenario: dict | None = None) -> dict:
    scen = load_scenario() if scenario is None else scenario
    return {n: load(n, scen) for n in CONFIG_NAMES if (config_dir() / f'{n}.yaml').exists()}


def config_hash(cfg: dict) -> str:
    blob = json.dumps(cfg, sort_keys=True, default=str).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


def require(cfg: dict, *keys):
    """Fetch a nested key, raising a clear error on corrupted/missing configuration."""
    node = cfg
    for k in keys:
        if not isinstance(node, dict) or k not in node:
            raise KeyError(f"missing configuration key: {'.'.join(map(str, keys))}")
        node = node[k]
    return node
