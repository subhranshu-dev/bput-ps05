"""Make every ROS package importable for pure-Python unit tests (no colcon build needed)."""
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault('PS05_ROOT', str(ROOT))
for pkg in sorted((ROOT / 'ros2_ws' / 'src').iterdir()):
    if (pkg / 'setup.py').exists():
        sys.path.insert(0, str(pkg))
sys.path.insert(0, str(ROOT / 'backend'))
