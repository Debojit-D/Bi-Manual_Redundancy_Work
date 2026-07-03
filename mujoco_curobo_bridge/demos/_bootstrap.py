"""Make the bridge packages importable when a demo is run as a file."""

import sys
from pathlib import Path


BRIDGE_ROOT = Path(__file__).resolve().parents[1]

if str(BRIDGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRIDGE_ROOT))
