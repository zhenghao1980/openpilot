import sys
from pathlib import Path
ROOT = Path("/home/zheng/openpilot")
for name in ["opendbc_repo", "msgq_repo", "panda", "rednose_repo", "tinygrad_repo", "teleoprtc_repo"]:
    p = ROOT / name
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
