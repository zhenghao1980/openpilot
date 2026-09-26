import sys
from openpilot.common.hardware import PC
print("PC =", PC)
import openpilot.system.manager.process_config as pc
for name, p, _, _ in pc.CONFIGS:
    if 'mapd' in name:
        print(f"{name}: enabled={p.enabled}, cmd={p.cmdline}")
