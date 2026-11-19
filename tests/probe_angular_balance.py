"""Offline legal acceleration-request probe; never commands individual jets."""
import json
from pathlib import Path
import numpy as np
from benchmark_final import Allocator

if __name__ == '__main__':
    allocator = Allocator(Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv'), 'revised simplex')
    rng = np.random.default_rng(5519)
    rows = []
    for axis in (2, 0, 1):
        best = 0.
        for index in range(500):
            command = np.eye(3)[axis]*.5
            angular = np.zeros(3) if index == 0 else rng.normal(size=3)*10.**rng.uniform(-9., -3.)
            accel, alpha, fuel = allocator.apply(command, angular)
            row = dict(axis=axis, request_angular=angular.tolist(), accel=accel.tolist(),
                       angular=alpha.tolist(), fuel_rate=fuel, fuel_per_forward_dv=fuel/accel[axis])
            rows.append(row)
            if accel[axis] > best:
                best = float(accel[axis])
                print(json.dumps(row), flush=True)
    Path('tests/score55_angular_balance.json').write_text(json.dumps(dict(kind='offline_allowed_command_probe', rows=rows), indent=2)+'\n')
