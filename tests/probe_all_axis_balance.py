"""Compare allowed near-axis requests through the unchanged selector model."""
import json
from pathlib import Path
import numpy as np
from benchmark_final import Allocator

if __name__ == '__main__':
    allocator = Allocator(Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv'), 'revised simplex')
    rng = np.random.default_rng(5520)
    rows = []
    for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
        best = 0.
        for index in range(600):
            direction = np.eye(3)[axis]*sign
            if index:
                transverse = rng.choice((-1., 1.), 3)*10.**rng.uniform(-8., -2., 3)
                transverse[axis] = 0.
                direction += transverse
            command = .5*direction/np.linalg.norm(direction)
            accel, alpha, fuel = allocator.apply(command, np.zeros(3))
            row = dict(axis=axis, sign=sign, direction=direction.tolist(), accel=accel.tolist(),
                       angular=alpha.tolist(), fuel_rate=fuel, fuel_per_forward_dv=fuel/(sign*accel[axis]))
            rows.append(row)
            if sign*accel[axis] > best:
                best = float(sign*accel[axis])
                print(json.dumps(row), flush=True)
    Path('tests/score55_all_axis_balance.json').write_text(json.dumps(dict(kind='offline_allowed_command_probe', rows=rows), indent=2)+'\n')
