"""Legal command-direction calibration through the official revised-simplex model."""
import json
from pathlib import Path
import numpy as np
from benchmark_final import Allocator


if __name__ == '__main__':
    jets = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv')
    allocator = Allocator(jets, 'revised simplex')
    values = np.r_[-np.array([.01, .001, .0002, .0001, .00002, .00001, .000002, .000001, .0000002]),
                   0., np.array([.0000002, .000001, .000002, .00001, .00002, .0001, .0002, .001, .01])]
    rows, best = [], 0.
    for x in values:
        for y in values:
            direction = np.array([x, y, 1.])
            command = .5*direction/np.linalg.norm(direction)
            accel, alpha, rate = allocator.apply(command, np.zeros(3))
            row = dict(x_fraction=float(x), y_fraction=float(y),
                accel=accel.tolist(), angular=alpha.tolist(), fuel_rate=float(rate),
                fuel_per_forward_dv=float(rate/accel[2]))
            rows.append(row)
            if accel[2] > best+1e-7:
                best = float(accel[2])
                print(json.dumps(row), flush=True)
    rows.sort(key=lambda r: r['accel'][2], reverse=True)
    Path('tests/score55_direction_balance.json').write_text(json.dumps(dict(
        kind='modeled_official_revised_simplex_command_direction_probe',
        samples=len(rows), rows=rows), indent=2)+'\n')
