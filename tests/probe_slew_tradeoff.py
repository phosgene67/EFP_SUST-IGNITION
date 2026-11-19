"""Modeled actuator-only slew costs; no debris or native score claims."""
import json
from pathlib import Path
import numpy as np
from controller_loader import load_namespace
from benchmark_final import Allocator, rotate
from benchmark_final_submission import load_scene


if __name__ == '__main__':
    assets = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim')
    scene = load_scene(assets/'Sims/FINAL_SUBMISSION.simconf')
    source = (Path(__file__).resolve().parents[1]/'Controller.py').read_text()
    heading = scene['target']-scene['loc']
    heading /= np.linalg.norm(heading)
    rows = []
    for cap, stopping, settling in ((.4, .03, 1.5), (.6, .05, 4.), (.8, .06, 4.), (.6, .04, 4.)):
        variant = source.replace('rate = min(.4, np.sqrt(2.*.03*max(angle-.01, 0.)))',
            f'rate = min({cap}, np.sqrt(2.*{stopping}*max(angle-.01, 0.)))')
        variant = variant.replace('-1.5*np.asarray(omega)/60.', f'-{settling}*np.asarray(omega)/60.')
        ns = load_namespace(variant)
        for body_axis in (0, 1):
            turn = ns['MainAxisSlew'](heading, 0., body_axis)
            allocator = Allocator(assets/'Data/SpaceBuggyJetConfigNoMains.csv')
            rot, omega = scene['rotation'].copy(), np.zeros(3)
            loc, vel = scene['loc']+2.5*scene['loc']/np.linalg.norm(scene['loc']), .7*heading
            dt, fuel = 1./60., 0.
            for tick in range(2400):
                angular = turn.angular_command(rot, omega, .02)
                if turn.complete:
                    break
                linear = np.zeros(3) if turn.settling else rot@(.025*heading)
                accel, alpha, rate = allocator.apply(linear, angular)
                gravity = -711000.*loc/np.linalg.norm(loc)**3
                acc = rot.T@accel+gravity
                loc += dt*vel+.5*dt*dt*acc
                vel += dt*acc
                rot = rotate(rot, dt*omega+.5*dt*dt*alpha)
                omega += dt*alpha
                fuel += dt*rate
            elapsed = tick*dt
            useful = (vel-.7*heading)@heading+.01*elapsed
            row = dict(rate_cap=cap, stopping=stopping, settling_gain=settling, body_axis=body_axis,
                complete=turn.complete, time_s=elapsed, fuel_kg=fuel, useful_thrust_dv=float(useful),
                overhead_kg=float(fuel-3388.7/(80*9.8)*useful), final_rate=float(np.linalg.norm(omega)))
            rows.append(row)
            print(json.dumps(row), flush=True)
    Path('tests/score55_slew_tradeoff.json').write_text(json.dumps(dict(kind='modeled_no_obstacles', rows=rows), indent=2)+'\n')
