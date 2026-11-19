"""Actuator-model score comparison of positive/negative main-axis turns."""
import json
from pathlib import Path
import numpy as np
from controller_loader import load_namespace
from benchmark_final import Allocator, rotate
from benchmark_final_submission import load_scene

if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    assets = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim')
    scene = load_scene(assets/'Sims/FINAL_SUBMISSION.simconf')
    ns = load_namespace((root/'Controller.py').read_text())
    bias, rows = np.array([.0023, -.0008, .0016]), []
    for axis, sign, calibrated in ((0, 1., False), (1, 1., False), (0, -1., True),
                                  (1, -1., True), (2, 1., True)):
        up = scene['loc']/np.linalg.norm(scene['loc'])
        loc, vel = scene['loc']+2.8*up, .74*up
        rotation, omega = scene['rotation'].copy(), np.zeros(3)
        heading = scene['target']-loc
        heading /= np.linalg.norm(heading)
        turn = ns['MainAxisSlew'](sign*heading, 0., axis)
        allocator = Allocator(assets/'Data/SpaceBuggyJetConfigNoMains.csv')
        dt, fuel = 1./60., 0.
        for tick in range(2400):
            angular = turn.angular_command(rotation, omega, .02)
            if turn.complete:
                break
            body = np.zeros(3) if turn.settling else rotation@(.025*heading)
            a, alpha, rate = allocator.apply(body, angular)
            acceleration = rotation.T@a-711000.*loc/np.linalg.norm(loc)**3+bias
            loc += dt*vel+.5*dt*dt*acceleration
            vel += dt*acceleration
            rotation = rotate(rotation, dt*omega+.5*dt*dt*alpha)
            omega += dt*alpha
            fuel += dt*rate
        elapsed = tick*dt
        p = ns['ShootingNavigator'](launch=False)
        p.body_rotation, p.dynamics.residual = rotation, bias
        if calibrated:
            p.axis_accels[1, 0] = .1606023181
            p.axis_accels[1, 1] = .1796371225
            p.axis_accels[0, 2] = .13467559
            p.main_accel_preloaded = True
        p.policy.observe(20.2+elapsed, vel, 4.83+fuel)
        p.command(20.2+elapsed, loc, vel, scene['target'], np.zeros(3), [])
        row = dict(axis=axis, sign=sign, calibrated_main=calibrated, complete=turn.complete,
            stage_s=elapsed, stage_fuel_kg=fuel, stage_speed=float(np.linalg.norm(vel)),
            modeled_clean_score=float(p.plan_score), modeled_arrival_s=p.previous_eta)
        rows.append(row)
        print(json.dumps(row), flush=True)
    (root/'tests/score55_signed_axis_slew_probe.json').write_text(json.dumps(dict(
        kind='modeled_constant_bias_no_obstacles', assumed_start_s=20.2,
        assumed_start_fuel_kg=4.83, assumed_bias=bias.tolist(), rows=rows), indent=2)+'\n')
