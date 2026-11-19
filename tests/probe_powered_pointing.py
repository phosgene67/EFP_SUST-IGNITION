"""Model a small main-axis turn while thrusting; no live simulator commands."""
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
    bias = np.array([.0023, -.0008, .0016])
    rows = []
    for kp, kd in ((.3, 1.1), (.6, 1.55), (.9, 1.9)):
        up = scene['loc']/np.linalg.norm(scene['loc'])
        loc, vel = scene['loc']+2.7*up, .74*up
        rotation, omega = scene['rotation'].copy(), np.zeros(3)
        gravity = -711000.*loc/np.linalg.norm(loc)**3+bias
        forward = scene['target']-loc
        forward /= np.linalg.norm(forward)
        horizon = 185.
        # This diagnostic supplies the true assumed constant bias; flight code
        # would have to estimate it. The result is an optimistic continuation.
        force = scene['target']-48.*forward-loc-vel*horizon-.5*gravity*horizon*horizon
        heading = force/np.linalg.norm(force)
        allocator = Allocator(assets/'Data/SpaceBuggyJetConfigNoMains.csv')
        dt, fuel, complete = 1./60., 0., False
        for tick in range(1800):
            body_heading = rotation@heading
            angle = np.arccos(np.clip(body_heading[2], -1., 1.))
            axis = np.cross(np.array([0., 0., 1.]), body_heading)
            error = angle*axis/max(np.linalg.norm(axis), 1e-12)
            angular = ns['limit_vector']((kp*error-kd*omega)/.25, .05)
            if angle < np.deg2rad(.5) and np.linalg.norm(omega) < .0001:
                complete = True
                break
            body = .5*np.array([.0002, 0., 1.])/np.sqrt(1.+.0002**2)
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
        p.efficient_main, p.main_accel_preloaded = True, True
        p.axis_accels[0, 2] = .13467559
        p.near_entry_goal = p.accept_safe_pass = True
        p.policy.observe(14.+elapsed, vel, 4.05+fuel)
        p.command(14.+elapsed, loc, vel, scene['target'], np.zeros(3), [])
        row = dict(kp=kp, kd=kd, complete=complete, stage_s=elapsed,
            stage_fuel_kg=fuel, end_speed=float(np.linalg.norm(vel)),
            angle_deg=float(np.rad2deg(angle)), modeled_score=float(p.plan_score),
            modeled_arrival_s=p.previous_eta)
        rows.append(row)
        print(json.dumps(row), flush=True)
    (root/'tests/score55_powered_pointing_probe.json').write_text(json.dumps(dict(
        kind='modeled_no_obstacles_optimistic_known_bias_boundary_aim',
        assumed_start_time_s=14., assumed_start_fuel_kg=4.05,
        assumed_bias_m_s2=bias.tolist(), rows=rows), indent=2)+'\n')
