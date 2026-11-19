"""Actuator-model boost/slew comparisons; no simulator files or live truth."""
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
    # Assumed constant continuation of the residual observed in native coasts.
    bias = np.array([.0023, -.0008, .0016])
    rows = []
    settings = ((.025, .02, False), (.05, .05, False), (.1, .05, False),
                (.2, .05, False), (.3, .05, False), (.2, .05, True), (.3, .05, True))
    for linear_request, angular_limit, sparse in settings:
        up = scene['loc']/np.linalg.norm(scene['loc'])
        loc, vel = scene['loc']+2.8*up, .74*up
        rotation, omega = scene['rotation'].copy(), np.zeros(3)
        heading = scene['target']-loc
        heading /= np.linalg.norm(heading)
        turn = ns['MainAxisSlew'](heading, 0.)
        allocator = Allocator(assets/'Data/SpaceBuggyJetConfigNoMains.csv')
        dt, fuel = 1./60., 0.
        for tick in range(2400):
            angular = turn.angular_command(rotation, omega, angular_limit)
            if turn.complete:
                break
            body = np.zeros(3) if turn.settling else rotation@(linear_request*heading)
            if sparse and not turn.settling:
                component = int(np.argmax(np.abs(body)))
                axis = np.zeros(3)
                axis[component] = np.sign(body[component])*linear_request
                body = axis
            a, alpha, rate = allocator.apply(body, angular)
            acceleration = rotation.T@a-711000.*loc/np.linalg.norm(loc)**3+bias
            loc += dt*vel+.5*dt*dt*acceleration
            vel += dt*acceleration
            rotation = rotate(rotation, dt*omega+.5*dt*dt*alpha)
            omega += dt*alpha
            fuel += dt*rate
        elapsed = tick*dt
        planner = ns['ShootingNavigator'](launch=False)
        planner.body_rotation, planner.dynamics.residual = rotation, bias
        planner.policy.observe(20.2+elapsed, vel, 4.83+fuel)
        planner.command(20.2+elapsed, loc, vel, scene['target'], np.zeros(3), [])
        row = dict(linear_request=linear_request, angular_limit=angular_limit,
            sparse_body_axis=sparse, complete=turn.complete, stage_time_s=elapsed,
            stage_fuel_kg=fuel, stage_end_speed=float(np.linalg.norm(vel)),
            modeled_clean_continuation_score=float(planner.plan_score),
            modeled_arrival_s=planner.previous_eta)
        rows.append(row)
        print(json.dumps(row), flush=True)
    (root/'tests/score55_boost_slew_probe.json').write_text(json.dumps(dict(
        kind='modeled_actuator_and_constant_bias_no_obstacles',
        assumed_initial_time_s=20.2, assumed_initial_fuel_kg=4.83,
        assumed_residual_m_s2=bias.tolist(), rows=rows), indent=2)+'\n')
