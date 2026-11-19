"""Offline cost of a single main-axis slew; no simulation files are modified."""
from pathlib import Path
import json
import numpy as np
from benchmark_final import Allocator, rotate
from benchmark_final_submission import load_scene


def slew_command(rotation, heading, omega, maximum=.02):
    body_heading = rotation@heading
    angle = np.arccos(np.clip(body_heading[0], -1., 1.))
    axis = np.cross(np.array([1., 0., 0.]), body_heading)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        axis = np.array([0., 1., 0.])
    else:
        axis /= norm
    rate = min(.4, np.sqrt(2.*.03*max(angle-.01, 0.)))
    desired = axis*rate if angle > .02 else np.zeros(3)
    command = 1.5*(desired-omega)
    command *= min(1., maximum/max(np.linalg.norm(command), 1e-9))
    if angle < np.deg2rad(2.) and np.linalg.norm(omega) < .005:
        return np.zeros(3), True
    return command, False


def evaluate(scene, jets, translation):
    allocator = Allocator(jets)
    rotation, omega = scene['rotation'].copy(), np.zeros(3)
    heading = scene['target']-scene['loc']
    heading /= np.linalg.norm(heading)
    initial = scene['loc']+2.5*scene['loc']/np.linalg.norm(scene['loc'])
    loc = initial.copy()
    velocity = .7*heading
    dt, fuel, now, done = 1./60., 0., 0., False
    while now < 40.:
        angular, done = slew_command(rotation, heading, omega)
        if done:
            break
        achieved, alpha, rate = allocator.apply(rotation@(translation*heading), angular)
        gravity = -711000.*loc/np.linalg.norm(loc)**3
        accel = rotation.T@achieved+gravity
        loc += dt*velocity+.5*dt*dt*accel
        velocity += dt*accel
        rotation = rotate(rotation, dt*omega+.5*dt*dt*alpha)
        omega += dt*alpha
        fuel += dt*rate
        now += dt
    return dict(translation=translation, complete=done, time_s=now, fuel_kg=fuel,
                useful_thrust_dv=float((velocity-.7*heading)@heading+.01*now),
                final_speed=float(np.linalg.norm(velocity)),
                position_gain=float((loc-initial)@heading),
                angle_deg=float(np.rad2deg(np.arccos(np.clip(rotation[0]@heading, -1., 1.)))),
                rate=float(np.linalg.norm(omega)))


if __name__ == '__main__':
    assets=Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim')
    scene=load_scene(assets/'Sims/FINAL_SUBMISSION.simconf')
    rows=[evaluate(scene,assets/'Data/SpaceBuggyJetConfigNoMains.csv',value)
          for value in (0.,.025,.05,.1,.2,.3,.5)]
    Path('tests/score55_axis_slew_probe.json').write_text(json.dumps(
        dict(kind='offline_slew_diagnostic_no_obstacles',results=rows),indent=2)+'\n')
    for row in rows:
        print(json.dumps(row),flush=True)
