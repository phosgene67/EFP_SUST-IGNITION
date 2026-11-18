"""Assumption-limited radial burn/coast diagnostic, not a flight benchmark.

No obstacles, terrain, attitude cost or camera gate is simulated. The +X case
assumes free alignment with travel; it is not an executable controller plan.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from benchmark_final_submission import load_scene


def evaluate(scene):
    initial_radius = float(np.linalg.norm(scene['loc']))
    distance = float(np.linalg.norm(scene['target']-scene['loc'])-50.)
    results = []
    for name, acceleration in (('+Z', .09581339692790716),
                               ('+X aligned diagnostic', .17456252354760046)):
        for peak in (8., 9.):
            elapsed = position = speed = fuel = burn = 0.
            burning, dt = True, .005
            while position < distance and elapsed < 1000.:
                if speed >= peak:
                    burning = False
                thrust = acceleration if burning else 0.
                net_acceleration = thrust-711000./(initial_radius+position)**2
                position += speed*dt+.5*net_acceleration*dt*dt
                speed += net_acceleration*dt
                fuel += (3388.7/(80.*9.8))*thrust*dt
                burn += dt if burning else 0.
                elapsed += dt
            results.append(dict(axis=name, peak_speed_m_s=peak, time_s=elapsed,
                fuel_kg=fuel, burn_s=burn, entry_speed_m_s=speed,
                score=100.*(.5+120./elapsed-fuel/75.),
                score_if_total_fuel_50kg=100.*(.5+120./elapsed-50./75.)))
    return dict(kind='modeled_1d_diagnostic',
        assumptions='Radial launch from rest; inverse-square gravity; fixed achieved thrust acceleration and reference-mass fuel rate; single uninterrupted burn to specified peak then coast. Published scene range minus 50 m. No terrain, debris, camera gate, station hull, launch delay, slewing or lateral motion. +X aligned case assumes free axis alignment and is not a flyable command sequence.',
        initial_radius_m=initial_radius, distance_to_entry_m=distance, results=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-config', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = evaluate(load_scene(args.sim_config))
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
