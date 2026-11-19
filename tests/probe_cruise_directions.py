"""Optimistic one-dimensional score screen for legal fixed-body directions."""
import json
from pathlib import Path
import numpy as np
from benchmark_final import Allocator

if __name__ == '__main__':
    allocator = Allocator(Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv'), 'revised simplex')
    rows = []
    for azimuth in np.arange(0., 360., 30.):
        for tilt in np.arange(0., 180.1, 5.):
            r, az = np.deg2rad(tilt), np.deg2rad(azimuth)
            direction = np.array([np.sin(r)*np.cos(az), np.sin(r)*np.sin(az), np.cos(r)])
            accel, angular, rate = allocator.apply(.5*direction, np.zeros(3))
            thrust = float(accel@direction)
            # Assumed empirical slew interpolation; exclude debris and lateral errors.
            startup = 20.2+15.*tilt/90.
            initial_fuel = 4.83+6.13*tilt/90.
            initial_v = .74+.87*tilt/90.
            drag = .01232
            burn = np.arange(.1, 100., .1)
            peak = initial_v+(thrust-drag)*burn
            remaining = 1250.-initial_v*burn-.5*(thrust-drag)*burn**2
            discriminant = peak**2-2*drag*remaining
            coast = (peak-np.sqrt(np.maximum(discriminant, 0.)))/drag
            times = startup+burn+coast
            fuel = initial_fuel+rate*burn
            score = 100*(.5+120/times-fuel/75.)
            score[(discriminant<0)|(remaining<0)|(coast<0)] = -np.inf
            k = int(np.argmax(score))
            rows.append(dict(tilt=float(tilt), azimuth=float(azimuth), direction=direction.tolist(),
                thrust=thrust, fuel_per_dv=rate/thrust, score=float(score[k]), time=float(times[k]), fuel=float(fuel[k])))
    rows.sort(key=lambda r:r['score'],reverse=True)
    Path('tests/score55_cruise_directions.json').write_text(json.dumps(dict(kind='optimistic_1d_with_assumed_slew_interpolation',rows=rows),indent=2)+'\n')
    print(json.dumps(rows[:12],indent=2),flush=True)
