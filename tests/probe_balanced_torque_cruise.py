"""Offline opposing angular requests through the unchanged official selector.

Only acceleration/angular-acceleration requests are evaluated. No jet on-times
are commanded. Averaged cancellation does not establish closed-loop stability.
"""
import itertools
import json
from pathlib import Path
import numpy as np
from benchmark_final import Allocator

if __name__ == '__main__':
    a = Allocator(Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim/Data/SpaceBuggyJetConfigNoMains.csv'), 'revised simplex')
    rows = []
    for raw in itertools.product((-1., 0., 1.), repeat=3):
        axis = np.array(raw)
        if not np.any(axis) or tuple(axis) > tuple(-axis):
            continue
        axis /= np.linalg.norm(axis)
        for request in (.0001, .001, .003, .01, .02, .05):
            force = .5*np.array([.0002, 0., 1.])/np.sqrt(1.+.0002**2)
            vp, wp, fp = a.apply(force, request*axis)
            vn, wn, fn = a.apply(force, -request*axis)
            positive_fraction = np.linalg.norm(wn)/(np.linalg.norm(wp)+np.linalg.norm(wn))
            v = positive_fraction*vp+(1.-positive_fraction)*vn
            w = positive_fraction*wp+(1.-positive_fraction)*wn
            fuel = positive_fraction*fp+(1.-positive_fraction)*fn
            rows.append(dict(request_axis=axis.tolist(), request_magnitude=request,
                positive_fraction=float(positive_fraction), average_accel=v.tolist(),
                average_angular=w.tolist(), fuel_per_forward_dv=float(fuel/v[2])))
    rows.sort(key=lambda r:r['average_accel'][2], reverse=True)
    Path('tests/score55_balanced_torque_probe.json').write_text(json.dumps(dict(
        kind='modeled_averaged_allowed_requests_not_a_flight_test', rows=rows), indent=2)+'\n')
    print(json.dumps(rows[:10], indent=2))
