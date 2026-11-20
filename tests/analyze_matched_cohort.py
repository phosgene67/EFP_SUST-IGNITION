"""Compare identical proxy seeds; keep failures visible and label model limits."""
import argparse
import json
from pathlib import Path
import numpy as np


def compare(root, baseline_name, candidate_name, seeds):
    profiles, pairs = {}, []
    for name in (baseline_name, candidate_name):
        records = [json.loads((root/f'score55_{name}_{seed}.json').read_text()) for seed in seeds]
        hashes = {r['controller_sha256'] for r in records}
        if len(hashes) != 1:
            raise ValueError(f'Mixed controller revisions in {name}')
        profiles[name] = records
    phase_names = set()
    for before, after in zip(profiles[baseline_name], profiles[candidate_name]):
        if before['seed'] != after['seed']:
            raise ValueError('Unmatched seeds')
        a, b = before['result'], after['result']
        af = a.get('fuel_audit', {}).get('segments', {})
        bf = b.get('fuel_audit', {}).get('segments', {})
        names = set(af) | set(bf)
        phase_names.update(names)
        pairs.append(dict(seed=before['seed'], baseline_arrived=a['arrived'], candidate_arrived=b['arrived'],
            candidate_contact=b['pre_entry_collision'],
            score_delta=(b['zero_damage_score_estimate']-a['zero_damage_score_estimate']
                         if a['arrived'] and b['arrived'] else None),
            fuel_delta_kg=b['fuel_kg']-a['fuel_kg'], time_delta_s=b['time_s']-a['time_s'],
            phase_fuel_delta_kg={name: bf.get(name, {}).get('fuel_kg', 0.)-af.get(name, {}).get('fuel_kg', 0.)
                                 for name in names}))

    def spread(values):
        values = np.asarray(values, float)
        return dict(mean=float(values.mean()), median=float(np.median(values)),
                    p10=float(np.percentile(values, 10)), p90=float(np.percentile(values, 90)),
                    minimum=float(values.min()), maximum=float(values.max()))

    summary = dict(kind='offline_proxy_not_native', baseline=baseline_name, candidate=candidate_name,
        seeds=list(seeds), profiles={}, pairs=pairs,
        limitations='Simplified gravity/synthetic radar/cameras/ellipsoid hull. Wall compute times are not flight-score evidence.')
    for name, records in profiles.items():
        result = [r['result'] for r in records]
        scores = [r['zero_damage_score_estimate'] for r in result if r['arrived']]
        summary['profiles'][name] = dict(controller_sha256=records[0]['controller_sha256'],
            controller_file_sha256=records[0]['controller_file_sha256'],
            arrivals=sum(r['arrived'] for r in result), contacts=sum(r['pre_entry_collision'] for r in result),
            success_scores=spread(scores) if scores else None,
            fuel_kg=spread([r['fuel_kg'] for r in result]), time_s=spread([r['time_s'] for r in result]))
    complete = all(r['score_delta'] is not None for r in pairs)
    summary['all_seeds_have_comparable_arrival_scores'] = complete
    if complete:
        delta = np.array([r['score_delta'] for r in pairs])
        boot = np.random.default_rng(55).choice(delta, size=(10000, len(delta)))
        summary['paired_score_delta'] = spread(delta)
        summary['paired_score_mean_bootstrap95'] = np.percentile(boot.mean(axis=1), [2.5, 97.5]).tolist()
        summary['paired_score_median_bootstrap95'] = np.percentile(np.median(boot, axis=1), [2.5, 97.5]).tolist()
    summary['paired_fuel_delta_kg'] = spread([r['fuel_delta_kg'] for r in pairs])
    summary['paired_time_delta_s'] = spread([r['time_delta_s'] for r in pairs])
    summary['phase_mean_fuel_delta_kg'] = {name: float(np.mean([
        r['phase_fuel_delta_kg'].get(name, 0.) for r in pairs])) for name in sorted(phase_names)}
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    summary = compare(Path(__file__).parent, args.baseline, args.candidate, range(100, 120))
    args.output.write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k not in ('pairs', 'profiles')}))
