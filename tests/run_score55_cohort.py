"""Matched offline cohort, source-checked reuse, bounded local parallelism."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--experiment', action='append', default=[])
    parser.add_argument('--start', type=int, default=100)
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256((root/args.source).read_text(encoding='utf-8').encode()).hexdigest()

    def trial(seed):
        output = root/'tests'/f'score55_{args.name}_{seed}.json'
        if output.is_file():
            old = json.loads(output.read_text())
            if old['controller_sha256'] == digest and set(old['experiments']) == set(args.experiment):
                return seed, old
            raise ValueError(f'Stale or mismatched cohort output: {output}')
        command = [sys.executable, '-u', '-B', str(root/'tests/run_score55_trial.py'),
                   '--name', args.name, '--source', args.source, '--seed', str(seed)]
        for flag in args.experiment:
            command.extend(['--experiment', flag])
        with (root/'tests/score_sweeps'/f'score55_{args.name}_worker{seed}.txt').open('w') as log:
            subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=True)
        return seed, json.loads(output.read_text())

    rows = []
    with ThreadPoolExecutor(max_workers=max(1, min(args.workers, 4))) as pool:
        jobs = [pool.submit(trial, seed) for seed in range(args.start, args.start+args.count)]
        for job in as_completed(jobs):
            seed, record = job.result()
            result = record['result']
            rows.append(dict(seed=seed, **result))
            print(json.dumps(dict(seed=seed, score=result['zero_damage_score_estimate'],
                                  time_s=result['time_s'], fuel_kg=result['fuel_kg'],
                                  contact=result['pre_entry_collision'])), flush=True)
    rows.sort(key=lambda r:r['seed'])
    def spread(key):
        values = [r[key] for r in rows if r.get(key) is not None]
        return dict(n=len(values), median=float(np.median(values)),
                    p10=float(np.percentile(values, 10)), p90=float(np.percentile(values, 90)),
                    minimum=min(values), maximum=max(values)) if values else None
    summary = dict(kind='offline_proxy_not_native', controller_sha256=digest,
        experiments=args.experiment, seeds=[r['seed'] for r in rows],
        arrivals=sum(r['arrived'] for r in rows), contacts=sum(bool(r['pre_entry_collision']) for r in rows),
        score=spread('zero_damage_score_estimate'), time_s=spread('time_s'), fuel_kg=spread('fuel_kg'))
    pairs = []
    for row in rows:
        baseline = json.loads((root/'tests'/f"score55_fine_axis_risk_{row['seed']}.json").read_text())['result']
        if row['zero_damage_score_estimate'] is not None and baseline['zero_damage_score_estimate'] is not None:
            pairs.append(dict(seed=row['seed'], score=row['zero_damage_score_estimate']-baseline['zero_damage_score_estimate'],
                time_s=row['time_s']-baseline['time_s'], fuel_kg=row['fuel_kg']-baseline['fuel_kg']))
    summary['paired_against_strong_axis_risk'] = pairs
    if pairs:
        deltas = np.array([r['score'] for r in pairs])
        samples = np.random.default_rng(55).choice(deltas, (10000, len(deltas)))
        summary['paired_score_median'] = float(np.median(deltas))
        summary['paired_score_bootstrap95'] = np.percentile(np.median(samples, axis=1), [2.5, 97.5]).tolist()
    (root/'tests'/f'score55_{args.name}_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary), flush=True)
