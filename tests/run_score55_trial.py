"""Run one reproducible offline trial with a saved trace and live progress."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--experiment', action='append', default=[])
    parser.add_argument('--seed', type=int, default=100)
    parser.add_argument('--rng-mode', choices=('legacy', 'native-order'), default='legacy')
    parser.add_argument('--radar-clock-offset', type=float, default=0.)
    parser.add_argument('--drift-bias', type=float, nargs=3, default=(0., 0., 0.))
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    assets = Path('C:/Users/Asus/AppData/Local/SimDynamX/STPro/Assets/Repos/STU4-Sim')
    output = root/'tests'/f'score55_{args.name}_{args.seed}.json'
    trace = root/'tests/score_sweeps'/f'score55_{args.name}_trace{args.seed}.txt'
    command = [sys.executable, '-u', '-B', str(root/'tests/benchmark_final_submission.py'),
        '--sim-config', str(assets/'Sims/FINAL_SUBMISSION.simconf'),
        '--jet-config', str(assets/'Data/SpaceBuggyJetConfigNoMains.csv'),
        '--ground-plane', '2.5', '--fuel-audit', '--baseline-file', args.source,
        '--seed', str(args.seed), '--rng-mode', args.rng_mode,
        '--radar-clock-offset', str(args.radar_clock_offset), '--output', str(output)]
    command.extend(['--drift-bias', *map(str, args.drift_bias)])
    for flag in args.experiment:
        command.extend(['--experiment', flag])
    env = os.environ.copy()
    for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
        env[name] = '1'
    print(f'Starting offline {args.name}, seed {args.seed}', flush=True)
    with trace.open('w') as log:
        process = subprocess.Popen(command, cwd=root, env=env, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        for line in process.stdout:
            log.write(line)
            log.flush()
            if line.startswith('t=') or line.startswith('Traceback') or 'Error:' in line:
                print(line.rstrip(), flush=True)
        status = process.wait()
    if status:
        print(f'Failed with exit {status}; see {trace}', flush=True)
        raise SystemExit(status)
    result = json.loads(output.read_text())['result']
    print(json.dumps({k: result.get(k) for k in
        ('time_s', 'fuel_kg', 'zero_damage_score_estimate', 'pre_entry_collision')}), flush=True)
