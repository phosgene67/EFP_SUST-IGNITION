"""Read official success alerts without assuming their field order.

This is a read-only log audit. It never connects to or commands the simulator.
Missing source hashes, seeds or entry fields remain unknown.
"""
import argparse
import json
from pathlib import Path
import re
import statistics


NUMBER = r'([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)'
FIELDS = {
    'score': r'Successfully escaped Phobos with a score of '+NUMBER,
    'time_s': r'Sim elapsed time\s*=\s*'+NUMBER,
    'fuel_kg': r'Total fuel used\s*=\s*'+NUMBER,
    'damage_J': r'Total damage taken\s*=\s*'+NUMBER,
    'entry_distance_m': r'Distance to ship\s*=\s*'+NUMBER,
}


def read_runs(path):
    lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    successes = [i for i, line in enumerate(lines) if re.search(FIELDS['score'], line)]
    records = []
    for index in successes or [len(lines)]:
        earlier = '\n'.join(lines[:index+1])
        profiles = re.findall(r'profile=(\w+)', earlier)
        hashes = re.findall(r'sourceSHA256=([a-fA-F0-9]{64})', earlier)
        record = dict(directory=str(path.parent), profile=profiles[-1] if profiles else None,
                      source_sha256=hashes[-1] if hashes else None, seed=None,
                      has_fuel_limit='fuelLimit=' in earlier,
                      clean_main_burn='cleanMainBurn=True' in earlier,
                      order_diversity='orderDiversity=True' in earlier,
                      recheck_coast='recheckCoast=True' in earlier,
                      status='official_success' if successes else 'no_official_success_alert')
        if successes:
            # Alerts can put fuel/time/damage BEFORE the success message.
            # Keep only the local alert block at its wall-clock timestamp.
            stamp = lines[index].split(' ', 1)[0]
            alerts = '\n'.join(line for line in lines[max(0, index-12):index+13]
                               if line.startswith(stamp+' ') and 'OnScreenAlert:' in line)
            for field, pattern in FIELDS.items():
                match = re.search(pattern, alerts)
                record[field] = float(match.group(1)) if match else None
            if all(record[field] is not None for field in ('time_s', 'fuel_kg', 'damage_J')):
                record['score_from_reported_fields'] = 100.*(
                    .5+120./record['time_s']-record['fuel_kg']/75.-2.*record['damage_J']/209200.)
                record['score_rounding_difference'] = record['score']-record['score_from_reported_fields']
        records.append(record)
    return records


def inventory(root):
    runs = [record for path in sorted(root.glob('*/FINAL_SUBMISSION.log'))
            for record in read_runs(path)]
    profiles = sorted({record['profile'] for record in runs if record['profile'] is not None})
    summaries = {}
    for profile in profiles:
        arrived = [r for r in runs if r['profile'] == profile and r['status'] == 'official_success']
        scores = [r['score'] for r in arrived]
        summaries[profile] = dict(successes=len(arrived),
            median_score=statistics.median(scores) if scores else None,
            min_score=min(scores) if scores else None, max_score=max(scores) if scores else None,
            source_hashes=sorted({r['source_sha256'] for r in arrived if r['source_sha256']}),
            note='Descriptive mixed-version successes; incomplete runs are listed separately and this is not a matched trial cohort.')
    return dict(kind='measured_native_log_records', root=str(root),
        note='Missing seeds/source identity remain unknown. Profile names alone cannot identify a controller revision. This inventory cannot establish a matched 20-run baseline.',
        runs=runs, profile_summaries=summaries)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.root.is_dir():
        parser.error('Log root does not exist')
    result = inventory(args.root)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result['profile_summaries'], indent=2))
