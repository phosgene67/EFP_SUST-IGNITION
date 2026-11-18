"""Summarize a complete modeled seed cohort and a matched optional experiment."""
import argparse
import json
from pathlib import Path

import numpy as np


def read_cohort(prefix):
    prefix_path = Path(prefix)
    records = []
    for path in sorted(prefix_path.parent.glob(prefix_path.name+'*.json')):
        suffix = path.stem[len(prefix_path.name):]
        if not suffix.isdecimal():
            continue  # E.g. arcs_v2_100 must not enter the arcs_ cohort.
        record = json.loads(path.read_text())
        if int(suffix) != record['seed']:
            raise ValueError('Filename seed disagrees with record seed')
        records.append(record)
    if len(records) < 20:
        raise ValueError('A cohort must contain at least 20 completed seeds')
    seeds = [record['seed'] for record in records]
    if len(set(seeds)) != len(seeds):
        raise ValueError('Duplicate seed')
    if len({record['controller_sha256'] for record in records}) != 1:
        raise ValueError('Controller source changed during the cohort')
    if len({record.get('rng_mode', 'legacy') for record in records}) != 1:
        raise ValueError('Random-noise model changed during the cohort')
    if len({tuple(sorted(set(record.get('experiments', [])))) for record in records}) != 1:
        raise ValueError('Experiment flags changed during the cohort')
    return sorted(records, key=lambda record: record['seed'])


def distribution(values):
    values = np.asarray(values, float)
    if not len(values):
        return None
    return dict(count=len(values), median=float(np.median(values)), mean=float(np.mean(values)),
                p10=float(np.percentile(values, 10)), p90=float(np.percentile(values, 90)),
                q25=float(np.percentile(values, 25)), q75=float(np.percentile(values, 75)),
                minimum=float(values.min()), maximum=float(values.max()))


def summarize(records):
    results = [record['result'] for record in records]
    clean = [result for result in results if result['zero_damage_score_estimate'] is not None]
    crossing = [result['milestones_by_remaining_metres']['600'] for result in results
                if '600' in result.get('milestones_by_remaining_metres', {})]
    return dict(
        seeds=[record['seed'] for record in records],
        controller_sha256=records[0]['controller_sha256'],
        rng_mode=records[0].get('rng_mode', 'legacy'),
        arrivals=sum(result['arrived'] for result in results),
        pre_entry_contacts=sum(bool(result['pre_entry_collision'] if result['arrived'] else result['collision'])
                               for result in results),
        incomplete_runs=sum(not result['arrived'] for result in results),
        clean_arrival_score=distribution([result['zero_damage_score_estimate'] for result in clean]),
        entry_time_s=distribution([result['time_s'] for result in results if result['arrived']]),
        entry_fuel_kg=distribution([result['fuel_kg'] for result in results if result['arrived']]),
        at_600m=dict(count=len(crossing),
                    time_s=distribution([point['time_s'] for point in crossing]),
                    fuel_kg=distribution([point['fuel_kg'] for point in crossing]),
                    speed_m_s=distribution([point['speed_m_s'] for point in crossing]),
                    below_30kg=sum(point['fuel_kg'] < 30. for point in crossing),
                    below_35kg=sum(point['fuel_kg'] < 35. for point in crossing)),
        native_damage_J=None,
        damage_note='No contact implies modeled zero damage. Contact energy and native damage are not simulated.',
        planner_p95_ms=distribution([result['plan_p95_ms'] for result in results]))


def compare(baseline, candidate):
    if [r['seed'] for r in baseline] != [r['seed'] for r in candidate]:
        raise ValueError('Matched seed sets are required')
    for old, new in zip(baseline, candidate):
        if old.get('rng_mode', 'legacy') != new.get('rng_mode', 'legacy'):
            raise ValueError('Random-noise models must match')
        for field in ('scenario_sha256', 'control_hz', 'allocator_solver', 'ground_plane_m',
                      'drift_bias_m_s2', 'plan_latency_s', 'planning_mode', 'scene_variant'):
            if old[field] != new[field]:
                raise ValueError('Mismatched benchmark setting '+field)
    pairs = [(old['result'], new['result']) for old, new in zip(baseline, candidate)]
    clean = [(old, new) for old, new in pairs
             if old['zero_damage_score_estimate'] is not None and new['zero_damage_score_estimate'] is not None]
    if not clean:
        return dict(clean_pairs=0, decision='inconclusive')
    old_scores = np.array([old['zero_damage_score_estimate'] for old, _ in clean])
    new_scores = np.array([new['zero_damage_score_estimate'] for _, new in clean])
    resamples = np.random.default_rng(20261118).integers(0, len(clean), (10000, len(clean)))
    bootstrap_delta = np.median(new_scores[resamples], axis=1)-np.median(old_scores[resamples], axis=1)
    interval = np.percentile(bootstrap_delta, [2.5, 97.5]).tolist()
    # Each legacy flight score is rounded to 0.01 points. Differences of two
    # cohort medians can therefore shift by 0.01 before any seed variation.
    precision_interval = [interval[0]-.01, interval[1]+.01]
    new_contacts_or_failures = sum(not new['arrived'] or bool(new['pre_entry_collision']) for _, new in pairs)
    phases = sorted({phase for old, new in pairs for result in (old, new)
                     for phase in result['fuel_audit']['segments']})
    phases.append('CRUISE_ALL')
    phase_delta = {}
    for phase in phases:
        def fuel_for(result):
            segments = result['fuel_audit']['segments']
            if phase == 'CRUISE_ALL':
                return sum(value['fuel_kg'] for name, value in segments.items() if name.startswith('CRUISE/'))
            return segments.get(phase, {}).get('fuel_kg', 0.)
        differences = [fuel_for(new)-fuel_for(old) for old, new in clean]
        phase_delta[phase] = distribution(differences)
    crossing_pairs = [(old['milestones_by_remaining_metres']['600'], new['milestones_by_remaining_metres']['600'])
                      for old, new in clean if all('600' in result.get('milestones_by_remaining_metres', {})
                                                  for result in (old, new))]
    decision = 'inconclusive'
    if new_contacts_or_failures:
        decision = 'reject_for_contact_or_incomplete_run'
    elif len(clean) >= 20 and precision_interval[0] > 1e-12:
        decision = 'modeled_gain_above_paired_seed_noise'
    elif interval[1] < 0.:
        decision = 'reject_for_modeled_score_loss'
    return dict(
        clean_pairs=len(clean), median_score_delta=float(np.median(new_scores)-np.median(old_scores)),
        paired_score_deltas=distribution(new_scores-old_scores),
        bootstrap_95pct_median_delta=interval,
        rounding_adjusted_95pct_median_delta=precision_interval,
        score_precision_note='Legacy scores are rounded to 0.01; promotion also requires a positive lower bound after allowing 0.01 for median-difference rounding.',
        paired_time_delta_s=distribution([new['time_s']-old['time_s'] for old, new in clean]),
        paired_fuel_delta_kg=distribution([new['fuel_kg']-old['fuel_kg'] for old, new in clean]),
        phase_fuel_deltas_kg=phase_delta, new_contacts_or_failures=new_contacts_or_failures,
        resolved_incomplete_runs=sum(not old['arrived'] and new['arrived'] for old, new in pairs),
        introduced_incomplete_runs=sum(old['arrived'] and not new['arrived'] for old, new in pairs),
        identical_reported_entry_metrics=sum(
            all(old[key] == new[key] for key in ('arrived', 'time_s', 'fuel_kg',
                                                'pre_entry_collision', 'zero_damage_score_estimate'))
            for old, new in pairs),
        paired_600m_deltas={key: distribution([new[key]-old[key] for old, new in crossing_pairs])
                           for key in ('time_s', 'fuel_kg', 'speed_m_s')},
        decision=decision,
        caveat='Paired bootstrap uses clean arrival pairs only and excludes systematic simulator-model error. Incomplete runs are reported separately; a harness timeout is not an official competition disqualification.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-prefix', required=True)
    parser.add_argument('--candidate-prefix')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    baseline = read_cohort(args.baseline_prefix)
    report = dict(kind='modeled', baseline=summarize(baseline), baseline_runs=baseline)
    if args.candidate_prefix:
        candidate = read_cohort(args.candidate_prefix)
        report.update(candidate=summarize(candidate), candidate_runs=candidate,
                      comparison=compare(baseline, candidate))
    Path(args.output).write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in report.items() if not key.endswith('_runs')}, indent=2))
