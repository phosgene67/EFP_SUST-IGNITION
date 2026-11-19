"""Summarize Task 1 telemetry without treating repeated failures as independent debris."""
import argparse
import json
from pathlib import Path

from summarize_score_sweep import compare, distribution, read_cohort, summarize


def audit(records, reference):
    rows, failures, first_failures = [], [], []
    for record in records:
        data = record['result']['dogleg']
        events = data['events']
        rejected = [event for event in events if event['event'] == 'no_safe_lane']
        if len(rejected) != data['no_safe_lane_events']:
            raise ValueError('Failure records truncated or counter disagrees: seed '+str(record['seed']))
        summary = data['run_summary']
        if summary['searches'] != data['searches'] or summary['no_safe_lane_events'] != len(rejected):
            raise ValueError('Run summary disagrees with event counts')
        row = dict(seed=record['seed'], searches=data['searches'],
                   no_safe_lane_events=len(rejected), run_summary=summary)
        rows.append(row)
        for event in rejected:
            failures.append(dict(seed=record['seed'], state_time_s=event['state_time_s'],
                                 **event['debris_distribution']))
        if rejected:
            first = min(rejected, key=lambda event: event['state_time_s'])
            first_failures.append(dict(seed=record['seed'], state_time_s=first['state_time_s'],
                                       **first['debris_distribution']))
    keys = ('slab_count', 'lat_p50_m', 'lat_p90_m', 'lat_p100_m')
    def slabs(events):
        nonempty = [event for event in events if event['slab_count'] > 0]
        return dict(events=len(events), nonempty_slabs=len(nonempty),
                    statistics={key: distribution([event[key] for event in nonempty]) for key in keys})
    pair = compare(reference, records)
    phase_equal = sum(old['result']['fuel_audit'] == new['result']['fuel_audit']
                      for old, new in zip(reference, records))
    return dict(
        kind='modeled_flight_and_sensor_estimates_with_measured_compute',
        baseline=summarize(records),
        settings=dict(experiments=records[0]['experiments'], scenario_sha256=records[0]['scenario_sha256'],
                      controller_sha256=records[0]['controller_sha256'],
                      controller_file_sha256=records[0].get('controller_file_sha256'),
                      rng_mode=records[0].get('rng_mode', 'legacy'),
                      note='No native run, debris truth or native damage-energy estimate used.'),
        no_safe_lane_runs=sum(row['no_safe_lane_events'] > 0 for row in rows),
        no_safe_lane_run_fraction=sum(row['no_safe_lane_events'] > 0 for row in rows)/len(rows),
        searches_per_run=distribution([row['searches'] for row in rows]),
        no_safe_lane_events_per_run=distribution([row['no_safe_lane_events'] for row in rows]),
        total_no_safe_lane_events=len(failures),
        first_failure_per_run=dict(slabs(first_failures), records=first_failures),
        all_failure_events=dict(slabs(failures), records=failures,
            note='Statistics of event-wise percentiles, with repeated correlated observations; not a pooled debris percentile.'),
        first_failure_lat90_above_100m=sum(event['lat_p90_m'] is not None and event['lat_p90_m'] > 100.
                                         for event in first_failures),
        pre_diagnostic_comparison=pair,
        identical_complete_fuel_audits=phase_equal,
        runs=rows,
        interpretation='Nominal radar-center spread is not the minimum bypass offset, occupied volume, or proof that no internal gap exists. Inflation, motion and time-aware clearance are not included in these percentiles.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='tests/score_dogleg_diag_')
    parser.add_argument('--reference-prefix', default='tests/score_dogleg_')
    parser.add_argument('--output', default='tests/dogleg_diagnostics_report.json')
    args = parser.parse_args()
    result = audit(read_cohort(args.prefix), read_cohort(args.reference_prefix))
    Path(args.output).write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in ('baseline', 'no_safe_lane_runs',
        'no_safe_lane_run_fraction', 'total_no_safe_lane_events', 'searches_per_run',
        'no_safe_lane_events_per_run', 'identical_complete_fuel_audits')}, indent=2))
