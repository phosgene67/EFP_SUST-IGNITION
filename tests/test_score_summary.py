"""Prevent mixed models or reporting precision from creating a false win."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from summarize_score_sweep import compare, read_cohort


def cohort(score):
    records = []
    for seed in range(100, 120):
        records.append(dict(seed=seed, scenario_sha256='fixed', control_hz=60.,
            allocator_solver='highs', ground_plane_m=2.5, drift_bias_m_s2=[0., 0., 0.],
            plan_latency_s=0., planning_mode='async', scene_variant='final-scene',
            result=dict(arrived=True, pre_entry_collision=False, time_s=250., fuel_kg=40.,
                        zero_damage_score_estimate=score,
                        fuel_audit=dict(segments={'CRUISE/IMPULSE_BURN': {'fuel_kg': 35.}}))))
    return records


class ScoreSummaryTests(unittest.TestCase):
    def test_nested_experiment_names_do_not_enter_the_parent_cohort(self):
        with tempfile.TemporaryDirectory() as directory:
            for record in cohort(40.):
                record['controller_sha256'] = 'baseline-source'
                (Path(directory)/('trial_'+str(record['seed'])+'.json')).write_text(json.dumps(record))
            different = copy.deepcopy(record)
            different.update(seed=100, controller_sha256='different-source')
            (Path(directory)/'trial_v2_100.json').write_text(json.dumps(different))
            self.assertEqual(len(read_cohort(str(Path(directory)/'trial_'))), 20)

    def test_clear_gain_is_accepted(self):
        result = compare(cohort(40.), cohort(45.))
        self.assertEqual(result['median_score_delta'], 5.)
        self.assertEqual(result['decision'], 'modeled_gain_above_paired_seed_noise')

    def test_one_reporting_quantum_is_inconclusive(self):
        result = compare(cohort(40.), cohort(40.01))
        self.assertEqual(result['decision'], 'inconclusive')

    def test_contact_rejects_an_otherwise_high_scoring_candidate(self):
        candidate = cohort(60.)
        candidate[0]['result'].update(pre_entry_collision=True, zero_damage_score_estimate=None)
        result = compare(cohort(40.), candidate)
        self.assertEqual(result['decision'], 'reject_for_contact_or_incomplete_run')

    def test_rng_model_mismatch_is_rejected(self):
        old = cohort(40.)
        candidate = copy.deepcopy(old)
        for record in candidate:
            record['rng_mode'] = 'native-order'
        with self.assertRaisesRegex(ValueError, 'Random-noise models'):
            compare(old, candidate)


if __name__ == '__main__':
    unittest.main()
