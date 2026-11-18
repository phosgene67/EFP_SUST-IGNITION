"""Guard against attributing post-entry fuel or unordered alerts to entry."""
from pathlib import Path
import tempfile
import unittest

from summarize_native_logs import read_runs


class NativeLogAuditTests(unittest.TestCase):
    def parse(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'FINAL_SUBMISSION.log'
            path.write_text(content, encoding='utf-8')
            return read_runs(path)

    def test_success_fields_before_message_and_later_damage(self):
        lines = ['2026-11-18T07:20:00 [INFO] IGNITION COAST profile=SCORE_MOMENTUM']
        for alert in ('Total damage taken = 0.0 J', 'Total fuel used = 40.0 kg',
                      'Sim elapsed time = 190.0 s', 'Distance to ship = 49.9 m',
                      'Successfully escaped Phobos with a score of 59.8; congratulations!'):
            lines.append('2026-11-18T07:23:48 [WARN] OnScreenAlert: '+alert)
        lines.append('2026-11-18T07:23:49 [INFO] damage=50000.0J fuel=60.0kg')
        result = self.parse('\n'.join(lines))[0]
        self.assertEqual(result['time_s'], 190.)
        self.assertEqual(result['fuel_kg'], 40.)
        self.assertEqual(result['damage_J'], 0.)
        self.assertIsNone(result['source_sha256'])
        self.assertAlmostEqual(result['score_from_reported_fields'], 59.82456, places=4)

    def test_missing_entry_fields_remain_unknown(self):
        result = self.parse('2026-11-18T07:23:48 [WARN] OnScreenAlert: '
                            'Successfully escaped Phobos with a score of 45.0; congratulations!')[0]
        self.assertIsNone(result['time_s'])
        self.assertIsNone(result['damage_J'])
        self.assertNotIn('score_from_reported_fields', result)

    def test_incomplete_is_not_success(self):
        result = self.parse('2026-11-18T07:23:48 [INFO] IGNITION IMPULSE_COAST '
                            'profile=SCORE_MOMENTUM distance=60m')[0]
        self.assertEqual(result['status'], 'no_official_success_alert')
        self.assertNotIn('score', result)


if __name__ == '__main__':
    unittest.main()
