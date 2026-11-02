"""Execute the actual connection/loop/command code against a small API double."""
import datetime
from types import SimpleNamespace
from unittest import mock
import unittest
import numpy as np
from test_controller import SOURCE


class StopSimulation(Exception):
    pass


class AdapterTests(unittest.TestCase):
    def run_adapter(self, radar):
        commands = []
        now = [0.]
        st = mock.MagicMock()
        frame = object()
        shuttle = mock.MagicMock()
        target = mock.MagicMock()
        phobos = mock.MagicMock()
        phobos.GetBodyFixedFrame.return_value = frame
        entities = {'Shuttle': shuttle, 'Target': target, 'Phobos': phobos,
                    'TopCamera': mock.MagicMock(), 'BottomCamera': mock.MagicMock()}
        st.GetThisSystem.return_value.GetParam.side_effect = lambda kind, key: (
            60. if key == 'LoopFreqHz' else entities[key])
        st.SimGlobals.GetSimEntity.return_value.GetParam.return_value = False
        def state(position):
            return SimpleNamespace(loc_WRT_ExprIn=lambda f: np.array(position),
                                   vel_WRT_ExprIn=lambda f: np.zeros(3))
        shuttle.getLocVelAcc.return_value = state([0., 0., 0.])
        target.getLocVelAcc.return_value = state([300., 0., 0.])
        shuttle.GetParam.side_effect = lambda kind, key: .5 if key[-1] == 'MaxAccel_m_s2' else .05
        shuttle.GetParamArray.return_value = radar
        shuttle.SetParam.side_effect = lambda kind, key, value: commands.append((key[-1], np.array(value)))
        shuttle.getAngVelocity.return_value.WRT.return_value.ExprIn.return_value = np.zeros(3)
        # A rotated shuttle: world +X maps to body -Y.
        transform = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
        shuttle.getRotation.return_value.DCM_WRT.return_value = transform
        st.frames.FramedVector.side_effect = lambda v, f: SimpleNamespace(ExprIn=lambda b: transform@v)
        epoch = datetime.datetime(2026, 1, 1)
        st.SimGlobals.SimClock.GetTimeNow.side_effect = lambda: SimpleNamespace(
            as_datetime=lambda: epoch+datetime.timedelta(seconds=now[0]))
        def tick(seconds):
            now[0] += .05
            if now[0] > 2.:
                raise StopSimulation()
        with mock.patch.dict('sys.modules', {'spaceteams': st}), mock.patch('time.sleep', tick):
            with self.assertRaises(StopSimulation):
                exec(compile(SOURCE.read_text(), str(SOURCE), 'exec'), {})
        st.connect_to_sim.assert_called_once()
        st.leave_sim.assert_called_once()
        return commands

    def test_frame_conversion_limits_and_cleanup(self):
        report = np.array([[1000., 0., 0.], [990., 0., 20.], [0., 0., 0.]])
        commands = self.run_adapter([report])
        linear = [v for name, v in commands if name == 'CommandedAccel']
        self.assertGreater(len(linear), 20)
        np.testing.assert_allclose(linear[0], [0., -.25, 0.])
        for _, value in commands:
            self.assertTrue(np.isfinite(value).all())
            self.assertLessEqual(np.linalg.norm(value), .250001)
        np.testing.assert_allclose(linear[-1], np.zeros(3))

    def test_empty_radar_holds_position(self):
        commands = self.run_adapter([])
        for name, value in commands:
            if name == 'CommandedAccel':
                np.testing.assert_allclose(value, np.zeros(3))

    def test_invalid_radar_holds_position(self):
        commands = self.run_adapter([np.full((3, 3), np.nan), np.zeros((3, 3))])
        for name, value in commands:
            if name == 'CommandedAccel':
                np.testing.assert_allclose(value, np.zeros(3))


if __name__ == '__main__':
    unittest.main()
