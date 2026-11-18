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
    def run_adapter(self, radar, camera_feed=False, target_position=None, scoring_distance=None, fuel_used=.05):
        commands = []
        now = [0.]
        st = mock.MagicMock()
        camera_api = None
        if camera_feed:
            from test_camera_fusion import CameraAPI
            camera_api = CameraAPI()
            st.camera = camera_api
            st.OutputMode = camera_api.OutputMode
            st.CaptureImageProperties = camera_api.CaptureImageProperties
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
        shuttle.getLocVelAcc.return_value = state([8000., 0., 0.])
        target.getLocVelAcc.return_value = state([9305.8, 0., 0.] if target_position is None else target_position)
        if scoring_distance is not None:
            shuttle.getLocation.return_value.WRT_ExprIn.return_value = np.array([scoring_distance, 0., 0.])
        shuttle.GetParam.side_effect = lambda kind, key: (
            fuel_used if key[-1] == 'FuelUsed' else .5 if key[-1] == 'MaxAccel_m_s2' else .05)
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
            if camera_api is not None:
                camera_api.now = now[0]
            if now[0] > 2.:
                raise StopSimulation()
        with mock.patch.dict('sys.modules', {'spaceteams': st}), mock.patch('time.sleep', tick), \
                mock.patch.dict('os.environ', {'EFP_CAMERA_GUI': '0'}):
            self.runtime = {'__file__': str(SOURCE)}
            with self.assertRaises(StopSimulation):
                exec(compile(SOURCE.read_text(), str(SOURCE), 'exec'), self.runtime)
        st.connect_to_sim.assert_called_once()
        st.leave_sim.assert_called_once()
        return commands

    def test_frame_conversion_limits_and_cleanup(self):
        report = np.array([[1000., 0., 0.], [990., 0., 20.], [0., 0., 0.]])
        commands = self.run_adapter([report])
        linear = [v for name, v in commands if name == 'CommandedAccel']
        self.assertGreater(len(linear), 20)
        # The radar fixture lies on the direct route, so A* may request a small
        # lateral detour. World-forward thrust must still map chiefly to body -Y.
        self.assertLess(linear[0][1], -.25)
        self.assertGreater(np.linalg.norm(linear[0]), .25)  # Launch may use partial thrust.
        for _, value in commands:
            self.assertTrue(np.isfinite(value).all())
            self.assertLessEqual(np.linalg.norm(value), .500001)
        np.testing.assert_allclose(linear[-1], np.zeros(3))

    def test_empty_radar_holds_position(self):
        commands = self.run_adapter([])
        for name, value in commands:
            if name == 'CommandedAccel':
                np.testing.assert_allclose(value, np.zeros(3))

    def test_fuel_limit_stops_linear_and_angular_commands_in_actual_runtime(self):
        report = np.array([[1000., 0., 0.], [990., 0., 20.], [0., 0., 0.]])
        for fuel in (49.5, 50., float('nan')):
            with self.subTest(fuel=fuel):
                commands = self.run_adapter([report], fuel_used=fuel)
                self.assertTrue(self.runtime['fuel_budget'].exhausted)
                for _, value in commands:
                    np.testing.assert_array_equal(value, np.zeros(3))

    def test_arrival_uses_official_distance_between_planning_ticks(self):
        report = np.array([[1000., 0., 0.], [990., 0., 20.], [0., 0., 0.]])
        commands = self.run_adapter([report], target_position=[8050.1, 0., 0.], scoring_distance=49.9)
        self.assertIsNotNone(self.runtime['planner'].arrival_time)
        self.assertTrue(self.runtime['planner'].terminal.entered)
        self.assertTrue(self.runtime['fuel_ledger'].frozen)
        self.assertEqual(self.runtime['mode'], 'MISSION_COMPLETE_COAST')
        for _, value in commands:
            np.testing.assert_array_equal(value, np.zeros(3))

    def test_invalid_radar_holds_position(self):
        commands = self.run_adapter([np.full((3, 3), np.nan), np.zeros((3, 3))])
        for name, value in commands:
            if name == 'CommandedAccel':
                np.testing.assert_allclose(value, np.zeros(3))

    def test_actual_runtime_processes_async_camera_when_radar_missing(self):
        commands = self.run_adapter([], camera_feed=True)
        linear = [v for name, v in commands if name == 'CommandedAccel']
        self.assertTrue(any(np.linalg.norm(v) > .01 for v in linear[:-1]))
        np.testing.assert_allclose(linear[-1], np.zeros(3))
        # The high-speed profile must not outrun the only remaining sensor.
        self.assertLessEqual(self.runtime['cruise'], np.sqrt(2.*.12*(220.-40.))+1e-6)


if __name__ == '__main__':
    unittest.main()
