import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.aeb import AEBMPC, AEBPID
from town10_adas.config import load_config
from town10_adas.lateral import Pose2D, PurePursuitController, RoutePoint
from town10_adas.longitudinal import LongitudinalPID
from town10_adas.safety import SafetySupervisor


class ControllerTests(unittest.TestCase):
    def test_configuration_loads(self) -> None:
        config = load_config(ROOT / "config" / "town10_adas.json")
        self.assertEqual(config.carla.map, "Town10HD_Opt")
        self.assertEqual(config.scenario.ego_speed_mph, 15.0)

    def test_pure_pursuit_is_neutral_on_straight_route(self) -> None:
        route = [RoutePoint(float(x), 0.0) for x in range(0, 31, 2)]
        controller = PurePursuitController(max_steer=0.18)
        steer = controller.compute(Pose2D(0.0, 0.0, 0.0), 6.0, route)
        self.assertAlmostEqual(steer, 0.0, places=6)

    def test_pure_pursuit_is_bounded(self) -> None:
        route = [RoutePoint(0.0, 20.0), RoutePoint(1.0, 22.0)]
        controller = PurePursuitController(max_steer=0.18)
        steer = controller.compute(Pose2D(0.0, 0.0, 0.0), 6.0, route)
        self.assertLessEqual(abs(steer), 0.18)

    def test_speed_pid_brakes_when_overspeeding(self) -> None:
        controller = LongitudinalPID(0.3, 0.0, 0.0)
        demand = controller.compute(desired_mps=4.0, actual_mps=8.0, dt_s=0.05)
        self.assertEqual(demand.throttle, 0.0)
        self.assertGreater(demand.brake, 0.0)

    def test_supervisor_thresholds_and_latches_aeb(self) -> None:
        supervisor = SafetySupervisor(fcw_ttc_s=3.0, aeb_ttc_s=2.0)
        warning = supervisor.evaluate(ego_speed_mps=10.0, target_speed_mps=5.0, gap_m=14.0)
        self.assertTrue(warning.fcw)
        self.assertFalse(warning.aeb)
        emergency = supervisor.evaluate(ego_speed_mps=10.0, target_speed_mps=5.0, gap_m=9.0)
        self.assertTrue(emergency.aeb)
        latched = supervisor.evaluate(ego_speed_mps=2.0, target_speed_mps=2.0, gap_m=20.0)
        self.assertTrue(latched.aeb)

    def test_infinite_ttc_without_closing_speed(self) -> None:
        result = SafetySupervisor().evaluate(5.0, 5.0, 20.0)
        self.assertTrue(math.isinf(result.ttc_s))
        self.assertFalse(result.fcw)

    def test_aeb_controllers_return_normalized_brake(self) -> None:
        pid_brake = AEBPID(0.45, 0.07, 0.02).compute(actual_accel_mps2=0.0, dt_s=0.05)
        mpc_brake = AEBMPC().compute(ego_speed_mps=12.0, target_speed_mps=0.0, gap_m=12.0)
        self.assertGreater(pid_brake, 0.0)
        self.assertLessEqual(pid_brake, 1.0)
        self.assertGreater(mpc_brake, 0.0)
        self.assertLessEqual(mpc_brake, 1.0)


if __name__ == "__main__":
    unittest.main()
