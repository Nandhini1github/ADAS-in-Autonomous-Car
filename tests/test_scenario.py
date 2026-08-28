from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from town10_adas.scenario import ScenarioObservation, ScenarioOrchestrator, ScenarioPhase


class ScenarioTests(unittest.TestCase):
    def setUp(self) -> None:
        self.scenario = ScenarioOrchestrator(
            ego_target_mph=15.0,
            target_target_mph=10.0,
            stable_tolerance_mph=1.0,
            stable_hold_s=3.0,
            stop_speed_mph=1.0,
            stop_hold_s=1.0,
            timeout_s=60.0,
        )

    def observation(self, time_s: float, ego: float, target: float, aeb: bool = False) -> ScenarioObservation:
        return ScenarioObservation(time_s, ego, target, aeb)

    def test_full_scenario_sequence(self) -> None:
        self.assertEqual(self.scenario.update(self.observation(0.0, 0.0, 0.0)), ScenarioPhase.ACCELERATE)
        self.assertEqual(self.scenario.update(self.observation(10.0, 15.0, 10.0)), ScenarioPhase.HOLD)
        self.assertEqual(self.scenario.update(self.observation(13.1, 15.1, 10.2)), ScenarioPhase.THREAT)
        self.assertTrue(self.scenario.target_hard_brake)
        self.assertEqual(self.scenario.update(self.observation(15.0, 14.0, 0.0, True)), ScenarioPhase.AEB)
        self.assertEqual(self.scenario.update(self.observation(20.0, 0.5, 0.0, True)), ScenarioPhase.AEB)
        self.assertEqual(self.scenario.update(self.observation(21.1, 0.4, 0.0, True)), ScenarioPhase.COMPLETE)
        self.assertTrue(self.scenario.finished)

    def test_unstable_hold_returns_to_accelerate(self) -> None:
        self.scenario.update(self.observation(5.0, 15.0, 10.0))
        result = self.scenario.update(self.observation(6.0, 17.0, 10.0))
        self.assertEqual(result, ScenarioPhase.ACCELERATE)

    def test_timeout_is_explicit_abort_not_route_end(self) -> None:
        result = self.scenario.update(self.observation(60.0, 5.0, 4.0))
        self.assertEqual(result, ScenarioPhase.ABORTED)


if __name__ == "__main__":
    unittest.main()
