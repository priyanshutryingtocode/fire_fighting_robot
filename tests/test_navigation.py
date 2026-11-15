"""Run with: python -m unittest discover -s tests"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import unittest
from navigation import Navigator, SensorData, Action, RobotState


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.nav = Navigator()
        self.no_fire = {"confirmed": False, "fire_direction": "NONE", "strongest": None}

    def test_idle_stops(self):
        d = self.nav.update(self.no_fire, SensorData(100, 100), start=False)
        self.assertEqual(d.action, Action.STOP)
        self.assertEqual(d.state, RobotState.IDLE)

    def test_missing_front_range_stops(self):
        d = self.nav.update(self.no_fire, SensorData(None, 100), start=True)
        self.assertEqual(d.action, Action.STOP)
        self.assertEqual(d.state, RobotState.SAFE_STOP)

    def test_clear_path_searches(self):
        d = self.nav.update(self.no_fire, SensorData(100, 100), start=True)
        self.assertEqual(d.action, Action.FORWARD)

    def test_obstacle_and_clear_rear_reverses(self):
        d = self.nav.update(self.no_fire, SensorData(10, 100), start=True)
        self.assertEqual(d.action, Action.REVERSE)

    def test_obstacle_without_rear_clearance_stops(self):
        d = self.nav.update(self.no_fire, SensorData(10, None), start=True)
        self.assertEqual(d.action, Action.STOP)

    def test_confirmed_fire_left_turns_left(self):
        fire = {"confirmed": True, "fire_direction": "LEFT",
                "strongest": {"class_name": "FIRE"}}
        d = self.nav.update(fire, SensorData(100, 100), start=True)
        self.assertEqual(d.action, Action.TURN_LEFT)


if __name__ == "__main__":
    unittest.main()
