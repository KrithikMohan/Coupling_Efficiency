"""Offline proof that the fake voltage drives a nontrivial motor search."""

import runpy
import unittest
from pathlib import Path

from alignment_routine import search_axis


SCRIPT = runpy.run_path(str(Path(__file__).with_name("Simulate motor movement.py")))


class FakeStage:
    def __init__(self):
        self.counts = SCRIPT["FAKE_START"].copy()
        self.moves = []

    def get_position(self, axis):
        return self.counts[axis]

    def move_to(self, axis, target):
        low, high = SCRIPT["FAKE_BOUNDS"][axis]
        assert low <= target <= high
        self.counts[axis] = target
        self.moves.append((axis, target))
        return target


class FakeVoltageRunTests(unittest.TestCase):
    def test_search_uses_fake_voltage_to_find_each_axis_peak(self):
        stage = FakeStage()
        events = []

        def measure(event, axis="", delta=0):
            events.append(event)
            return SCRIPT["fake_voltage"](stage.counts), 0.0

        best, _ = measure("start")
        budget = [40]
        for axis in SCRIPT["FAKE_ORDER"]:
            best, _ = search_axis(stage, axis, measure, SCRIPT["FAKE_STEPS"][axis],
                                  25000, 1e-5, 101.0, budget, best, SCRIPT["FAKE_BOUNDS"])
        self.assertLessEqual(abs(stage.counts["X"] - SCRIPT["FAKE_PEAK"]["X"]), 5000)
        self.assertEqual(stage.counts["Y"], SCRIPT["FAKE_PEAK"]["Y"])
        self.assertEqual(stage.counts["Z"], SCRIPT["FAKE_PEAK"]["Z"])
        self.assertGreater(len(stage.moves), 9)
        self.assertIn("recover_peak", events)
        self.assertGreater(best, 0.2)


if __name__ == "__main__":
    unittest.main()
