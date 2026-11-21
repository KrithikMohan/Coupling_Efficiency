"""Offline checks for the meter-guided search decisions."""

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from alignment_routine import PZC200Stage, ThorlabsPM100USB, efficiency_percent, run_alignment, search_axis


class FakeStage:
    def __init__(self):
        self.y = 250000
        self.moves = []

    def get_position(self, axis):
        assert axis == "Y"
        return self.y

    def move_to(self, axis, target):
        assert axis == "Y"
        self.y = target
        self.moves.append(target)
        return target


class SearchTests(unittest.TestCase):
    def test_efficiency_uses_dark_corrected_project_reference(self):
        dark = 4e-9
        self.assertAlmostEqual(efficiency_percent(dark + 0.65**2 * 10e-6 * 0.86, dark), 65.0)

    def test_recovers_peak_after_power_falls(self):
        stage = FakeStage()
        events = []

        def measure(event, axis, delta):
            events.append(event)
            return 1e-6 - abs(stage.y - 150000) * 1e-12, 0.0

        power, reached = search_axis(stage, "Y", measure, 100000, 16000,
                                     1e-10, 65.0, [60], measure("start", "Y", 0)[0],
                                     {"Y": (-1, 416700)})
        self.assertFalse(reached)
        self.assertEqual(stage.y, 150000)
        self.assertAlmostEqual(power, 1e-6)
        self.assertIn("recover_peak", events)

    def test_stops_when_efficiency_target_is_measured(self):
        stage = FakeStage()

        def measure(event, axis, delta):
            return 1e-6 - abs(stage.y - 150000) * 1e-12, 70.0 if stage.y == 150000 else 0.0

        _, reached = search_axis(stage, "Y", measure, 100000, 16000,
                                 1e-10, 65.0, [60], measure("start", "Y", 0)[0],
                                 {"Y": (-1, 416700)})
        self.assertTrue(reached)
        self.assertEqual(stage.y, 150000)

    def test_nonidle_origin_sends_no_move(self):
        class Meter:
            def connect(self):
                return "fake meter"

            def set_wavelength(self, value):
                self.wavelength = value

            def close(self):
                pass

        class Stage:
            def origin_candidate(self):
                raise RuntimeError("axis not idle")

            def move_to(self, axis, target):
                raise AssertionError("Movement was sent before READY")

            def close(self):
                pass

        args = SimpleNamespace(dark_power_w=4e-9, samples=3, min_step=16000,
                               max_moves=60, target_efficiency=65.0,
                               min_improvement_v=1e-5, x_step=30000,
                               y_step=100000, z_step=100000,
                               z_sweep_step=50000, signal_rise_v=1e-8,
                               output=Path("unused-coupling-test.csv"))
        with patch("alignment_routine.ThorlabsPM100USB", return_value=Meter()), \
                patch("alignment_routine.PZC200Stage", return_value=Stage()):
            with self.assertRaisesRegex(RuntimeError, "not idle"):
                run_alignment(args)

    def test_noisy_dark_signal_scans_y_then_x_after_z_cap(self):
        class Meter:
            def connect(self):
                return "fake meter"

            def set_wavelength(self, _value):
                pass

            def read_voltage(self):
                return -5e-10

            def read_power(self):
                return -6e-10

            def close(self):
                pass

        class Stage:
            def __init__(self):
                self.counts = {"X": 0, "Y": 0, "Z": 0}
                self.moves = []

            def origin_candidate(self):
                return self.counts.copy()

            def establish_origin(self, expected):
                self.asserted = expected.copy()
                self.counts = {"X": 0, "Y": 0, "Z": 0}
                return self.counts.copy()

            def get_position(self, axis):
                return self.counts[axis]

            def positions(self):
                return self.counts.copy()

            def get_limits(self, axis):
                return {"X": (-1, 61450), "Y": (-1, 416700), "Z": (-1, 150000)}[axis]

            def move_to(self, axis, target):
                self.moves.append((axis, target))
                self.counts[axis] = target
                return target

            def close(self):
                pass

        stage = Stage()
        with TemporaryDirectory() as directory:
            args = SimpleNamespace(dark_power_w=None, samples=3, min_step=16000,
                                   max_moves=60, target_efficiency=65.0,
                                   min_improvement_v=2e-10, x_step=30000,
                                   y_step=100000, z_step=100000,
                                   z_sweep_step=50000, signal_rise_v=1e-8,
                                   output=Path(directory) / "run.csv")
            with patch("alignment_routine.ThorlabsPM100USB", return_value=Meter()), \
                    patch("alignment_routine.PZC200Stage", return_value=stage), \
                    patch("alignment_routine.time.sleep"):
                run_alignment(args)
        self.assertEqual(stage.moves[:3], [("Z", count) for count in range(50000, 150001, 50000)])
        self.assertEqual(stage.moves[3][0], "Y")
        first_x = next(index for index, (axis, _) in enumerate(stage.moves) if axis == "X")
        self.assertTrue(all(axis == "Y" for axis, _ in stage.moves[3:first_x]))
        self.assertEqual(stage.moves[first_x + 1][0], "Y")
        self.assertLess(stage.moves[first_x + 1][1], stage.moves[first_x - 1][1])
        self.assertLessEqual(stage.counts["X"], 61450)
        self.assertLessEqual(stage.counts["Z"], 150000)

    def test_origin_selection_uses_or_and_installs_limits_without_pr(self):
        stage = PZC200Stage.__new__(PZC200Stage)
        stage.ser = object()
        positions = {1: 123, 2: -45, 3: 1000}
        limits = {axis: [-10000000, 10000000] for axis in positions}
        commands = []

        def read_number(_serial, command):
            axis = int(command[0])
            suffix = command[1:]
            if suffix == "TP?":
                return positions[axis]
            if suffix == "TE?":
                return 0
            if suffix == "SL?":
                return limits[axis][0]
            if suffix == "SR?":
                return limits[axis][1]
            raise AssertionError(command)

        def query(_serial, command):
            return f"{command} PZA12" if command.endswith("ID?") else f"{command} @"

        def send(_serial, command):
            commands.append(command)
            axis = int(command[0])
            suffix = command[1:]
            if suffix == "OR":
                positions[axis] = 0
            elif suffix.startswith("SL"):
                limits[axis][0] = int(suffix[2:])
            elif suffix.startswith("SR"):
                limits[axis][1] = int(suffix[2:])

        expected = {"X": 123, "Y": -45, "Z": 1000}
        with patch("alignment_routine.read_number", side_effect=read_number), \
                patch("alignment_routine.query", side_effect=query), \
                patch("alignment_routine.send", side_effect=send), \
                patch("alignment_routine.check_error"), \
                patch("alignment_routine.time.sleep"):
            self.assertEqual(stage.establish_origin(expected), {"X": 0, "Y": 0, "Z": 0})
        self.assertEqual([cmd for cmd in commands if cmd.endswith("OR")], ["1OR", "2OR", "3OR"])
        self.assertFalse(any("PR" in cmd for cmd in commands))
        self.assertEqual(limits, {1: [-1, 61450], 2: [-10000000, 10000000],
                                  3: [-1, 150000]})

    def test_live_stage_splits_move_and_never_commands_past_cap(self):
        stage = PZC200Stage.__new__(PZC200Stage)
        stage.ser = object()
        state = {"position": 0, "high": 61450, "commands": []}

        def read_number(_serial, command):
            if command == "1TP?":
                return state["position"]
            if command == "1SL?":
                return -1
            if command == "1SR?":
                return state["high"]
            raise AssertionError(command)

        def send(_serial, command):
            state["commands"].append(command)
            if command.startswith("1PR"):
                state["position"] += int(command[3:])

        with patch("alignment_routine.read_number", side_effect=read_number), \
                patch("alignment_routine.query", return_value="1TS? Q"), \
                patch("alignment_routine.send", side_effect=send), \
                patch("alignment_routine.check_error"), \
                patch("alignment_routine.time.sleep"):
            final = stage.move_to("X", 61450)
        deltas = [int(command[3:]) for command in state["commands"] if command.startswith("1PR")]
        self.assertGreater(len(deltas), 1)
        self.assertTrue(all(0 < delta <= 50000 for delta in deltas))
        self.assertLessEqual(final, 61450)
        self.assertLessEqual(61450 - final, 16)

    def test_live_stage_stops_if_controller_limit_changes(self):
        stage = PZC200Stage.__new__(PZC200Stage)
        stage.ser = object()
        state = {"position": 0, "high": 61450, "moves": 0}

        def read_number(_serial, command):
            return {"1TP?": state["position"], "1SL?": -1, "1SR?": state["high"]}[command]

        def send(_serial, command):
            if command.startswith("1PR"):
                state["position"] += int(command[3:])
                state["moves"] += 1
                state["high"] = 70000

        with patch("alignment_routine.read_number", side_effect=read_number), \
                patch("alignment_routine.query", return_value="1TS? Q"), \
                patch("alignment_routine.send", side_effect=send), \
                patch("alignment_routine.check_error"), \
                patch("alignment_routine.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "limits changed"):
                stage.move_to("X", 61450)
        self.assertEqual(state["moves"], 1)

    def test_meter_wrapper_reads_measured_voltage_and_power(self):
        class Instrument:
            def measVoltage(self):
                return 0, 0.123

            def measPower(self):
                return 0, 4e-9

        meter = ThorlabsPM100USB()
        meter.instrument = Instrument()
        self.assertEqual(meter.read_voltage(), 0.123)
        self.assertEqual(meter.read_power(), 4e-9)


if __name__ == "__main__":
    unittest.main()
