import math
import sys
import unittest
from pathlib import Path


SIMULATOR_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIMULATOR_DIR))

from simulator import FlightSimulation, PositionHoldRelease  # noqa: E402


class FlightSimulatorTests(unittest.TestCase):
    def run_scenario(self, name: str):
        rows, summary = FlightSimulation(name).run()
        self.assertTrue(summary.all_outputs_finite)
        self.assertTrue(all(abs(row.roll_setpoint_deg) <= 8.01 for row in rows if row.position_hold_active))
        self.assertTrue(all(abs(row.pitch_setpoint_deg) <= 8.01 for row in rows if row.position_hold_active))
        return rows, summary

    def test_altitude_hold_rejects_vertical_gust(self):
        _, summary = self.run_scenario("altitude_hold")
        self.assertLess(abs(summary.final_vertical_speed_mps), 0.10)
        self.assertLess(abs(summary.final_altitude_m - 1.0), 0.45)

    def test_position_hold_stops_horizontal_gust(self):
        _, summary = self.run_scenario("position_hold")
        self.assertGreater(summary.active_samples, 250)
        self.assertLess(summary.final_horizontal_speed_mps, 0.18)
        self.assertEqual([], summary.release_reasons)

    def test_flow_dropout_releases_and_requires_mode_cycle(self):
        rows, summary = self.run_scenario("flow_dropout")
        self.assertIn(PositionHoldRelease.SENSOR.value, summary.release_reasons)
        self.assertTrue(summary.recovered_after_release)
        stale_rows = [row for row in rows if row.position_hold_fault == "FLOW_STALE"]
        self.assertTrue(stale_rows)
        self.assertTrue(all(not row.position_hold_active for row in stale_rows))

    def test_missing_barometer_releases_position_hold(self):
        rows, summary = self.run_scenario("barometer_dropout")
        self.assertIn(PositionHoldRelease.SENSOR.value, summary.release_reasons)
        self.assertTrue(summary.recovered_after_release)
        self.assertTrue(any(row.position_hold_fault == "NO_BARO" for row in rows))

    def test_invalid_range_never_reaches_controller(self):
        rows, summary = self.run_scenario("invalid_range")
        invalid = [row for row in rows if row.position_hold_fault == "RANGE_INVALID"]
        self.assertTrue(invalid)
        self.assertTrue(all(not row.position_hold_active for row in invalid))
        self.assertTrue(summary.recovered_after_release)

    def test_low_quality_flow_never_reaches_controller(self):
        rows, summary = self.run_scenario("low_flow_quality")
        rejected = [row for row in rows if row.position_hold_fault == "FLOW_QUALITY"]
        self.assertTrue(rejected)
        self.assertTrue(all(not row.position_hold_active for row in rejected))
        self.assertTrue(summary.recovered_after_release)

    def test_excessive_motion_latches_release(self):
        rows, summary = self.run_scenario("excessive_motion")
        self.assertIn(PositionHoldRelease.EXCESSIVE_MOTION.value, summary.release_reasons)
        self.assertTrue(summary.recovered_after_release)
        released = [row for row in rows if row.position_hold_release == "EXCESSIVE_MOTION"]
        self.assertTrue(released)
        self.assertTrue(all(not row.position_hold_active for row in released))

    def test_simulation_is_deterministic(self):
        rows_a, summary_a = FlightSimulation("position_hold").run()
        rows_b, summary_b = FlightSimulation("position_hold").run()
        self.assertEqual(summary_a, summary_b)
        self.assertTrue(math.isclose(rows_a[-1].true_x_m, rows_b[-1].true_x_m, abs_tol=1e-12))


if __name__ == "__main__":
    unittest.main()
