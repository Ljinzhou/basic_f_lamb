import tempfile
import unittest
from pathlib import Path

from debug_backend import DaemonStatus, GimbalSnapshot, IMUSnapshot, MotorSnapshot, SystemSnapshot, export_snapshots_to_csv, flatten_snapshot, parse_dji_measure, parse_dm_measure


class DebugBackendTest(unittest.TestCase):
    def test_parse_dji_measure(self) -> None:
        payload = bytes.fromhex("34 12 78 56 00 00 20 41 00 00 A0 41 38 FF 3C 00 00 00 F0 41 02 00 00 00")
        data = parse_dji_measure(payload)
        self.assertEqual(data["ecd"], 0x5678)
        self.assertAlmostEqual(float(data["angle_single_round"]), 10.0, places=5)
        self.assertAlmostEqual(float(data["speed_aps"]), 20.0, places=5)
        self.assertEqual(data["real_current"], -200)
        self.assertEqual(data["temperature"], 60)

    def test_parse_dm_measure(self) -> None:
        payload = bytes.fromhex("01 02 00 00 00 00 80 3F 00 00 00 40 00 00 40 40 00 00 80 40 00 00 A0 40 00 00 C0 40 03 00 00 00")
        data = parse_dm_measure(payload)
        self.assertEqual(data["state"], 2)
        self.assertAlmostEqual(float(data["velocity"]), 1.0, places=5)
        self.assertAlmostEqual(float(data["position"]), 3.0, places=5)
        self.assertEqual(data["total_round"], 3)

    def test_flatten_and_export_snapshot(self) -> None:
        snapshot = SystemSnapshot(
            timestamp=1.23,
            robot_state=1,
            imu=IMUSnapshot(1, 2, 3, 4, (0.1, 0.2, 0.3, 0.4), (1, 2, 3), (4, 5, 6), (0, 0, 0), (0, 0, 0), (7, 8, 9), (10, 11, 12), 36.5),
            gimbal=GimbalSnapshot(10, 20, 30, 40, 50, 60, 2),
            motors=[MotorSnapshot("yaw_motor", "DJI", True, True, 100, 1.0, 2.0, 3.0, 40.0, 5.0, 6)],
            daemons=[DaemonStatus("vision", True, 5, 4)],
            tasks=[],
            notes=["ok"],
        )
        row = flatten_snapshot(snapshot)
        self.assertEqual(row["robot_state"], 1)
        self.assertEqual(row["motor_0_yaw_motor_online"], True)
        with tempfile.TemporaryDirectory() as tmp_dir:
            output = Path(tmp_dir) / "out.csv"
            export_snapshots_to_csv(output, [snapshot])
            self.assertTrue(output.exists())
            self.assertIn("motor_0_yaw_motor_online", output.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
