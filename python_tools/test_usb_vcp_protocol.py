import unittest

from usb_vcp_host import (
    SeaSkyProtocol,
    USB_CTRL_CHASSIS_CMD_ID,
    USB_CTRL_GIMBAL_CMD_ID,
    USB_CTRL_SHOOT_CMD_ID,
    USB_STATUS_CHASSIS_CMD_ID,
    USB_STATUS_GIMBAL_CMD_ID,
    USB_STATUS_SHOOT_CMD_ID,
    VISION_RECV_CMD_ID,
    VISION_SEND_CMD_ID,
    build_chassis_frame,
    build_gimbal_frame,
    build_legacy_vision_frame,
    build_shoot_frame,
    decode_status_text,
)


class USBVCPProtocolTest(unittest.TestCase):
    def test_legacy_vision_frame_round_trip(self) -> None:
        frame = build_legacy_vision_frame(2, 1, 3, pitch=4.5, yaw=-6.25)
        decoded = SeaSkyProtocol.unpack_frame(frame)
        self.assertEqual(decoded.cmd_id, VISION_RECV_CMD_ID)
        self.assertEqual(decoded.flags & 0x03, 2)
        self.assertAlmostEqual(decoded.floats[0], 4.5, places=5)
        self.assertAlmostEqual(decoded.floats[1], -6.25, places=5)

    def test_gimbal_frame_round_trip(self) -> None:
        frame = build_gimbal_frame(yaw=15.0, pitch=-3.0, mode=2, active=True, auto_aim=True, relative=True)
        decoded = SeaSkyProtocol.unpack_frame(frame)
        self.assertEqual(decoded.cmd_id, USB_CTRL_GIMBAL_CMD_ID)
        self.assertEqual((decoded.flags >> 4) & 0x0F, 2)
        self.assertAlmostEqual(decoded.floats[0], 15.0, places=5)
        self.assertAlmostEqual(decoded.floats[1], -3.0, places=5)

    def test_chassis_frame_round_trip(self) -> None:
        frame = build_chassis_frame(vx=300.0, vy=-120.0, wz=45.0, speed_scale=80.0, mode=3, active=True)
        decoded = SeaSkyProtocol.unpack_frame(frame)
        self.assertEqual(decoded.cmd_id, USB_CTRL_CHASSIS_CMD_ID)
        self.assertAlmostEqual(decoded.floats[0], 300.0, places=5)
        self.assertAlmostEqual(decoded.floats[1], -120.0, places=5)
        self.assertAlmostEqual(decoded.floats[2], 45.0, places=5)
        self.assertAlmostEqual(decoded.floats[3], 80.0, places=5)

    def test_shoot_frame_round_trip(self) -> None:
        frame = build_shoot_frame(28000.0, 1500.0, 8.0, 5, True, True, True)
        decoded = SeaSkyProtocol.unpack_frame(frame)
        self.assertEqual(decoded.cmd_id, USB_CTRL_SHOOT_CMD_ID)
        self.assertAlmostEqual(decoded.floats[0], 28000.0, places=5)
        self.assertAlmostEqual(decoded.floats[1], 1500.0, places=5)
        self.assertAlmostEqual(decoded.floats[2], 8.0, places=5)

    def test_stream_stability_simulation(self) -> None:
        for index in range(1000):
            frame = build_chassis_frame(vx=float(index), vy=float(-index), wz=float(index % 360), speed_scale=100.0)
            decoded = SeaSkyProtocol.unpack_frame(frame)
            self.assertEqual(decoded.cmd_id, USB_CTRL_CHASSIS_CMD_ID)
            self.assertAlmostEqual(decoded.floats[0], float(index), places=5)

    def test_status_text_decode(self) -> None:
        vision = decode_status_text(SeaSkyProtocol.unpack_frame(SeaSkyProtocol.pack_frame(VISION_SEND_CMD_ID, 0x1E01, [1.0, 2.0, 3.0])))
        gimbal = decode_status_text(SeaSkyProtocol.unpack_frame(SeaSkyProtocol.pack_frame(USB_STATUS_GIMBAL_CMD_ID, 0x0102, [1.0, 2.0, 3.0, 4.0])))
        chassis = decode_status_text(SeaSkyProtocol.unpack_frame(SeaSkyProtocol.pack_frame(USB_STATUS_CHASSIS_CMD_ID, 0x0103, [1.0, 2.0, 3.0, 4.0])))
        shoot = decode_status_text(SeaSkyProtocol.unpack_frame(SeaSkyProtocol.pack_frame(USB_STATUS_SHOOT_CMD_ID, 0x8531, [1.0, 2.0, 3.0, 4.0])))
        self.assertIn("VISION", vision)
        self.assertIn("GIMBAL", gimbal)
        self.assertIn("CHASSIS", chassis)
        self.assertIn("SHOOT", shoot)


if __name__ == "__main__":
    unittest.main()
