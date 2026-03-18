#!/usr/bin/env python3
import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover
    serial = None


PROTOCOL_CMD_ID = 0xA5
VISION_RECV_CMD_ID = 0x0001
VISION_SEND_CMD_ID = 0x0002
USB_CTRL_GIMBAL_CMD_ID = 0x0101
USB_CTRL_CHASSIS_CMD_ID = 0x0102
USB_CTRL_SHOOT_CMD_ID = 0x0103
USB_STATUS_GIMBAL_CMD_ID = 0x0201
USB_STATUS_CHASSIS_CMD_ID = 0x0202
USB_STATUS_SHOOT_CMD_ID = 0x0203


CRC8_TABLE = [
    0, 49, 98, 83, 196, 245, 166, 151, 185, 136, 219, 234, 125, 76, 31, 46,
    67, 114, 33, 16, 135, 182, 229, 212, 250, 203, 152, 169, 62, 15, 92, 109,
    134, 183, 228, 213, 66, 115, 32, 17, 63, 14, 93, 108, 251, 202, 153, 168,
    197, 244, 167, 150, 1, 48, 99, 82, 124, 77, 30, 47, 184, 137, 218, 235,
    61, 12, 95, 110, 249, 200, 155, 170, 132, 181, 230, 215, 64, 113, 34, 19,
    126, 79, 28, 45, 186, 139, 216, 233, 199, 246, 165, 148, 3, 50, 97, 80,
    187, 138, 217, 232, 127, 78, 29, 44, 2, 51, 96, 81, 198, 247, 164, 149,
    248, 201, 154, 171, 60, 13, 94, 111, 65, 112, 35, 18, 133, 180, 231, 214,
    122, 75, 24, 41, 190, 143, 220, 237, 195, 242, 161, 144, 7, 54, 101, 84,
    57, 8, 91, 106, 253, 204, 159, 174, 128, 177, 226, 211, 68, 117, 38, 23,
    252, 205, 158, 175, 56, 9, 90, 107, 69, 116, 39, 22, 129, 176, 227, 210,
    191, 142, 221, 236, 123, 74, 25, 40, 6, 55, 100, 85, 194, 243, 160, 145,
    71, 118, 37, 20, 131, 178, 225, 208, 254, 207, 156, 173, 58, 11, 88, 105,
    4, 53, 102, 87, 192, 241, 162, 147, 189, 140, 223, 238, 121, 72, 27, 42,
    193, 240, 163, 146, 5, 52, 103, 86, 120, 73, 26, 43, 188, 141, 222, 239,
    130, 179, 224, 209, 70, 119, 36, 21, 59, 10, 89, 104, 255, 206, 157, 172,
]


class USBControlFlags:
    ACTIVE = 1 << 0
    AUTO_AIM = 1 << 1
    RELATIVE = 1 << 2
    STOP = 1 << 1
    SHOOT_ENABLE = 1 << 0
    FRICTION_ON = 1 << 1
    LOADER_ON = 1 << 2
    LID_OPEN = 1 << 3
    MODE_SHIFT = 4


@dataclass
class DecodedFrame:
    cmd_id: int
    flags: int
    floats: Tuple[float, ...]


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value = CRC8_TABLE[byte ^ value]
    return value


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            if value & 1:
                value = (value >> 1) ^ 0xA001
            else:
                value >>= 1
    return value & 0xFFFF


class SeaSkyProtocol:
    @staticmethod
    def pack_frame(cmd_id: int, flags: int, floats: Sequence[float]) -> bytes:
        payload = struct.pack("<H", flags) + b"".join(struct.pack("<f", value) for value in floats)
        header = bytearray(4)
        header[0] = PROTOCOL_CMD_ID
        header[1:3] = struct.pack("<H", len(payload))
        header[3] = crc8(bytes(header[:3]))
        frame = bytes(header) + struct.pack("<H", cmd_id) + payload
        return frame + struct.pack("<H", crc16(frame))

    @staticmethod
    def unpack_frame(frame: bytes) -> DecodedFrame:
        if len(frame) < 10 or frame[0] != PROTOCOL_CMD_ID:
            raise ValueError("invalid frame")
        if crc8(frame[:3]) != frame[3]:
            raise ValueError("invalid crc8")
        data_len = struct.unpack("<H", frame[1:3])[0]
        if len(frame) != data_len + 8:
            raise ValueError("invalid length")
        if crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
            raise ValueError("invalid crc16")
        cmd_id = struct.unpack("<H", frame[4:6])[0]
        flags = struct.unpack("<H", frame[6:8])[0]
        payload = frame[8:-2]
        if len(payload) % 4 != 0:
            raise ValueError("invalid float payload")
        floats = tuple(struct.unpack("<" + "f" * (len(payload) // 4), payload))
        return DecodedFrame(cmd_id, flags, floats)


def build_legacy_vision_frame(fire_mode: int, target_state: int, target_type: int, pitch: float, yaw: float) -> bytes:
    flags = (fire_mode & 0x03) | ((target_state & 0x03) << 2) | ((target_type & 0x0F) << 4)
    return SeaSkyProtocol.pack_frame(VISION_RECV_CMD_ID, flags, [pitch, yaw])


def build_gimbal_frame(yaw: float, pitch: float, mode: int = 2, active: bool = True, auto_aim: bool = False, relative: bool = False) -> bytes:
    flags = mode << USBControlFlags.MODE_SHIFT
    if active:
        flags |= USBControlFlags.ACTIVE
    if auto_aim:
        flags |= USBControlFlags.AUTO_AIM
    if relative:
        flags |= USBControlFlags.RELATIVE
    return SeaSkyProtocol.pack_frame(USB_CTRL_GIMBAL_CMD_ID, flags, [yaw, pitch, 0.0, 0.0])


def build_chassis_frame(vx: float, vy: float, wz: float, speed_scale: float = 100.0, mode: int = 3, active: bool = True, stop: bool = False) -> bytes:
    flags = mode << USBControlFlags.MODE_SHIFT
    if active:
        flags |= USBControlFlags.ACTIVE
    if stop:
        flags |= USBControlFlags.STOP
    return SeaSkyProtocol.pack_frame(USB_CTRL_CHASSIS_CMD_ID, flags, [vx, vy, wz, speed_scale])


def build_shoot_frame(friction_speed: float, loader_speed: float, shoot_rate: float, loader_mode: int = 5, shoot_enable: bool = True, friction_on: bool = True, loader_on: bool = True, lid_open: bool = False, active: bool = True) -> bytes:
    flags = loader_mode << USBControlFlags.MODE_SHIFT
    if active:
        flags |= USBControlFlags.ACTIVE
    if shoot_enable:
        flags |= USBControlFlags.SHOOT_ENABLE
    if friction_on:
        flags |= USBControlFlags.FRICTION_ON
    if loader_on:
        flags |= USBControlFlags.LOADER_ON
    if lid_open:
        flags |= USBControlFlags.LID_OPEN
    return SeaSkyProtocol.pack_frame(USB_CTRL_SHOOT_CMD_ID, flags, [friction_speed, loader_speed, shoot_rate, 0.0])


def frame_to_hex(frame: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in frame)


def repeated_frames(frame: bytes, repeat: int) -> List[bytes]:
    return [frame for _ in range(max(repeat, 1))]


def send_frames(port: str, baudrate: int, frames: Iterable[bytes], interval: float) -> None:
    if serial is None:
        raise RuntimeError("pyserial not installed, please run: pip install pyserial")
    with serial.Serial(port=port, baudrate=baudrate, timeout=0.2) as ser:
        for frame in frames:
            ser.write(frame)
            ser.flush()
            if interval > 0:
                time.sleep(interval)


def read_one_frame(ser) -> Optional[bytes]:
    first = ser.read(1)
    if not first:
        return None
    if first[0] != PROTOCOL_CMD_ID:
        return None
    header_tail = ser.read(3)
    if len(header_tail) != 3:
        return None
    header = first + header_tail
    if crc8(header[:3]) != header[3]:
        return None
    data_len = struct.unpack("<H", header[1:3])[0]
    body = ser.read(data_len + 4)
    if len(body) != data_len + 4:
        return None
    return header + body


def decode_status_text(decoded: DecodedFrame) -> str:
    if decoded.cmd_id == VISION_SEND_CMD_ID:
        return f"VISION yaw={decoded.floats[0]:.2f} pitch={decoded.floats[1]:.2f} roll={decoded.floats[2]:.2f}"
    if decoded.cmd_id == USB_STATUS_GIMBAL_CMD_ID:
        mode = decoded.flags & 0x0F
        online = (decoded.flags >> 8) & 0x01
        return f"GIMBAL online={online} mode={mode} yaw={decoded.floats[0]:.2f} pitch={decoded.floats[1]:.2f} ecd={decoded.floats[2]:.2f} yaw_total={decoded.floats[3]:.2f}"
    if decoded.cmd_id == USB_STATUS_CHASSIS_CMD_ID:
        mode = decoded.flags & 0x0F
        online = (decoded.flags >> 8) & 0x01
        return f"CHASSIS online={online} mode={mode} vx={decoded.floats[0]:.2f} vy={decoded.floats[1]:.2f} wz={decoded.floats[2]:.2f} offset={decoded.floats[3]:.2f}"
    if decoded.cmd_id == USB_STATUS_SHOOT_CMD_ID:
        shoot_mode = decoded.flags & 0x0F
        friction_mode = (decoded.flags >> 4) & 0x03
        loader_mode = (decoded.flags >> 8) & 0x0F
        online = (decoded.flags >> 15) & 0x01
        return f"SHOOT online={online} shoot={shoot_mode} friction={friction_mode} loader={loader_mode} friction_spd={decoded.floats[0]:.2f} loader_spd={decoded.floats[1]:.2f} rate={decoded.floats[2]:.2f} bullet={decoded.floats[3]:.2f}"
    return f"CMD 0x{decoded.cmd_id:04X} flags=0x{decoded.flags:04X} floats={decoded.floats}"


def monitor_status(port: str, baudrate: int, count: int = 0) -> None:
    if serial is None:
        raise RuntimeError("pyserial not installed, please run: pip install pyserial")
    with serial.Serial(port=port, baudrate=baudrate, timeout=0.5) as ser:
        received = 0
        while count == 0 or received < count:
            frame = read_one_frame(ser)
            if not frame:
                continue
            decoded = SeaSkyProtocol.unpack_frame(frame)
            print(decode_status_text(decoded))
            received += 1


def prompt_text(label: str, default: str) -> str:
    value = input(f"{label} [{default}]: ").strip()
    return value if value else default


def prompt_float(label: str, default: float) -> float:
    return float(prompt_text(label, str(default)))


def prompt_int(label: str, default: int) -> int:
    return int(prompt_text(label, str(default)))


def prompt_bool(label: str, default: bool) -> bool:
    suffix = "Y/n" if default else "y/N"
    value = input(f"{label} [{suffix}]: ").strip().lower()
    if not value:
        return default
    return value in {"y", "yes", "1", "true"}


def choose_port_and_speed() -> Tuple[str, int]:
    port = prompt_text("串口号", "COM5")
    baudrate = prompt_int("波特率", 115200)
    return port, baudrate


def interactive_menu() -> int:
    while True:
        print("\nUSB VCP 控制菜单")
        print("1. 发送云台控制")
        print("2. 发送底盘控制")
        print("3. 发送发射控制")
        print("4. 发送旧版视觉帧")
        print("5. 监听机器人状态回传")
        print("0. 退出")
        choice = input("选择功能: ").strip()

        if choice == "0":
            return 0
        if choice == "5":
            port, baudrate = choose_port_and_speed()
            count = prompt_int("接收帧数(0为持续)", 0)
            monitor_status(port, baudrate, count)
            continue

        repeat = prompt_int("发送次数", 1)
        interval = prompt_float("发送间隔(秒)", 0.02)
        dry_run = prompt_bool("仅打印不发送", False)

        if choice == "1":
            frame = build_gimbal_frame(
                yaw=prompt_float("yaw角度/增量", 15.0),
                pitch=prompt_float("pitch角度/增量", -3.0),
                mode=prompt_int("模式(0零力/1自由/2陀螺)", 2),
                active=not prompt_bool("禁用该控制帧", False),
                auto_aim=prompt_bool("开启自瞄", True),
                relative=prompt_bool("相对角控制", False),
            )
        elif choice == "2":
            frame = build_chassis_frame(
                vx=prompt_float("vx", 300.0),
                vy=prompt_float("vy", 0.0),
                wz=prompt_float("wz", 45.0),
                speed_scale=prompt_float("速度缩放", 100.0),
                mode=prompt_int("模式(0零力/1不跟随/2跟随/3直控/4自旋)", 3),
                active=not prompt_bool("禁用该控制帧", False),
                stop=prompt_bool("请求停止", False),
            )
        elif choice == "3":
            frame = build_shoot_frame(
                friction_speed=prompt_float("摩擦轮转速", 28000.0),
                loader_speed=prompt_float("拨盘转速", 1500.0),
                shoot_rate=prompt_float("射频", 8.0),
                loader_mode=prompt_int("拨盘模式(0停/1单发/2三发/3连发/4反转/5速度)", 5),
                shoot_enable=prompt_bool("发射总使能", True),
                friction_on=prompt_bool("摩擦轮开启", True),
                loader_on=prompt_bool("拨盘开启", True),
                lid_open=prompt_bool("弹舱盖打开", False),
                active=not prompt_bool("禁用该控制帧", False),
            )
        elif choice == "4":
            frame = build_legacy_vision_frame(
                fire_mode=prompt_int("fire_mode", 2),
                target_state=prompt_int("target_state", 2),
                target_type=prompt_int("target_type", 3),
                pitch=prompt_float("pitch", 4.5),
                yaw=prompt_float("yaw", -6.25),
            )
        else:
            print("无效选择")
            continue

        print(frame_to_hex(frame))
        if dry_run:
            continue
        port, baudrate = choose_port_and_speed()
        send_frames(port, baudrate, repeated_frames(frame, repeat), interval)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="USB VCP host control sender for the robot")
    parser.add_argument("--menu", action="store_true", help="launch interactive menu")
    parser.add_argument("--port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.02)
    parser.add_argument("--dry-run", action="store_true")
    sub = parser.add_subparsers(dest="command")

    vision = sub.add_parser("vision")
    vision.add_argument("--fire-mode", type=int, default=2)
    vision.add_argument("--target-state", type=int, default=2)
    vision.add_argument("--target-type", type=int, default=3)
    vision.add_argument("--pitch", type=float, required=True)
    vision.add_argument("--yaw", type=float, required=True)

    gimbal = sub.add_parser("gimbal")
    gimbal.add_argument("--yaw", type=float, required=True)
    gimbal.add_argument("--pitch", type=float, required=True)
    gimbal.add_argument("--mode", type=int, default=2, choices=[0, 1, 2])
    gimbal.add_argument("--auto-aim", action="store_true")
    gimbal.add_argument("--relative", action="store_true")
    gimbal.add_argument("--disable", action="store_true")

    chassis = sub.add_parser("chassis")
    chassis.add_argument("--vx", type=float, required=True)
    chassis.add_argument("--vy", type=float, required=True)
    chassis.add_argument("--wz", type=float, required=True)
    chassis.add_argument("--speed-scale", type=float, default=100.0)
    chassis.add_argument("--mode", type=int, default=3, choices=[0, 1, 2, 3, 4])
    chassis.add_argument("--stop", action="store_true")
    chassis.add_argument("--disable", action="store_true")

    shoot = sub.add_parser("shoot")
    shoot.add_argument("--friction-speed", type=float, required=True)
    shoot.add_argument("--loader-speed", type=float, default=0.0)
    shoot.add_argument("--shoot-rate", type=float, default=0.0)
    shoot.add_argument("--loader-mode", type=int, default=5, choices=[0, 1, 2, 3, 4, 5])
    shoot.add_argument("--shoot-enable", action="store_true")
    shoot.add_argument("--friction-on", action="store_true")
    shoot.add_argument("--loader-on", action="store_true")
    shoot.add_argument("--lid-open", action="store_true")
    shoot.add_argument("--disable", action="store_true")

    monitor = sub.add_parser("monitor")
    monitor.add_argument("--count", type=int, default=0)
    return parser


def build_frame_from_args(args: argparse.Namespace) -> bytes:
    if args.command == "vision":
        return build_legacy_vision_frame(args.fire_mode, args.target_state, args.target_type, args.pitch, args.yaw)
    if args.command == "gimbal":
        return build_gimbal_frame(args.yaw, args.pitch, args.mode, not args.disable, args.auto_aim, args.relative)
    if args.command == "chassis":
        return build_chassis_frame(args.vx, args.vy, args.wz, args.speed_scale, args.mode, not args.disable, args.stop)
    if args.command == "shoot":
        return build_shoot_frame(args.friction_speed, args.loader_speed, args.shoot_rate, args.loader_mode, args.shoot_enable, args.friction_on, args.loader_on, args.lid_open, not args.disable)
    raise ValueError(f"unsupported command {args.command}")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    if args.menu or args.command is None:
        return interactive_menu()
    if args.command == "monitor":
        if not args.port:
            parser.error("--port is required for monitor")
        monitor_status(args.port, args.baudrate, args.count)
        return 0

    frame = build_frame_from_args(args)
    print(frame_to_hex(frame))
    if args.dry_run:
        return 0
    if not args.port:
        parser.error("--port is required unless --dry-run is used")
    send_frames(args.port, args.baudrate, repeated_frames(frame, args.repeat), args.interval)
    return 0


if __name__ == "__main__":
    sys.exit(main())
