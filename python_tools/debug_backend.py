#!/usr/bin/env python3
from __future__ import annotations

import csv
import re
import socket
import struct
import subprocess
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BUILD_DIR = PROJECT_ROOT / "build"
ELF_FILE = BUILD_DIR / "basic_framework.elf"
OPENOCD_CONFIG = PROJECT_ROOT / "openocd_stlink.cfg"
ARM_TOOLCHAIN_PATH = Path(r"C:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin")
OPENOCD_TELNET_PORT = 4444

DJI_MEASURE_SIZE = 24
DJI_STOP_FLAG_OFFSET = 440
DJI_DAEMON_OFFSET = 444
DM_MEASURE_SIZE = 32
DM_STOP_FLAG_OFFSET = 444
DM_DAEMON_OFFSET = 452
DAEMON_STRUCT_SIZE = 16


@dataclass
class SymbolInfo:
    name: str
    address: int
    size: int
    type_: str


@dataclass
class DaemonStatus:
    name: str
    online: bool
    reload_count: int
    temp_count: int


@dataclass
class MotorSnapshot:
    name: str
    motor_type: str
    online: bool
    enabled: bool
    ecd: int
    angle_deg: float
    speed_deg_s: float
    current: float
    temperature: float
    total_angle: float
    total_round: int


@dataclass
class IMUSnapshot:
    roll: float
    pitch: float
    yaw: float
    yaw_total: float
    quaternion: Tuple[float, float, float, float]
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    motion_accel_body: Tuple[float, float, float]
    motion_accel_world: Tuple[float, float, float]
    bmi088_gyro: Tuple[float, float, float]
    bmi088_accel: Tuple[float, float, float]
    bmi088_temperature: float


@dataclass
class GimbalSnapshot:
    yaw: float
    pitch: float
    yaw_total: float
    yaw_motor_angle: float
    yaw_ecd: int
    pitch_ecd: int
    mode: int


@dataclass
class TaskSnapshot:
    name: str
    created: bool
    handle: int


@dataclass
class SystemSnapshot:
    timestamp: float
    robot_state: Optional[int]
    imu: IMUSnapshot
    gimbal: GimbalSnapshot
    motors: List[MotorSnapshot] = field(default_factory=list)
    daemons: List[DaemonStatus] = field(default_factory=list)
    tasks: List[TaskSnapshot] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)


def bytes_to_float(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


def bytes_to_u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def bytes_to_i16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<h", data, offset)[0]


def bytes_to_i32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<i", data, offset)[0]


def parse_dji_measure(data: bytes) -> Dict[str, float | int]:
    return {
        "ecd": bytes_to_u16(data, 2),
        "angle_single_round": bytes_to_float(data, 4),
        "speed_aps": bytes_to_float(data, 8),
        "real_current": bytes_to_i16(data, 12),
        "temperature": data[14],
        "total_angle": bytes_to_float(data, 16),
        "total_round": bytes_to_i32(data, 20),
    }


def parse_dm_measure(data: bytes) -> Dict[str, float | int]:
    return {
        "state": data[1],
        "velocity": bytes_to_float(data, 4),
        "position": bytes_to_float(data, 12),
        "torque": bytes_to_float(data, 16),
        "t_mos": bytes_to_float(data, 20),
        "t_rotor": bytes_to_float(data, 24),
        "total_round": bytes_to_i32(data, 28),
    }


def force_tuple3(values: Sequence[float]) -> Tuple[float, float, float]:
    data = [float(value) for value in values[:3]]
    while len(data) < 3:
        data.append(0.0)
    return (data[0], data[1], data[2])


def force_tuple4(values: Sequence[float]) -> Tuple[float, float, float, float]:
    data = [float(value) for value in values[:4]]
    while len(data) < 4:
        data.append(0.0)
    return (data[0], data[1], data[2], data[3])


def flatten_snapshot(snapshot: SystemSnapshot) -> Dict[str, object]:
    row: Dict[str, object] = {
        "timestamp": snapshot.timestamp,
        "robot_state": snapshot.robot_state,
        "imu_roll": snapshot.imu.roll,
        "imu_pitch": snapshot.imu.pitch,
        "imu_yaw": snapshot.imu.yaw,
        "imu_yaw_total": snapshot.imu.yaw_total,
        "gimbal_mode": snapshot.gimbal.mode,
        "gimbal_yaw": snapshot.gimbal.yaw,
        "gimbal_pitch": snapshot.gimbal.pitch,
        "gimbal_yaw_motor_angle": snapshot.gimbal.yaw_motor_angle,
    }
    for index, motor in enumerate(snapshot.motors):
        prefix = f"motor_{index}_{motor.name}"
        row[f"{prefix}_online"] = motor.online
        row[f"{prefix}_enabled"] = motor.enabled
        row[f"{prefix}_ecd"] = motor.ecd
        row[f"{prefix}_speed"] = motor.speed_deg_s
        row[f"{prefix}_current"] = motor.current
        row[f"{prefix}_temperature"] = motor.temperature
    for daemon in snapshot.daemons:
        row[f"daemon_{daemon.name}_online"] = daemon.online
        row[f"daemon_{daemon.name}_temp"] = daemon.temp_count
    for task in snapshot.tasks:
        row[f"task_{task.name}_created"] = task.created
    return row


def export_snapshots_to_csv(file_path: Path, snapshots: Sequence[SystemSnapshot]) -> None:
    rows = [flatten_snapshot(snapshot) for snapshot in snapshots]
    if not rows:
        return
    headers: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in headers:
                headers.append(key)
    with file_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)


class ELFParser:
    def __init__(self, elf_path: Path):
        self.elf_path = elf_path
        self.symbols: Dict[str, SymbolInfo] = {}

    def parse(self) -> bool:
        if not self.elf_path.exists():
            return False
        nm_exe = ARM_TOOLCHAIN_PATH / "arm-none-eabi-nm.exe"
        if not nm_exe.exists():
            return False
        result = subprocess.run([str(nm_exe), "-S", "-C", str(self.elf_path)], capture_output=True, text=True, encoding="utf-8", errors="replace")
        if result.returncode != 0:
            return False
        self.symbols.clear()
        for line in result.stdout.splitlines():
            parts = line.split()
            if len(parts) < 3:
                continue
            try:
                address = int(parts[0], 16)
                if len(parts) > 3:
                    size = int(parts[1], 16)
                    type_ = parts[2]
                    name = parts[3]
                else:
                    size = 0
                    type_ = parts[1]
                    name = parts[2]
            except ValueError:
                continue
            self.symbols[name] = SymbolInfo(name, address, size, type_)
        return True

    def get_symbol_address(self, name: str) -> Optional[int]:
        if name in self.symbols:
            return self.symbols[name].address
        for symbol_name, info in self.symbols.items():
            if name in symbol_name or symbol_name.endswith(name):
                return info.address
        return None

    def get_symbol_size(self, name: str) -> int:
        if name in self.symbols:
            return self.symbols[name].size
        for symbol_name, info in self.symbols.items():
            if name in symbol_name or symbol_name.endswith(name):
                return info.size
        return 0


class OpenOCDClient:
    def __init__(self, host: str = "localhost", port: int = OPENOCD_TELNET_PORT):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.connected = False

    def connect(self, timeout: float = 5.0) -> bool:
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self._recv_until(b"> ")
            return True
        except OSError:
            self.connected = False
            return False

    def disconnect(self) -> None:
        if self.socket:
            try:
                self.socket.close()
            except OSError:
                pass
        self.socket = None
        self.connected = False

    def _recv_until(self, delimiter: bytes, timeout: float = 5.0) -> bytes:
        if not self.socket:
            return b""
        self.socket.settimeout(timeout)
        data = b""
        while delimiter not in data:
            try:
                chunk = self.socket.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                break
            data += chunk
        return data

    def send_command(self, command: str) -> str:
        if not self.connected or not self.socket:
            return ""
        try:
            self.socket.sendall((command + "\n").encode("utf-8"))
            return self._recv_until(b"> ").decode("utf-8", errors="replace")
        except OSError:
            return ""

    def read_memory_32(self, address: int, count: int = 1) -> List[int]:
        response = self.send_command(f"mdw {address:#x} {count}")
        values: List[int] = []
        for line in response.splitlines():
            match = re.search(r"0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]+\s*)+)", line)
            if not match:
                continue
            for token in match.group(1).split():
                try:
                    values.append(int(token, 16))
                except ValueError:
                    pass
        return values

    def read_memory_8(self, address: int, count: int = 1) -> List[int]:
        response = self.send_command(f"mdb {address:#x} {count}")
        values: List[int] = []
        for line in response.splitlines():
            match = re.search(r"0x[0-9a-fA-F]+:\s+((?:0x[0-9a-fA-F]+\s*)+)", line)
            if not match:
                continue
            for token in match.group(1).split():
                try:
                    values.append(int(token, 16))
                except ValueError:
                    pass
        return values

    def read_u8(self, address: int) -> int:
        values = self.read_memory_8(address, 1)
        return values[0] if values else 0

    def read_u16(self, address: int) -> int:
        raw = self.read_bytes(address, 2)
        return struct.unpack("<H", raw)[0] if len(raw) == 2 else 0

    def read_u32(self, address: int) -> int:
        values = self.read_memory_32(address, 1)
        return values[0] if values else 0

    def read_float(self, address: int) -> float:
        values = self.read_memory_32(address, 1)
        if not values:
            return 0.0
        return struct.unpack("<f", struct.pack("<I", values[0]))[0]

    def read_float_array(self, address: int, count: int) -> List[float]:
        return [struct.unpack("<f", struct.pack("<I", value))[0] for value in self.read_memory_32(address, count)]

    def read_bytes(self, address: int, count: int) -> bytes:
        return bytes(self.read_memory_8(address, count))

    def halt(self) -> bool:
        response = self.send_command("halt").lower()
        return "target halted" in response or "already halted" in response

    def resume(self) -> bool:
        self.send_command("resume")
        return True


class STLinkDebugBackend:
    DJI_MOTORS = [
        "yaw_motor",
        "pitch_motor",
        "motor_lf",
        "motor_rf",
        "motor_lb",
        "motor_rb",
        "friction_l",
        "friction_r",
        "friction_3",
        "loader",
    ]
    DM_MOTORS = ["loader_dm"]
    TASK_SYMBOLS = ["insTaskHandle", "robotTaskHandle", "motorTaskHandle", "daemonTaskHandle", "uiTaskHandle", "defaultTaskHandle"]
    DAEMON_SYMBOLS = ["vision_daemon_instance", "rc_daemon_instance", "referee_daemon", "bmi088_daemon_instance"]

    def __init__(self, elf_path: Path = ELF_FILE, config_path: Path = OPENOCD_CONFIG):
        self.elf_path = elf_path
        self.config_path = config_path
        self.elf_parser = ELFParser(elf_path)
        self.symbol_cache: Dict[str, int] = {}
        self.openocd = OpenOCDClient()
        self.openocd_process: Optional[subprocess.Popen[str]] = None

    def parse_elf(self) -> bool:
        return self.elf_parser.parse()

    def start_openocd(self) -> bool:
        if not self.config_path.exists():
            return False
        self.openocd_process = subprocess.Popen(["openocd", "-f", str(self.config_path).replace("\\", "/")], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace")
        time.sleep(2)
        return self.openocd_process.poll() is None

    def stop_openocd(self) -> None:
        self.openocd.disconnect()
        if self.openocd_process:
            self.openocd_process.terminate()
            try:
                self.openocd_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.openocd_process.kill()
            self.openocd_process = None

    def connect(self) -> bool:
        return self.openocd.connect()

    def get_symbol_address(self, name: str) -> Optional[int]:
        if name in self.symbol_cache:
            return self.symbol_cache[name]
        address = self.elf_parser.get_symbol_address(name)
        if address is not None:
            self.symbol_cache[name] = address
        return address

    def read_pointer_symbol(self, name: str) -> Optional[int]:
        address = self.get_symbol_address(name)
        if address is None:
            return None
        values = self.openocd.read_memory_32(address, 1)
        return values[0] if values else None

    def read_daemon(self, symbol_name: str) -> Optional[DaemonStatus]:
        pointer = self.read_pointer_symbol(symbol_name)
        if not pointer:
            return None
        data = self.openocd.read_bytes(pointer, DAEMON_STRUCT_SIZE)
        if len(data) < DAEMON_STRUCT_SIZE:
            return None
        reload_count = struct.unpack_from("<H", data, 0)[0]
        temp_count = struct.unpack_from("<H", data, 8)[0]
        return DaemonStatus(symbol_name, temp_count > 0, reload_count, temp_count)

    def read_task_handle(self, symbol_name: str) -> TaskSnapshot:
        address = self.get_symbol_address(symbol_name)
        handle = self.openocd.read_u32(address) if address is not None else 0
        return TaskSnapshot(symbol_name, handle != 0, handle)

    def read_dji_motor(self, symbol_name: str) -> Optional[MotorSnapshot]:
        pointer = self.read_pointer_symbol(symbol_name)
        if not pointer:
            return None
        measure_data = self.openocd.read_bytes(pointer, DJI_MEASURE_SIZE)
        if len(measure_data) < DJI_MEASURE_SIZE:
            return None
        measure = parse_dji_measure(measure_data)
        stop_flag = self.openocd.read_memory_32(pointer + DJI_STOP_FLAG_OFFSET, 1)
        daemon_ptr = self.openocd.read_memory_32(pointer + DJI_DAEMON_OFFSET, 1)
        daemon_status = None
        if daemon_ptr and daemon_ptr[0]:
            raw = self.openocd.read_bytes(daemon_ptr[0], DAEMON_STRUCT_SIZE)
            if len(raw) >= DAEMON_STRUCT_SIZE:
                temp_count = struct.unpack_from("<H", raw, 8)[0]
                daemon_status = temp_count > 0
        return MotorSnapshot(
            name=symbol_name,
            motor_type="DJI",
            online=bool(daemon_status),
            enabled=(stop_flag[0] if stop_flag else 0) == 1,
            ecd=int(measure["ecd"]),
            angle_deg=float(measure["angle_single_round"]),
            speed_deg_s=float(measure["speed_aps"]),
            current=float(measure["real_current"]),
            temperature=float(measure["temperature"]),
            total_angle=float(measure["total_angle"]),
            total_round=int(measure["total_round"]),
        )

    def read_dm_motor(self, symbol_name: str) -> Optional[MotorSnapshot]:
        pointer = self.read_pointer_symbol(symbol_name)
        if not pointer:
            return None
        measure_data = self.openocd.read_bytes(pointer, DM_MEASURE_SIZE)
        if len(measure_data) < DM_MEASURE_SIZE:
            return None
        measure = parse_dm_measure(measure_data)
        stop_flag = self.openocd.read_memory_32(pointer + DM_STOP_FLAG_OFFSET, 1)
        daemon_ptr = self.openocd.read_memory_32(pointer + DM_DAEMON_OFFSET, 1)
        daemon_status = None
        if daemon_ptr and daemon_ptr[0]:
            raw = self.openocd.read_bytes(daemon_ptr[0], DAEMON_STRUCT_SIZE)
            if len(raw) >= DAEMON_STRUCT_SIZE:
                temp_count = struct.unpack_from("<H", raw, 8)[0]
                daemon_status = temp_count > 0
        return MotorSnapshot(
            name=symbol_name,
            motor_type="DM",
            online=bool(daemon_status),
            enabled=(stop_flag[0] if stop_flag else 0) == 1,
            ecd=0,
            angle_deg=float(measure["position"]),
            speed_deg_s=float(measure["velocity"]),
            current=float(measure["torque"]),
            temperature=float(measure["t_mos"]),
            total_angle=float(measure["position"]),
            total_round=int(measure["total_round"]),
        )

    def read_motors(self) -> List[MotorSnapshot]:
        motors: List[MotorSnapshot] = []
        for symbol in self.DJI_MOTORS:
            motor = self.read_dji_motor(symbol)
            if motor is not None:
                motors.append(motor)
        for symbol in self.DM_MOTORS:
            motor = self.read_dm_motor(symbol)
            if motor is not None:
                motors.append(motor)
        return motors

    def read_imu(self) -> IMUSnapshot:
        ins_addr = self.get_symbol_address("INS")
        bmi_addr = self.get_symbol_address("BMI088")
        if ins_addr is None:
            raise RuntimeError("未找到 INS 符号")
        q = force_tuple4(self.openocd.read_float_array(ins_addr + 0, 4))
        motion_accel_body = force_tuple3(self.openocd.read_float_array(ins_addr + 16, 3))
        motion_accel_world = force_tuple3(self.openocd.read_float_array(ins_addr + 28, 3))
        gyro = force_tuple3(self.openocd.read_float_array(ins_addr + 80, 3))
        accel = force_tuple3(self.openocd.read_float_array(ins_addr + 92, 3))
        roll = self.openocd.read_float(ins_addr + 104)
        pitch = self.openocd.read_float(ins_addr + 108)
        yaw = self.openocd.read_float(ins_addr + 112)
        yaw_total = self.openocd.read_float(ins_addr + 116)
        bmi_gyro = (0.0, 0.0, 0.0)
        bmi_accel = (0.0, 0.0, 0.0)
        bmi_temp = 0.0
        if bmi_addr is not None:
            bmi_gyro = force_tuple3(self.openocd.read_float_array(bmi_addr + 0, 3))
            bmi_accel = force_tuple3(self.openocd.read_float_array(bmi_addr + 12, 3))
            bmi_temp = self.openocd.read_float(bmi_addr + 24)
        return IMUSnapshot(roll, pitch, yaw, yaw_total, q, gyro, accel, motion_accel_body, motion_accel_world, bmi_gyro, bmi_accel, bmi_temp)

    def read_gimbal(self, motors: Optional[Sequence[MotorSnapshot]] = None) -> GimbalSnapshot:
        feedback_addr = self.get_symbol_address("gimbal_feedback_data")
        mode_addr = self.get_symbol_address("gimbal_cmd_send")
        if feedback_addr is None:
            raise RuntimeError("未找到 gimbal_feedback_data 符号")
        yaw = self.openocd.read_float(feedback_addr + 32)
        pitch = self.openocd.read_float(feedback_addr + 28)
        yaw_total = self.openocd.read_float(feedback_addr + 36)
        yaw_ecd = self.openocd.read_u16(feedback_addr + 40)
        mode = self.openocd.read_u8(mode_addr + 12) if mode_addr is not None else 0
        motor_map = {motor.name: motor for motor in motors or []}
        yaw_motor = motor_map.get("yaw_motor")
        pitch_motor = motor_map.get("pitch_motor")
        return GimbalSnapshot(
            yaw=yaw,
            pitch=pitch,
            yaw_total=yaw_total,
            yaw_motor_angle=yaw_motor.angle_deg if yaw_motor else float(yaw_ecd),
            yaw_ecd=yaw_ecd,
            pitch_ecd=pitch_motor.ecd if pitch_motor else 0,
            mode=mode,
        )

    def read_robot_state(self) -> Optional[int]:
        address = self.get_symbol_address("robot_state")
        if address is None:
            return None
        return self.openocd.read_u8(address)

    def read_snapshot(self) -> SystemSnapshot:
        halted_here = self.openocd.halt()
        try:
            motors = self.read_motors()
            daemons = [status for status in (self.read_daemon(name) for name in self.DAEMON_SYMBOLS) if status is not None]
            tasks = [self.read_task_handle(name) for name in self.TASK_SYMBOLS]
            imu = self.read_imu()
            gimbal = self.read_gimbal(motors)
            notes: List[str] = []
            if not any(task.created for task in tasks):
                notes.append("未检测到任务句柄，可能尚未启动调度器")
            if daemons and not any(daemon.online for daemon in daemons):
                notes.append("所有已发现 daemon 当前均离线")
            return SystemSnapshot(time.time(), self.read_robot_state(), imu, gimbal, motors, daemons, tasks, notes)
        finally:
            if halted_here:
                self.openocd.resume()

    def list_symbols(self, keyword: Optional[str] = None) -> List[SymbolInfo]:
        symbols = list(self.elf_parser.symbols.values())
        if keyword:
            keyword_lower = keyword.lower()
            symbols = [info for info in symbols if keyword_lower in info.name.lower()]
        return sorted(symbols, key=lambda item: item.address)


def snapshot_to_text(snapshot: SystemSnapshot) -> str:
    lines = [
        f"timestamp: {snapshot.timestamp:.3f}",
        f"robot_state: {snapshot.robot_state}",
        f"imu: roll={snapshot.imu.roll:.2f} pitch={snapshot.imu.pitch:.2f} yaw={snapshot.imu.yaw:.2f} yaw_total={snapshot.imu.yaw_total:.2f}",
        f"gimbal: mode={snapshot.gimbal.mode} yaw={snapshot.gimbal.yaw:.2f} pitch={snapshot.gimbal.pitch:.2f} yaw_ecd={snapshot.gimbal.yaw_ecd}",
    ]
    if snapshot.daemons:
        lines.append("daemons: " + ", ".join(f"{item.name}={'online' if item.online else 'offline'}({item.temp_count}/{item.reload_count})" for item in snapshot.daemons))
    if snapshot.tasks:
        lines.append("tasks: " + ", ".join(f"{item.name}={'ok' if item.created else 'none'}" for item in snapshot.tasks))
    if snapshot.motors:
        for motor in snapshot.motors:
            lines.append(f"motor[{motor.name}]: online={motor.online} enabled={motor.enabled} ecd={motor.ecd} angle={motor.angle_deg:.2f} speed={motor.speed_deg_s:.2f} current={motor.current:.2f} temp={motor.temperature:.2f}")
    for note in snapshot.notes:
        lines.append(f"note: {note}")
    return "\n".join(lines)
