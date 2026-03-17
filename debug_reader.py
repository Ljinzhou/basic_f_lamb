#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 Debug Reader - 使用ST-Link读取C板debug信息
支持读取IMU数据、姿态数据等
"""

import subprocess
import socket
import time
import struct
import argparse
import re
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import os

PROJECT_DIR = Path(__file__).parent.resolve()
BUILD_DIR = PROJECT_DIR / "build"
ELF_FILE = BUILD_DIR / "basic_framework.elf"
OPENOCD_CONFIG = PROJECT_DIR / "openocd_stlink.cfg"

ARM_TOOLCHAIN_PATH = r"C:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin"

OPENOCD_TELNET_PORT = 4444
OPENOCD_GDB_PORT = 3333


@dataclass
class IMUData:
    """IMU数据结构"""
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    roll: float
    pitch: float
    yaw: float
    yaw_total: float
    temperature: float


@dataclass
class GimbalData:
    """云台数据结构"""
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    roll: float
    pitch: float
    yaw: float
    yaw_total: float
    yaw_motor_angle: int
    temperature: float


@dataclass
class SymbolInfo:
    """符号信息"""
    name: str
    address: int
    size: int
    type_: str


class ELFParser:
    """ELF文件解析器，获取变量地址"""
    
    def __init__(self, elf_path: Path):
        self.elf_path = elf_path
        self.symbols: Dict[str, SymbolInfo] = {}
        
    def parse(self) -> bool:
        """解析ELF文件获取符号表"""
        if not self.elf_path.exists():
            print(f"[ERROR] ELF文件不存在: {self.elf_path}")
            return False
        
        nm_exe = Path(ARM_TOOLCHAIN_PATH) / "arm-none-eabi-nm.exe"
        if not nm_exe.exists():
            print(f"[ERROR] 未找到工具链: {nm_exe}")
            return False
        
        try:
            result = subprocess.run(
                [str(nm_exe), "-S", "-C", str(self.elf_path)],
                capture_output=True,
                text=True,
                encoding='utf-8',
                errors='replace'
            )
            
            if result.returncode != 0:
                print(f"[ERROR] nm命令执行失败: {result.stderr}")
                return False
            
            for line in result.stdout.strip().split('\n'):
                if not line:
                    continue
                    
                parts = line.split()
                if len(parts) >= 3:
                    try:
                        address = int(parts[0], 16)
                        size = int(parts[1], 16) if len(parts) > 3 else 0
                        type_ = parts[2] if len(parts) > 3 else parts[1]
                        name = parts[3] if len(parts) > 3 else parts[2]
                        
                        self.symbols[name] = SymbolInfo(
                            name=name,
                            address=address,
                            size=size,
                            type_=type_
                        )
                    except (ValueError, IndexError):
                        continue
            
            print(f"[INFO] 解析到 {len(self.symbols)} 个符号")
            return True
            
        except FileNotFoundError:
            print(f"[ERROR] 未找到 arm-none-eabi-nm 命令，请检查工具链路径: {ARM_TOOLCHAIN_PATH}")
            return False
    
    def get_symbol_address(self, name: str) -> Optional[int]:
        """获取符号地址"""
        if name in self.symbols:
            return self.symbols[name].address
        
        for sym_name, sym_info in self.symbols.items():
            if name in sym_name or sym_name.endswith(name):
                return sym_info.address
        
        return None
    
    def get_symbol_size(self, name: str) -> int:
        """获取符号大小"""
        if name in self.symbols:
            return self.symbols[name].size
        
        for sym_name, sym_info in self.symbols.items():
            if name in sym_name or sym_name.endswith(name):
                return sym_info.size
        
        return 0


class OpenOCDClient:
    """OpenOCD Telnet客户端"""
    
    def __init__(self, host: str = "localhost", port: int = OPENOCD_TELNET_PORT):
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self.connected = False
        
    def connect(self, timeout: float = 5.0) -> bool:
        """连接到OpenOCD Telnet服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(timeout)
            self.socket.connect((self.host, self.port))
            self.connected = True
            self._recv_until(b"> ")
            print(f"[INFO] 已连接到OpenOCD Telnet服务器 {self.host}:{self.port}")
            return True
        except socket.error as e:
            print(f"[ERROR] 连接OpenOCD失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
        self.socket = None
        self.connected = False
    
    def _recv_until(self, delimiter: bytes, timeout: float = 5.0) -> bytes:
        """接收数据直到遇到分隔符"""
        if not self.socket:
            return b""
        
        data = b""
        self.socket.settimeout(timeout)
        
        while delimiter not in data:
            try:
                chunk = self.socket.recv(4096)
                if not chunk:
                    break
                data += chunk
            except socket.timeout:
                break
        
        return data
    
    def send_command(self, command: str) -> str:
        """发送命令并获取响应"""
        if not self.connected or not self.socket:
            return ""
        
        try:
            self.socket.sendall((command + "\n").encode('utf-8'))
            response = self._recv_until(b"> ")
            return response.decode('utf-8', errors='replace')
        except socket.error as e:
            print(f"[ERROR] 发送命令失败: {e}")
            return ""
    
    def read_memory_32(self, address: int, count: int = 1) -> List[int]:
        """读取32位内存数据"""
        cmd = f"mdw {address:#x} {count}"
        response = self.send_command(cmd)
        
        if not response:
            print(f"[DEBUG] mdw响应为空，地址: 0x{address:08X}")
            return []
        
        values = []
        for line in response.split('\n'):
            match = re.search(r'0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]+\s*)+)', line)
            if match:
                hex_values = match.group(1).split()
                for hv in hex_values:
                    try:
                        values.append(int(hv, 16))
                    except ValueError:
                        continue
        
        return values
    
    def read_memory_8(self, address: int, count: int = 1) -> List[int]:
        """读取8位内存数据"""
        cmd = f"mdb {address:#x} {count}"
        response = self.send_command(cmd)
        
        values = []
        for line in response.split('\n'):
            match = re.search(r'0x[0-9a-fA-F]+:\s+((?:0x[0-9a-fA-F]+\s*)+)', line)
            if match:
                hex_values = match.group(1).split()
                for hv in hex_values:
                    try:
                        values.append(int(hv, 16))
                    except ValueError:
                        continue
        
        return values
    
    def read_float(self, address: int) -> float:
        """读取单个float值"""
        values = self.read_memory_32(address, 1)
        if values:
            return struct.unpack('<f', struct.pack('<I', values[0]))[0]
        return 0.0
    
    def read_float_array(self, address: int, count: int) -> List[float]:
        """读取float数组"""
        values = self.read_memory_32(address, count)
        floats = []
        for v in values:
            floats.append(struct.unpack('<f', struct.pack('<I', v))[0])
        return floats
    
    def read_memory_16(self, address: int, count: int = 1) -> List[int]:
        """读取16位内存数据"""
        cmd = f"mdh {address:#x} {count}"
        response = self.send_command(cmd)
        
        if not response:
            return []
        
        values = []
        for line in response.split('\n'):
            match = re.search(r'0x[0-9a-fA-F]+:\s+((?:[0-9a-fA-F]+\s*)+)', line)
            if match:
                hex_values = match.group(1).split()
                for hv in hex_values:
                    try:
                        values.append(int(hv, 16))
                    except ValueError:
                        continue
        
        return values
    
    def halt(self) -> bool:
        """暂停CPU"""
        response = self.send_command("halt")
        return "target halted" in response.lower() or "already halted" in response.lower()
    
    def resume(self) -> bool:
        """恢复CPU运行"""
        response = self.send_command("resume")
        return True


class STLinkDebugReader:
    """ST-Link Debug读取器"""
    
    IMU_VARIABLES = {
        'INS': {
            'Gyro': {'offset': 0, 'count': 3, 'desc': '角速度 (rad/s)'},
            'Accel': {'offset': 12, 'count': 3, 'desc': '加速度 (m/s²)'},
            'Roll': {'offset': 24, 'count': 1, 'desc': '横滚角 (°)'},
            'Pitch': {'offset': 28, 'count': 1, 'desc': '俯仰角 (°)'},
            'Yaw': {'offset': 32, 'count': 1, 'desc': '偏航角 (°)'},
            'YawTotalAngle': {'offset': 36, 'count': 1, 'desc': '偏航总角度 (°)'},
        },
        'BMI088': {
            'Gyro': {'offset': 0, 'count': 3, 'desc': '陀螺仪原始数据 (rad/s)'},
            'Accel': {'offset': 12, 'count': 3, 'desc': '加速度计原始数据 (m/s²)'},
            'Temperature': {'offset': 32, 'count': 1, 'desc': '温度 (°C)'},
        }
    }
    
    def __init__(self, elf_path: Path = ELF_FILE, config_path: Path = OPENOCD_CONFIG):
        self.elf_path = elf_path
        self.config_path = config_path
        self.elf_parser = ELFParser(elf_path)
        self.openocd: Optional[OpenOCDClient] = None
        self.openocd_process: Optional[subprocess.Popen] = None
        self.symbol_cache: Dict[str, int] = {}
        
    def start_openocd(self) -> bool:
        """启动OpenOCD服务器"""
        if not self.config_path.exists():
            print(f"[ERROR] OpenOCD配置文件不存在: {self.config_path}")
            return False
        
        print("[INFO] 启动OpenOCD...")
        
        config_str = str(self.config_path).replace("\\", "/")
        
        self.openocd_process = subprocess.Popen(
            ["openocd", "-f", config_str],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding='utf-8',
            errors='replace'
        )
        
        print("[INFO] 等待OpenOCD启动...")
        time.sleep(2)
        
        if self.openocd_process.poll() is not None:
            output = self.openocd_process.stdout.read()
            print(f"[ERROR] OpenOCD启动失败:\n{output}")
            return False
        
        print("[INFO] OpenOCD已启动")
        return True
    
    def stop_openocd(self):
        """停止OpenOCD"""
        if self.openocd:
            self.openocd.disconnect()
            self.openocd = None
        
        if self.openocd_process:
            self.openocd_process.terminate()
            try:
                self.openocd_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.openocd_process.kill()
            self.openocd_process = None
        
        print("[INFO] OpenOCD已停止")
    
    def connect(self) -> bool:
        """连接到目标板"""
        self.openocd = OpenOCDClient()
        
        for i in range(5):
            if self.openocd.connect():
                return True
            print(f"[WARN] 连接失败，重试 {i+1}/5...")
            time.sleep(1)
        
        return False
    
    def parse_elf(self) -> bool:
        """解析ELF文件"""
        return self.elf_parser.parse()
    
    def get_variable_address(self, var_name: str) -> Optional[int]:
        """获取变量地址"""
        if var_name in self.symbol_cache:
            return self.symbol_cache[var_name]
        
        addr = self.elf_parser.get_symbol_address(var_name)
        if addr is not None:
            self.symbol_cache[var_name] = addr
        
        return addr
    
    def read_imu_data(self) -> Optional[IMUData]:
        """读取IMU数据"""
        ins_addr = self.get_variable_address('INS')
        bmi088_addr = self.get_variable_address('BMI088')
        
        if ins_addr is None and bmi088_addr is None:
            print("[ERROR] 未找到INS或BMI088变量")
            print("[DEBUG] 尝试查找其他符号...")
            for name in ['gimba_IMU_data', 'QEKF_INS', 'ins_data']:
                addr = self.get_variable_address(name)
                if addr:
                    print(f"[DEBUG] 找到 {name} @ 0x{addr:08X}")
                    ins_addr = addr
                    break
        
        if ins_addr is None and bmi088_addr is None:
            return None
        
        imu_data = IMUData(
            gyro=(0.0, 0.0, 0.0),
            accel=(0.0, 0.0, 0.0),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            yaw_total=0.0,
            temperature=0.0
        )
        
        INS_GYRO_OFFSET = 80
        INS_ACCEL_OFFSET = 92
        INS_ROLL_OFFSET = 104
        INS_PITCH_OFFSET = 108
        INS_YAW_OFFSET = 112
        INS_YAW_TOTAL_OFFSET = 116
        
        BMI088_TEMP_OFFSET = 28
        
        if bmi088_addr is not None:
            gyro_raw = self.openocd.read_float_array(bmi088_addr + 0, 3)
            accel_raw = self.openocd.read_float_array(bmi088_addr + 12, 3)
            temp = self.openocd.read_float(bmi088_addr + BMI088_TEMP_OFFSET)
            
            if gyro_raw and len(gyro_raw) == 3:
                imu_data.gyro = tuple(gyro_raw)
            if accel_raw and len(accel_raw) == 3:
                imu_data.accel = tuple(accel_raw)
            imu_data.temperature = temp
        
        if ins_addr is not None:
            gyro = self.openocd.read_float_array(ins_addr + INS_GYRO_OFFSET, 3)
            accel = self.openocd.read_float_array(ins_addr + INS_ACCEL_OFFSET, 3)
            roll = self.openocd.read_float(ins_addr + INS_ROLL_OFFSET)
            pitch = self.openocd.read_float(ins_addr + INS_PITCH_OFFSET)
            yaw = self.openocd.read_float(ins_addr + INS_YAW_OFFSET)
            yaw_total = self.openocd.read_float(ins_addr + INS_YAW_TOTAL_OFFSET)
            
            if gyro and len(gyro) == 3:
                imu_data.gyro = tuple(gyro)
            if accel and len(accel) == 3:
                imu_data.accel = tuple(accel)
            imu_data.roll = roll
            imu_data.pitch = pitch
            imu_data.yaw = yaw
            imu_data.yaw_total = yaw_total
        
        return imu_data
    
    def read_gimbal_data(self) -> Optional[GimbalData]:
        """读取云台数据"""
        gimbal_feedback_addr = self.get_variable_address('gimbal_feedback_data')
        
        if gimbal_feedback_addr is None:
            return None
        
        gimbal_data = GimbalData(
            gyro=(0.0, 0.0, 0.0),
            accel=(0.0, 0.0, 0.0),
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            yaw_total=0.0,
            yaw_motor_angle=0,
            temperature=0.0
        )
        
        gyro = self.openocd.read_float_array(gimbal_feedback_addr + 0, 3)
        accel = self.openocd.read_float_array(gimbal_feedback_addr + 12, 3)
        roll = self.openocd.read_float(gimbal_feedback_addr + 24)
        pitch = self.openocd.read_float(gimbal_feedback_addr + 28)
        yaw = self.openocd.read_float(gimbal_feedback_addr + 32)
        yaw_total = self.openocd.read_float(gimbal_feedback_addr + 36)
        
        yaw_motor_values = self.openocd.read_memory_16(gimbal_feedback_addr + 40, 1)
        yaw_motor_angle = yaw_motor_values[0] if yaw_motor_values else 0
        
        if gyro and len(gyro) == 3:
            gimbal_data.gyro = tuple(gyro)
        if accel and len(accel) == 3:
            gimbal_data.accel = tuple(accel)
        gimbal_data.roll = roll
        gimbal_data.pitch = pitch
        gimbal_data.yaw = yaw
        gimbal_data.yaw_total = yaw_total
        gimbal_data.yaw_motor_angle = yaw_motor_angle
        
        bmi088_addr = self.get_variable_address('BMI088')
        if bmi088_addr:
            gimbal_data.temperature = self.openocd.read_float(bmi088_addr + 28)
        
        return gimbal_data
    
    def read_custom_variable(self, var_name: str, var_type: str = 'float') -> Optional[any]:
        """读取自定义变量"""
        addr = self.get_variable_address(var_name)
        if addr is None:
            print(f"[ERROR] 未找到变量: {var_name}")
            return None
        
        size = self.elf_parser.get_symbol_size(var_name)
        
        if var_type == 'float':
            return self.openocd.read_float(addr)
        elif var_type == 'int':
            values = self.openocd.read_memory_32(addr, max(1, size // 4))
            return values[0] if values else None
        elif var_type == 'float_array' and size > 0:
            count = size // 4
            return self.openocd.read_float_array(addr, count)
        
        return None
    
    def list_imu_symbols(self):
        """列出IMU相关符号"""
        print("\n[INFO] IMU相关符号:")
        print("-" * 60)
        
        keywords = ['INS', 'BMI088', 'IMU', 'gyro', 'accel', 'Gyro', 'Accel', 
                    'Roll', 'Pitch', 'Yaw', 'temperature', 'Temperature']
        
        found = []
        for name, info in self.elf_parser.symbols.items():
            for kw in keywords:
                if kw.lower() in name.lower():
                    found.append((name, info))
                    break
        
        found.sort(key=lambda x: x[1].address)
        
        for name, info in found[:30]:
            print(f"  {name}: 0x{info.address:08X} (size: {info.size})")
        
        if len(found) > 30:
            print(f"  ... 还有 {len(found) - 30} 个符号")
    
    def monitor_imu(self, interval: float = 0.1, count: int = 0):
        """持续监控云台IMU数据"""
        gimbal_feedback_addr = self.get_variable_address('gimbal_feedback_data')
        ins_addr = self.get_variable_address('INS')
        bmi088_addr = self.get_variable_address('BMI088')
        
        if gimbal_feedback_addr is None and ins_addr is None and bmi088_addr is None:
            print("[ERROR] 未找到任何IMU变量，请先运行 --list 查看可用符号")
            return
        
        print("\n" + "=" * 100)
        print("云台IMU数据监控 (按Ctrl+C退出)")
        print("=" * 100)
        
        try:
            i = 0
            while count == 0 or i < count:
                data = self.read_gimbal_data()
                if data is None:
                    data = self.read_imu_data()
                    if data:
                        data = GimbalData(
                            gyro=data.gyro,
                            accel=data.accel,
                            roll=data.roll,
                            pitch=data.pitch,
                            yaw=data.yaw,
                            yaw_total=data.yaw_total,
                            yaw_motor_angle=0,
                            temperature=data.temperature
                        )
                
                if data:
                    attitude_str = f"Pitch:{data.pitch:7.2f}° Yaw:{data.yaw:7.2f}° YawTotal:{data.yaw_total:8.2f}°"
                    gyro_str = f"Gyro: X:{data.gyro[0]:7.3f} Y:{data.gyro[1]:7.3f} Z:{data.gyro[2]:7.3f}"
                    temp_str = f"T:{data.temperature:.1f}°C"
                    
                    line = f"{attitude_str} | {gyro_str} | {temp_str}"
                    print(f"\r{line:<120}", end="", flush=True)
                else:
                    print(f"\r{'读取失败':<120}", end="", flush=True)
                
                time.sleep(interval)
                i += 1
                
        except KeyboardInterrupt:
            print("\n\n[INFO] 停止监控")
    
    def calibrate_pitch(self):
        """校准Pitch参数"""
        print("\n" + "=" * 80)
        print("Pitch校准模式")
        print("=" * 80)
        print("\n此模式将帮助你设置 robot_def.h 中的以下参数:")
        print("  - PITCH_HORIZON_ECD: 云台水平位置时编码器值")
        print("  - PITCH_MAX_ANGLE: 云台最大仰角")
        print("  - PITCH_MIN_ANGLE: 云台最大俯角")
        print("\n操作步骤:")
        print("  1. 将云台转到水平位置，按Enter记录")
        print("  2. 将云台转到最大仰角位置，按Enter记录")
        print("  3. 将云台转到最大俯角位置，按Enter记录")
        print("-" * 80)
        
        pitch_motor_addr = self.get_variable_address('pitch_motor')
        ins_addr = self.get_variable_address('INS')
        gimbal_feedback_addr = self.get_variable_address('gimbal_feedback_data')
        
        if pitch_motor_addr is None:
            print("[ERROR] 未找到pitch_motor变量")
            return
        
        if ins_addr is None and gimbal_feedback_addr is None:
            print("[ERROR] 未找到INS或gimbal_feedback_data变量")
            return
        
        input("\n准备好后按Enter开始...")
        
        print("\n[步骤1] 请将云台转到水平位置，然后按Enter...")
        input()
        horizon_ecd = self._read_pitch_ecd(pitch_motor_addr)
        print(f"  水平位置编码器值: {horizon_ecd}")
        
        print("\n[步骤2] 请将云台转到最大仰角位置(向上)，然后按Enter...")
        input()
        max_ecd = self._read_pitch_ecd(pitch_motor_addr)
        max_pitch = self._read_pitch_angle(ins_addr, gimbal_feedback_addr)
        print(f"  最大仰角位置编码器值: {max_ecd}")
        print(f"  当前IMU Pitch角度: {max_pitch:.2f}°")
        
        print("\n[步骤3] 请将云台转到最大俯角位置(向下)，然后按Enter...")
        input()
        min_ecd = self._read_pitch_ecd(pitch_motor_addr)
        min_pitch = self._read_pitch_angle(ins_addr, gimbal_feedback_addr)
        print(f"  最大俯角位置编码器值: {min_ecd}")
        print(f"  当前IMU Pitch角度: {min_pitch:.2f}°")
        
        print("\n" + "=" * 80)
        print("校准结果 - 请将以下值复制到 robot_def.h:")
        print("=" * 80)
        print(f"#define PITCH_HORIZON_ECD {horizon_ecd}      // 云台处于水平位置时编码器值")
        print(f"#define PITCH_MAX_ANGLE {max_pitch:.1f}f       // 云台竖直方向最大角度,向上为正")
        print(f"#define PITCH_MIN_ANGLE {min_pitch:.1f}f      // 云台竖直方向最小角度,向下为负")
        print("=" * 80)
    
    def _read_pitch_ecd(self, pitch_motor_addr: int) -> int:
        """读取pitch电机编码器值"""
        ptr_values = self.openocd.read_memory_32(pitch_motor_addr, 1)
        if not ptr_values:
            return 0
        
        motor_instance_addr = ptr_values[0]
        if motor_instance_addr == 0:
            return 0
        
        ecd_offset = 2
        ecd_values = self.openocd.read_memory_16(motor_instance_addr + ecd_offset, 1)
        return ecd_values[0] if ecd_values else 0
    
    def _read_pitch_angle(self, ins_addr: int, gimbal_feedback_addr: int) -> float:
        """读取pitch角度"""
        if gimbal_feedback_addr:
            return self.openocd.read_float(gimbal_feedback_addr + 28)
        elif ins_addr:
            return self.openocd.read_float(ins_addr + 108)
        return 0.0


def main():
    parser = argparse.ArgumentParser(
        description="STM32 Debug Reader - 使用ST-Link读取C板debug信息",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python debug_reader.py --monitor              # 持续监控IMU数据
  python debug_reader.py --monitor -i 0.05      # 以20Hz频率监控
  python debug_reader.py --read INS             # 读取INS变量
  python debug_reader.py --read BMI088          # 读取BMI088变量
  python debug_reader.py --list                 # 列出IMU相关符号
  python debug_reader.py --calibrate            # 校准Pitch参数
  python debug_reader.py --variable gyro --type float_array  # 读取自定义变量
        """
    )
    
    parser.add_argument(
        "--elf", "-e",
        type=str,
        default=str(ELF_FILE),
        help=f"ELF文件路径 (默认: {ELF_FILE})"
    )
    
    parser.add_argument(
        "--config", "-c",
        type=str,
        default=str(OPENOCD_CONFIG),
        help=f"OpenOCD配置文件路径 (默认: {OPENOCD_CONFIG})"
    )
    
    parser.add_argument(
        "--monitor", "-m",
        action="store_true",
        help="持续监控IMU数据"
    )
    
    parser.add_argument(
        "--interval", "-i",
        type=float,
        default=0.1,
        help="监控间隔(秒) (默认: 0.1)"
    )
    
    parser.add_argument(
        "--count",
        type=int,
        default=0,
        help="读取次数 (默认: 0=无限)"
    )
    
    parser.add_argument(
        "--read", "-r",
        type=str,
        choices=['INS', 'BMI088', 'imu'],
        help="读取指定变量组"
    )
    
    parser.add_argument(
        "--variable", "-v",
        type=str,
        help="读取自定义变量名"
    )
    
    parser.add_argument(
        "--type", "-t",
        type=str,
        choices=['float', 'int', 'float_array'],
        default='float',
        help="变量类型 (默认: float)"
    )
    
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="列出IMU相关符号"
    )
    
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="校准Pitch参数"
    )
    
    parser.add_argument(
        "--no-start-openocd",
        action="store_true",
        help="不自动启动OpenOCD (假设OpenOCD已在运行)"
    )
    
    args = parser.parse_args()
    
    elf_path = Path(args.elf)
    config_path = Path(args.config)
    
    reader = STLinkDebugReader(elf_path, config_path)
    
    print("\n" + "=" * 50)
    print("STM32 Debug Reader")
    print("=" * 50)
    
    if not reader.parse_elf():
        sys.exit(1)
    
    if args.list:
        reader.list_imu_symbols()
        sys.exit(0)
    
    start_openocd = not args.no_start_openocd
    
    if start_openocd:
        if not reader.start_openocd():
            sys.exit(1)
    
    try:
        if not reader.connect():
            print("[ERROR] 无法连接到OpenOCD")
            sys.exit(1)
        
        if args.monitor:
            reader.monitor_imu(interval=args.interval, count=args.count)
        
        elif args.read:
            if args.read.upper() == 'INS' or args.read == 'imu':
                data = reader.read_imu_data()
                if data:
                    print("\n[INS数据]")
                    print(f"  角速度: X={data.gyro[0]:.4f} Y={data.gyro[1]:.4f} Z={data.gyro[2]:.4f}")
                    print(f"  加速度: X={data.accel[0]:.4f} Y={data.accel[1]:.4f} Z={data.accel[2]:.4f}")
                    print(f"  姿态角: Roll={data.roll:.2f}° Pitch={data.pitch:.2f}° Yaw={data.yaw:.2f}°")
                    print(f"  偏航总角度: {data.yaw_total:.2f}°")
            
            elif args.read.upper() == 'BMI088':
                bmi088_addr = reader.get_variable_address('BMI088')
                if bmi088_addr:
                    gyro = reader.openocd.read_float_array(bmi088_addr + 0, 3)
                    accel = reader.openocd.read_float_array(bmi088_addr + 12, 3)
                    temp = reader.openocd.read_float(bmi088_addr + 32)
                    
                    print("\n[BMI088数据]")
                    print(f"  陀螺仪: X={gyro[0]:.4f} Y={gyro[1]:.4f} Z={gyro[2]:.4f}")
                    print(f"  加速度计: X={accel[0]:.4f} Y={accel[1]:.4f} Z={accel[2]:.4f}")
                    print(f"  温度: {temp:.2f}°C")
        
        elif args.variable:
            value = reader.read_custom_variable(args.variable, args.type)
            if value is not None:
                addr = reader.get_variable_address(args.variable)
                print(f"\n[{args.variable}] @ 0x{addr:08X}")
                if isinstance(value, list):
                    print(f"  值: {value}")
                else:
                    print(f"  值: {value}")
        
        elif args.calibrate:
            reader.calibrate_pitch()
        
        else:
            parser.print_help()
    
    finally:
        if start_openocd:
            reader.stop_openocd()


if __name__ == "__main__":
    main()
