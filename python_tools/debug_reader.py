#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from debug_backend import ELF_FILE, OPENOCD_CONFIG, STLinkDebugBackend, snapshot_to_text


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STM32 Robot Debug Reader")
    parser.add_argument("--elf", default=str(ELF_FILE), help="ELF 文件路径")
    parser.add_argument("--config", default=str(OPENOCD_CONFIG), help="OpenOCD 配置路径")
    parser.add_argument("--gui", action="store_true", help="启动 PyQt 图形界面")
    parser.add_argument("--list", action="store_true", help="列出匹配符号")
    parser.add_argument("--keyword", default="", help="符号过滤关键字")
    parser.add_argument("--read", choices=["imu", "gimbal"], help="兼容旧版的快速读取模式")
    parser.add_argument("--variable", help="读取自定义变量名")
    parser.add_argument("--type", choices=["float", "int", "float_array"], default="float", help="自定义变量读取类型")
    parser.add_argument("--snapshot", action="store_true", help="读取一次系统快照")
    parser.add_argument("--monitor", action="store_true", help="持续监控系统快照")
    parser.add_argument("--interval", type=float, default=0.5, help="监控间隔秒数")
    parser.add_argument("--count", type=int, default=0, help="监控次数，0 为持续")
    parser.add_argument("--no-start-openocd", action="store_true", help="假设 OpenOCD 已经启动")
    return parser


def create_backend(args: argparse.Namespace) -> STLinkDebugBackend:
    backend = STLinkDebugBackend(Path(args.elf), Path(args.config))
    if not backend.parse_elf():
        raise RuntimeError("ELF 解析失败，请先确认已生成带符号的 ELF")
    return backend


def connect_backend(backend: STLinkDebugBackend, auto_start: bool) -> None:
    if auto_start and not backend.start_openocd():
        raise RuntimeError("OpenOCD 启动失败")
    if not backend.connect():
        raise RuntimeError("无法连接到 OpenOCD")


def run_snapshot_loop(backend: STLinkDebugBackend, interval: float, count: int) -> None:
    index = 0
    while count == 0 or index < count:
        snapshot = backend.read_snapshot()
        print("=" * 100)
        print(snapshot_to_text(snapshot))
        index += 1
        if count == 0 or index < count:
            time.sleep(interval)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    backend = create_backend(args)

    if args.list:
        for symbol in backend.list_symbols(args.keyword):
            print(f"{symbol.address:08X} {symbol.size:6d} {symbol.type_:>2} {symbol.name}")
        return 0

    if args.gui:
        connect_backend(backend, not args.no_start_openocd)
        from debug_gui import launch_gui

        try:
            return launch_gui(backend)
        finally:
            backend.stop_openocd()

    connect_backend(backend, not args.no_start_openocd)
    try:
        if args.read == "imu":
            print(backend.read_imu())
            return 0
        if args.read == "gimbal":
            motors = backend.read_motors()
            print(backend.read_gimbal(motors))
            return 0
        if args.variable:
            address = backend.get_symbol_address(args.variable)
            if address is None:
                raise RuntimeError(f"未找到变量: {args.variable}")
            if args.type == "float":
                print(backend.openocd.read_float(address))
            elif args.type == "int":
                values = backend.openocd.read_memory_32(address, 1)
                print(values[0] if values else None)
            else:
                size = backend.elf_parser.get_symbol_size(args.variable)
                count = max(size // 4, 1)
                print(backend.openocd.read_float_array(address, count))
            return 0
        if args.snapshot:
            print(snapshot_to_text(backend.read_snapshot()))
            return 0
        if args.monitor:
            run_snapshot_loop(backend, args.interval, args.count)
            return 0
        parser.print_help()
        return 0
    finally:
        backend.stop_openocd()


if __name__ == "__main__":
    sys.exit(main())
