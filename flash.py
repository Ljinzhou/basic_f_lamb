#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 一键编译烧录脚本
支持 CMake + Ninja 编译，OpenOCD + DAPLink 烧录
"""

import subprocess
import os
import sys
import argparse
import time
from pathlib import Path

PROJECT_DIR = Path(__file__).parent.resolve()
BUILD_DIR = PROJECT_DIR / "build"
ELF_FILE = BUILD_DIR / "basic_framework.elf"
HEX_FILE = BUILD_DIR / "basic_framework.hex"

ARM_TOOLCHAIN_PATH = r"C:\Program Files (x86)\Arm\GNU Toolchain mingw-w64-i686-arm-none-eabi\bin"
NINJA_PATH = str(Path.home() / "scoop" / "apps" / "ninja" / "current")

OPENOCD_CONFIG_DAP = PROJECT_DIR / "openocd_dap.cfg"
OPENOCD_CONFIG_STLINK = PROJECT_DIR / "openocd_stlink.cfg"
OPENOCD_CONFIG_JLINK = PROJECT_DIR / "openocd_jlink.cfg"


def run_command(cmd, cwd=None, env=None):
    """运行命令并实时输出"""
    print(f"\n[CMD] {' '.join(cmd)}")
    
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    
    process = subprocess.Popen(
        cmd,
        cwd=cwd,
        env=merged_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding='utf-8',
        errors='replace'
    )
    
    for line in process.stdout:
        print(line, end='')
    
    process.wait()
    return process.returncode


def build(clean=False):
    """编译项目"""
    print("\n" + "="*50)
    print("开始编译...")
    print("="*50)
    
    env = {
        "PATH": f"{NINJA_PATH};{ARM_TOOLCHAIN_PATH};{os.environ.get('PATH', '')}"
    }
    
    if clean or not BUILD_DIR.exists():
        print("\n[INFO] 清理并重新配置...")
        if BUILD_DIR.exists():
            import shutil
            shutil.rmtree(BUILD_DIR)
        BUILD_DIR.mkdir(parents=True, exist_ok=True)
        
        ret = run_command(
            ["cmake", "-G", "Ninja", ".."],
            cwd=BUILD_DIR,
            env=env
        )
        if ret != 0:
            print(f"\n[ERROR] CMake 配置失败! 返回码: {ret}")
            return False
    
    print("\n[INFO] 开始编译...")
    ret = run_command(
        ["cmake", "--build", "."],
        cwd=BUILD_DIR,
        env=env
    )
    
    if ret != 0:
        print(f"\n[ERROR] 编译失败! 返回码: {ret}")
        return False
    
    if not ELF_FILE.exists():
        print(f"\n[ERROR] 编译产物不存在: {ELF_FILE}")
        return False
    
    print("\n[SUCCESS] 编译成功!")
    return True


def flash(file_path=None, programmer="dap"):
    """烧录固件
    
    Args:
        file_path: 要烧录的文件路径
        programmer: 烧录器类型，可选 "dap", "stlink", "jlink"
    """
    print("\n" + "="*50)
    print("开始烧录...")
    print("="*50)
    
    # 根据烧录器选择配置文件
    programmer = programmer.lower()
    if programmer == "stlink":
        openocd_config = OPENOCD_CONFIG_STLINK
    elif programmer == "jlink":
        openocd_config = OPENOCD_CONFIG_JLINK
    else:
        openocd_config = OPENOCD_CONFIG_DAP
    
    if not openocd_config.exists():
        print(f"\n[ERROR] OpenOCD配置文件不存在: {openocd_config}")
        return False
    
    if file_path is None:
        file_path = ELF_FILE
    
    if not file_path.exists():
        print(f"\n[ERROR] 文件不存在: {file_path}")
        return False
    
    print(f"[INFO] 使用烧录器: {programmer}")
    print(f"[INFO] 配置文件: {openocd_config}")
    
    file_path_str = str(file_path).replace("\\", "/")
    openocd_config_str = str(openocd_config).replace("\\", "/")
    
    cmd = [
        "openocd",
        "-f", openocd_config_str,
        "-c", f"program {file_path_str} verify reset exit"
    ]
    
    ret = run_command(cmd, cwd=PROJECT_DIR)
    
    if ret != 0:
        print(f"\n[ERROR] 烧录失败! 返回码: {ret}")
        print("\n[提示] 请检查:")
        print("  1. 调试器是否正确连接")
        print("  2. OpenOCD是否已安装并添加到PATH")
        print("  3. 目标板是否上电")
        return False
    
    print("\n[SUCCESS] 烧录成功!")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="STM32 一键编译烧录脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python flash.py                      # 编译并烧录 (默认DAPLink)
  python flash.py --build               # 仅编译
  python flash.py --flash              # 仅烧录
  python flash.py --flash hex          # 烧录hex文件
  python flash.py --clean               # 清理后重新编译
  python flash.py --build --flash      # 编译后烧录
  python flash.py --programmer stlink   # 使用STLink烧录
  python flash.py --programmer jlink    # 使用JLink烧录
  python flash.py -p stlink -f hex     # 使用STLink烧录hex文件
        """
    )
    
    parser.add_argument(
        "--build", "-b",
        action="store_true",
        help="仅编译"
    )
    parser.add_argument(
        "--flash", "-f",
        nargs="?",
        const="elf",
        choices=["elf", "hex"],
        help="仅烧录 (可选: elf 或 hex，默认 elf)"
    )
    parser.add_argument(
        "--clean", "-c",
        action="store_true",
        help="清理后重新编译"
    )
    
    parser.add_argument(
        "--programmer", "-p",
        dest="programmer",
        default="dap",
        choices=["dap", "stlink", "jlink"],
        help="烧录器类型 (默认: dap)"
    )
    
    args = parser.parse_args()
    
    do_build = args.build or args.clean or (not args.flash)
    do_flash = args.flash is not None or (not args.build and not args.flash and not args.clean)
    
    start_time = time.time()
    
    if do_build:
        if not build(clean=args.clean):
            sys.exit(1)
    
    if do_flash:
        flash_file = HEX_FILE if args.flash == "hex" else ELF_FILE
        if not flash(file_path=flash_file, programmer=args.programmer):
            sys.exit(1)
    
    elapsed = time.time() - start_time
    print(f"\n{'='*50}")
    print(f"完成! 耗时: {elapsed:.2f} 秒")
    print("="*50)


if __name__ == "__main__":
    main()
