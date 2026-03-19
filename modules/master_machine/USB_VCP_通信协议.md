# USB 虚拟串口通信协议

## 一、概述

- **物理链路**：STM32 USB CDC 虚拟 COM 端口
- **数据包格式**：Seasky 协议，固定 `SOF=0xA5`，小端字节序
- **数据布局**：`header(4) + cmd_id(2) + flags(2) + float[4] + crc16(2)` 共 16 字节
- **帧头结构**：
  | 字段 | 偏移 | 长度 | 说明 |
  |------|------|------|------|
  | SOF | 0 | 1 | 起始字节 `0xA5` |
  | data_length | 1 | 2 | payload 长度 |
  | crc8 | 3 | 1 | 帧头 CRC8 校验 |
- **完整帧 CRC**：`crc16(frame[0:data_len+6])`

---

## 二、上位机控制接口（主机 → 机器人）

### 2.1 云台控制 `0x0101`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bit0 | 帧有效 (active) |
| bit1 | 自动瞄准启用 (auto_aim) |
| bit2 | 相对目标 (relative) |
| bits[7:4] | 云台模式 |

**云台模式值**：
| 值 | 模式 | 说明 |
|----|------|------|
| 0 | ZERO_FORCE | 无力矩 |
| 1 | FREE_MODE | 自由 |
| 2 | GYRO_MODE | 陀螺仪 |

**float 数组**：
| 索引 | 字段 | 说明 | 单位 |
|------|------|------|------|
| float[0] | yaw | 偏航目标/增量 | 度 |
| float[1] | pitch | 俯仰目标/增量 | 度 |
| float[2] | - | 保留 | - |
| float[3] | - | 保留 | - |

---

### 2.2 底盘控制 `0x0102`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bit0 | 帧有效 (active) |
| bit1 | 停止请求 (stop) |
| bits[7:4] | 底盘模式 |

**底盘模式值**：
| 值 | 模式 | 说明 |
|----|------|------|
| 0 | ZERO_FORCE | 无力矩 |
| 1 | NO_FOLLOW | 不跟随 |
| 2 | FOLLOW_GIMBAL | 跟随云台 |
| 3 | RAW | 原始（直接 wz 控制） |
| 4 | ROTATE | 旋转 |

**float 数组**：
| 索引 | 字段 | 说明 | 单位 |
|------|------|------|------|
| float[0] | vx | X轴速度 | mm/s |
| float[1] | vy | Y轴速度 | mm/s |
| float[2] | wz | Z轴角速度 | 度/s |
| float[3] | speed_scale | 速度缩放 | % |

---

### 2.3 射击控制 `0x0103`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bit0 | 射击启用 (shoot_enable) |
| bit1 | 摩擦轮开启 (friction_on) |
| bit2 | 装弹机开启 (loader_on) |
| bit3 | 盖子打开 (lid_open) |
| bits[7:4] | 装弹机模式 |

**装弹机模式值**：
| 值 | 模式 | 说明 |
|----|------|------|
| 0 | STOP | 停止 |
| 1 | SINGLE | 单发 |
| 2 | TRIPLE | 三发 |
| 3 | BURST | 连射 |
| 4 | REVERSE | 反向 |
| 5 | EXTERNAL_SPEED | 外部速度 |

**float 数组**：
| 索引 | 字段 | 说明 | 单位 |
|------|------|------|------|
| float[0] | friction_speed | 摩擦轮目标速度 | rpm |
| float[1] | loader_speed | 装弹机目标速度 | rpm |
| float[2] | shoot_rate | 射击频率 | Hz |
| float[3] | - | 保留 | - |

---

## 三、机器人状态回传（机器人 → 主机）

### 3.1 云台状态 `0x0201`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bits[3:0] | 云台模式 |
| bit8 | 云台在线状态 |

**float 数组**：
| 索引 | 字段 | 说明 |
|------|------|------|
| float[0] | yaw | 当前 yaw 总角度 |
| float[1] | pitch | 当前 pitch 角度 |
| float[2] | yaw_motor_angle | yaw 电机单圈角度 |
| float[3] | yaw_total_angle_mirror | yaw 总角度镜像 |

---

### 3.2 底盘状态 `0x0202`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bits[3:0] | 底盘模式 |
| bit8 | 底盘在线状态 |

**float 数组**：
| 索引 | 字段 | 说明 |
|------|------|------|
| float[0] | vx | 当前/指令 vx |
| float[1] | vy | 当前/指令 vy |
| float[2] | wz | 当前/指令 wz |
| float[3] | offset_angle | 云台-底盘夹角 |

---

### 3.3 发射状态 `0x0203`

**flags 位定义**：
| bit | 功能 |
|-----|------|
| bits[3:0] | 发射模式 |
| bits[5:4] | 摩擦轮模式 |
| bits[11:8] | 拨盘模式 |
| bit15 | 发射在线状态 |

**float 数组**：
| 索引 | 字段 | 说明 |
|------|------|------|
| float[0] | friction_speed | 摩擦轮目标转速 |
| float[1] | loader_speed | 拨盘目标转速 |
| float[2] | shoot_rate | 射频 |
| float[3] | bullet_speed | 弹速限制 |

---

## 四、使用示例

### 4.1 Python 上位机调用

```python
from usb_vcp_host import SeaSkyProtocol, build_gimbal_frame, build_chassis_frame, build_shoot_frame

# 云台控制 - 绝对角度
frame = build_gimbal_frame(yaw=15.0, pitch=-3.0, mode=2, active=True, auto_aim=True, relative=False)
# 发送: serial.write(frame)

# 底盘控制 - 速度控制
frame = build_chassis_frame(vx=300.0, vy=0.0, wz=45.0, speed_scale=100.0, mode=3, active=True)

# 射击控制
frame = build_shoot_frame(friction_speed=28000.0, loader_speed=1500.0, shoot_rate=8.0,
                          loader_mode=5, shoot_enable=True, friction_on=True, loader_on=True)
```

### 4.2 命令行工具

```bash
# 交互式菜单
python python_tools/usb_vcp_host.py --menu

# 监听机器人状态
python python_tools/usb_vcp_host.py monitor --port COM5 --count 20

# 干跑测试（仅打印不发送）
python python_tools/usb_vcp_host.py --dry-run gimbal --yaw 15 --pitch -3 --auto-aim
python python_tools/usb_vcp_host.py --dry-run chassis --vx 300 --vy 0 --wz 45 --mode 3
python python_tools/usb_vcp_host.py --dry-run shoot --friction-speed 28000 --shoot-enable
```

---

## 五、协议命令码汇总

| CMD_ID | 方向 | 功能 |
|--------|------|------|
| 0x0001 | 视觉→机器人 | 传统视觉接收（自瞄数据） |
| 0x0002 | 机器人→主机 | 机器人遥测发送（姿态） |
| **0x0101** | **主机→机器人** | **云台控制** |
| **0x0102** | **主机→机器人** | **底盘控制** |
| **0x0103** | **主机→机器人** | **射击控制** |
| 0x0201 | 机器人→主机 | 云台状态回传 |
| 0x0202 | 机器人→主机 | 底盘状态回传 |
| 0x0203 | 机器人→主机 | 发射状态回传 |

---

## 六、CRC 校验说明

### CRC8（帧头校验）

用于校验帧头的前 3 字节（SOF + data_length + crc8 本身）

### CRC16（整帧校验）

用于校验从帧头到 payload 的所有字节，不包含 CRC16 本身

---

## 七、数据打包示例

```python
import struct

PROTOCOL_SOF = 0xA5

def pack_frame(cmd_id, flags, floats):
    # 1. 打包 payload: flags(2) + float[4](16)
    payload = struct.pack("<H", flags) + b"".join(struct.pack("<f", v) for v in floats)

    # 2. 构建帧头
    header = bytearray(4)
    header[0] = PROTOCOL_SOF
    header[1:3] = struct.pack("<H", len(payload))
    header[3] = crc8(bytes(header[:3]))

    # 3. 组装完整帧
    frame = bytes(header) + struct.pack("<H", cmd_id) + payload
    return frame + struct.pack("<H", crc16(frame))
```
