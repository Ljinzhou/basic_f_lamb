# USB VCP 控制协议

## 当前机制

- 物理链路：STM32 USB CDC 虚拟 COM 端口。
- 数据包格式：Seasky 协议，固定 `SOF=0xA5`，小端字节序。
- 布局：`header(4) + cmd_id(2) + flags(2) + float[n] + crc16(2)`。
- 头部 CRC：`crc8(frame[0:3])`。
- 完整帧 CRC：`crc16(frame[0:data_len+6])`。

## 现有兼容消息

### `0x0001` 传统视觉接收

- `flags[1:0]`：`fire_mode`（射击模式）
- `flags[3:2]`：`target_state`（目标状态）
- `flags[7:4]`：`target_type`（目标类型）
- `float[0]`：`pitch`（俯仰角）
- `float[1]`：`yaw`（偏航角）

### `0x0002` 机器人遥测发送

- 现有的机器人到主机遥测。
- `float[0]`：`yaw`（偏航角）
- `float[1]`：`pitch`（俯仰角）
- `float[2]`：`roll`（横滚角）

## 新的主机控制消息

### `0x0101` 云台控制

- `bit0`：帧有效
- `bit1`：自动瞄准启用
- `bit2`：相对目标
- `bits[7:4]`：云台模式（`0=无力矩`，`1=自由`，`2=陀螺仪`）
- `float[0]`：偏航目标或偏航增量，单位：度
- `float[1]`：俯仰目标或俯仰增量，单位：度
- `float[2]`：保留
- `float[3]`：保留

### `0x0102` 底盘控制

- `bit0`：帧有效
- `bit1`：停止请求
- `bits[7:4]`：底盘模式（`0=无力矩`，`1=不跟随`，`2=跟随`，`3=原始`，`4=旋转`）
- `float[0]`：`vx`（X轴速度）
- `float[1]`：`vy`（Y轴速度）
- `float[2]`：`wz`（Z轴角速度）
- `float[3]`：速度缩放

### `0x0103` 射击控制

- `bit0`：射击启用
- `bit1`：摩擦轮开启
- `bit2`：装弹机开启
- `bit3`：盖子打开
- `bits[7:4]`：装弹机模式（`0=停止`，`1=单发`，`2=三发`，`3=连射`，`4=反向`，`5=外部速度`）
- `float[0]`：摩擦轮目标速度
- `float[1]`：装弹机目标速度
- `float[2]`：射击频率
- `float[3]`：保留

## 新增机器人状态回传消息

### `0x0201` 云台状态

- `flags[3:0]`：云台模式
- `flags[8]`：云台在线状态
- `float[0]`：当前 yaw 总角度
- `float[1]`：当前 pitch 角度
- `float[2]`：yaw 电机单圈角度
- `float[3]`：当前 yaw 总角度镜像

### `0x0202` 底盘状态

- `flags[3:0]`：底盘模式
- `flags[8]`：底盘在线状态
- `float[0]`：当前/当前指令 `vx`
- `float[1]`：当前/当前指令 `vy`
- `float[2]`：当前/当前指令 `wz`
- `float[3]`：云台-底盘夹角

### `0x0203` 发射状态

- `flags[3:0]`：发射模式
- `flags[5:4]`：摩擦轮模式
- `flags[11:8]`：拨盘模式
- `flags[15]`：发射在线状态
- `float[0]`：摩擦轮目标转速
- `float[1]`：拨盘目标转速
- `float[2]`：射频
- `float[3]`：弹速限制

## 控制映射

- 云台：映射到 `Gimbal_Ctrl_Cmd_s`，支持绝对和相对角度控制以及自动瞄准状态切换。
- 底盘：映射到 `Chassis_Ctrl_Cmd_s`，添加了 `CHASSIS_RAW` 用于直接 `wz` 控制。
- 射击：映射到 `Shoot_Ctrl_Cmd_s`，添加了自定义摩擦速度和 `LOAD_EXTERNAL_SPEED` 装弹机模式。

## 主机工具

- 发送脚本：`tools/usb_vcp_host.py`
- 菜单模式：`python tools/usb_vcp_host.py --menu`
- 状态监听：`python tools/usb_vcp_host.py monitor --port COM5 --count 20`
- 示例：

```bash
python tools/usb_vcp_host.py --dry-run gimbal --yaw 15 --pitch -3 --auto-aim
python tools/usb_vcp_host.py --dry-run chassis --vx 300 --vy 0 --wz 45 --mode 3
python tools/usb_vcp_host.py --dry-run shoot --friction-speed 28000 --loader-speed 1500 --shoot-rate 8 --shoot-enable --friction-on --loader-on
```
