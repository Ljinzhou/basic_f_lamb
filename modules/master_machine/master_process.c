/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data
 * @version beta
 * @date 2022-11-03
 * @todo 增加对串口调试助手协议的支持,包括vofa和serial debug
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Send_s send_data;
static USB_Control_Data_s usb_ctrl_data;
static USB_Gimbal_Status_s usb_gimbal_status;
static USB_Chassis_Status_s usb_chassis_status;
static USB_Shoot_Status_s usb_shoot_status;
static DaemonInstance *vision_daemon_instance;
static uint8_t usb_status_round_robin;

enum
{
    VISION_FLAG_FIRE_MODE_MASK = 0x0003,
    VISION_FLAG_TARGET_STATE_MASK = 0x000C,
    VISION_FLAG_TARGET_TYPE_MASK = 0x00F0,
    USB_CTRL_FLAG_ACTIVE = 0x0001,
    USB_CTRL_FLAG_AUTO_AIM = 0x0002,
    USB_CTRL_FLAG_RELATIVE = 0x0004,
    USB_CTRL_FLAG_STOP = 0x0002,
    USB_CTRL_FLAG_SHOOT_ENABLE = 0x0001,
    USB_CTRL_FLAG_FRICTION_ON = 0x0002,
    USB_CTRL_FLAG_LOADER_ON = 0x0004,
    USB_CTRL_FLAG_LID_OPEN = 0x0008,
    USB_CTRL_MODE_SHIFT = 4,
    USB_CTRL_MODE_MASK = 0x00F0,
};

static uint8_t ProtocolDataLengthValid(const uint8_t *raw_buf)
{
    uint16_t data_length = (uint16_t)raw_buf[1] | ((uint16_t)raw_buf[2] << 8);
    return data_length <= (uint16_t)(2u + USB_CONTROL_FLOAT_COUNT * sizeof(float));
}

static void UpdateUSBControlEnableState(void)
{
    usb_ctrl_data.host_enabled = usb_ctrl_data.gimbal.active || usb_ctrl_data.chassis.active || usb_ctrl_data.shoot.active;
}

static void DecodeLegacyVisionFlags(uint16_t flag_register)
{
    recv_data.fire_mode = (Fire_Mode_e)(flag_register & VISION_FLAG_FIRE_MODE_MASK);
    recv_data.target_state = (Target_State_e)((flag_register & VISION_FLAG_TARGET_STATE_MASK) >> 2);
    recv_data.target_type = (Target_Type_e)((flag_register & VISION_FLAG_TARGET_TYPE_MASK) >> 4);
}

static void DecodeUSBGimbalCommand(uint16_t flag_register, const float *rx_data)
{
    usb_ctrl_data.gimbal.active = (flag_register & USB_CTRL_FLAG_ACTIVE) != 0u;
    usb_ctrl_data.gimbal.auto_aim_enabled = (flag_register & USB_CTRL_FLAG_AUTO_AIM) != 0u;
    usb_ctrl_data.gimbal.relative_target = (flag_register & USB_CTRL_FLAG_RELATIVE) != 0u;
    usb_ctrl_data.gimbal.mode = (Host_Gimbal_Mode_e)((flag_register & USB_CTRL_MODE_MASK) >> USB_CTRL_MODE_SHIFT);
    usb_ctrl_data.gimbal.yaw = rx_data[0];
    usb_ctrl_data.gimbal.pitch = rx_data[1];
    UpdateUSBControlEnableState();
}

static void DecodeUSBChassisCommand(uint16_t flag_register, const float *rx_data)
{
    usb_ctrl_data.chassis.active = (flag_register & USB_CTRL_FLAG_ACTIVE) != 0u;
    usb_ctrl_data.chassis.stop_request = (flag_register & USB_CTRL_FLAG_STOP) != 0u;
    usb_ctrl_data.chassis.mode = (Host_Chassis_Mode_e)((flag_register & USB_CTRL_MODE_MASK) >> USB_CTRL_MODE_SHIFT);
    usb_ctrl_data.chassis.vx = rx_data[0];
    usb_ctrl_data.chassis.vy = rx_data[1];
    usb_ctrl_data.chassis.wz = rx_data[2];
    usb_ctrl_data.chassis.speed_scale = rx_data[3];
    UpdateUSBControlEnableState();
}

static void DecodeUSBShootCommand(uint16_t flag_register, const float *rx_data)
{
    usb_ctrl_data.shoot.active = (flag_register & USB_CTRL_FLAG_ACTIVE) != 0u;
    usb_ctrl_data.shoot.shoot_enable = (flag_register & USB_CTRL_FLAG_SHOOT_ENABLE) != 0u;
    usb_ctrl_data.shoot.friction_on = (flag_register & USB_CTRL_FLAG_FRICTION_ON) != 0u;
    usb_ctrl_data.shoot.loader_on = (flag_register & USB_CTRL_FLAG_LOADER_ON) != 0u;
    usb_ctrl_data.shoot.lid_open = (flag_register & USB_CTRL_FLAG_LID_OPEN) != 0u;
    usb_ctrl_data.shoot.loader_mode = (Host_Loader_Mode_e)((flag_register & USB_CTRL_MODE_MASK) >> USB_CTRL_MODE_SHIFT);
    usb_ctrl_data.shoot.friction_speed = rx_data[0];
    usb_ctrl_data.shoot.loader_speed = rx_data[1];
    usb_ctrl_data.shoot.shoot_rate = rx_data[2];
    UpdateUSBControlEnableState();
}

static void DecodeIncomingFrame(uint8_t *raw_buf)
{
    uint16_t flag_register = 0;
    uint16_t cmd_id;
    float rx_data[USB_CONTROL_FLOAT_COUNT] = {0};

    if (!ProtocolDataLengthValid(raw_buf))
        return;

    cmd_id = get_protocol_info(raw_buf, &flag_register, (uint8_t *)rx_data);
    if (cmd_id == 0u)
        return;

    DaemonReload(vision_daemon_instance);
    switch (cmd_id)
    {
    case VISION_RECV_CMD_ID:
        DecodeLegacyVisionFlags(flag_register);
        recv_data.pitch = rx_data[0];
        recv_data.yaw = rx_data[1];
        break;
    case USB_CTRL_GIMBAL_CMD_ID:
        DecodeUSBGimbalCommand(flag_register, rx_data);
        break;
    case USB_CTRL_CHASSIS_CMD_ID:
        DecodeUSBChassisCommand(flag_register, rx_data);
        break;
    case USB_CTRL_SHOOT_CMD_ID:
        DecodeUSBShootCommand(flag_register, rx_data);
        break;
    default:
        break;
    }
}

void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed)
{
    send_data.enemy_color = enemy_color;
    send_data.work_mode = work_mode;
    send_data.bullet_speed = bullet_speed;
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_data.yaw = yaw;
    send_data.pitch = pitch;
    send_data.roll = roll;
}

void USBSetGimbalStatus(float yaw, float pitch, float yaw_motor_angle, float yaw_total_angle, uint8_t mode, uint8_t online)
{
    usb_gimbal_status.yaw = yaw;
    usb_gimbal_status.pitch = pitch;
    usb_gimbal_status.yaw_motor_angle = yaw_motor_angle;
    usb_gimbal_status.yaw_total_angle = yaw_total_angle;
    usb_gimbal_status.mode = mode;
    usb_gimbal_status.online = online;
}

void USBSetChassisStatus(float vx, float vy, float wz, float offset_angle, uint8_t mode, uint8_t online)
{
    usb_chassis_status.vx = vx;
    usb_chassis_status.vy = vy;
    usb_chassis_status.wz = wz;
    usb_chassis_status.offset_angle = offset_angle;
    usb_chassis_status.mode = mode;
    usb_chassis_status.online = online;
}

void USBSetShootStatus(float friction_speed, float loader_speed, float shoot_rate, float bullet_speed, uint8_t shoot_mode, uint8_t friction_mode, uint8_t loader_mode, uint8_t online)
{
    usb_shoot_status.friction_speed = friction_speed;
    usb_shoot_status.loader_speed = loader_speed;
    usb_shoot_status.shoot_rate = shoot_rate;
    usb_shoot_status.bullet_speed = bullet_speed;
    usb_shoot_status.shoot_mode = shoot_mode;
    usb_shoot_status.friction_mode = friction_mode;
    usb_shoot_status.loader_mode = loader_mode;
    usb_shoot_status.online = online;
}

static void EncodeVisionStatusFlags(uint16_t *flag_register)
{
    *flag_register = 30 << 8 | 0x0001;
}

static void EncodeGimbalStatusFlags(uint16_t *flag_register)
{
    *flag_register = (uint16_t)(usb_gimbal_status.mode & 0x0F) | (uint16_t)((usb_gimbal_status.online & 0x01) << 8);
}

static void EncodeChassisStatusFlags(uint16_t *flag_register)
{
    *flag_register = (uint16_t)(usb_chassis_status.mode & 0x0F) | (uint16_t)((usb_chassis_status.online & 0x01) << 8);
}

static void EncodeShootStatusFlags(uint16_t *flag_register)
{
    *flag_register = (uint16_t)(usb_shoot_status.shoot_mode & 0x0F) |
                     (uint16_t)((usb_shoot_status.friction_mode & 0x03) << 4) |
                     (uint16_t)((usb_shoot_status.loader_mode & 0x0F) << 8) |
                     (uint16_t)((usb_shoot_status.online & 0x01) << 15);
}

static void FillStatusFrame(uint16_t *cmd_id, uint16_t *flag_register, float *tx_data)
{
    switch (usb_status_round_robin)
    {
    case 0:
        *cmd_id = VISION_SEND_CMD_ID;
        EncodeVisionStatusFlags(flag_register);
        tx_data[0] = send_data.yaw;
        tx_data[1] = send_data.pitch;
        tx_data[2] = send_data.roll;
        break;
    case 1:
        *cmd_id = USB_STATUS_GIMBAL_CMD_ID;
        EncodeGimbalStatusFlags(flag_register);
        tx_data[0] = usb_gimbal_status.yaw;
        tx_data[1] = usb_gimbal_status.pitch;
        tx_data[2] = usb_gimbal_status.yaw_motor_angle;
        tx_data[3] = usb_gimbal_status.yaw_total_angle;
        break;
    case 2:
        *cmd_id = USB_STATUS_CHASSIS_CMD_ID;
        EncodeChassisStatusFlags(flag_register);
        tx_data[0] = usb_chassis_status.vx;
        tx_data[1] = usb_chassis_status.vy;
        tx_data[2] = usb_chassis_status.wz;
        tx_data[3] = usb_chassis_status.offset_angle;
        break;
    default:
        *cmd_id = USB_STATUS_SHOOT_CMD_ID;
        EncodeShootStatusFlags(flag_register);
        tx_data[0] = usb_shoot_status.friction_speed;
        tx_data[1] = usb_shoot_status.loader_speed;
        tx_data[2] = usb_shoot_status.shoot_rate;
        tx_data[3] = usb_shoot_status.bullet_speed;
        break;
    }

    usb_status_round_robin = (uint8_t)((usb_status_round_robin + 1u) % 4u);
}

/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void VisionOfflineCallback(void *id)
{
#ifdef VISION_USE_UART
    USARTServiceInit(vision_usart_instance);
#endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}

#ifdef VISION_USE_UART

#include "bsp_usart.h"

static USARTInstance *vision_usart_instance;

/**
 * @brief 接收解包回调函数,将在bsp_usart.c中被usart rx callback调用
 * @todo  1.提高可读性,将get_protocol_info的第四个参数增加一个float类型buffer
 *        2.添加标志位解码
 */
static void DecodeVision()
{
    DecodeIncomingFrame(vision_usart_instance->recv_buff);
}

Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = DecodeVision;
    conf.recv_buff_size = VISION_RECV_SIZE;
    conf.usart_handle = _handle;
    vision_usart_instance = USARTRegister(&conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = vision_usart_instance,
        .reload_count = 10,
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 发送函数
 *
 * @param send 待发送数据
 *
 */
void VisionSend()
{
    static uint16_t cmd_id;
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    static float tx_data[USB_CONTROL_FLOAT_COUNT];
    memset(tx_data, 0, sizeof(tx_data));
    FillStatusFrame(&cmd_id, &flag_register, tx_data);
    get_protocol_send_data(cmd_id, flag_register, tx_data, cmd_id == VISION_SEND_CMD_ID ? 3 : 4, send_buff, &tx_len);
    USARTSend(vision_usart_instance, send_buff, tx_len, USART_TRANSFER_DMA); // 和视觉通信使用IT,防止和接收使用的DMA冲突
    // 此处为HAL设计的缺陷,DMASTOP会停止发送和接收,导致再也无法进入接收中断.
    // 也可在发送完成中断中重新启动DMA接收,但较为复杂.因此,此处使用IT发送.
    // 若使用了daemon,则也可以使用DMA发送.
}

#endif // VISION_USE_UART

#ifdef VISION_USE_VCP

#include "bsp_usb.h"
static uint8_t *vis_recv_buff;

static void DecodeVision(uint16_t recv_len)
{
    UNUSED(recv_len);
    DecodeIncomingFrame(vis_recv_buff);
}

/* 视觉通信初始化 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle)
{
    UNUSED(_handle); // 仅为了消除警告
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

void VisionSend()
{
    static uint16_t cmd_id;
    static uint16_t flag_register;
    static uint8_t send_buff[VISION_SEND_SIZE];
    static uint16_t tx_len;
    static float tx_data[USB_CONTROL_FLOAT_COUNT];
    memset(tx_data, 0, sizeof(tx_data));
    FillStatusFrame(&cmd_id, &flag_register, tx_data);
    get_protocol_send_data(cmd_id, flag_register, tx_data, cmd_id == VISION_SEND_CMD_ID ? 3 : 4, send_buff, &tx_len);
    USBTransmit(send_buff, tx_len);
}

#endif // VISION_USE_VCP

const USB_Control_Data_s *USBControlGetData(void)
{
    return &usb_ctrl_data;
}

uint8_t USBControlIsOnline(void)
{
    return vision_daemon_instance != NULL && DaemonIsOnline(vision_daemon_instance);
}
