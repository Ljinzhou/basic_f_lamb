#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 18u // 当前为固定值,36字节
#define VISION_SEND_SIZE 36u
#define USB_CONTROL_FLOAT_COUNT 4u

#define VISION_RECV_CMD_ID 0x0001u
#define VISION_SEND_CMD_ID 0x0002u
#define USB_CTRL_GIMBAL_CMD_ID 0x0101u
#define USB_CTRL_CHASSIS_CMD_ID 0x0102u
#define USB_CTRL_SHOOT_CMD_ID 0x0103u
#define USB_STATUS_GIMBAL_CMD_ID 0x0201u
#define USB_STATUS_CHASSIS_CMD_ID 0x0202u
#define USB_STATUS_SHOOT_CMD_ID 0x0203u

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	float pitch;
	float yaw;
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;

typedef enum
{
	HOST_GIMBAL_ZERO_FORCE = 0,
	HOST_GIMBAL_FREE_MODE = 1,
	HOST_GIMBAL_GYRO_MODE = 2,
} Host_Gimbal_Mode_e;

typedef enum
{
	HOST_CHASSIS_ZERO_FORCE = 0,
	HOST_CHASSIS_NO_FOLLOW = 1,
	HOST_CHASSIS_FOLLOW_GIMBAL = 2,
	HOST_CHASSIS_RAW = 3,
	HOST_CHASSIS_ROTATE = 4,
} Host_Chassis_Mode_e;

typedef enum
{
	HOST_LOADER_STOP = 0,
	HOST_LOADER_SINGLE = 1,
	HOST_LOADER_TRIPLE = 2,
	HOST_LOADER_BURST = 3,
	HOST_LOADER_REVERSE = 4,
	HOST_LOADER_SPEED = 5,
} Host_Loader_Mode_e;

typedef struct
{
	uint8_t active;
	uint8_t auto_aim_enabled;
	uint8_t relative_target;
	Host_Gimbal_Mode_e mode;
	float yaw;
	float pitch;
} USB_Gimbal_Ctrl_s;

typedef struct
{
	uint8_t active;
	uint8_t stop_request;
	Host_Chassis_Mode_e mode;
	float vx;
	float vy;
	float wz;
	float speed_scale;
} USB_Chassis_Ctrl_s;

typedef struct
{
	uint8_t active;
	uint8_t shoot_enable;
	uint8_t friction_on;
	uint8_t loader_on;
	uint8_t lid_open;
	Host_Loader_Mode_e loader_mode;
	float friction_speed;
	float loader_speed;
	float shoot_rate;
} USB_Shoot_Ctrl_s;

typedef struct
{
	uint8_t host_enabled;
	USB_Gimbal_Ctrl_s gimbal;
	USB_Chassis_Ctrl_s chassis;
	USB_Shoot_Ctrl_s shoot;
} USB_Control_Data_s;

typedef struct
{
	uint8_t mode;
	uint8_t online;
	float yaw;
	float pitch;
	float yaw_motor_angle;
	float yaw_total_angle;
} USB_Gimbal_Status_s;

typedef struct
{
	uint8_t mode;
	uint8_t online;
	float vx;
	float vy;
	float wz;
	float offset_angle;
} USB_Chassis_Status_s;

typedef struct
{
	uint8_t shoot_mode;
	uint8_t friction_mode;
	uint8_t loader_mode;
	uint8_t online;
	float friction_speed;
	float loader_speed;
	float shoot_rate;
	float bullet_speed;
} USB_Shoot_Status_s;
#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/**
 * @brief 设置发送数据的姿态部分
 *
 * @param yaw
 * @param pitch
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

const USB_Control_Data_s *USBControlGetData(void);

uint8_t USBControlIsOnline(void);

void USBSetGimbalStatus(float yaw, float pitch, float yaw_motor_angle, float yaw_total_angle, uint8_t mode, uint8_t online);

void USBSetChassisStatus(float vx, float vy, float wz, float offset_angle, uint8_t mode, uint8_t online);

void USBSetShootStatus(float friction_speed, float loader_speed, float shoot_rate, float bullet_speed, uint8_t shoot_mode, uint8_t friction_mode, uint8_t loader_mode, uint8_t online);

#endif // !MASTER_PROCESS_H
