#ifndef SENTRY_CONFIG_H
#define SENTRY_CONFIG_H

#define ROBOT_SENTRY

#include "controller.h"
#include "motor_def.h"
#include "bsp_can.h"

/* ============================== 哨兵机器人电机配置 ============================== */
/*
 * CAN1总线:
 *   - 底盘: 4个3508电机, ID: 1, 2, 3, 4
 *   - 云台Yaw: 1个6020电机 + 1个达妙4310电机 (ID: 0x11, Master ID: 0x21)
 *   - 云台Pitch: 1个6020电机
 * CAN2总线:
 *   - 拨弹轮: 1个达妙2325电机 (ID: 0x10, Master ID: 0x20)
 *
 * TODO: 达妙电机和双Yaw云台结构尚未实现，需要后续完善
 */

/* ============================== 底盘电机配置 ============================== */
#define SENTRY_CHASSIS_MOTOR_TYPE       M3508
#define SENTRY_CHASSIS_CAN_HANDLE       hcan1

#define SENTRY_CHASSIS_LF_ID            1
#define SENTRY_CHASSIS_RF_ID            2
#define SENTRY_CHASSIS_LB_ID            3
#define SENTRY_CHASSIS_RB_ID            4

static inline Motor_Init_Config_s SentryChassisMotorConfig(uint8_t motor_id, Motor_Reverse_Flag_e reverse_flag)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &SENTRY_CHASSIS_CAN_HANDLE,
            .tx_id = motor_id,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 4.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000.0f,
                .Output_LPF_RC = 0.3f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = reverse_flag,
        },
        .motor_type = SENTRY_CHASSIS_MOTOR_TYPE,
    };
    return config;
}

/* ============================== 云台电机配置 ============================== */
/*
 * 哨兵云台为双Yaw结构:
 *   - Yaw主电机: GM6020
 *   - Yaw从电机: 达妙4310 (ID: 0x11, Master ID: 0x21)
 *   - Pitch电机: GM6020
 */

#define SENTRY_GIMBAL_CAN_HANDLE        hcan1

#define SENTRY_GIMBAL_YAW_6020_ID       1
#define SENTRY_GIMBAL_PITCH_ID          2

#define SENTRY_GIMBAL_YAW_DM4310_ID     0x11
#define SENTRY_GIMBAL_YAW_DM4310_MASTER_ID  0x21

static inline Motor_Init_Config_s SentryYaw6020MotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &SENTRY_GIMBAL_CAN_HANDLE,
            .tx_id = SENTRY_GIMBAL_YAW_6020_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 200.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000.0f,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    return config;
}

static inline Motor_Init_Config_s SentryPitchMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &SENTRY_GIMBAL_CAN_HANDLE,
            .tx_id = SENTRY_GIMBAL_PITCH_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 350.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500.0f,
                .MaxOut = 20000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    return config;
}

/* ============================== 达妙电机配置 (TODO: 待实现) ============================== */
/*
 * TODO: 达妙4310电机 (云台Yaw从电机)
 * - CAN ID: 0x11
 * - Master ID: 0x21
 * - 需要实现双Yaw云台的同步控制逻辑
 * - 需要在dmmotor.c中添加对4310型号的支持
 * - 需要实现主从电机的力矩/位置同步策略
 */
typedef struct {
    uint16_t can_id;
    uint16_t master_id;
    float kp;
    float kd;
    float pos_ref;
    float vel_ref;
    float torq_ref;
} DM4310_Config_s;

static inline DM4310_Config_s SentryYawDM4310Config(void)
{
    DM4310_Config_s config = {
        .can_id = SENTRY_GIMBAL_YAW_DM4310_ID,
        .master_id = SENTRY_GIMBAL_YAW_DM4310_MASTER_ID,
        .kp = 0.0f,
        .kd = 0.0f,
        .pos_ref = 0.0f,
        .vel_ref = 0.0f,
        .torq_ref = 0.0f,
    };
    return config;
}

/*
 * TODO: 达妙2325电机 (拨弹轮)
 * - CAN ID: 0x10
 * - Master ID: 0x20
 * - 需要实现拨弹轮的速度/位置控制
 * - 需要在dmmotor.c中添加对2325型号的支持
 */
typedef struct {
    uint16_t can_id;
    uint16_t master_id;
    float kp;
    float kd;
    float pos_ref;
    float vel_ref;
    float torq_ref;
} DM2325_Config_s;

#define SENTRY_LOADER_DM2325_ID         0x10
#define SENTRY_LOADER_DM2325_MASTER_ID  0x20

static inline DM2325_Config_s SentryLoaderDM2325Config(void)
{
    DM2325_Config_s config = {
        .can_id = SENTRY_LOADER_DM2325_ID,
        .master_id = SENTRY_LOADER_DM2325_MASTER_ID,
        .kp = 0.0f,
        .kd = 0.0f,
        .pos_ref = 0.0f,
        .vel_ref = 0.0f,
        .torq_ref = 0.0f,
    };
    return config;
}

/* ============================== 发射机构电机配置 ============================== */
/*
 * TODO: 哨兵发射机构使用达妙2325电机控制拨弹轮
 * - 摩擦轮配置待确认
 * - 拨弹轮使用达妙2325 (ID: 0x10, Master ID: 0x20)
 */
#define SENTRY_SHOOT_CAN_HANDLE         hcan2

/* ============================== 哨兵机器人机械参数 ============================== */
#define SENTRY_WHEEL_BASE               350.0f
#define SENTRY_TRACK_WIDTH              300.0f
#define SENTRY_RADIUS_WHEEL             60.0f
#define SENTRY_REDUCTION_RATIO_WHEEL    19.0f

/* ============================== 待实现功能 TODO ============================== */
/*
 * [ ] 1. 在 dmmotor.c 中添加达妙4310和2325型号的支持
 * [ ] 2. 实现双Yaw云台的同步控制逻辑 (主从电机协同)
 * [ ] 3. 实现哨兵云台初始化函数 SentryGimbalInit()
 * [ ] 4. 实现哨兵云台控制任务 SentryGimbalTask()
 * [ ] 5. 实现哨兵发射机构初始化函数 SentryShootInit()
 * [ ] 6. 实现哨兵发射机构控制任务 SentryShootTask()
 * [ ] 7. 添加达妙电机的离线检测和错误处理
 * [ ] 8. 实现双Yaw电机的力矩均衡分配策略
 */

#endif
