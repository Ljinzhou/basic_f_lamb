#ifndef HERO_CONFIG_H
#define HERO_CONFIG_H

#define ROBOT_HERO

#include "controller.h"
#include "motor_def.h"
#include "bsp_can.h"

/* ============================== 英雄机器人电机配置 ============================== */
/* 
 * CAN1总线:
 *   - 底盘: 4个3508电机, ID: 1, 2, 3, 4
 *   - 云台: 2个6020电机, ID: 1(yaw), 2(pitch)
 * CAN2总线:
 *   - 摩擦轮: 3个3508电机, ID: 1, 2, 3
 *   - 拨弹轮: 1个3508电机, ID: 7
 */

/* ============================== 底盘电机配置 ============================== */
#define HERO_CHASSIS_MOTOR_TYPE       M3508
#define HERO_CHASSIS_CAN_HANDLE       hcan1

#define HERO_CHASSIS_LF_ID            1
#define HERO_CHASSIS_RF_ID            2
#define HERO_CHASSIS_LB_ID            3
#define HERO_CHASSIS_RB_ID            4

static inline Motor_Init_Config_s HeroChassisMotorConfig(uint8_t motor_id, Motor_Reverse_Flag_e reverse_flag)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &HERO_CHASSIS_CAN_HANDLE,
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
        .motor_type = HERO_CHASSIS_MOTOR_TYPE,
    };
    return config;
}

/* ============================== 云台电机配置 ============================== */
#define HERO_GIMBAL_YAW_MOTOR_TYPE    GM6020
#define HERO_GIMBAL_PITCH_MOTOR_TYPE  GM6020
#define HERO_GIMBAL_CAN_HANDLE        hcan1

#define HERO_GIMBAL_YAW_ID            1
#define HERO_GIMBAL_PITCH_ID          2

static inline Motor_Init_Config_s HeroYawMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &HERO_GIMBAL_CAN_HANDLE,
            .tx_id = HERO_GIMBAL_YAW_ID,
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
        .motor_type = HERO_GIMBAL_YAW_MOTOR_TYPE,
    };
    return config;
}

static inline Motor_Init_Config_s HeroPitchMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &HERO_GIMBAL_CAN_HANDLE,
            .tx_id = HERO_GIMBAL_PITCH_ID,
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
        .motor_type = HERO_GIMBAL_PITCH_MOTOR_TYPE,
    };
    return config;
}

/* ============================== 发射机构电机配置 ============================== */
#define HERO_FRICTION_MOTOR_TYPE      M3508
#define HERO_LOADER_MOTOR_TYPE        M3508
#define HERO_SHOOT_CAN_HANDLE         hcan2

#define HERO_FRICTION_L_ID            1
#define HERO_FRICTION_R_ID            2
#define HERO_FRICTION_3_ID            3
#define HERO_LOADER_ID                7

static inline Motor_Init_Config_s HeroFrictionMotorConfig(uint8_t motor_id, Motor_Reverse_Flag_e reverse_flag)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &HERO_SHOOT_CAN_HANDLE,
            .tx_id = motor_id,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20.0f,
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
                .MaxOut = 15000.0f,
            },
            .current_PID = {
                .Kp = 0.7f,
                .Ki = 0.1f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000.0f,
                .MaxOut = 15000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = reverse_flag,
        },
        .motor_type = HERO_FRICTION_MOTOR_TYPE,
    };
    return config;
}

static inline Motor_Init_Config_s HeroLoaderMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &HERO_SHOOT_CAN_HANDLE,
            .tx_id = HERO_LOADER_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 200.0f,
            },
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 1.0f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 5000.0f,
            },
            .current_PID = {
                .Kp = 0.7f,
                .Ki = 0.1f,
                .Kd = 0.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 5000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = HERO_LOADER_MOTOR_TYPE,
    };
    return config;
}

/* ============================== 英雄机器人机械参数 ============================== */
#define HERO_WHEEL_BASE               400.0f
#define HERO_TRACK_WIDTH              350.0f
#define HERO_RADIUS_WHEEL             60.0f
#define HERO_REDUCTION_RATIO_WHEEL    19.0f

#define HERO_REDUCTION_RATIO_LOADER   19.0f
#define HERO_ONE_BULLET_DELTA_ANGLE   36

#endif
