#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;
static float gimbal_yaw_angle_feedback;
static float gimbal_yaw_speed_feedback;
static float gimbal_pitch_angle_feedback;
static float gimbal_pitch_speed_feedback;
static float pitch_current_feedforward;
static uint8_t pitch_limit_protect_active;
static uint8_t pitch_upper_limit_protect_active;
static gimbal_mode_e last_gimbal_mode = GIMBAL_ZERO_FORCE;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static void ClearPIDState(PIDInstance *pid)
{
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

static void ResetPitchMotorControlState(void)
{
    ClearPIDState(&pitch_motor->motor_controller.angle_PID);
    ClearPIDState(&pitch_motor->motor_controller.speed_PID);
    ClearPIDState(&pitch_motor->motor_controller.current_PID);
}

static void UpdateGimbalFeedback(void)
{
    gimbal_yaw_angle_feedback = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->YawTotalAngle;
    gimbal_yaw_speed_feedback = GYRO2GIMBAL_DIR_YAW * gimba_IMU_data->Gyro[2] * RAD_2_DEGREE;
    gimbal_pitch_angle_feedback = GYRO2GIMBAL_DIR_PITCH * gimba_IMU_data->Pitch;
    gimbal_pitch_speed_feedback = GYRO2GIMBAL_DIR_PITCH * gimba_IMU_data->Gyro[0] * RAD_2_DEGREE;
}

static float ClampPitchRef(float pitch_ref)
{
    LIMIT_MIN_MAX(pitch_ref, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
    return pitch_ref;
}

static float ApplyPitchLimitProtection(float pitch_ref)
{
    uint8_t lower_protect_active = 0;
    uint8_t upper_protect_active = 0;

    pitch_ref = ClampPitchRef(pitch_ref);

    if (gimbal_pitch_angle_feedback >= (PITCH_MAX_ANGLE - PITCH_UPPER_LIMIT_SOFT_ZONE) &&
        pitch_ref >= (PITCH_MAX_ANGLE - PITCH_UPPER_LIMIT_SOFT_ZONE))
    {
        upper_protect_active = 1;
        if (gimbal_pitch_angle_feedback >= PITCH_MAX_ANGLE)
        {
            pitch_ref = PITCH_MAX_ANGLE - PITCH_UPPER_LIMIT_RELEASE_ANGLE;
        }
        else if (pitch_ref > gimbal_pitch_angle_feedback)
        {
            pitch_ref = gimbal_pitch_angle_feedback;
        }
    }

    if (gimbal_pitch_angle_feedback <= (PITCH_MIN_ANGLE + PITCH_LIMIT_SOFT_ZONE) &&
        pitch_ref <= (PITCH_MIN_ANGLE + PITCH_LIMIT_SOFT_ZONE))
    {
        lower_protect_active = 1;
        if (gimbal_pitch_angle_feedback <= PITCH_MIN_ANGLE)
        {
            pitch_ref = PITCH_MIN_ANGLE + PITCH_LIMIT_RELEASE_ANGLE;
        }
        else if (pitch_ref < gimbal_pitch_angle_feedback)
        {
            pitch_ref = gimbal_pitch_angle_feedback;
        }
    }

    if ((lower_protect_active && !pitch_limit_protect_active) ||
        (upper_protect_active && !pitch_upper_limit_protect_active))
        ResetPitchMotorControlState();

    pitch_limit_protect_active = lower_protect_active;
    pitch_upper_limit_protect_active = upper_protect_active;
    return pitch_ref;
}

static void UpdatePitchFeedforward(void)
{
    float pitch_rad = gimbal_pitch_angle_feedback * DEGREE_2_RAD;
    pitch_current_feedforward = PITCH_GRAVITY_COMP_BASE_CURRENT +
                                PITCH_GRAVITY_COMP_COS_GAIN * cosf(pitch_rad);

    if (fabsf(gimbal_cmd_recv.pitch - gimbal_pitch_angle_feedback) > 1.5f)
    {
        pitch_current_feedforward +=
            (gimbal_cmd_recv.pitch > gimbal_pitch_angle_feedback) ? PITCH_STALL_ASSIST_CURRENT : -PITCH_STALL_ASSIST_CURRENT;
    }

    if (pitch_limit_protect_active || pitch_upper_limit_protect_active)
        pitch_current_feedforward *= 0.5f;
}

// static BMI088Instance *bmi088; // 云台IMU
void GimbalInit()
{   
    gimba_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    UpdateGimbalFeedback();
#if defined(ROBOT_SENTRY)
    yaw_motor = NULL;
    pitch_motor = NULL;
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    return;
#endif

    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,

                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_yaw_angle_feedback,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_yaw_speed_feedback,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 14,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = PITCH_MAX_SPEED_REF,
            },
            .speed_PID = {
                .Kp = 45,
                .Ki = 180,
                .Kd = 0,    // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000,
                .MaxOut = 16000,
            },
            .other_angle_feedback_ptr = &gimbal_pitch_angle_feedback,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_pitch_speed_feedback,
            .current_feedforward_ptr = &pitch_current_feedforward,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
#if defined(ROBOT_SENTRY)
    UpdateGimbalFeedback();
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = 0.0f;
    last_gimbal_mode = GIMBAL_ZERO_FORCE;
    pitch_limit_protect_active = 0;
    pitch_upper_limit_protect_active = 0;
    pitch_current_feedforward = 0.0f;
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
    return;
#endif

    float pitch_ref;

    UpdateGimbalFeedback();

    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    pitch_ref = ApplyPitchLimitProtection(gimbal_cmd_recv.pitch);
    UpdatePitchFeedforward();

    if (last_gimbal_mode != gimbal_cmd_recv.gimbal_mode)
    {
        ResetPitchMotorControlState();
        last_gimbal_mode = gimbal_cmd_recv.gimbal_mode;
    }

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        pitch_limit_protect_active = 0;
        pitch_upper_limit_protect_active = 0;
        pitch_current_feedforward = 0.0f;
        ResetPitchMotorControlState();
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, pitch_ref);
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, pitch_ref);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
