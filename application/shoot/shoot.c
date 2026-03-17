#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r; // 摩擦轮电机
static DMMotorInstance *loader; // 拨盘电机(达妙2325)
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20,
                .Ki = 1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 7,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 8; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机(达妙2325, CAN1, ID:0x10, Master ID:0x20)
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = SENTRY_LOADER_DM2325_ID,
            .rx_id = SENTRY_LOADER_DM2325_MASTER_ID,
        },
        .controller_setting_init_config = {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    loader = DMMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // 对shoot mode等于SHOOT_OFF的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DMMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DMMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    //     return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DMMotorSetRef(loader, 0); // 力矩为0,停止
        break;
    // 单发模式
    case LOAD_1_BULLET:
        DMMotorSetRef(loader, 2.0f); // 施加固定力矩推动一段时间
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 150;
        break;
    // 三连发
    case LOAD_3_BULLET:
        DMMotorSetRef(loader, 2.0f);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 450;
        break;
    // 连发模式,持续施加力矩
    case LOAD_BURSTFIRE:
        DMMotorSetRef(loader, 2.0f); // 力矩值需根据实际情况调整
        break;
    // 拨盘反转
    case LOAD_REVERSE:
        DMMotorSetRef(loader, -2.0f);
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 30000);
            DJIMotorSetRef(friction_r, 30000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}