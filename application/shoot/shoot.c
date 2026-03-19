#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *friction_3; // 摩擦轮电机
static DJIMotorInstance *loader; // 拨盘电机
static DMMotorInstance *loader_dm; // 拨盘电机(达妙2325哨兵专用)
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

void ShootInit()
{
#if defined(ROBOT_HERO)
    // 英雄机器人: 摩擦轮3个M3508( CAN2, ID 1/2/3 ) + 拨盘1个M3508( CAN2, ID 7 )
    Motor_Init_Config_s friction_l_config = HeroFrictionMotorConfig(HERO_FRICTION_L_ID, MOTOR_DIRECTION_NORMAL);
    Motor_Init_Config_s friction_r_config = HeroFrictionMotorConfig(HERO_FRICTION_R_ID, MOTOR_DIRECTION_REVERSE);
    Motor_Init_Config_s friction_3_config = HeroFrictionMotorConfig(HERO_FRICTION_3_ID, MOTOR_DIRECTION_NORMAL);
    Motor_Init_Config_s loader_config = HeroLoaderMotorConfig();
    friction_l = DJIMotorInit(&friction_l_config);
    friction_r = DJIMotorInit(&friction_r_config);
    friction_3 = DJIMotorInit(&friction_3_config);
    loader = DJIMotorInit(&loader_config);
    (void)loader_dm; // 哨兵专用变量

#elif defined(ROBOT_SENTRY)
    // 哨兵机器人: 摩擦轮2个M3508( CAN1, ID 7/8 ) + 拨盘达妙2325( CAN1, ID 0x10/0x20 )
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &SENTRY_SHOOT_CAN_HANDLE,
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
    friction_config.can_init_config.tx_id = SENTRY_FRICTION_L_ID;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = SENTRY_FRICTION_R_ID;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    friction_3 = NULL;

    // 拨盘电机(达妙2325, CAN1, ID:0x10, Master ID:0x20)
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &SENTRY_SHOOT_CAN_HANDLE,
            .tx_id = SENTRY_LOADER_DM2325_ID,
            .rx_id = SENTRY_LOADER_DM2325_MASTER_ID,
        },
        .controller_setting_init_config = {
            .outer_loop_type = OPEN_LOOP,
            .close_loop_type = OPEN_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
    };
    loader = NULL;
    loader_dm = DMMotorInit(&loader_config);
    (void)loader; // 英雄专用变量
#endif

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
        if (friction_3 != NULL)
            DJIMotorStop(friction_3);
#if defined(ROBOT_HERO)
        DJIMotorStop(loader);
#elif defined(ROBOT_SENTRY)
        DMMotorStop(loader_dm);
#endif
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        if (friction_3 != NULL)
            DJIMotorEnable(friction_3);
#if defined(ROBOT_HERO)
        DJIMotorEnable(loader);
#elif defined(ROBOT_SENTRY)
        DMMotorEnable(loader_dm);
#endif
    }

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, 0);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, 0);
#endif
        break;
    // 单发模式
    case LOAD_1_BULLET:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, 1000.0f);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, 2.0f);
#endif
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 150;
        break;
    // 三连发
    case LOAD_3_BULLET:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, 1000.0f);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, 2.0f);
#endif
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 450;
        break;
    // 连发模式,持续施加力矩
    case LOAD_BURSTFIRE:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, 1000.0f);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, 2.0f);
#endif
        break;
    // 拨盘反转
    case LOAD_REVERSE:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, -1000.0f);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, -2.0f);
#endif
        break;
    case LOAD_EXTERNAL_SPEED:
#if defined(ROBOT_HERO)
        DJIMotorSetRef(loader, shoot_cmd_recv.loader_speed);
#elif defined(ROBOT_SENTRY)
        DMMotorSetRef(loader_dm, shoot_cmd_recv.loader_speed);
#endif
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        if (shoot_cmd_recv.use_custom_friction_speed)
        {
            DJIMotorSetRef(friction_l, shoot_cmd_recv.friction_speed);
            DJIMotorSetRef(friction_r, shoot_cmd_recv.friction_speed);
            if (friction_3 != NULL)
                DJIMotorSetRef(friction_3, shoot_cmd_recv.friction_speed);
            goto friction_done;
        }
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            if (friction_3 != NULL)
                DJIMotorSetRef(friction_3, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            if (friction_3 != NULL)
                DJIMotorSetRef(friction_3, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            if (friction_3 != NULL)
                DJIMotorSetRef(friction_3, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 30000);
            DJIMotorSetRef(friction_r, 30000);
            if (friction_3 != NULL)
                DJIMotorSetRef(friction_3, 30000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
        if (friction_3 != NULL)
            DJIMotorSetRef(friction_3, 0);
    }

friction_done:

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
