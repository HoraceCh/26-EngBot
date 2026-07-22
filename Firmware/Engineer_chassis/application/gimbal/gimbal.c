#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ysmotor.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "servo_motor.h"

static DMMotorInstance *arm_4310_4, *arm_4310_5;                     // 4310电机
static DJIMotorInstance *arm_3508_1, *arm_2006_6;

static Publisher_t *gimbal_pub;                         // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                        // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data;       // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;               // 来自cmd的控制信息
static ServoInstance *img_pitch, *img_yaw;              // 图传舵机

// static BMI088Instance *bmi088;                          // 云台IMU

void GimbalInit()
{   
    /*---初始化M3508电机结构体---*/
    Motor_Init_Config_s m3508_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30, // 8
                .Ki = 8,
                .Kd = 0.01,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 10000,
                .MaxOut = 11000,
            },
            .speed_PID = {
                .Kp = 3,  // 50
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 12000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };
    arm_3508_1 = DJIMotorInit(&m3508_motor_config); // can2

    /*---初始化M8010电机结构体---*/
    // RS485_Motor_Init_Config_s m8010_2_config = {
    //     .rs485_init_config.usart_handle = &huart1,
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 8.5, // 13
    //             .Ki = 0.43,//8
    //             .Kd = 0.8,
    //             .DeadBand = 0.01,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 180,
    //             .MaxOut = 200,
    //         },
    //         .speed_PID = {
    //             .Kp = 0.028,//0.028
    //             .Ki = 0.1,//0.005
    //             .Kd = 0,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 7,
    //             .MaxOut = 20,
    //         },
    //         // .other_angle_feedback_ptr = &encoder_1->angle,
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP, //后续如果加入其它环，需在此也修改
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //         .angle_max = PI,
    //         .angle_min = -PI,
    //     },
    //     .motor_type = M8010,
    //     .id = 0,
    // };
    // arm_8010_2 = YSMotorInit(&m8010_2_config);
    // RS485_Motor_Init_Config_s m8010_3_config = {
    //     .rs485_init_config.usart_handle = &huart1,
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 6, // 14
    //             .Ki = 0.15,
    //             .Kd = 0.6,
    //             .DeadBand = 0.1,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 180,
    //             .MaxOut = 200,
    //         },
    //         .speed_PID = {
    //             .Kp = 0.022,
    //             .Ki = 0.01,
    //             .Kd = 0,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 7,
    //             .MaxOut = 20,
    //         },
    //         // .other_angle_feedback_ptr = &encoder_2->angle,
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP, //后续如果加入其它环，需在此也修改
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //         .angle_max = PI,
    //         .angle_min = -PI,
    //     },
    //     .motor_type = M8010,
    //     .id = 1,
    // };
    // arm_8010_3 = YSMotorInit(&m8010_3_config);

    /*---初始化DM4310电机结构体---*/
    Motor_Init_Config_s m4310_4_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4, // can2
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };
    arm_4310_4 = DMMotorInit(&m4310_4_motor_config);
    Motor_Init_Config_s m4310_5_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 5, // can2
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };
    arm_4310_5 = DMMotorInit(&m4310_5_motor_config);

    /*---初始化M2006电机结构体---*/
    // Motor_Init_Config_s m2006_5_motor_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 2,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 40, // 8
    //             .Ki = 2,
    //             .Kd = 0.01,
    //             .DeadBand = 0.1,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 2000,
    //             .MaxOut = 10000,
    //         },
    //         .speed_PID = {
    //             .Kp = 3,  // 50
    //             .Ki = 0.1, // 200
    //             .Kd = 0,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
    //             .IntegralLimit = 3000,
    //             .MaxOut = 30000,
    //         },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED,
    //         .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
    //     },
    //     .motor_type = M2006,
    // };
    // arm_2006_5 = DJIMotorInit(&m2006_5_motor_config); // can2
    Motor_Init_Config_s m2006_6_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30, // 8
                .Ki = 2,
                .Kd = 0.01,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000,
                .MaxOut = 10000,
            },
            .speed_PID = {
                .Kp = 2.8,  // 50
                .Ki = 0.1, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 3000,
                .MaxOut = 30000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006,
    };
    arm_2006_6 = DJIMotorInit(&m2006_6_motor_config); // can2

    Servo_Init_Config_s img_pitch_init_config = {
        .pwm_init_config = {
            .htim = &htim8,
            .channel = TIM_CHANNEL_2,
            .period = 0.02f,
            .dutyratio = 0.0675f,
        },
        .servo_type = PWM_Servo,
        .servo_id = 0,
    };
    img_pitch = ServoInit(&img_pitch_init_config);
    Servo_Init_Config_s img_yaw_init_config = {
        .pwm_init_config = {
            .htim = &htim8,
            .channel = TIM_CHANNEL_3,
            .period = 0.02f,
            .dutyratio = 0.1f,
        },
        .servo_type = PWM_Servo,
        .servo_id = 1,
    };
    img_yaw = ServoInit(&img_yaw_init_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/**
 * @brief 机械臂电机存活检测
 * @note 该函数用于检测机械臂电机是否存活，仅检测四个电机
 */
static void GimbalAliveCheck()
{
    if (DaemonIsOnline(arm_3508_1->daemon) == 0 &&
        DaemonIsOnline(arm_4310_4->daemon) == 0 &&
        DaemonIsOnline(arm_4310_5->daemon) == 0 &&
        DaemonIsOnline(arm_2006_6->daemon) == 0)
    {
        gimbal_feedback_data.gimbal_state = GIMBAL_DIE;
        DJIMotorStop(arm_3508_1);
        DMMotorStop(arm_4310_4);
        DMMotorStop(arm_4310_5);
        DJIMotorStop(arm_2006_6);
    }
    else
    {
        gimbal_feedback_data.gimbal_state = GIMBAL_ALIVE;
    }
}

int a = 0;

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    if(gimbal_cmd_recv.motor_init_finish) // 等待六个电机收到反馈值，且目标值被初始化为当前值
    {
        switch (gimbal_cmd_recv.gimbal_mode)
        {
            case GIMBAL_MODE_STOP:
                DJIMotorStop(arm_3508_1);
                DMMotorStop(arm_4310_4);
                DMMotorStop(arm_4310_5);
                DJIMotorStop(arm_2006_6);

                DJIMotorSetRef(arm_3508_1, gimbal_cmd_recv.target_arm_angle[0]);
                DMMotorSetAngle(arm_4310_4, gimbal_cmd_recv.target_arm_angle[3]);
                DMMotorSetAngle(arm_4310_5, gimbal_cmd_recv.target_arm_angle[4]);
                DJIMotorSetRef(arm_2006_6, gimbal_cmd_recv.target_arm_angle[5]);
                break;
            case GIMBAL_MODE_ENABLE: 
                DJIMotorEnable(arm_3508_1);
                DMMotorEnable(arm_4310_4);
                DMMotorEnable(arm_4310_5);
                DJIMotorEnable(arm_2006_6);

                DJIMotorSetRef(arm_3508_1, gimbal_cmd_recv.target_arm_angle[0]);

                DMMotorSetCtrlType(arm_4310_4, DM_CTRL_TYPE_PosVel);
                DMMotorSetSpeed(arm_4310_4, 4.3f);
                DMMotorSetAngle(arm_4310_4, gimbal_cmd_recv.target_arm_angle[3]);

                DMMotorSetCtrlType(arm_4310_5, DM_CTRL_TYPE_PosVel);
                DMMotorSetSpeed(arm_4310_5, 4.3f);
                DMMotorSetAngle(arm_4310_5, gimbal_cmd_recv.target_arm_angle[4]);

                DJIMotorSetRef(arm_2006_6, gimbal_cmd_recv.target_arm_angle[5]);

                ServoSetAngle(img_pitch, gimbal_cmd_recv.img_pitch_set);
                ServoSetAngle(img_yaw, gimbal_cmd_recv.img_yaw_set);
                break;
            default:
                break;
        }
    }
    // else
    // {
    //     DMMotorSetMode(DM_CMD_MOTOR_MODE, arm_4310_4);  // 使能电机
    //     DMMotorSetMode(DM_CMD_MOTOR_MODE, arm_4310_5);  // 使能电机
    // }

    // 机械臂电机存活检测
    GimbalAliveCheck();

    if ((gimbal_feedback_data.gimbal_state == GIMBAL_DIE) || (gimbal_cmd_recv.DM4310ReCmd))
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, arm_4310_4);  // 使能电机
        DMMotorSetMode(DM_CMD_MOTOR_MODE, arm_4310_5);  // 使能电机
    }   

    // 
    //     DMMotorSetMode(DM_CMD_ZERO_POSITION, arm_4310_4);
    if (a)
    {
        DMMotorSetMode(DM_CMD_ZERO_POSITION, arm_4310_5);
        a = 0;
    }

    gimbal_feedback_data.feedback_arm_angle[0] = arm_3508_1->measure.total_angle;
    gimbal_feedback_data.feedback_arm_angle[3] = arm_4310_4->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[4] = arm_4310_5->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[5] = arm_2006_6->measure.total_angle;
    gimbal_feedback_data.ecd = arm_3508_1->measure.ecd;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

