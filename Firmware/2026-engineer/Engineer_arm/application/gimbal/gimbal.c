#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "rsmotor.h"
#include "LKmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"
#include "servo_motor.h"
#include "trajectory_planning.h"
#include "host_communication.h"

static RSMotorInstance *arm_04_0, *arm_06_1, *arm_04_2, *arm_06_3;   // rs电机:0-大yaw, 1-小yaw, 2-J2, 3-J3
static DMMotorInstance *arm_4310_4, *arm_4310_5;                     // 4310电机:4-J4, 5-J5
static LKMotorInstance *arm_4005_6, *arm_4005_7;                     // 4005电机:6-J6, 7-末端执行器

static Publisher_t *gimbal_pub;                         // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                        // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data;       // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;               // 来自cmd的控制信息
static ServoInstance *img_pitch, *img_yaw;              // 图传舵机

// static BMI088Instance *bmi088;                          // 云台IMU

void GimbalInit()
{   

    /*---初始化RS电机结构体---*/
    Motor_Init_Config_s motor_1_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
            .IDE = CAN_ID_EXT,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 28, // 4.5
                .Ki = 8, 
                .Kd = 0.5, 
                .IntegralLimit = 10,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20,
                .DeadBand = 0.00001,
            },
            .speed_PID = {
                .Kp = 0.95, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10,
                .DeadBand = 0.00001,
            },
            .Kp = 0,
            .Kd = 0,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = RS,
    };
    arm_06_1 = RSMotorInit(&motor_1_config);
    Motor_Init_Config_s motor_2_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 3,
            .IDE = CAN_ID_EXT,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 200, // 4.5
                .Ki = 300,  // 0
                .Kd = -0.8,  // 0
                .IntegralLimit = 5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 30,
                .DeadBand = 0.00001,
            },
            .speed_PID = {
                .Kp = 0.32, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10,
                .DeadBand = 0.00001,
            },
            .Kp = 0,
            .Kd = 0,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = RS,
    };
    arm_04_2 = RSMotorInit(&motor_2_config);
    Motor_Init_Config_s motor_3_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 4,
            .IDE = CAN_ID_EXT,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 50, // 4.5
                .Ki = 3.5,  // 0
                .Kd = 0.8,  // 0
                .IntegralLimit = 5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 20,
                .DeadBand = 0.00001,
            },
            .speed_PID = {
                .Kp = 0.8, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 5,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10,
                .DeadBand = 0.00001,
            },
            .Kp = 0,
            .Kd = 0,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = RS,
    };
    arm_06_3 = RSMotorInit(&motor_3_config);

    /*---初始化DM4310电机结构体---*/
    Motor_Init_Config_s m4310_4_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 5, // can2
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30,
                .Ki = 0,
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 8,
                .DeadBand = 0.00001,
            },
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10,
                .DeadBand = 0.00001,
            },
            .Kp = 0,
            .Kd = 0,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = DM4310,
    };
    arm_4310_4 = DMMotorInit(&m4310_4_motor_config);
    Motor_Init_Config_s m4310_5_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 6, // can2
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 30,
                .Ki = 0,
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 5,
                .DeadBand = 0.00001,
            },
            .speed_PID = {
                .Kp = 1, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 5,
                .DeadBand = 0.00001,
            },
            .Kp = 0,
            .Kd = 0,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = DM4310,
    };
    arm_4310_5 = DMMotorInit(&m4310_5_motor_config);

    /*---初始化LK4005电机结构体---*/
    Motor_Init_Config_s m4005_6_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 7,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 400, // 8
                .Ki = 0,//2,
                .Kd = 2,
                .DeadBand = 0.0001,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 5,  // 50
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 500,
                .MaxOut = 1000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK4005,
    };
    arm_4005_6 = LKMotorInit(&m4005_6_motor_config);
    Motor_Init_Config_s m4005_7_motor_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 8,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 120, // 8
                .Ki = 0,//2,
                .Kd = 2,
                .DeadBand = 0.0001,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 1.5,  // 50
                .Ki = 0, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle,
                .IntegralLimit = 500,
                .MaxOut = 500,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,//ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP,//ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK4005,
    };
    arm_4005_7 = LKMotorInit(&m4005_7_motor_config);

    Servo_Init_Config_s img_pitch_init_config = {
        .pwm_init_config = {
            .htim = &htim8,
            .channel = TIM_CHANNEL_2,
            .period = 0.02f,
            .dutyratio = 0.075f,
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
            .dutyratio = 0.095f,
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
    static uint16_t delay_time = 0; // 延时计数，避免上电瞬间检测误判
    if ((DaemonIsOnline(arm_06_1->daemon) == 0) &&
        (DaemonIsOnline(arm_04_2->daemon) == 0) &&
        (DaemonIsOnline(arm_06_3->daemon) == 0) &&
        (DaemonIsOnline(arm_4310_4->daemon) == 0))
    {
        gimbal_feedback_data.gimbal_state = GIMBAL_DIE;
        RSMotorStop(arm_06_1);
        RSMotorStop(arm_04_2);
        RSMotorStop(arm_06_3);
        DMMotorStop(arm_4310_4);
    }
    else if ((DaemonIsOnline(arm_06_1->daemon) == 1) &&
        (DaemonIsOnline(arm_04_2->daemon) == 1) &&
        (DaemonIsOnline(arm_06_3->daemon) == 1) &&
        (DaemonIsOnline(arm_4310_4->daemon) == 1))
    {
        if (delay_time < 300) // 上电后前200次检测不做处理
        {
            delay_time++;
            return;
        }
        delay_time = 0;
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

    if(gimbal_cmd_recv.motor_init_finish) // 等待电机收到反馈值，且目标值被初始化为当前值
    {
        switch (gimbal_cmd_recv.gimbal_mode)
        {
            case GIMBAL_MODE_STOP:
                RSMotorStop(arm_06_1);
                RSMotorStop(arm_04_2);
                RSMotorStop(arm_06_3);
                DMMotorStop(arm_4310_4);
                DMMotorStop(arm_4310_5);
                LKMotorStop(arm_4005_6);
                LKMotorStop(arm_4005_7);

                RSMotorSetRef(arm_06_1, gimbal_cmd_recv.target_arm_angle[0]);
                RSMotorSetRef(arm_04_2, gimbal_cmd_recv.target_arm_angle[1]);
                RSMotorSetRef(arm_06_3, gimbal_cmd_recv.target_arm_angle[2]);
                DMMotorSetRef(arm_4310_4, gimbal_cmd_recv.target_arm_angle[3]);
                DMMotorSetRef(arm_4310_5, gimbal_cmd_recv.target_arm_angle[4]);
                LKMotorSetRef(arm_4005_6, gimbal_cmd_recv.target_arm_angle[5]);
                LKMotorSetRef(arm_4005_7, gimbal_cmd_recv.target_arm_angle[6]);

                break;
            case GIMBAL_MODE_ENABLE: 
                RSMotorEnable(arm_06_1);
                RSMotorEnable(arm_04_2);
                RSMotorEnable(arm_06_3);
                DMMotorEnable(arm_4310_4);
                DMMotorEnable(arm_4310_5);
                LKMotorEnable(arm_4005_6);
                LKMotorEnable(arm_4005_7);

                RSMotorSetRef(arm_06_1, gimbal_cmd_recv.target_arm_angle[0]);
                RSMotorSetRef(arm_04_2, gimbal_cmd_recv.target_arm_angle[1]);
                RSMotorSetRef(arm_06_3, gimbal_cmd_recv.target_arm_angle[2]);

                DMMotorSetRef(arm_4310_4, gimbal_cmd_recv.target_arm_angle[3]);
                DMMotorSetRef(arm_4310_5, gimbal_cmd_recv.target_arm_angle[4]);

                LKMotorSetRef(arm_4005_6, gimbal_cmd_recv.target_arm_angle[5]);
                LKMotorSetRef(arm_4005_7, gimbal_cmd_recv.target_arm_angle[6]);

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

    gimbal_feedback_data.feedback_arm_angle[0] = arm_06_1->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[1] = arm_04_2->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[2] = arm_06_3->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[3] = arm_4310_4->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[4] = arm_4310_5->measure.angle;
    gimbal_feedback_data.feedback_arm_angle[5] = arm_4005_6->measure.total_angle;
    gimbal_feedback_data.feedback_arm_angle[6] = arm_4005_7->measure.total_angle;
    gimbal_feedback_data.motor_blocked = arm_4005_7->speed_PID.ERRORHandler.ERRORType;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}

