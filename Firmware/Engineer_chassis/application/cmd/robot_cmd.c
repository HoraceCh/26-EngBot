// app
#include "robot_def.h"
#include "robot_cmd.h"
#include "gimbal.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
#include "host_communication.h"
#include "user_lib.h"
#include "custom_controller.h"
#include "pump.h"
#include "can_comm.h"
#include "auto_action.h"
#include "rm_referee.h"
#include "referee_task.h"
#include "kinematics.h"
#include "trajectory_planning.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* 底盘 */
static Publisher_t *chassis_cmd_pub;                // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;              // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;         // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data;    // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static uint8_t chassis_direct;
static PIDInstance chassis_leg_PID; // 底盘wz的PID


/* 遥控器 视觉(没用到) */
static RC_ctrl_t *rc_data;                          // 遥控器数据,初始化时返回
// static custom_controller_t *cus_ctrl_data;          // 自定义控制器数据,初始化时返回
// static Vision_Recv_s *vision_recv_data;             // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;              // 视觉发送数据

/* 上位机 */
// static Host_Recv_s *host_recv_data;                 // 上位机接收数据指针，初始化返回，已解码
// static Host_Send_s host_send_data;                  // 上位机发送数据

/* 机械臂相关 */
static Publisher_t *gimbal_cmd_pub;                 // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;               // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;           // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;      // 从云台获取的反馈信息
static float offset_arm_angle[6];                   // 机械臂电机零位偏差角度
static float IK_target_angle[6];                    // 逆解关节角度
static float IK_target_pos[6] = {0.4f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f};   // 末端目标位姿
static float FK_current_pos[3];                     // 正解末端位姿   
static float elbow = 1.0f;                          // 机械臂肘部方向，1为正，-1为负
static float direct = 1.0f;                         // 矿槽方向，1为左，-1为右
static AutoActionInstance *auto_motion;             // 自动动作实例

/* 发射(无用) */
static Publisher_t *shoot_cmd_pub;                  // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;                // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;             // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data;        // 从发射获取的反馈信息

/* 主从控板通信 */
static CANCommInstance *cmd_can_comm;               // 双板can通信实例
static Master_Cmd_Data_s master_cmd_recv;           // 主控板发送给从控板的数据
static Slave_Upload_Data_s slave_feedback_data;     // 从控板发送给主控板的数据

/* 图传链路和裁判系统数据 */
static ImgTrans_info_t *img_data;                   // 图传链路数据,包含键鼠数据和自定义控制器数据
// static referee_info_t* referee_data;                // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;          // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI


/* 状态&标志位 */
static Robot_Status_e robot_state;                  // 机器人整体状态
static uint8_t gimbal_motor_init_flag[6];           // 机械臂电机初始化标志
static uint8_t chassis_motor_init_flag[5];          // 抬升小臂电机初始化标志
// static uint8_t *pump_state;

CubicTrajectory traj;


/**
 * @brief 用户状态,按位赋值
 *       0-底盘8010中值初始化 1-图传跟随 2-自定义控制器控制 3-arm8010电机初始化
 */
static uint8_t user_state;  // 用户状态,按位赋值

/* 私有函数声明 */
static void MouseKeySet();

// BMI088Instance *bmi088_test; // 云台IMU
// BMI088_Data_t bmi088_data;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);               // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1);         // 视觉通信串口
    // host_recv_data = HostInit();                    // 上位机通信，使用USB通信
    // cus_ctrl_data = CustomControllerInit(&huart6);  // 自定义控制器
    // img_data = ImgTransInit(&huart1);               // 图传链路数据
    auto_motion = AutoActionInit(&traj);                // 自动动作初始化
    // pump_state = get_pump_state_ptr();                  // 获取气泵状态指针
    // referee_data = UITaskInit(&huart6,&ui_data);    // 裁判系统初始化,会同时初始化UI

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    // gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    // gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    // shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2, // 后面改成can2
            .tx_id = 0x312,
            .rx_id = 0x311,
            .IDE = CAN_ID_STD,
        },
        .recv_data_len = sizeof(Master_Cmd_Data_s),       // 接收数据长度
        .send_data_len = sizeof(Slave_Upload_Data_s),         // 发送数据长度
        .daemon_count = 50,
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    // 后腿PID
    // PID_Init_Config_s chassis_leg_PID_config = {
    //     .Kp = 0.1,
    //     .Ki = 0,
    //     .Kd = 0,
    //     .DeadBand = 0.0001,
    //     .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //     .IntegralLimit = 0,
    //     .MaxOut = 0.001,
    // };
    // PIDInit(&chassis_leg_PID, &chassis_leg_PID_config);

    memset(&chassis_cmd_send, 0, sizeof(chassis_cmd_send));
    memset(&chassis_fetch_data, 0, sizeof(chassis_fetch_data));
}



/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    //angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
#if YAW_ECD_GREATER_THAN_4096                               // 如果大于180度
    if (angle > YAW_ALIGN_ANGLE && angle <= 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle > 180.0f + YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE - 360.0f;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
#else // 小于180度
    if (angle > YAW_ALIGN_ANGLE)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else if (angle <= YAW_ALIGN_ANGLE && angle >= YAW_ALIGN_ANGLE - 180.0f)
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE;
    else
        chassis_cmd_send.offset_angle = angle - YAW_ALIGN_ANGLE + 360.0f;
#endif
}


/**
 * @brief  图传跟随控制
 *
 */
static void ImgControlSet()
{
    float img_y = 0.16f;
    float img_z = 0.3f;
    float theta2;
    if (chassis_direct)
        theta2 = (gimbal_fetch_data.feedback_arm_angle[1] - offset_arm_angle[1] + 19.886f) / (-12.66f);
    else 
        theta2 = (gimbal_fetch_data.feedback_arm_angle[1] - offset_arm_angle[1]) / (-12.66f);

    Robotics_FK(FK_current_pos, gimbal_fetch_data.feedback_arm_angle, offset_arm_angle, chassis_direct); // 计算当前末端位姿

    gimbal_cmd_send.img_yaw_set = 0.1f + 0.032f * (atan2f((FK_current_pos[1] - img_y), FK_current_pos[0]) - theta2); // 计算当前yaw角度
    gimbal_cmd_send.img_pitch_set = 0.0675f + 0.032f * atan2f((FK_current_pos[2] - img_z), FK_current_pos[0]); // 计算当前yaw角度
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置,左侧开关状态[下]
 *
 */
static void RemoteControlSet()
{
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_SB))       // 左侧开关状态[下],控制底盘
    {    
        if (switch_is_down(rc_data[TEMP].rc.switch_SC))
        {
            // 底盘参数,目前没有加入小陀螺
            chassis_cmd_send.vx = master_cmd_recv.vx;            // _水平方向
            chassis_cmd_send.vy = master_cmd_recv.vy;            // 1竖直方向
            // chassis_cmd_send.theta_z = master_cmd_recv.theta_z;    // 旋转方向
            chassis_cmd_send.wz = master_cmd_recv.wz;            // 旋转速度

            chassis_cmd_send.target_chassis_angle[4] = master_cmd_recv.target_chassis_angle[4];
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_SC))
        {
            chassis_cmd_send.target_chassis_angle[0] = master_cmd_recv.target_chassis_angle[0]; // 摇臂电机目标角度
            chassis_cmd_send.target_chassis_angle[1] = master_cmd_recv.target_chassis_angle[1]; // 摇臂电机目标角度

            chassis_cmd_send.target_chassis_angle[2] = master_cmd_recv.target_chassis_angle[2]; // 后腿电机目标角度
            chassis_cmd_send.target_chassis_angle[3] = master_cmd_recv.target_chassis_angle[3]; // 后腿电机目标角度

            if (master_cmd_recv.event & START_TRACK)
            {
                chassis_cmd_send.target_chassis_angle[5] = 8000; // 履带电机目标速度
            }
            else
            {
                chassis_cmd_send.target_chassis_angle[5] = 0; // 履带电机目标速度
            }
        }

        // PumpControl(); // 气泵控制
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_SB))   // 左侧开关状态[中],控制云台机械臂
    {
        // direct = switch_is_down(rc_data[TEMP].rc.switch_SD) ? 1.0f : -1.0f; // 矿槽方向,左侧开关状态[下]为左槽,上为右槽
        if (switch_is_down(rc_data[TEMP].rc.switch_SC))
         { // 遥控器直接控制电机角度
            // gimbal_cmd_send.target_arm_angle[0] += 0.1f * (float)rc_data[TEMP].rc.rocker_l1;
            // gimbal_cmd_send.target_arm_angle[0] = float_constrain(gimbal_cmd_send.target_arm_angle[0], offset_arm_angle[0], offset_arm_angle[0] + 18000.0f);

            // gimbal_cmd_send.target_arm_angle[1] += 0.0001f * (float)rc_data[TEMP].rc.rocker_l_;

            // gimbal_cmd_send.target_arm_angle[2] += 0.00001f * (float)rc_data[TEMP].rc.rocker_r_;

            // gimbal_cmd_send.target_arm_angle[3] += 0.00002f * (float)rc_data[TEMP].rc.rocker_r1;

            // ImgControlSet();
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_SC))
        { // 末端仅使用直接控制
            // actuator_pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_r1; // 单位为度
            // Math_Constrain(&actuator_pitch, 0.0f, 100.0f);
            // actuator_roll += 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
            // Math_Constrain(&actuator_roll, -180.0f, 180.0f);

            // gimbal_cmd_send.target_arm_angle[4] += 0.00002f * (float)rc_data[TEMP].rc.rocker_r1;
            // gimbal_cmd_send.target_arm_angle[5] += 0.1f * (float)rc_data[TEMP].rc.rocker_r_;

            // 图传舵机控制
            // gimbal_cmd_send.img_pitch_set += 0.000001f * (float)rc_data[TEMP].rc.rocker_l1;
            // gimbal_cmd_send.img_yaw_set -= 0.000001f * (float)rc_data[TEMP].rc.rocker_l_;

            // IK_target_pos[4] -= 0.0001f * (float)rc_data[TEMP].rc.rocker_r1;
            // IK_target_pos[5] += 0.0001f * (float)rc_data[TEMP].rc.rocker_r_;

            // Math_Constrain(&IK_target_pos[4], -1.57f, 1.57f);
            // Math_Constrain(&IK_target_pos[5], -1.57f, 1.57f);

            // if (!ResolveInverseKinematics(IK_target_pos, IK_target_angle, elbow, direct))
            // {   // 若不可解，则退回遥控器的输入控制量
            //     IK_target_pos[4] += 0.0001f * (float)rc_data[TEMP].rc.rocker_l1;
            //     IK_target_pos[5] -= 0.0001f * (float)rc_data[TEMP].rc.rocker_l_;
            // }

            // // 从关节空间变换到驱动器空间
            // gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + (IK_target_angle[0] * 25226.28f);
            // gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1] - (IK_target_angle[1] * 12.66f);
            // gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2] - IK_target_angle[2];
            // gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3] + IK_target_angle[3];
            // gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] + IK_target_angle[4];
            // gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] - IK_target_angle[5] * 4125.30f;
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_SC))
        { // 使用IK控制
            // IK_target_pos[0] += 0.000004f * (float)rc_data[TEMP].rc.rocker_l1;
            // IK_target_pos[1] -= 0.000004f * (float)rc_data[TEMP].rc.rocker_l_;
            // IK_target_pos[2] += 0.000004f * (float)rc_data[TEMP].rc.rocker_r1;
            // IK_target_pos[3] -= 0.0001f * (float)rc_data[TEMP].rc.rocker_r_;

            // Math_Constrain(&IK_target_pos[0], -0.71f, 0.71f);
            // Math_Constrain(&IK_target_pos[1], -0.71f, 0.71f);
            // Math_Constrain(&IK_target_pos[2], 0.0f, 0.69f);
            // Math_Constrain(&IK_target_pos[3], -2.09f, 2.09f);

            // if (!ResolveInverseKinematics(IK_target_pos, IK_target_angle, elbow, direct))
            // {   // 若不可解，则退回遥控器的输入控制量
            //     IK_target_pos[0] -= 0.000004f * (float)rc_data[TEMP].rc.rocker_l1;
            //     IK_target_pos[1] += 0.000004f * (float)rc_data[TEMP].rc.rocker_l_;
            //     IK_target_pos[2] -= 0.000004f * (float)rc_data[TEMP].rc.rocker_r1;
            //     IK_target_pos[3] += 0.0001f * (float)rc_data[TEMP].rc.rocker_r_;    
            //     return;
            // }

            // // 从关节空间变换到驱动器空间
            // gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + (IK_target_angle[0] * 25226.28f);
            // gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1] - (IK_target_angle[1] * 12.66f);
            // gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2] - IK_target_angle[2];
            // gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3] + IK_target_angle[3];
            // gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] + IK_target_angle[4];
            // gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] - IK_target_angle[5] * 4125.30f;
            // ImgControlSet();
        }
    }
}



/**
 * @brief 输入为自定义控制器时的控制量设置
 *
 */
static void CustomControllerSet()
{
    // if (!CustomControllerIsOnline())
    //     return;

    float controller_data[6];
    memcpy(controller_data, &(img_data->Controller.data), sizeof(controller_data));

    if ((controller_data[0] == 0) && (controller_data[1] == 0) && (controller_data[2] == 0))
    {
        return;
    }

    // 将自定义控制器的输入量映射到机械臂的目标位姿
    IK_target_pos[0] = 3.266f + controller_data[0] * 14.2f; //1.634 6.6      (-0.230, -0.128)  5.5  -0.23,-0.18
    IK_target_pos[1] = controller_data[1] * 6.0f;                   // (-0.0861, 0.0729)
    IK_target_pos[2] = 0.618f + controller_data[2] * 5.154f;         // (-0.13, 0.0545)      -0.12,0.01
    IK_target_pos[3] = controller_data[3];// * 2.0f;
    IK_target_pos[4] = controller_data[4];// * 2.0f;
    IK_target_pos[5] = controller_data[5];// * 4.0f;

    Math_Constrain(&IK_target_pos[0], 0.05f, 0.71f);
    Math_Constrain(&IK_target_pos[1], -0.71f, 0.71f);
    Math_Constrain(&IK_target_pos[2], 0.0f, 0.67f);
    Math_Constrain(&IK_target_pos[3], -2.1f, 2.1f);
    Math_Constrain(&IK_target_pos[4], -1.57f, 1.57f);
    // Math_Constrain(&IK_target_pos[5], -1.571f, 1.571f);

    /* 防止输入越界,出现无解的情况 */
    float l_input = sqrtf(IK_target_pos[0] * IK_target_pos[0] + IK_target_pos[1] * IK_target_pos[1]); // 计算自定义控制器输入坐标点到原点的距离

    if (l_input > 0.71f) // 如果输入坐标点到原点的距离大于臂长(0.59f),即必定超出臂的工作空间导致无解,则将其缩放到0.59f
    {
        IK_target_pos[0] = 0.71f * IK_target_pos[0] / l_input;
        IK_target_pos[1] = 0.71f * IK_target_pos[1] / l_input;
    }
    else if (l_input < 0.15f) // 如果输入坐标点到原点的距离小于最小工作空间(0.26f),则将其缩放到0.26f
    {
        IK_target_pos[0] = 0.15f * IK_target_pos[0] / l_input;
        IK_target_pos[1] = 0.15f * IK_target_pos[1] / l_input;
    }

    if (!ResolveInverseKinematics(IK_target_pos, IK_target_angle, elbow, direct))
    {
        LOGINFO("[CMD] Robotics IK failed!");
        return;
    }
    else
    {
        LOGINFO("[CMD] Robotics IK success!");

        Math_Constrain(&IK_target_angle[0], 0.0f, 0.67f);
        // Math_Constrain(&IK_target_angle[2], -1.92f, 1.92f);
        // Math_Constrain(&IK_target_angle[3], -2.618f, 2.618f);
    }

    // 从关节空间变换到驱动器空间
    gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + (IK_target_angle[0] * 25226.28f);
    gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1] - (IK_target_angle[1] * 12.66f) - 19.886f;
    gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2] - IK_target_angle[2];
    gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3] + IK_target_angle[3];
    gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] + IK_target_angle[4];
    gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] - IK_target_angle[5] * 4125.30f;
    // gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] - IK_target_pos[5] * 4125.30f;
}


/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{     
    if (robot_state != ROBOT_DIE)
    {
        if (!RemoteControlIsOnline() || (!switch_is_down(rc_data[TEMP].rc.switch_SA))) // 还需添加重要应用和模块离线的判断
        {   // 遥控器离线
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_STOP;
            chassis_cmd_send.chassis_mode = CHASSIS_MODE_STOP;
            // shoot_cmd_send.shoot_mode = SHOOT_OFF;
            // shoot_cmd_send.friction_mode = FRICTION_OFF;
            // shoot_cmd_send.load_mode = LOAD_STOP;
            LOGERROR("[CMD] emergency stop!");
        }
        else
        {
            robot_state = ROBOT_READY;
            chassis_cmd_send.chassis_mode = CHASSIS_MODE_ENABLE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_ENABLE;
            // shoot_cmd_send.shoot_mode = SHOOT_ON;
            LOGINFO("[CMD] reinstate, robot ready");
        }
    }

    // /* 机器人死亡处理 */
    // if (gimbal_fetch_data.gimbal_state == GIMBAL_DIE)
    // { // 机械臂电机掉线后，初始化完成标志复位，机器人状态置为死亡
    //     robot_state = ROBOT_DIE;
    //     gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_STOP;
    // }
    // else if (robot_state == ROBOT_DIE)
    // { // 当机器人为死亡状态时，若电机重新上线，则将机器人状态置为初始化，进入电机初始化流程
    //     if (gimbal_fetch_data.gimbal_state == GIMBAL_ALIVE)
    //     {
    //         robot_state = ROBOT_INIT;
    //         // gimbal_cmd_send.DM4310ReCmd = 1; // 重新初始化机械臂电机
    //     }
    // }

    /* 如果有XXX_MODE_STOP，则将电机目标值更新为电机当前反馈值 */
    // if (gimbal_cmd_send.gimbal_mode == GIMBAL_MODE_STOP)
    // {
    //     for (uint8_t i = 0; i < 6; i++)
    //         gimbal_cmd_send.target_arm_angle[i] = gimbal_fetch_data.feedback_arm_angle[i];
    // }
    // if (chassis_cmd_send.chassis_mode == CHASSIS_MODE_STOP)
    // {
    //     for (uint8_t i = 0; i < 2; i++)
    //     {
    //         chassis_cmd_send.target_leg_angle[i] = chassis_fetch_data.feedback_leg_angle[i];
    //         chassis_cmd_send.target_rocker_angle[i] = chassis_fetch_data.feedback_rocker_angle[i];
    //     }
    //     chassis_cmd_send.target_yaw_angle = chassis_fetch_data.feedback_yaw_angle;
    // }
}

/**
 * @brief 自动动作事件转换
 */
static void AutoActionEventTransfer()
{
    if (auto_motion->event == MANUAL_EVENT_NONE)
    {
        slave_feedback_data.event &= ~CHASSIS_FSM_BUSY;
        if (master_cmd_recv.event & CHASSIS_REPOSITION)
        {
            auto_motion->event = REPOSITION;
        }
        else if (master_cmd_recv.event & CHASSIS_CLIMB)
        {
            auto_motion->event = START_CLIMB;
        }
    }
    else
    {
        slave_feedback_data.event |= CHASSIS_FSM_BUSY;
        if (master_cmd_recv.event & CHASSIS_REFORMAT)
        {
            auto_motion->event = REFORMAT;
            memcpy(chassis_cmd_send.target_chassis_angle, chassis_fetch_data.feedback_chassis_angle, sizeof(chassis_fetch_data.feedback_chassis_angle));
        }
    }
}

/** 
 * @brief  自动动作处理函数
 * 
 */
static void AutoActionHandler()
{
    AutoActionEventTransfer();  // 根据输入进行事件转换
    AutoActionRun();            // 运行自动动作状态机

    if (traj.is_active)
    {
        memcpy(chassis_cmd_send.target_chassis_angle, traj.pos, sizeof(traj.pos));
    }
    else
    {
        memcpy(traj.pos, chassis_fetch_data.feedback_chassis_angle, sizeof(traj.pos));
    }
   
    if (auto_motion->start_track)
    {
        chassis_cmd_send.target_chassis_angle[5] = 6000; // 履带电机目标速度
        chassis_cmd_send.vy = 9000;
    }
    
}

/**
 * @brief 电机控制数据初始化
 * 
 */
static void MotorCmdDataInit()
{
    static uint16_t init_time = 0;
    if (!(slave_feedback_data.event & CHASSIS_INIT_FINISH))//robot_state == ROBOT_INIT) // (gimbal_cmd_send.motor_init_finish == 0)
    {
        uint8_t isALLone;
        for (uint8_t i = 0; i < 5 ; i++)
        {
            if (chassis_fetch_data.feedback_chassis_angle[i])
            {
                chassis_motor_init_flag[i] = 1;    // 收到反馈值，初始化标志置1
                chassis_cmd_send.target_chassis_angle[i] = chassis_fetch_data.feedback_chassis_angle[i]; // 同时将目标角度赋当前值为初始值
            }
            isALLone += chassis_motor_init_flag[i];  // 累加各个标志位的值
        }
        // 若累加值为6，则六个电机均已收到反馈值，初始化完成
        if((isALLone == 5) || (init_time++ > 10000))
        {
            slave_feedback_data.event |= CHASSIS_INIT_FINISH;
            chassis_cmd_send.motor_init_finish = 1;
            // gimbal_cmd_send.motor_init_finish |= 0x01;
            // 舵机赋初值
            // gimbal_cmd_send.img_pitch_set = 0.0675f;
            // gimbal_cmd_send.img_yaw_set = 0.1f;
            // robot_state = ROBOT_READY;
        }
    }
}

static void MotorAngleLimit()
{
    // 底盘后腿和摇臂电机角度限幅,根据实际安装位置设置合理的限幅值
    chassis_cmd_send.target_chassis_angle[0] = float_constrain(chassis_cmd_send.target_chassis_angle[0], 0.05f, 5.8f);
    chassis_cmd_send.target_chassis_angle[1] = float_constrain(chassis_cmd_send.target_chassis_angle[1], -5.8f, -0.05f);
    // chassis_cmd_send.target_chassis_angle[2] = float_constrain(chassis_cmd_send.target_chassis_angle[2], 0.05f, 1.3f);
    // chassis_cmd_send.target_chassis_angle[3] = float_constrain(chassis_cmd_send.target_chassis_angle[3], -1.3f, -0.05f);
    // chassis_cmd_send.target_chassis_angle[4] = float_constrain(chassis_cmd_send.target_chassis_angle[3], -1.3f, -0.05f);

    // 图传舵机限幅,根据实际安装位置和需要设置合理的限幅值
    // gimbal_cmd_send.img_yaw_set = float_constrain(gimbal_cmd_send.img_yaw_set, 0.05f, 0.13f);
    // gimbal_cmd_send.img_pitch_set = float_constrain(gimbal_cmd_send.img_pitch_set, 0.04f, 0.075f);

}


/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;

    // 从其他应用获取回传数据
    // SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    master_cmd_recv = *(Master_Cmd_Data_s *)CANCommGet(cmd_can_comm);                // 获取从控板反馈的数据
    chassis_cmd_send.chassis_mode = master_cmd_recv.chassis_mode;
    memcpy(&chassis_cmd_send, &master_cmd_recv, 9 * sizeof(float));

    // if (master_cmd_recv.event & START_TRACK)
    // {
    //    chassis_cmd_send.target_chassis_angle[5] = 4000; // 履带电机目标速度
    // }
    // else
    // {
    //     chassis_cmd_send.target_chassis_angle[5] = 0; // 履带电机目标速度
    // }

    if (master_cmd_recv.event & START_TRACK_INVERSE)
    {
        chassis_cmd_send.target_chassis_angle[5] = -4000; // 履带电机目标速度
    }
    else if (master_cmd_recv.event & START_TRACK)
    {
       chassis_cmd_send.target_chassis_angle[5] = 4000; // 履带电机目标速度
    }
    else
    {
        chassis_cmd_send.target_chassis_angle[5] = 0; // 履带电机目标速度
    }


    MotorCmdDataInit(); // 电机控制数据初始化,等待电机反馈值

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    //CalcOffsetAngle();

    CubicTrajectory_Update(&traj);

    AutoActionHandler();    // 处理自动动作状态机

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    // if (switch_is_up(rc_data[TEMP].rc.switch_SB))     // 遥控器左侧开关状态为[上],键鼠和自定义控制器控制
    // {
    //     // CustomControllerSet();
    //     // MouseKeySet();
    //     AutoActionHandler();    // 处理自动动作状态机
    // }
    // else                                                // 遥控器左侧开关状态为[下]或[中],均为遥控器控制
    // {
    //     RemoteControlSet();
    // }

    // if (robot_state != ROBOT_INIT)
        // EmergencyHandler();     // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,chassis_fetch_data.bullet_speed)

    MotorAngleLimit(); // 限幅目标角度

    // 推送消息,双板通信,视觉通信等
    // PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);           // 发布云台机械臂控制消息
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);         // 发布底盘控制消息

    // ui_data.pump_state = *pump_state; // 更新泵状态到UI数据
    // ui_data.user_state = user_state; // 更新用户状态到UI数据

    // 更新从控板的控制数据
    memcpy(slave_feedback_data.feedback_chassis_angle, chassis_fetch_data.feedback_chassis_angle, 5 * sizeof(float));
    CANCommSend(cmd_can_comm, (uint8_t *)&slave_feedback_data);  
}



/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    if ((auto_motion->event == 12) || (user_state & 0b00100))
        chassis_direct = 1;
    else if (auto_motion->event == 19)
        chassis_direct = 0;
    else if (auto_motion->event == 11)
        chassis_direct = 0;

    if (chassis_direct)
    {
        if (img_data->KeyMouseData[TEMP][SINGLE].a == 1)
        {
            if (chassis_cmd_send.vy < 30000)
                chassis_cmd_send.vy += 300;
        }
        else
        {
            if (chassis_cmd_send.vy > 0)
                chassis_cmd_send.vy -= 150;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].d == 1)
        {
            if (chassis_cmd_send.vy > -30000)
                chassis_cmd_send.vy -= 300;
        }
        else
        {
            if (chassis_cmd_send.vy < 0)
                chassis_cmd_send.vy += 150;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].s == 1)
        {
            if (chassis_cmd_send.vx < 20000)
                chassis_cmd_send.vx += 200;
        }
        else
        {
            if (chassis_cmd_send.vx > 0)
                chassis_cmd_send.vx -= 100;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].w == 1)
        {
            if (chassis_cmd_send.vx > -20000)
                chassis_cmd_send.vx -= 200;
        }
        else
        {
            if (chassis_cmd_send.vx < 0)
                chassis_cmd_send.vx += 100;
        }    

        if (img_data->KeyMouseRaw.shift == 1)
        {
            chassis_cmd_send.vx = img_data->KeyMouseData[TEMP][SHIFT].s * 6000 - img_data->KeyMouseData[TEMP][SHIFT].w * 6000;  
            chassis_cmd_send.vy = img_data->KeyMouseData[TEMP][SHIFT].a * 6000 - img_data->KeyMouseData[TEMP][SHIFT].d * 6000;
        }
        chassis_cmd_send.wz = img_data->KeyMouseData[TEMP][SINGLE].mouse_x * 50 + img_data->KeyMouseData[TEMP][SHIFT].mouse_x * 10
                            + img_data->KeyMouseData[TEMP][SHIFT].e * 1000 - img_data->KeyMouseData[TEMP][SHIFT].q * 1000;
    
    }
    else
    {
        if (img_data->KeyMouseData[TEMP][SINGLE].w == 1)
        {
            if (chassis_cmd_send.vy < 37000)
                chassis_cmd_send.vy += 300;
        }
        else
        {
            if (chassis_cmd_send.vy > 0)
                chassis_cmd_send.vy -= 150;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].s == 1)
        {
            if (chassis_cmd_send.vy > -30000)
                chassis_cmd_send.vy -= 300;
        }
        else
        {
            if (chassis_cmd_send.vy < 0)
                chassis_cmd_send.vy += 150;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].a == 1)
        {
            if (chassis_cmd_send.vx < 20000)
                chassis_cmd_send.vx += 200;
        }
        else
        {
            if (chassis_cmd_send.vx > 0)
                chassis_cmd_send.vx -= 100;
        }
    
        if (img_data->KeyMouseData[TEMP][SINGLE].d == 1)
        {
            if (chassis_cmd_send.vx > -20000)
                chassis_cmd_send.vx -= 200;
        }
        else
        {
            if (chassis_cmd_send.vx < 0)
                chassis_cmd_send.vx += 100;
        }
    
        if (img_data->KeyMouseRaw.shift == 1)
        {
            chassis_cmd_send.vx = img_data->KeyMouseData[TEMP][SHIFT].a * 6000 - img_data->KeyMouseData[TEMP][SHIFT].d * 6000;  
            chassis_cmd_send.vy = img_data->KeyMouseData[TEMP][SHIFT].w * 6000 - img_data->KeyMouseData[TEMP][SHIFT].s * 6000;
        }
        chassis_cmd_send.wz = img_data->KeyMouseData[TEMP][SINGLE].mouse_x * 50 + img_data->KeyMouseData[TEMP][SHIFT].mouse_x * 10
                            + img_data->KeyMouseData[TEMP][SHIFT].e * 1000 - img_data->KeyMouseData[TEMP][SHIFT].q * 1000;
    }


    // chassis_cmd_send.vx = img_data->KeyMouseData[TEMP][SINGLE].a * 30000 - img_data->KeyMouseData[TEMP][SINGLE].d * 30000
    //                     + img_data->KeyMouseData[TEMP][SHIFT].a * 6000 - img_data->KeyMouseData[TEMP][SHIFT].d * 6000;  
    // chassis_cmd_send.vy = img_data->KeyMouseData[TEMP][SINGLE].w * 30000 - img_data->KeyMouseData[TEMP][SINGLE].s * 30000
    //                     + img_data->KeyMouseData[TEMP][SHIFT].w * 6000 - img_data->KeyMouseData[TEMP][SHIFT].s * 6000;
    // chassis_cmd_send.wz = img_data->KeyMouseData[TEMP][SINGLE].mouse_x * 50 + img_data->KeyMouseData[TEMP][SHIFT].mouse_x * 10
    //                     + img_data->KeyMouseData[TEMP][SHIFT].e * 1000 - img_data->KeyMouseData[TEMP][SHIFT].q * 1000;

    if (auto_motion->event == 0)
    {
        // actuator_pitch += 0.001f * img_data->KeyMouseData[TEMP][SINGLE].mouse_z; // 单位为度
        // Math_Constrain(&actuator_pitch, 0.0f, 100.0f);
        gimbal_cmd_send.target_arm_angle[0] += img_data->KeyMouseData[TEMP][CTRL].mouse_z;
        // gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] - (actuator_pitch) * 36.0f;    
        // gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] + (actuator_pitch) * 36.0f;
        gimbal_cmd_send.target_arm_angle[4] -= 0.5f * img_data->KeyMouseData[TEMP][SINGLE].mouse_z;
        // gimbal_cmd_send.target_arm_angle[5] += 0.5f * img_data->KeyMouseData[TEMP][SINGLE].mouse_z;
        // gimbal_cmd_send.target_arm_angle[1] -= img_data->KeyMouseData[TEMP][CTRL].left_button_down * 0.05f - img_data->KeyMouseData[TEMP][CTRL].right_button_down * 0.05f;
        // gimbal_cmd_send.target_arm_angle[2] += img_data->KeyMouseData[TEMP][SINGLE].left_button_down * 0.05f - img_data->KeyMouseData[TEMP][SINGLE].right_button_down * 0.05f;
        // gimbal_cmd_send.target_arm_angle[3] += img_data->KeyMouseData[TEMP][SHIFT].z * 0.05f - img_data->KeyMouseData[TEMP][SHIFT].c * 0.05f;
        
        if (img_data->KeyMouseData[TEMP][SHIFT].z)
        {
            offset_arm_angle[0] = gimbal_fetch_data.feedback_arm_angle[0];
        }
        if (img_data->KeyMouseData[TEMP][SINGLE].b)
        {
            // offset_arm_angle[4] = gimbal_fetch_data.feedback_arm_angle[4];
            // offset_arm_angle[5] = gimbal_fetch_data.feedback_arm_angle[5];
            gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + 2000.0f;
            gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1] + 8.686f;
            gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2] - 1.739f;
            gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3] - 1.63f;
            gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] + 1.571f;    
        }

        if (img_data->KeyMouseData[TEMP][CTRL].r)
            gimbal_cmd_send.DM4310ReCmd = 1;
        else
            gimbal_cmd_send.DM4310ReCmd = 0;

        if (img_data->KeyMouseData[TEMP][CTRL].v)
        {
            gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1];
            gimbal_cmd_send.img_yaw_set = 0.1f;

            if (user_state & 0b00010)
            {
                img_data->key_count[SINGLE][14]++;
            }
        }

        // switch (img_data->key_count[SINGLE][13] % 2) // c 取大资源岛 臂伸直
        // {
        //     case 0:
        //         break;
        //     default:
        //         gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + 3000.0f;
        //         gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1];
        //         gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2];
        //         gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3];
        //         actuator_pitch = 95.0f; // 单位为度
        //         gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4] - (actuator_pitch - actuator_roll * 2.0f) * 36.0f;        
        //         gimbal_cmd_send.target_arm_angle[5] = offset_arm_angle[5] + (actuator_pitch + actuator_roll * 2.0f) * 36.0f;
    
        //         img_data->key_count[SINGLE][13]++;
        //         break;
        // }
        
        if (img_data->KeyMouseData[TEMP][SINGLE].c)
        {
            gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + 4200.0f;
            gimbal_cmd_send.target_arm_angle[1] = offset_arm_angle[1] - 19.886f;
            gimbal_cmd_send.target_arm_angle[2] = offset_arm_angle[2];
            gimbal_cmd_send.target_arm_angle[3] = offset_arm_angle[3] - 1.571f;
            gimbal_cmd_send.target_arm_angle[4] = offset_arm_angle[4];     
            
            chassis_direct = 1;

        }

        if (img_data->KeyMouseData[TEMP][SHIFT].c)
        {
            gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + 2000.0f;
        }

        if (img_data->KeyMouseData[TEMP][SHIFT].v)
        {
            offset_arm_angle[5] = gimbal_fetch_data.feedback_arm_angle[5];
        }

        // if (img_data->KeyMouseData[TEMP][SHIFT].b)
        // {
        //     // offset_arm_angle[2] = gimbal_fetch_data.feedback_arm_angle[2];
        //     // offset_arm_angle[1] = gimbal_fetch_data.feedback_arm_angle[1];
        //     offset_arm_angle[4] += 0.2855f;
        //     gimbal_cmd_send.target_arm_angle[4] += 0.2855f;
        // }
        // if (img_data->KeyMouseData[TEMP][CTRL].b)
        // {
        //     // offset_arm_angle[0] = gimbal_fetch_data.feedback_arm_angle[0];
        //     offset_arm_angle[4] -= 0.2855f;
        //     gimbal_cmd_send.target_arm_angle[4] -= 0.2855f;
        // }

        switch (img_data->key_count[SHIFT][15] % 2) //
        {
            case 0:
                break;
            default:
                offset_arm_angle[4] += 0.2856f;
                gimbal_cmd_send.target_arm_angle[4] += 0.2856f;
                img_data->key_count[SHIFT][15]++;
                break;
        }

        switch (img_data->key_count[CTRL][15] % 2) //
        {
            case 0:
                break;
            default:
                offset_arm_angle[4] -= 0.2856f;
                gimbal_cmd_send.target_arm_angle[4] -= 0.2856f;
                img_data->key_count[CTRL][15]++;
                break;
        }


    }

    // if (!(user_state & 0b00001))
    // {
    //     if (!slave_fetch_data.offset_8010_angle[1])
    //         gimbal_cmd_send.target_arm_angle[2] += img_data->KeyMouseData[TEMP][SINGLE].left_button_down * 0.1f - img_data->KeyMouseData[TEMP][SINGLE].right_button_down * 0.1f;
    //     else if ((slave_fetch_data.offset_8010_angle[1] > 0.01f) || (slave_fetch_data.offset_8010_angle[1] < -0.01f))
    //         gimbal_cmd_send.target_arm_angle[1] -= img_data->KeyMouseData[TEMP][CTRL].left_button_down * 0.1f - img_data->KeyMouseData[TEMP][CTRL].right_button_down * 0.1f;
    //     else
    //         user_state |= 0b00001;
    // }

    // if (!((slave_fetch_data.offset_8010_angle[0] > 0.01f) || (slave_fetch_data.offset_8010_angle[0] < -0.01f)))
        // gimbal_cmd_send.target_arm_angle[1] -= img_data->KeyMouseData[TEMP][CTRL].left_button_down * 0.1f - img_data->KeyMouseData[TEMP][CTRL].right_button_down * 0.1f;
    // else
        // gimbal_cmd_send.target_arm_angle[2] += img_data->KeyMouseData[TEMP][SINGLE].left_button_down * 0.1f - img_data->KeyMouseData[TEMP][SINGLE].right_button_down * 0.1f;

    switch (img_data->key_count[SHIFT][8] % 2) // shift+R键重新初始化UI
    {
        case 0:
            break;
        default:
            UIReset(&ui_data);
            img_data->key_count[SHIFT][8]++;
            break;
    }
    
    switch (img_data->key_count[SHIFT][9] % 2) // shift+F键正转roll90度
    {
        case 0:
            break;
        default:
            // offset_arm_angle[4] += 1.571f * 4125.30f;
            // offset_arm_angle[5] += 1.571f * 4125.30f;
            offset_arm_angle[5] += 1.571f * 4125.30f;
            img_data->key_count[SHIFT][9]++;
            break;
    }

    switch (img_data->key_count[CTRL][9] % 2) // ctrl+F键反转roll90度
    {
        case 0:
            break;
        default:
            // offset_arm_angle[4] -= 1.571f * 4125.30f;
            // offset_arm_angle[5] -= 1.571f * 4125.30f;
            offset_arm_angle[5] -= 1.571f * 4125.30f;
            img_data->key_count[CTRL][9]++;
            break;
    }

    switch (img_data->key_count[SINGLE][9] % 2) // F键开关自定义控制器，进入自动动作直接退出
    {
        case 0:
            user_state &= ~0b00100;
            break;
        default:
            if (!(user_state & 0b00010))
                img_data->key_count[SINGLE][14]++;

            user_state |= 0b00100;

            if (img_data->KeyMouseData[TEMP][SINGLE].left_button_down)
                direct = 1.0f;
            else if (img_data->KeyMouseData[TEMP][SINGLE].right_button_down)
                direct = -1.0f;

            CustomControllerSet();
            if (auto_motion->event != 0)
                img_data->key_count[SINGLE][9]++;

            break;
    }

    switch (img_data->key_count[SINGLE][10] % 2) // G键开关大泵
    {
        case 0:
            // if (*pump_state & 0b0010000)
            // {
            //     pump_stop(1);
            //     *pump_state &= ~0b0010000;
            // }
            break;
        default:
            // if (!(*pump_state & 0b0001))
            //     pump_run(1);
            // else
            //     pump_stop(1);

            img_data->key_count[SINGLE][10]++;
            break;
    }

    switch (img_data->key_count[SHIFT][10] % 2) // shift+G键开关左泵
    {
        case 0:
            // if (*pump_state & 0b0100000)
            // {
            //     pump_stop(3);
            //     *pump_state &= ~0b0100000;
            // }
            break;
        default:
            // if (!(*pump_state & 0b0100))
            //     pump_run(3);
            // else
            //     pump_stop(3);

            img_data->key_count[SHIFT][10]++;
            break;
    }

    switch (img_data->key_count[CTRL][10] % 2) // ctrl+G键开关右泵
    {
        case 0:
            // if (*pump_state & 0b1000000)
            // {
            //     pump_stop(2);
            //     *pump_state &= ~0b1000000;
            // }
            break;
        default:
            // if (!(*pump_state & 0b0010))
            //     pump_run(2);
            // else
            //     pump_stop(2);

            img_data->key_count[CTRL][10]++;
            break;
    }

    switch (img_data->key_count[SINGLE][14] % 2) // V键开关视角跟随末端
    {
        case 0:
            if (user_state & 0b00010)
            {
                // gimbal_cmd_send.img_yaw_set = 0.1f + (4.660f / (-12.66f));
                user_state &= ~0b00010;
            }
            gimbal_cmd_send.img_yaw_set += img_data->KeyMouseData[TEMP][CTRL].d * 0.0003f - img_data->KeyMouseData[TEMP][CTRL].a * 0.0003f;
            gimbal_cmd_send.img_pitch_set += (img_data->KeyMouseData[TEMP][SINGLE].mouse_y + img_data->KeyMouseData[TEMP][CTRL].mouse_y + img_data->KeyMouseData[TEMP][SHIFT].mouse_y)* 0.00001f; // img_data->KeyMouseData[TEMP][CTRL].w * 0.0003f - img_data->KeyMouseData[TEMP][CTRL].s * 0.0003f;
            break;
        default:
            user_state |= 0b00010;
            ImgControlSet();
            break;
    }

    switch (img_data->key_count[SINGLE][11] % 2) // Z键更换左右手型
    {
        case 0:
            if (elbow == -1.0f)
            {
                direct = 1.0f;          // 机械臂肘部方向，1为正，-1为负
                elbow = 1.0f;
            }
            break;
        default:
            if (elbow == 1.0f)
            {
                direct = -1.0f;
                elbow = -1.0f;
            }
            break;
    }

}
