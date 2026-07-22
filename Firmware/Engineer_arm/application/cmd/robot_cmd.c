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

/* 遥控器 视觉(没用到) */
static RC_ctrl_t *rc_data;                          // 遥控器数据,初始化时返回
// static custom_controller_t *cus_ctrl_data;          // 自定义控制器数据,初始化时返回
// static Vision_Recv_s *vision_recv_data;             // 视觉接收数据指针,初始化时返回
// static Vision_Send_s vision_send_data;              // 视觉发送数据

/* 上位机 */
static Host_Recv_s *host_recv_data;                 // 上位机接收数据指针，初始化返回，已解码
static Host_Send_s host_send_data;                  // 上位机发送数据

/* 机械臂相关 */
static Publisher_t *gimbal_cmd_pub;                 // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;               // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;           // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;      // 从云台获取的反馈信息
static float offset_arm_angle[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};                   // 机械臂电机零位偏差角度
static float IK_target_angle[6];                    // 逆解关节角度
static float IK_target_pos[6] = {0.4f, 0.0f, 0.2f, 0.0f, 0.0f, 0.0f};   // 末端目标位姿
static float FK_current_pos[3];                     // 正解末端位姿   
static float actuator_pitch;
static float actuator_roll;
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
static Master_Cmd_Data_s master_cmd_data;           // 主控板发送给从控板的数据
static Slave_Upload_Data_s slave_fetch_data;        // 从控板发送给主控板的数据

/* 图传链路和裁判系统数据 */
static ImgTrans_info_t *img_data;                   // 图传链路数据,包含键鼠数据和自定义控制器数据
static referee_info_t* referee_data;                // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;          // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI


/* 状态&标志位 */
static Robot_Status_e robot_state;                  // 机器人整体状态
static uint8_t gimbal_motor_init_flag[7];           // 机械臂电机初始化标志
static uint8_t chassis_motor_init_flag[5];          // 抬升小臂电机初始化标志
// static motor_delta_angle_t motor_delta_angle;
static uint8_t *pump_state;
static uint8_t grabber_action = 2;   // 0-不运动 1-闭合 2-张开

static CubicTrajectory traj;   // 自定义控制器规划和自动动作规划共用这个结构体
/**
 * @brief 用户状态,按位赋值
 * 0-夹爪闭合 1-图传跟随 2-自定义控制器控制 3-
 */
static uint8_t user_state;  // 用户状态,按位赋值

/* 私有函数声明 */
static void MouseKeySet();

// BMI088Instance *bmi088_test; // 云台IMU
// BMI088_Data_t bmi088_data;

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);           // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    // vision_recv_data = VisionInit(&huart1);         // 视觉通信串口
    host_recv_data = HostInit(&host_send_data);                    // 上位机通信，使用USB通信
    // cus_ctrl_data = CustomControllerInit(&huart6);  // 自定义控制器
    img_data = ImgTransInit(&huart1);               // 图传链路数据
    auto_motion = AutoActionInit(&traj);                 // 自动动作初始化
    pump_state = get_pump_state_ptr();              // 获取气泵状态指针
    referee_data = UITaskInit(&huart6, &ui_data);    // 裁判系统初始化,会同时初始化UI

    // chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    // chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));

    // shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1, // 后面改成can2
            .tx_id = 0x311,
            .rx_id = 0x312,
            .IDE = CAN_ID_STD,
        },
        .recv_data_len = sizeof(Slave_Upload_Data_s),       // 接收数据长度
        .send_data_len = sizeof(Master_Cmd_Data_s),         // 发送数据长度
        .daemon_count = 50,
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    // 清空,防止有残余
    memset(&gimbal_cmd_send, 0, sizeof(gimbal_cmd_send));
    memset(&gimbal_fetch_data, 0, sizeof(gimbal_fetch_data));
    memset(&chassis_cmd_send, 0, sizeof(chassis_cmd_send));
    memset(&slave_fetch_data, 0, sizeof(slave_fetch_data));
    // chassis_cmd_send.chassis_mode = CHASSIS_MODE_ENABLE; // 底盘直接启动
    offset_arm_angle[0] = -25.0f;
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
 * @brief 气泵控制,遥控器SD从上往下拨进行一次状态转换
 *
 */
static void PumpControl()
{
    if (switch_is_up(rc_data[TEMP].rc.switch_SD))
    {
        *pump_state |= 0b1000;
    }

    if (switch_is_down(rc_data[TEMP].rc.switch_SC))
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_SD) && (*pump_state & 0b1000))
        {
            if (*pump_state & 0b0001)
            {
                pump_stop(1);
            }
            else
            {
                pump_run(1);
            }
        }
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_SC))
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_SD) && (*pump_state & 0b1000))
        {
            if (*pump_state & 0b0010)
            {
                pump_stop(2);
            }
            else
            {
                pump_run(2);
            }
        } 
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_SC))
    {
        if (switch_is_down(rc_data[TEMP].rc.switch_SD) && (*pump_state & 0b1000))
        {
            if (*pump_state & 0b0100)
            {
                pump_stop(3);
            }
            else
            {
                pump_run(3);
            }
        }
    }

}

/**
 * @brief 	获取电机角度差值
 *
 */
static void get_delta_angle()
{
	// motor_delta_angle.delta_pos3508upraise = gimbal_fetch_data.feedback_arm_angle[0] - offset_arm_angle[0];
	// motor_delta_angle.delta_pos8010chassis = gimbal_fetch_data.feedback_arm_angle[1] - offset_arm_angle[1];
	// motor_delta_angle.delta_pos02arm = gimbal_fetch_data.feedback_arm_angle[2] - offset_arm_angle[2];
	// motor_delta_angle.delta_pos4310_1arm = gimbal_fetch_data.feedback_arm_angle[3] - offset_arm_angle[3];
	// motor_delta_angle.delta_pos4310_2arm = gimbal_fetch_data.feedback_arm_angle[4] - offset_arm_angle[4];
	// motor_delta_angle.delta_pos2006 = gimbal_fetch_data.feedback_arm_angle[5] - offset_arm_angle[5];

}

float theta = 0;
/**
 * @brief  图传跟随控制
 *
 */
static void ImgControlSet()
{
    // float img_y = 0.16f;
    // float img_z = 0.3f;
    
    // if (chassis_direct)
    //     theta2 = (gimbal_fetch_data.feedback_arm_angle[1] - offset_arm_angle[1] + 19.886f) / (-12.66f);
    // else 
    //     theta2 = (gimbal_fetch_data.feedback_arm_angle[1] - offset_arm_angle[1]) / (-12.66f);

    // Robotics_FK(FK_current_pos, gimbal_fetch_data.feedback_arm_angle, offset_arm_angle, chassis_direct); // 计算当前末端位姿

    theta = 1.571f + gimbal_fetch_data.feedback_arm_angle[1] - gimbal_fetch_data.feedback_arm_angle[2];
    gimbal_cmd_send.img_yaw_set = 0.08f ; // 计算当前yaw角度
    gimbal_cmd_send.img_pitch_set = 0.075f - 0.032f * theta;        // 计算当前yaw角度
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置,左侧开关状态[下]
 *
 */
static void RemoteControlSet()
{
    chassis_cmd_send.chassis_mode = CHASSIS_MODE_ENABLE;
    // 控制底盘和云台运行模式,云台待添加,云台是否始终使用IMU数据?
    if (switch_is_down(rc_data[TEMP].rc.switch_SB))       // 左侧开关状态[下],控制底盘
    {    
        if (switch_is_down(rc_data[TEMP].rc.switch_SC))
        {
            // 底盘参数
            chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_l_;            // _水平方向
            chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_l1;            // 1竖直方向
            // chassis_cmd_send.theta_z -= 0.0007f * (float)rc_data[TEMP].rc.rocker_r_;    // 旋转方向
            chassis_cmd_send.wz = 1.5f * (float)rc_data[TEMP].rc.rocker_r_;            // 旋转速度
            
            chassis_cmd_send.target_chassis_angle[4] += 0.00003f * (float)rc_data[TEMP].rc.rocker_r1;
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_SC))
        {
            chassis_cmd_send.target_chassis_angle[0] += 0.00005f * (float)rc_data[TEMP].rc.rocker_r1; // 摇臂电机目标角度
            chassis_cmd_send.target_chassis_angle[1] -= 0.00005f * (float)rc_data[TEMP].rc.rocker_r1; // 摇臂电机目标角度

            chassis_cmd_send.target_chassis_angle[2] += 0.00001f * (float)rc_data[TEMP].rc.rocker_l1; // 后腿电机目标角度
            chassis_cmd_send.target_chassis_angle[3] -= 0.00001f * (float)rc_data[TEMP].rc.rocker_l1; // 后腿电机目标角度

            chassis_cmd_send.vx = 5.0f * (float)rc_data[TEMP].rc.rocker_r_;            // _水平方向
            chassis_cmd_send.vy = 5.0f * (float)rc_data[TEMP].rc.rocker_l_;            // 1竖直方向

            if (switch_is_down(rc_data[TEMP].rc.switch_SD))
                master_cmd_data.event |= START_TRACK; // 启动履带事件
            else
                master_cmd_data.event &= ~START_TRACK; // 停止履带事件
        }
        else if (switch_is_up(rc_data[TEMP].rc.switch_SC))
        {
            if (switch_is_up(rc_data[TEMP].rc.switch_SD))
                chassis_cmd_send.chassis_mode = CHASSIS_MODE_FOLLOW;
            else
                chassis_cmd_send.chassis_mode = CHASSIS_MODE_ROTATE;

            chassis_cmd_send.vx = 5.0f * (float)rc_data[TEMP].rc.rocker_l_;            // _水平方向
            chassis_cmd_send.vy = 5.0f * (float)rc_data[TEMP].rc.rocker_l1;            // 1竖直方向
            chassis_cmd_send.target_chassis_angle[4] -= 0.00003f * (float)rc_data[TEMP].rc.rocker_r_; 
        }
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_SB))   // 左侧开关状态[中],控制云台机械臂
    {
        // direct = switch_is_down(rc_data[TEMP].rc.switch_SD) ? 1.0f : -1.0f; // 矿槽方向,左侧开关状态[下]为左槽,上为右槽
        if (switch_is_down(rc_data[TEMP].rc.switch_SC))
         { // 遥控器直接控制电机角度
            gimbal_cmd_send.target_arm_angle[0] -= 0.00001f * (float)rc_data[TEMP].rc.rocker_l_;

            gimbal_cmd_send.target_arm_angle[1] -= 0.00001f * (float)rc_data[TEMP].rc.rocker_l1;

            gimbal_cmd_send.target_arm_angle[2] -= 0.00002f * (float)rc_data[TEMP].rc.rocker_r1;

            ImgControlSet();
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_SC))
        { // 末端仅使用直接控制
            // actuator_pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_r1; // 单位为度
            // Math_Constrain(&actuator_pitch, 0.0f, 100.0f);
            // actuator_roll += 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
            // Math_Constrain(&actuator_roll, -180.0f, 180.0f);

            gimbal_cmd_send.target_arm_angle[3] += 0.00002f * (float)rc_data[TEMP].rc.rocker_r1;
            gimbal_cmd_send.target_arm_angle[4] += 0.00002f * (float)rc_data[TEMP].rc.rocker_r_;
            gimbal_cmd_send.target_arm_angle[5] += 0.00002f * (float)rc_data[TEMP].rc.rocker_l1;
            gimbal_cmd_send.target_arm_angle[6] = (float)rc_data[TEMP].rc.rocker_l_;//+= 0.0006f * (float)rc_data[TEMP].rc.rocker_l_;

            if (switch_is_down(rc_data[TEMP].rc.switch_SD) && (user_state & 0b001))
            {   // 开夹爪
                grabber_action = 2;
            }
            else if (switch_is_up(rc_data[TEMP].rc.switch_SD) && !(user_state & 0b001))
            {   // 关夹爪
                grabber_action = 1;
            }

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
        { 
            // 图传舵机控制
            gimbal_cmd_send.img_pitch_set += 0.000001f * (float)rc_data[TEMP].rc.rocker_l1;
            gimbal_cmd_send.img_yaw_set -= 0.000001f * (float)rc_data[TEMP].rc.rocker_l_;
            
            // 使用IK控制
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


static float controller_data[6];
/**
 * @brief 输入为自定义控制器时的控制量设置
 *
 */
static void CustomControllerSet()
{
    // if (!CustomControllerIsOnline())
    //     return;

    static float last_data[6];
    static float filter_data[6];
    static uint8_t cc_init = 0;
    // float controller_data[6]; // 前六个为机械臂控制量，后两个为键盘控制量

    memcpy(controller_data, &(img_data->ControllerToRobot), sizeof(controller_data));
    controller_data[0] -= 2.04f;
    controller_data[1] -= 1.571f;
    controller_data[2] = -controller_data[2];

    if ((controller_data[0] == 0) && (controller_data[1] == 0) && (controller_data[2] == 0))
    {
        return;
    }


    // if (fabs(error0) > 0.5f || fabs(error1) > 0.5f || fabs(error2) > 0.5f)
    // {   // 如果控制器输入与机械臂当前角度的误差过大，可能是因为控制器离线或者突然变化导致的，忽略这次更新，等待下一次更新
    //     return;
    // }

    uint8_t IsActive = traj.is_active;       // 轨迹规划是否激活有效

    if (IsActive)
    {
        memcpy(gimbal_cmd_send.target_arm_angle, traj.pos, sizeof(traj.pos));
    }
    else
    {
        memcpy(traj.pos, gimbal_fetch_data.feedback_arm_angle, sizeof(traj.pos));
        memcpy(last_data, gimbal_fetch_data.feedback_arm_angle, sizeof(last_data));   
        cc_init = 0;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        filter_data[i] = 0.05f * controller_data[i] + 0.95f * last_data[i];
        last_data[i] = filter_data[i];
    }

    float error0 = controller_data[0] - gimbal_fetch_data.feedback_arm_angle[0];
    float error1 = controller_data[1] - gimbal_fetch_data.feedback_arm_angle[1];
    float error2 = controller_data[2] - gimbal_fetch_data.feedback_arm_angle[2];


    if (img_data->CtrlDataIsUpdate)
    {
        // if (!cc_init)//fabs(error0) > 0.5f || fabs(error1) > 0.5f || fabs(error2) > 0.5f)
        // {
        //     float T = compute_segment_time(traj.pos, controller_data);
        //     CubicTrajectory_Init(&traj, traj.pos, controller_data, T);
        // }

        // if ((!cc_init) && (!IsActive))
        // {
        //     cc_init = 1;
        // }

        // if (cc_init)
        float T = 0;
        if ((abs(error0) > 0.35f) || (abs(error1) > 0.35f) || (abs(error2) > 0.35f))
        {
            T = 1.0f;
            
            img_data->CtrlDataIsUpdate = 0;

            if (!cc_init)
                CubicTrajectory_Init(&traj, traj.pos, controller_data, T);

            cc_init = 1;
            return;
        }
        else
            T = 0.15f;

        if (!cc_init) // 当自定义控制器不在初始化阶段时，才以0.15s的时间规划插值轨迹，否则直接初始化轨迹
            CubicTrajectory_Init(&traj, traj.pos, controller_data, T); // 以当前机械臂角度为初始点

        img_data->CtrlDataIsUpdate = 0;
    }

    // memcpy(gimbal_cmd_send.target_arm_angle, controller_data, sizeof(controller_data));
}


/**
 * @brief 处理上位机控制命令,使用插值进行平滑控制
 */
static void HostControlSet()
{
    // static uint16_t counter = 0;
    // if (counter++ > 100) // 每0.1秒有新的上位机命令
    // {   
    //     static float t = 0.0f;
    //     t += 0.3f;
    //     host_recv_data->cmd_arm_angle[5] = 6.0f*sin(t);
    //     counter = 0;
    //     HostSetUpdate();
    // }

    // if (0)
    // {   
    //     if (HostIsUpdate())
    //     {
    //         CubicTrajectory_Init(&traj, traj.pos, host_recv_data->cmd_arm_angle[5], 0.1f); // 以当前机械臂角度为初始点，目标点为上位机发送的角度，规划100ms的插值轨迹
    //     }
    //     // gimbal_cmd_send.target_arm_angle[4] = host_recv_data->cmd_arm_angle[3];
    //     // gimbal_cmd_send.target_arm_angle[5] = host_recv_data->cmd_arm_angle[4];
    //     CubicTrajectory_Update(&traj, 0.0009f);
    //     if (traj.is_active)
    //         gimbal_cmd_send.target_arm_angle[6] = traj.pos;
    // }
    // else
    // {
    //     traj.pos = gimbal_fetch_data.feedback_arm_angle[6]; // 如果未使能，则将插值的初始点设置为当前机械臂角度，避免未使能时机械臂突然跳动
    // }
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
    if ((robot_state == ROBOT_STOP) || (robot_state == ROBOT_READY))
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
            // chassis_cmd_send.chassis_mode = CHASSIS_MODE_ENABLE;
            gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_ENABLE;
            // shoot_cmd_send.shoot_mode = SHOOT_ON;
            LOGINFO("[CMD] reinstate, robot ready");
        }

        /* 如果有XXX_MODE_STOP，则将电机目标值更新为电机当前反馈值 */
        if (gimbal_cmd_send.gimbal_mode == GIMBAL_MODE_STOP)
        {
            memcpy(gimbal_cmd_send.target_arm_angle, gimbal_fetch_data.feedback_arm_angle, sizeof(gimbal_cmd_send.target_arm_angle));
            if (user_state & 0b00100)
            {
                img_data->key_count[SINGLE][9]++;
                user_state &= ~0b00100;
            }
        }
        if (chassis_cmd_send.chassis_mode == CHASSIS_MODE_STOP)
        {
            memcpy(chassis_cmd_send.target_chassis_angle, slave_fetch_data.feedback_chassis_angle, sizeof(chassis_cmd_send.target_chassis_angle));
        }
    }
    else if ((robot_state == ROBOT_INIT) || (robot_state == ROBOT_DIE))
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_MODE_STOP;

        if (user_state & 0b00100)
        {
            img_data->key_count[SINGLE][9]++;
            user_state &= ~0b00100;
        }
    }


        // /* 机器人死亡处理 */
    if (gimbal_fetch_data.gimbal_state == GIMBAL_DIE)
    { // 在非初始化状态下，机械臂电机掉线后，初始化完成标志复位，机器人状态置为死亡
        if (robot_state != ROBOT_INIT)
            robot_state = ROBOT_DIE;

        gimbal_cmd_send.gimbal_mode = GIMBAL_MODE_STOP;
        chassis_cmd_send.chassis_mode = CHASSIS_MODE_STOP;
    }
    else if (robot_state == ROBOT_DIE)
    { // 当机器人为死亡状态时，若电机重新上线，则将机器人状态置为初始化，进入电机初始化流程
        if (gimbal_fetch_data.gimbal_state == GIMBAL_ALIVE)
        {
            gimbal_cmd_send.motor_init_finish = 0;
            robot_state = ROBOT_INIT;
        }
    }

}

/**
 * @brief 自动动作事件转换
 */
static void AutoActionEventTransfer()
{
    if (auto_motion->event == MANUAL_EVENT_NONE)
    {
        if (rc_data[TEMP].rc.rocker_l1 > 600 || img_data->KeyMouseData[TEMP][SINGLE].x)
        {
            auto_motion->event = REPOSITION_EVENT;
            master_cmd_data.event |= CHASSIS_REPOSITION;
            if (user_state & 0b00010)
            {
                img_data->key_count[SINGLE][14]++;
            }
            gimbal_cmd_send.img_yaw_set = 0.095f;
        }
        else if (rc_data[TEMP].rc.rocker_l1 < -600  || img_data->KeyMouseData[TEMP][SHIFT].q)
        {
            auto_motion->event = START_GRAB_5;
            // auto_motion->event = REPOSITION_EVENT;
            // if (user_state & 0b00010)
            // {
            //     img_data->key_count[SINGLE][14]++;
            // }
            // gimbal_cmd_send.img_yaw_set = 0.119f;
        }
        else if (rc_data[TEMP].rc.rocker_l_ > 600  || img_data->KeyMouseData[TEMP][SHIFT].e)
        {
            auto_motion->event = START_GRAB_6;
        }
        else if (rc_data[TEMP].rc.rocker_l_ < -600 || img_data->KeyMouseData[TEMP][SINGLE].q)
        {
            auto_motion->event = START_STORE_L;
        }
        else if (rc_data[TEMP].rc.rocker_r1 > 600 || img_data->KeyMouseData[TEMP][SHIFT].r)
        {
            // auto_motion->event = CLIMB_EVENT;
            // master_cmd_data.event |= CHASSIS_CLIMB;
        }
        else if (rc_data[TEMP].rc.rocker_r1 < -600 || img_data->KeyMouseData[TEMP][CTRL].r)
        {
            // auto_motion->event = REPOSITION_EVENT;
            // master_cmd_data.event |= CHASSIS_CLIMB;
        }
        else if (rc_data[TEMP].rc.rocker_r_ > 600 || img_data->KeyMouseData[TEMP][SINGLE].e)
        {
            auto_motion->event = START_STORE_R;
        }
        // else if (img_data->KeyMouseData[TEMP][CTRL].e)
        // {
        //     auto_motion->event = 8;
        // }
        // else if (img_data->KeyMouseData[TEMP][CTRL].q)
        // {
        //     auto_motion->event = 9;
        // }
    }
    else
    {
        if (rc_data[TEMP].rc.rocker_r_ < -600  || img_data->KeyMouseData[TEMP][SINGLE].r)
        {
            auto_motion->event = REFORMAT;
            // memcpy(gimbal_cmd_send.target_arm_angle, gimbal_fetch_data.feedback_arm_angle, sizeof(gimbal_cmd_send.target_arm_angle));
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
        memcpy(gimbal_cmd_send.target_arm_angle, traj.pos, sizeof(traj.pos));
    }
    else
    {
        memcpy(traj.pos, gimbal_fetch_data.feedback_arm_angle, sizeof(traj.pos));
    }
}

/** 
 * @brief  处理底盘事件
 * 
 */
static void ChassisEventHandler()
{
    static bool chassis_init_reposition = false; // 底盘的初始化复位
    if (!chassis_init_reposition && robot_state == ROBOT_READY)
    {   // 初始化完成进行依次底盘关节复位
        // master_cmd_data.event |= CHASSIS_REPOSITION;
        chassis_init_reposition = true;

        // gimbal_cmd_send.target_arm_angle[6] = 400; // 同时进行夹爪张开复位
        // grabber_action = 1;
    }

    if (slave_fetch_data.event & CHASSIS_FSM_BUSY)
    {   // 底盘FSM不在空闲状态，则说明已经接收到event，并开始执行状态机
        // 将发送过去的事件清空，防止二次触发
        master_cmd_data.event &= ~CHASSIS_REPOSITION;
        master_cmd_data.event &= ~CHASSIS_CLIMB;
        // 如果有其它自动动作，就继续添加
        // 同步关节角度
        memcpy(chassis_cmd_send.target_chassis_angle, slave_fetch_data.feedback_chassis_angle, sizeof(chassis_cmd_send.target_chassis_angle));
    }
    else 
    {   // 底盘FSM在空闲状态，则清空REFORMAT事件，因为此时不需要REFORMAT
        master_cmd_data.event &= ~CHASSIS_REFORMAT;
    }
}

/** 
 * @brief  夹爪控制
 * 
 */
static void GrabberControlSet()
{
    if (auto_motion->grab_flow == Grabbing)
    {
        // user_state |= 0b001;
        grabber_action = 1;
        // gimbal_cmd_send.target_arm_angle[6] = -400;
        if (user_state & 0b001)
        {
            auto_motion->IsGrabbing = 1;
            grabber_action = 0;
        }
    }
    else if (auto_motion->store_flow == Releasing)
    {
        // user_state &= ~0b001;
        // gimbal_cmd_send.target_arm_angle[6] = 400;
        grabber_action = 2;
        if (!(user_state & 0b001))
        {
            auto_motion->IsGrabbing = 0;
            grabber_action = 0;
        }
    }

    switch (grabber_action)
    {
        case 0:
            break;
        case 1: // 闭合
            // user_state |= 0b001;
            gimbal_cmd_send.target_arm_angle[6] = -400;
            // grabber_action = 0;
            break;
        case 2: // 张开
            // user_state &= ~0b001;
            gimbal_cmd_send.target_arm_angle[6] = 400;
            // grabber_action = 0;
            break;
    }

    static uint8_t block_reset = 0;
    static uint8_t open_reset = 0;
    if (gimbal_fetch_data.motor_blocked && !block_reset)
    {   // 夹爪电机发生堵转，速度置0
        gimbal_cmd_send.target_arm_angle[6] = 0;
        block_reset = 1;
        if (grabber_action == 1)
        {
            offset_arm_angle[0] = gimbal_fetch_data.feedback_arm_angle[6];
            grabber_action = 0;
            user_state |= 0b001;
        }
    }
    else if (!gimbal_fetch_data.motor_blocked)
    {
        block_reset = 0;
    }

    if ((gimbal_fetch_data.feedback_arm_angle[6] > offset_arm_angle[0] + 24.0f) && !open_reset)
    {
        gimbal_cmd_send.target_arm_angle[6] = 0;
        open_reset = 1;
        if (grabber_action == 2)
        {
            grabber_action = 0;
            user_state &= ~0b001;
        }
    }
    else if (gimbal_fetch_data.feedback_arm_angle[6] < offset_arm_angle[0] + 24.0f)
    {
        open_reset = 0;
    }
}

/**
 * @brief 电机控制数据初始化
 * 
 */
static void MotorCmdDataInit()
{
    static uint16_t init_time = 0;
    if ((!gimbal_cmd_send.motor_init_finish) || (!chassis_cmd_send.motor_init_finish) || !(slave_fetch_data.event & CHASSIS_INIT_FINISH))
    {// 三方若未初始化完成, 继续进行初始化
        uint8_t isALLone;
        // 检测电机是否有回传数据，若有，则赋电机初值，并置位初始化标志
        for(uint8_t i = 0 ; i < 7 ; i++)
        {   
            if(gimbal_fetch_data.feedback_arm_angle[i])
            {
                gimbal_motor_init_flag[i] = 1;     // 收到反馈值，初始化标志置1
                gimbal_cmd_send.target_arm_angle[i] = gimbal_fetch_data.feedback_arm_angle[i]; // 同时将目标角度赋当前值为初始值
                if ((i == 1) || (i == 6)) // 06_1和4005_6电机
                    offset_arm_angle[i] = gimbal_fetch_data.feedback_arm_angle[i]; // 机械臂电机零位偏差角度
            }
            isALLone += gimbal_motor_init_flag[i]; // 累加各个标志位的值
        }
        for (uint8_t i = 0; i < 5 ; i++)
        {
            if (slave_fetch_data.feedback_chassis_angle[i])
            {
                chassis_motor_init_flag[i] = 1;    // 收到反馈值，初始化标志置1
                chassis_cmd_send.target_chassis_angle[i] = slave_fetch_data.feedback_chassis_angle[i]; // 同时将目标角度赋当前值为初始值
            }
            isALLone += chassis_motor_init_flag[i];  // 累加各个标志位的值
        }
        // 若累加值为12，则六个电机均已收到反馈值，初始化完成
        if((isALLone == 12))// || (init_time++ > 5000)) // 在等待一段时间后，也会直接进入就绪状态
        {
            gimbal_cmd_send.motor_init_finish = 1;
            chassis_cmd_send.motor_init_finish = 1;
            // 舵机赋初值
            gimbal_cmd_send.img_pitch_set = 0.09;//0.0675f;
            gimbal_cmd_send.img_yaw_set = 0.094;//0.1f;
            init_time = 0;
        }
        robot_state = ROBOT_INIT;
    }    
    else if (robot_state != ROBOT_DIE)
        robot_state = ROBOT_READY;  // 底盘和机械臂全部初始化才进入就绪状态
}

static void MotorAngleLimit()
{
    // 机械臂角度限幅,根据机械臂的实际工作空间和电机的安装位置设置合理的限幅值,避免出现无解或机械碰撞的情况
    Math_Constrain(&gimbal_cmd_send.target_arm_angle[0], -3.1f, 3.1f);
    Math_Constrain(&gimbal_cmd_send.target_arm_angle[1], -1.47f, 1.39f);//-1.7f, 0.85f
    // Math_Constrain(&gimbal_cmd_send.target_arm_angle[2], -2.43f, 2.94f);
    gimbal_cmd_send.target_arm_angle[2] = float_constrain(gimbal_cmd_send.target_arm_angle[2], -2.43f, 2.94f);
    Math_Constrain(&gimbal_cmd_send.target_arm_angle[3], -3.14f, 3.14f);
    Math_Constrain(&gimbal_cmd_send.target_arm_angle[4], -1.57f, 1.57f);
    Math_Constrain(&gimbal_cmd_send.target_arm_angle[5], -3.14f, 3.14f);

    // 底盘角度限幅
    chassis_cmd_send.target_chassis_angle[0] = float_constrain(chassis_cmd_send.target_chassis_angle[0], 0.05f, 5.8f);
    chassis_cmd_send.target_chassis_angle[1] = float_constrain(chassis_cmd_send.target_chassis_angle[1], -5.8f, -0.05f);
    chassis_cmd_send.target_chassis_angle[2] = float_constrain(chassis_cmd_send.target_chassis_angle[2], 0.12f, 0.9f);
    chassis_cmd_send.target_chassis_angle[3] = float_constrain(chassis_cmd_send.target_chassis_angle[3], -0.9f, -0.12f);


    // 图传舵机限幅,根据实际安装位置和需要设置合理的限幅值
    gimbal_cmd_send.img_yaw_set = float_constrain(gimbal_cmd_send.img_yaw_set, 0.05f, 0.13f);
    gimbal_cmd_send.img_pitch_set = float_constrain(gimbal_cmd_send.img_pitch_set, 0.055f, 0.09f);
}



/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
    SubGetMessage(gimbal_feed_sub, (void *)&gimbal_fetch_data);
    // SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    slave_fetch_data = *(Slave_Upload_Data_s *)CANCommGet(cmd_can_comm);                // 获取从控板反馈的数据

    MotorCmdDataInit(); // 电机控制数据初始化,等待电机反馈值

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    //CalcOffsetAngle();

    CubicTrajectory_Update(&traj);

    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_SB))     // 遥控器左侧开关状态为[上],键鼠和自定义控制器控制，添加上位机控制
    {
        // CustomControllerSet();
        MouseKeySet();
        AutoActionHandler();    // 处理自动动作状态机
    }
    else                                                // 遥控器左侧开关状态为[下]或[中],均为遥控器控制
    {
        RemoteControlSet();
    }

    ChassisEventHandler();
    // if (robot_state != ROBOT_INIT)
    EmergencyHandler();     // 处理模块离线和遥控器急停等紧急情况
    GrabberControlSet();

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,chassis_fetch_data.bullet_speed)
    
    // 先暂时放这，后续放入遥控器处理逻辑中
    // HostControlSet();       // 处理上位机控制

    MotorAngleLimit(); // 限幅目标角度

    // 推送消息,双板通信,视觉通信等
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);           // 发布云台机械臂控制消息
    // PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);         // 发布底盘控制消息

    ui_data.pump_state = *pump_state; // 更新泵状态到UI数据
    ui_data.user_state = user_state; // 更新用户状态到UI数据

    //////////////////
    // if (slave_fetch_data.feedback_8010_angle[1] && (gimbal_cmd_send.motor_init_finish == 0))
    // {
    //     gimbal_cmd_send.motor_init_finish = 1;
    //     gimbal_cmd_send.target_arm_angle[1] = slave_fetch_data.feedback_8010_angle[1]; // 将从控板反馈的8010电机数据更新到gimbal_cmd_send
    // }

    // 更新从控板的控制数据
    memcpy(&master_cmd_data, &chassis_cmd_send, 9 * sizeof(float) + sizeof(chassis_mode_e));
    CANCommSend(cmd_can_comm, (uint8_t *)&master_cmd_data);  

    // 更新返回自定义控制器的数据
    memcpy(img_data->RobotToController.data, gimbal_fetch_data.feedback_arm_angle, 12);

    // memcpy(host_send_data.current_arm_angle, gimbal_fetch_data.feedback_arm_angle + 1, sizeof(host_send_data.current_arm_angle));
    // HostSend();
}



/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    if (img_data->KeyMouseData[TEMP][SINGLE].w == 1)
    {
        if (chassis_cmd_send.vy < 6000)
            chassis_cmd_send.vy += 50;
    }
    else
    {
        if (chassis_cmd_send.vy > 0)
            chassis_cmd_send.vy -= 25;
    }
    
    if (img_data->KeyMouseData[TEMP][SINGLE].s == 1)
    {
        if (chassis_cmd_send.vy > -5000)
            chassis_cmd_send.vy -= 50;
    }
    else
    {
        if (chassis_cmd_send.vy < 0)
            chassis_cmd_send.vy += 25;
    }
    
    if (img_data->KeyMouseData[TEMP][SINGLE].a == 1)
    {
        if (chassis_cmd_send.vx < 3500)
            chassis_cmd_send.vx += 25;
    }
    else
    {
        if (chassis_cmd_send.vx > 0)
            chassis_cmd_send.vx -= 20;
    }
    
    if (img_data->KeyMouseData[TEMP][SINGLE].d == 1)
    {
        if (chassis_cmd_send.vx > -2500)
            chassis_cmd_send.vx -= 35;
    }
    else
    {
        if (chassis_cmd_send.vx < 0)
        chassis_cmd_send.vx += 16;
    }
    
    if (img_data->KeyMouseRaw.shift == 1)
    { // 按下shift，代表缓慢行动
        chassis_cmd_send.vx = img_data->KeyMouseData[TEMP][SHIFT].a * 1000 - img_data->KeyMouseData[TEMP][SHIFT].d * 1000;  
        chassis_cmd_send.vy = img_data->KeyMouseData[TEMP][SHIFT].w * 1000 - img_data->KeyMouseData[TEMP][SHIFT].s * 1000;
    }
        // chassis_cmd_send.wz = img_data->KeyMouseData[TEMP][SINGLE].mouse_x * 50 + img_data->KeyMouseData[TEMP][SHIFT].mouse_x * 10
        //                     + img_data->KeyMouseData[TEMP][SHIFT].e * 1000 - img_data->KeyMouseData[TEMP][SHIFT].q * 1000;
    chassis_cmd_send.target_chassis_angle[4] -= img_data->KeyMouseData[TEMP][SINGLE].mouse_x * 0.00008f + img_data->KeyMouseData[TEMP][SHIFT].mouse_x * 0.00008f;
                            //+ img_data->KeyMouseData[TEMP][SHIFT].e * 1000 - img_data->KeyMouseData[TEMP][SHIFT].q * 1000;


    if (auto_motion->event == 0)
    {        
        if (img_data->KeyMouseData[TEMP][SINGLE].z)
        {
            // chassis_cmd_send.target_chassis_angle[2] += 0.001f;
            // chassis_cmd_send.target_chassis_angle[3] -= 0.001f;
        }
        else if(img_data->KeyMouseData[TEMP][SINGLE].x)
        {
            // chassis_cmd_send.target_chassis_angle[2] -= 0.001f;
            // chassis_cmd_send.target_chassis_angle[3] += 0.001f;
        }

        if (img_data->KeyMouseData[TEMP][SINGLE].b)
        {
            master_cmd_data.event |= START_TRACK_INVERSE;
        }
        else
        {
            master_cmd_data.event &= ~START_TRACK_INVERSE;
        }

        // if (img_data->KeyMouseData[TEMP][CTRL].r)
        //     gimbal_cmd_send.DM4310ReCmd = 1;
        // else
        //     gimbal_cmd_send.DM4310ReCmd = 0;

        // if (img_data->KeyMouseData[TEMP][CTRL].v)
        // {
        //     gimbal_cmd_send.img_yaw_set = 0.1f;

        //     if (user_state & 0b00010)
        //     {
        //         img_data->key_count[SINGLE][14]++;
        //     }
        // }

        // switch (img_data->key_count[SINGLE][13] % 2) // c 切换底盘跟随和小陀螺
        // {
        //     case 0:
                chassis_cmd_send.chassis_mode = CHASSIS_MODE_FOLLOW;
        //         break;
        //     default:
        //         chassis_cmd_send.chassis_mode = CHASSIS_MODE_ROTATE;
        //         break;
        // }
        
        if (img_data->KeyMouseData[TEMP][SINGLE].c)
        {
        }

        if (img_data->KeyMouseData[TEMP][SHIFT].c)
        {
            // gimbal_cmd_send.target_arm_angle[0] = offset_arm_angle[0] + 2000.0f;
        }

        if (img_data->KeyMouseData[TEMP][SHIFT].v)
        {
            // offset_arm_angle[5] = gimbal_fetch_data.feedback_arm_angle[5];
        }

        if (img_data->KeyMouseData[TEMP][SINGLE].c)
            gimbal_cmd_send.DM4310ReCmd = 1;
        else
            gimbal_cmd_send.DM4310ReCmd = 0;

        switch (img_data->key_count[SHIFT][15] % 2) //
        {
            case 0:
                break;
            default:
                // img_data->key_count[SHIFT][15]++;
                break;
        }

        switch (img_data->key_count[CTRL][15] % 2) //
        {
            case 0:
                break;
            default:
                // offset_arm_angle[4] -= 0.2856f;
                // gimbal_cmd_send.target_arm_angle[4] -= 0.2856f;
                // img_data->key_count[CTRL][15]++;
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

    // switch (img_data->key_count[SHIFT][8] % 2) // shift+R键重新初始化UI
    // {
    //     case 0:
    //         break;
    //     default:
    //         UIReset(&ui_data);
    //         img_data->key_count[SHIFT][8]++;
    //         break;
    // }
    
    switch (img_data->key_count[SHIFT][9] % 2) // shift+F键正转roll90度
    {
        case 0:
            break;
        default:
            // offset_arm_angle[4] += 1.571f * 4125.30f;
            // offset_arm_angle[5] += 1.571f * 4125.30f;
            // offset_arm_angle[5] += 1.571f * 4125.30f;
            // img_data->key_count[SHIFT][9]++;
            break;
    }

    switch (img_data->key_count[CTRL][9] % 2) // ctrl+F键反转roll90度
    {
        case 0:
            break;
        default:
            // offset_arm_angle[4] -= 1.571f * 4125.30f;
            // offset_arm_angle[5] -= 1.571f * 4125.30f;
            // offset_arm_angle[5] -= 1.571f * 4125.30f;
            // img_data->key_count[CTRL][9]++;
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

            CustomControllerSet();
            if (auto_motion->event != 0)
                img_data->key_count[SINGLE][9]++;

            break;
    }

    switch (img_data->key_count[SINGLE][10] % 2) // G键开闭夹爪
    {
        // case 0: // 不动作
        //     break;

        // case 1: // 闭合
        //     // gimbal_cmd_send.target_arm_angle[6] = -400.0f;
        //     grabber_action = 1;
        //     img_data->key_count[SINGLE][10] ++;
        //     // user_state |= 0b100;
        //     break;

        // case 2: // 不动作
        //     break;
        
        // case 3: // 张开
        //     // gimbal_cmd_send.target_arm_angle[6] = 400.0f;
        //     grabber_action = 2;
        //     img_data->key_count[SINGLE][10] ++;
        //     // user_state &= ~0b100;
        //     break;

        case 0:
            break;
        case 1:
            if (user_state & 0b001)
                grabber_action = 2;
            else 
                grabber_action = 1;

            img_data->key_count[SINGLE][10] ++;
        default:
            break;
    }

    switch (img_data->key_count[SHIFT][15] % 2) // shift+B开一点
    {
        case 0:
            
            break;
        default:
            gimbal_cmd_send.target_arm_angle[6] = 400.0f;
            static uint8_t delay_time = 0;
            if (delay_time++ < 30)
            {
                break;
            }
            delay_time = 0;
            img_data->key_count[SHIFT][15]++;
            gimbal_cmd_send.target_arm_angle[6] = 0.0f;
            grabber_action = 1;
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

            // img_data->key_count[CTRL][10]++;
            // break;
    }

    switch (img_data->key_count[SINGLE][14] % 2) // V键开关视角跟随末端
    {
        case 0:
            if (user_state & 0b00010)
            {
                gimbal_cmd_send.img_yaw_set = 0.09f;//0.1f + (4.660f / (-12.66f));
                user_state &= ~0b00010;
            }
            // gimbal_cmd_send.img_yaw_set += img_data->KeyMouseData[TEMP][CTRL].d * 0.0003f - img_data->KeyMouseData[TEMP][CTRL].a * 0.0003f;
            gimbal_cmd_send.img_pitch_set -= (img_data->KeyMouseData[TEMP][SINGLE].mouse_y + img_data->KeyMouseData[TEMP][CTRL].mouse_y + img_data->KeyMouseData[TEMP][SHIFT].mouse_y)* 0.00001f; // img_data->KeyMouseData[TEMP][CTRL].w * 0.0003f - img_data->KeyMouseData[TEMP][CTRL].s * 0.0003f;
            break;
        default:
            user_state |= 0b00010;
            ImgControlSet();
            break;
    }

    switch (img_data->key_count[SINGLE][11] % 2) // Z键伸腿
    {
        case 0:
            break;
        default:
            if (chassis_cmd_send.target_chassis_angle[2] < 0.65f || chassis_cmd_send.target_chassis_angle[2] > -0.65f)
            {            
                chassis_cmd_send.target_chassis_angle[2] += 0.003f;
                chassis_cmd_send.target_chassis_angle[3] -= 0.003f;
            }
            else
            {
                chassis_cmd_send.target_chassis_angle[2] -= 0.003f;
                chassis_cmd_send.target_chassis_angle[3] += 0.003f;
            }

            if (if_reached(chassis_cmd_send.target_chassis_angle[2], 0.65, 0.01) && if_reached(chassis_cmd_send.target_chassis_angle[3], -0.65, 0.01))
            {
                chassis_cmd_send.target_chassis_angle[2] = 0.65f;
                chassis_cmd_send.target_chassis_angle[3] = -0.65f;
                img_data->key_count[SINGLE][11]++;
            }
            break;
    }

}
