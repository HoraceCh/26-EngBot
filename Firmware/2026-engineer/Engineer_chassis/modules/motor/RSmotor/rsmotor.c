#include "rsmotor.h"
#include "cmsis_os.h"
#include <stdarg.h>

static uint8_t idx;
static RSMotorInstance *rs_motor_instance[RS_MOTOR_CNT];
static osThreadId rs_task_handle[RS_MOTOR_CNT];

static uint16_t cmd_delay_time[RS_MOTOR_CNT] = {0};  // 电机上线后,延迟计时一段时间后再使能


static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief 写入单个参数
 * @param index: 参数编号索引
 * @param ref: 参数值
 */
static void RSMotorWriteParam(RSMotorInstance* motor, uint16_t index, float ref)
{
    motor->send_mailbox.txCanIdEx.mode = 0x12;
    motor->send_mailbox.txCanIdEx.data = motor->can_instace->rx_id;
    motor->can_instace->txconf.ExtId = *(uint32_t*)&motor->send_mailbox.txCanIdEx;
    memset(motor->can_instace->tx_buff, 0, 8);
    memcpy(motor->can_instace->tx_buff, (uint8_t*)&index, 2);
    switch (index)
    {
        case RS_RUN_MODE:
            uint8_t val = (uint8_t)ref;
            memcpy(motor->can_instace->tx_buff + 4, (uint8_t*)&val, 1);
            break;
        default:
            memcpy(motor->can_instace->tx_buff + 4, (uint8_t*)&ref, 4);
            break;
    }
    CANTransmit(motor->can_instace, 1);
    // DWT_Delay(0.0003f);
}

/**
 * @brief 运控模式电机控制,即MIT控制
 */
static void RSMotorControlMIT(RSMotorInstance* motor)
{
    uint16_t pos_tmp, vel_tmp, Kp_tmp, Kd_tmp, torque_tmp;

    LIMIT_MIN_MAX(motor->send_mailbox.angle_des, RS_P_MIN, RS_P_MAX);
    LIMIT_MIN_MAX(motor->send_mailbox.speed_des, RS_V_MIN, RS_V_MAX);
    LIMIT_MIN_MAX(motor->send_mailbox.torque_des, RS_T_MIN, RS_T_MAX);
    LIMIT_MIN_MAX(motor->send_mailbox.Kp, RS_KP_MIN, RS_KP_MAX);
    LIMIT_MIN_MAX(motor->send_mailbox.Kd, RS_KD_MIN, RS_KD_MAX);

    motor->send_mailbox.txCanIdEx.mode = 0x1;
    motor->send_mailbox.txCanIdEx.data = float_to_uint(motor->send_mailbox.torque_des, RS_T_MIN, RS_T_MAX, 16);
    motor->can_instace->txconf.ExtId = *(uint32_t*)&motor->send_mailbox.txCanIdEx;
   
    pos_tmp = float_to_uint(motor->send_mailbox.angle_des, RS_P_MIN, RS_P_MAX, 16);
    vel_tmp = float_to_uint(motor->send_mailbox.speed_des, RS_V_MIN, RS_V_MAX, 16);
    torque_tmp = float_to_uint(motor->send_mailbox.torque_des, RS_T_MIN, RS_T_MAX, 16);
    Kp_tmp = float_to_uint(motor->send_mailbox.Kp, RS_KP_MIN, RS_KP_MAX, 16);
    Kd_tmp = float_to_uint(motor->send_mailbox.Kd, RS_KD_MIN, RS_KD_MAX, 16);

    motor->can_instace->tx_buff[0] = (uint8_t)(pos_tmp >> 8);
    motor->can_instace->tx_buff[1] = (uint8_t)(pos_tmp);
    motor->can_instace->tx_buff[2] = (uint8_t)(vel_tmp >> 8);
    motor->can_instace->tx_buff[3] = (uint8_t)(vel_tmp);
    motor->can_instace->tx_buff[4] = (uint8_t)(Kp_tmp >> 8);
    motor->can_instace->tx_buff[5] = (uint8_t)(Kp_tmp);
    motor->can_instace->tx_buff[6] = (uint8_t)(Kd_tmp >> 8);
    motor->can_instace->tx_buff[7] = (uint8_t)(Kd_tmp);

    CANTransmit(motor->can_instace, 1);
    DWT_Delay(0.001f);
}

// /**
//  * @brief 调试使用，设置电机CAN_ID
//  * @param uint8_t _id: 新ID
//  */
// void RSMotorSetID(RSMotorInstance* motor, uint8_t _id)
// {
//     motor->send_mailbox.txCanIdEx.mode = 0x7;
//     motor->send_mailbox.txCanIdEx.data = (uint16_t)(motor->can_instace->rx_id | (_id << 8));
//     memset(motor->can_instace->tx_buff, 0, 8);
//     motor->can_instace->txconf.ExtId = *(uint32_t*)&motor->send_mailbox.txCanIdEx;
//     CANTransmit(motor->can_instace, 1);
//     DWT_Delay(0.1);
// }

/**
 * @brief 电机配置函数，包括：使能失能，设置零位，设置CANID
 */
void RSMotorConfig(RSMotorInstance* motor, RSMotor_ConfigType_e _type, ...)
{
    motor->send_mailbox.txCanIdEx.mode = _type;
    motor->send_mailbox.txCanIdEx.data = motor->can_instace->rx_id;
    memset(motor->can_instace->tx_buff, 0, 8);

    switch (_type)
    {
        case RS_SET_ZERO_POSITION:
            motor->can_instace->tx_buff[0] = 1;
            break;
        case RS_SET_CANID:
            va_list args;
            va_start(args, _type);
            uint8_t new_id = (uint8_t)va_arg(args, int);
            motor->send_mailbox.txCanIdEx.data = (uint16_t)(motor->can_instace->rx_id | (new_id << 8));
            va_end(args);
            break;
        default:
            break;
    }

    motor->can_instace->txconf.ExtId = *(uint32_t*)&motor->send_mailbox.txCanIdEx;
    CANTransmit(motor->can_instace, 1);
    DWT_Delay(0.0003f);
}

/**
 * @brief 接口函数,设置电机参数
 */
void RSMotorSetAngle(RSMotorInstance *motor, float _angle)
{   // 角度
    motor->send_mailbox.angle_des = _angle;
}
void RSMotorSetSpeed(RSMotorInstance *motor, float _speed)
{   // 速度
    motor->send_mailbox.speed_des = _speed;
}
void RSMotorSetTorque(RSMotorInstance *motor, float _torque)
{   // 力矩
    motor->send_mailbox.torque_des = _torque;
}
void RSMotorSetCoefficient(RSMotorInstance *motor, float _Kp, float _Kd)
{   // 比例系数和微分系数
    motor->send_mailbox.Kp = _Kp;
    motor->send_mailbox.Kd = _Kd;
}
void RSMotorSetRef(RSMotorInstance *motor, float ref)
{   // 设置电机期望值
     motor->motor_controller.pid_ref = ref;
}
void RSMotorEnable(RSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}
void RSMotorStop(RSMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}


static void RSMotorLostCallback(void *motor_ptr)
{
    LOGWARNING("[RSMotor] motor lost");
}

static void RSMotorDecode(CANInstance *motor_can)
{    
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;                       // 8字节数据
    RSMotorInstance *motor = (RSMotorInstance *)motor_can->id;  // 将can实例中保存的id转换成电机实例的指针
    RSMotor_Measure_s *measure = &(motor->measure);
    measure->rxCanIdEx = *(RSMotor_ExCanIdInfo_t*)&motor_can->rx_extid; // 将29位扩展帧ID解包为结构体

    DaemonReload(motor->daemon); // 喂狗

    if (measure->rxCanIdEx.mode != 0x2)
        return;

    measure->last_angle = measure->angle;
    tmp = (uint16_t)((rxbuff[0] << 8) | rxbuff[1]);
    measure->angle = uint_to_float(tmp, RS_P_MIN, RS_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[2] << 8) | rxbuff[3]);
    measure->speed = uint_to_float(tmp, RS_V_MIN, RS_V_MAX, 16);

    tmp = (uint16_t)((rxbuff[4] << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, RS_T_MIN, RS_T_MAX, 16);

    tmp = (uint16_t)((rxbuff[6] << 8) | rxbuff[7]);
    measure->temperature = (float)(tmp / 10.0f);

    memcpy(measure, (uint8_t*)&measure->rxCanIdEx + 1, 2);

    // 若配置了参数表的zero_stat为1,即编码器初始化范围为(-pi, pi),则无需使用total_angle
    // 若未配置,则建议使用total_angle
    // if (measure->angle - measure->last_angle > 3.1416f)
    //     measure->total_round--;
    // else if (measure->angle - measure->last_angle < -3.1416f)
    //     measure->total_round++;
    // measure->total_angle = measure->total_round * 6.2832f + measure->angle;
}

RSMotorInstance* RSMotorInit(Motor_Init_Config_s *config)
{
    RSMotorInstance *motor = (RSMotorInstance *)malloc(sizeof(RSMotorInstance));
    memset(motor, 0, sizeof(RSMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.Kp = config->controller_param_init_config.Kp;
    motor->motor_controller.Kd = config->controller_param_init_config.Kd;

    config->can_init_config.can_module_callback = RSMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.rx_id = 0x10 + config->can_init_config.tx_id; // 协议中的Master_ID
    motor->can_instace = CANRegister(&config->can_init_config);

    // 根据协议，扩展ID中不变的部分直接赋值
    motor->send_mailbox.txCanIdEx.id = config->can_init_config.tx_id;
    motor->send_mailbox.txCanIdEx.reserved = 0;

    Daemon_Init_Config_s conf = {
        .callback = RSMotorLostCallback,
        .owner_id = motor,
        .reload_count = 30,
    };
    motor->daemon = DaemonRegister(&conf);

    rs_motor_instance[idx++] = motor;
    return motor;
}


void RSMotorControl()
{
    RSMotorInstance* motor;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller;
    RSMotor_Measure_s *measure;
    float pid_measure, pid_ref;                             // 电机PID测量值和设定值

    for(size_t i = 0; i < idx; ++i)
    {
        motor = rs_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;

        // 若使用的是total_angle, 则取消注释后半段内容
        pid_ref = motor_controller->pid_ref;// - measure->total_round * 6.2832f; // 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

        // RSMotorSetAngle(motor, pid_ref); // 设置角度

        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->angle; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1; // 设置反转
            
        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->speed;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }


        RSMotorSetTorque(motor, pid_ref); // 设置转矩
        RSMotorSetCoefficient(motor, motor->motor_controller.Kp, motor->motor_controller.Kd); // 设置电机系数


        // STOP状态下，把速度值设为0
        if(motor->stop_flag == MOTOR_STOP)
        {
            RSMotorSetCoefficient(motor, 0, 0); // 设置电机系数
            RSMotorSetTorque(motor, 0); // 设置转矩为0
        }
        
        if (DaemonIsOnline(motor->daemon) == 0)
        {
            RSMotorWriteParam(motor, RS_RUN_MODE, 0);   // 运控模式
            // RSMotorConfig(motor, RS_ENABLE); // 使能电机
            motor->if_cmd = 0;
            cmd_delay_time[i] = 0;
            continue;
        }
        else if (motor->if_cmd == 0) // 电机在线后,如果没有命令,需要延时发送使能命令
        {
            RSMotorWriteParam(motor, RS_RUN_MODE, 0);   // 运控模式
            if(cmd_delay_time[i] ++ > 500)
            {
                // RSMotorWriteParam(motor, RS_RUN_MODE, 0); // 运控模式
                RSMotorConfig(motor, RS_ENABLE); // 使能电机
                motor->if_cmd = 1; // 设置标志位,防止重复发送
            }
            continue;
        }

        
        // RSMotorConfig(motor, RS_SET_ZERO_POSITION); // 设置零位
        // RSMotorWriteParam(motor, RS_RUN_MODE, 5);
        // RSMotorConfig(motor, RS_ENABLE);
        // RSMotorWriteParam(motor, RS_SPEED_LIMIT, 3.0f);//motor->send_mailbox.speed_des);
        // RSMotorWriteParam(motor, RS_ANGLE_SET, motor->send_mailbox.angle_des);
        if (motor->if_cmd)
            RSMotorControlMIT(motor);
    }
}

