#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];

/***********************************************************************************************************************
 * @brief 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用
 **********************************************************************************************************************/
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/***********************************************************************************************************************
 * @brief 解析电机反馈数据
 **********************************************************************************************************************/
static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id; // 将can实例中保存的id转换成电机实例的指针
    DM_Motor_Measure_s *measure = &(motor->measure);

    DaemonReload(motor->daemon); // 喂狗

    measure->id = rxbuff[0] & 0x07;

    switch (rxbuff[0] >> 4)
    {
        case (0x08):
            measure->error = DM_Motor_Error_OVERVOLTAGE;
            break;
        case (0x09):
            measure->error = DM_Motor_Error_UNDERVOLTAGE;
            break;
        case (0x0A):
            measure->error = DM_Motor_Error_OVERCURRENT;
            break;
        case (0x0B):
            measure->error = DM_Motor_Error_OVERTEMPERATURE_MOS;
            break;
        case (0x0C):
            measure->error = DM_Motor_Error_OVERTEMPERATURE_Rotor;
            break;
        case (0x0D):
            measure->error = DM_Motor_Error_NOCOMMUNICATION;
            break;
        case (0x0E):
            measure->error = DM_Motor_Error_OVERLOAD;
            break;
        default:
            measure->error = DM_Motor_Error_NOERROR;
            break;
    }

    measure->last_angle = measure->angle;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->angle = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->speed = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->temperature_MOS = (float)rxbuff[6];
    measure->temperature_Rotor = (float)rxbuff[7];
}

/***********************************************************************************************************************
 * @brief MIT控制模式
 **********************************************************************************************************************/
static void DMMotorControlMIT(DMMotorInstance *motor)
{
    uint16_t pos_tmp, vel_tmp, Kp_tmp, Kd_tmp, torque_tmp;

    LIMIT_MIN_MAX(motor->motor_send_mailbox.angle_des, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(motor->motor_send_mailbox.speed_des, DM_V_MIN, DM_V_MAX);
    LIMIT_MIN_MAX(motor->motor_send_mailbox.torque_des, DM_T_MIN, DM_T_MAX);
    LIMIT_MIN_MAX(motor->motor_send_mailbox.Kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(motor->motor_send_mailbox.Kd, DM_KD_MIN, DM_KD_MAX);

    pos_tmp = float_to_uint(motor->motor_send_mailbox.angle_des, DM_P_MIN, DM_P_MAX, 16);
    vel_tmp = float_to_uint(motor->motor_send_mailbox.speed_des, DM_V_MIN, DM_V_MAX, 12);
    torque_tmp = float_to_uint(motor->motor_send_mailbox.torque_des, DM_T_MIN, DM_T_MAX, 12);
    Kp_tmp = float_to_uint(motor->motor_send_mailbox.Kp, DM_KP_MIN, DM_KP_MAX, 12);
    Kd_tmp = float_to_uint(motor->motor_send_mailbox.Kd, DM_KD_MIN, DM_KD_MAX, 12);

    motor->motor_can_instace->tx_buff[0] = (uint8_t)(pos_tmp >> 8);
    motor->motor_can_instace->tx_buff[1] = (uint8_t)(pos_tmp);
    motor->motor_can_instace->tx_buff[2] = (uint8_t)(vel_tmp >> 4);
    motor->motor_can_instace->tx_buff[3] = (uint8_t)(((vel_tmp & 0xF) << 4) | (Kp_tmp >> 8));
    motor->motor_can_instace->tx_buff[4] = (uint8_t)(Kp_tmp);
    motor->motor_can_instace->tx_buff[5] = (uint8_t)(Kd_tmp >> 4);
    motor->motor_can_instace->tx_buff[6] = (uint8_t)(((Kd_tmp & 0xF) << 4) | (torque_tmp >> 8));
    motor->motor_can_instace->tx_buff[7] = (uint8_t)(torque_tmp);
    
    motor->motor_can_instace->txconf.StdId = motor->motor_can_instace->tx_id;
}

/***********************************************************************************************************************
 * @brief 位置速度控制模式
 **********************************************************************************************************************/
static void DMMotorControlPosVel(DMMotorInstance *motor)
{
    uint8_t * PBuf, * VBuf;

    LIMIT_MIN_MAX(motor->motor_send_mailbox.angle_des, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(motor->motor_send_mailbox.speed_des, DM_V_MIN, DM_V_MAX);
    PBuf = (uint8_t *)&motor->motor_send_mailbox.angle_des;
    VBuf = (uint8_t *)&motor->motor_send_mailbox.speed_des;

    motor->motor_can_instace->tx_buff[0] = PBuf[0];
    motor->motor_can_instace->tx_buff[1] = PBuf[1];
    motor->motor_can_instace->tx_buff[2] = PBuf[2]; 
    motor->motor_can_instace->tx_buff[3] = PBuf[3];
    motor->motor_can_instace->tx_buff[4] = VBuf[0];
    motor->motor_can_instace->tx_buff[5] = VBuf[1];
    motor->motor_can_instace->tx_buff[6] = VBuf[2];
    motor->motor_can_instace->tx_buff[7] = VBuf[3];

    motor->motor_can_instace->txconf.StdId = 0x100 + motor->motor_can_instace->tx_id;
}

/***********************************************************************************************************************
 * @brief 速度控制模式
 **********************************************************************************************************************/
static void DMMotorControlVel(DMMotorInstance *motor)
{
    uint8_t * VBuf;

    LIMIT_MIN_MAX(motor->motor_send_mailbox.speed_des, DM_V_MIN, DM_V_MAX);
    VBuf = (uint8_t *)&motor->motor_send_mailbox.speed_des;

    motor->motor_can_instace->tx_buff[0] = VBuf[0];
    motor->motor_can_instace->tx_buff[1] = VBuf[1];
    motor->motor_can_instace->tx_buff[2] = VBuf[2]; 
    motor->motor_can_instace->tx_buff[3] = VBuf[3];
    motor->motor_can_instace->tx_buff[4] = 0;
    motor->motor_can_instace->tx_buff[5] = 0;
    motor->motor_can_instace->tx_buff[6] = 0;
    motor->motor_can_instace->tx_buff[7] = 0;

    motor->motor_can_instace->txconf.StdId = 0x200 + motor->motor_can_instace->tx_id;
}

/***********************************************************************************************************************
 * @brief 电机丢失回调函数
 **********************************************************************************************************************/
static void DMMotorLostCallback(void *motor_ptr)
{
    LOGWARNING("[DMMotor] motor lost");
}

/***********************************************************************************************************************
 * @brief 电机初始化
 **********************************************************************************************************************/
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
        
    motor->motor_settings = config->controller_setting_init_config;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    config->can_init_config.rx_id = 0x10 + config->can_init_config.tx_id;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    motor->motor_controller.Kp = config->controller_param_init_config.Kp;
    motor->motor_controller.Kd = config->controller_param_init_config.Kd;

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 5,
    };
    motor->daemon = DaemonRegister(&conf);

    dm_motor_instance[idx++] = motor;
    return motor;
}


/***********************************************************************************************************************
 * @brief 设置电机模式
 * @param cmd: 使能,停止,清除错误等
 * @param motor: 电机实例
 * @note 使用后应该延时0.1      DWT_Delay(0.1);
 **********************************************************************************************************************/
void DMMotorSetMode(DM_Motor_Mode_e cmd, DMMotorInstance *motor)
{
    motor->motor_can_instace->txconf.StdId = 0x100 + motor->motor_can_instace->tx_id; // 发送id

    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
    // DWT_Delay(0.1);
}

/***********************************************************************************************************************
 * @brief 接口函数，设置电机控制参数
 **********************************************************************************************************************/
void DMMotorSetRef(DMMotorInstance *motor, float _ref)
{   // 角度
    motor->motor_controller.pid_ref = _ref;
}
 void DMMotorSetAngle(DMMotorInstance *motor, float _angle)
{   // 角度
    motor->motor_send_mailbox.angle_des = _angle;
}
void DMMotorSetSpeed(DMMotorInstance *motor, float _speed)
{   // 速度
    motor->motor_send_mailbox.speed_des = _speed;
}
void DMMotorSetTorque(DMMotorInstance *motor, float _torque)
{   // 力矩
    motor->motor_send_mailbox.torque_des = _torque;
}
void DMMotorSetCoefficient(DMMotorInstance *motor, float _kp, float _kd)
{   // 比例系数和微分系数
    motor->motor_send_mailbox.Kp = _kp;
    motor->motor_send_mailbox.Kd = _kd;
}
void DMMotorSetCtrlType(DMMotorInstance *motor, DM_Motor_Ctrl_Type_e _type)
{   // 控制类型
    motor->motor_ctrl_type = _type;
}

/***********************************************************************************************************************
 * @brief 电机使能置位
 **********************************************************************************************************************/
void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/***********************************************************************************************************************
 * @brief 电机停止置位
 **********************************************************************************************************************/
void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

/***********************************************************************************************************************
 * @brief 设置电机外层闭环类型
 **********************************************************************************************************************/
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

/***********************************************************************************************************************
 * @brief 电机控制任务
 **********************************************************************************************************************/
__attribute__((noreturn)) void DMMotorTask(void const *argument)
{
    DMMotorInstance *motor;
    Motor_Control_Setting_s *setting;
    uint8_t group = 0;                                      // 电机组号
    float pid_measure, pid_ref;
    DM_Motor_Measure_s *measure;                            // 电机返回的测量值
    Motor_Controller_s *motor_controller;                   // 电机控制器
    Motor_Control_Setting_s *motor_setting;                 // 电机控制参数

    while (1)
    {
        motor = dm_motor_instance[group++];           // 电机实例
        setting = &motor->motor_settings;             // 电机控制参数
        if (group == idx) group = 0;                  // 循环控制

        measure = &motor->measure;                      // 电机返回的测量值
        pid_ref = motor->motor_controller.pid_ref;      // 位置环的目标值,速度环的目标值由位置环输出提供
        motor_controller = &motor->motor_controller;    // 电机控制器
        motor_setting = &motor->motor_settings;         // 电机控制参数
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

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && motor_setting->outer_loop_type& (ANGLE_LOOP | SPEED_LOOP))
        {
            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed; // MOTOR_FEED,对total speed闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // DMMotorSetSpeed(motor, pid_ref); // 将速度环的目标值设为位置环的输出
        DMMotorSetTorque(motor, pid_ref);
        DMMotorSetCoefficient(motor, motor->motor_controller.Kp, motor->motor_controller.Kd);
        DMMotorSetCtrlType(motor, DM_CTRL_TYPE_MIT);

        // STOP状态下，把速度值设为0
        if(motor->stop_flag == MOTOR_STOP)
        {
            DMMotorSetTorque(motor, 0);
            DMMotorSetCoefficient(motor, 0.0f, 0.0f);
        }

        if (DaemonIsOnline(motor->daemon) == 0)
        {
            motor->if_cmd = 0;
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 使能电机
        }
        else if (motor->if_cmd == 0)
        {
            DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 使能电机
            motor->if_cmd = 1;
        }

        switch(motor->motor_ctrl_type)
        {
            case DM_CTRL_TYPE_MIT:
                DMMotorControlMIT(motor);
                break;
            case DM_CTRL_TYPE_PosVel:
                DMMotorControlPosVel(motor);
                break;
            case DM_CTRL_TYPE_Pos:
                DMMotorControlVel(motor);
                break;
            default:
                break;
        }

        if (motor->if_cmd)
            CANTransmit(motor->motor_can_instace, 1);
        osDelay(1); // 500hz发送频率，激发式，反馈也为500hz
    }
}

/***********************************************************************************************************************
 * @brief 电机控制任务初始化
 **********************************************************************************************************************/
void DMMotorControlInit()
{
    if (!idx)
        return;
    
    osThreadDef(dmmotor, DMMotorTask, osPriorityNormal, 0, 128);
    dm_task_handle[0] = osThreadCreate(osThread(dmmotor), NULL);

}