#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
#define DM_KP_MIN  0.0f
#define DM_KP_MAX  500.0f
#define DM_KD_MIN  0.0f
#define DM_KD_MAX  5.0f


/**
 * @brief 达妙电机错误枚举类型
 */
typedef enum 
{
    DM_Motor_Error_NOERROR = 0,
    DM_Motor_Error_OVERVOLTAGE,
    DM_Motor_Error_UNDERVOLTAGE,
    DM_Motor_Error_OVERCURRENT,
    DM_Motor_Error_OVERTEMPERATURE_MOS,
    DM_Motor_Error_OVERTEMPERATURE_Rotor,
    DM_Motor_Error_NOCOMMUNICATION,
    DM_Motor_Error_OVERLOAD,
}DM_Motor_Errors_e;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,       // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,       // 停止
    DM_CMD_ZERO_POSITION = 0xfe,    // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb       // 清除电机过热错误
}DM_Motor_Mode_e;

typedef enum
{
    DM_CTRL_TYPE_PosVel = 0,        // 速度位置控制
    DM_CTRL_TYPE_MIT,               // MIT控制
    DM_CTRL_TYPE_Pos,               // 位置控制
}DM_Motor_Ctrl_Type_e;

/**
 * @brief 达妙电机经过处理数据，均为标准单位
 */
typedef struct 
{
    uint8_t id;
    // uint8_t state;
    float last_angle;
    float angle;
    float speed;
    float torque;
    uint8_t temperature_MOS;
    uint8_t temperature_Rotor;
    DM_Motor_Errors_e error;
    int32_t total_round;
}DM_Motor_Measure_s;

typedef struct
{
    float angle_des;
    float speed_des;
    float torque_des;
    float Kp;
    float Kd;
}DM_Motor_Send_s;

typedef struct 
{
    DM_Motor_Measure_s measure;                 // 电机测量数据
    Motor_Control_Setting_s motor_settings;     // 电机控制设置
    Motor_Controller_s motor_controller;        // 电机控制器
    Motor_Working_Type_e stop_flag;             // 电机停止标志
    CANInstance *motor_can_instace;             // 电机CAN实例
    DaemonInstance* daemon;                     // 电机守护实例
    DM_Motor_Send_s motor_send_mailbox;         // 电机发送数据
    DM_Motor_Ctrl_Type_e motor_ctrl_type;       // 电机控制类型
    uint8_t if_cmd;                             // 电机是否使能
    uint32_t lost_cnt;
    
}DMMotorInstance;


DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);
void DMMotorSetRef(DMMotorInstance *motor, float ref);
void DMMotorSetAngle(DMMotorInstance *motor, float _angle);
void DMMotorSetSpeed(DMMotorInstance *motor, float _speed);
void DMMotorSetTorque(DMMotorInstance *motor, float _torque);
void DMMotorSetCoefficient(DMMotorInstance *motor, float _kp, float _kd);
void DMMotorSetMode(DM_Motor_Mode_e mode, DMMotorInstance *motor);
void DMMotorSetCtrlType(DMMotorInstance *motor, DM_Motor_Ctrl_Type_e _type);
void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);
void DMMotorEnable(DMMotorInstance *motor);
void DMMotorStop(DMMotorInstance *motor);
void DMMotorControlInit();
#endif // !DMMOTOR