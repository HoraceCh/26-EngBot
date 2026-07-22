/**
 * @file rsmotor.h
 * @author lh
 * @brief RS02电机
 * @version 1.0
 * @date 2025-06-26
 *
 */

#ifndef _RSMOTOR_H__
#define _RSMOTOR_H__

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define RS_MOTOR_CNT 6

#define RS_P_MIN -12.57f
#define RS_P_MAX 12.57f
#define RS_V_MIN -44.0f
#define RS_V_MAX 44.0f
#define RS_KP_MIN 0.0f
#define RS_KP_MAX 500.0f
#define RS_KD_MIN 0.0f
#define RS_KD_MAX 5.0f
#define RS_T_MIN -17.0f
#define RS_T_MAX 17.0f

/**
 * @brief 写入参数类型指引
 */
typedef enum
{
    RS_RUN_MODE = 0x7005,
    RS_ANGLE_SET = 0x7016,
    RS_SPEED_LIMIT = 0x7017,
} RSMotor_ParamIndex_e;

/**
 * @brief  电机配置类型
 */
typedef enum
{
    RS_ENABLE = 0x3,
    RS_DISABLE = 0x4,
    RS_SET_ZERO_POSITION = 0x6,
    RS_SET_CANID = 0x7,
} RSMotor_ConfigType_e;

/**
 * @brief RS02使用CAN扩展帧的29位CANID，该结构体为扩展帧29位CANID的组成
 */
typedef struct
{
    uint32_t id:8;          // 目标地址，电机ID
    uint32_t data:16;       // 数据区
    uint32_t mode:5;        // 通信类型
    uint32_t reserved:3;    // 保留位
} RSMotor_ExCanIdInfo_t;

typedef struct 
{
    uint16_t motor_ID:8;        // bit 8~15
    uint16_t undervoltage:1;    // bit 16
    uint16_t overcurrent:1;     // bit 17
    uint16_t overheat:1;        // bit 18
    uint16_t encoder_error:1;   // bit 19
    uint16_t overload:1;        // bit 20
    uint16_t no_calibrated:1;   // bit 21
    uint16_t motor_status:2;    // bit 22~23

    float last_angle;
    float angle;            // 角度，单位rad
    float total_angle;      // 总角度，单位rad
    float speed;            // 角速度，单位rad/s
    float torque;           // 力矩，单位Nm
    float temperature;      // 温度，摄氏度
    int16_t total_round;    // 记录电机转过的圈数,正转加1,反转减1,在计算总角度时使用
    RSMotor_ExCanIdInfo_t rxCanIdEx;
} RSMotor_Measure_s;

typedef struct 
{
    float angle_des;
    float speed_des;
    float torque_des;
    float Kp;
    float Kd;
    RSMotor_ExCanIdInfo_t txCanIdEx;
} RSMotor_Send_s;

typedef struct
{
    Motor_Control_Setting_s motor_settings;     // 电机控制设置
    Motor_Controller_s motor_controller;        // 电机控制器
    Motor_Working_Type_e stop_flag;             // 电机使能标志
    RSMotor_Measure_s measure;
    RSMotor_Send_s send_mailbox;
    CANInstance *can_instace;             // 电机CAN实例
    DaemonInstance* daemon;                     // 电机守护实例
    uint8_t if_cmd;
} RSMotorInstance;

extern void RSMotorSetAngle(RSMotorInstance *motor, float _angle);
extern void RSMotorSetSpeed(RSMotorInstance *motor, float _speed);
extern void RSMotorSetTorque(RSMotorInstance *motor, float _torque);
extern void RSMotorSetCoefficient(RSMotorInstance *motor, float _Kp, float _Kd);
extern void RSMotorSetRef(RSMotorInstance *motor, float ref);
extern void RSMotorEnable(RSMotorInstance *motor);
extern void RSMotorStop(RSMotorInstance *motor);
extern void RSMotorConfig(RSMotorInstance* motor, RSMotor_ConfigType_e _type, ...);
extern RSMotorInstance* RSMotorInit(Motor_Init_Config_s *config);
extern void RSMotorControl();

#endif // _RSMOTOR_H__