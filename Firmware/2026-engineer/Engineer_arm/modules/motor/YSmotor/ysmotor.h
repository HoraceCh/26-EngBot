#ifndef YSMOTOR_H
#define YSMOTOR_H
#include <stdint.h>
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "bsp_usart.h"

#define YS_MOTOR_CNT 4

#pragma pack(1)
/* 电机模式枚举 */
typedef enum
{
    M8010_MODE_LOCK = 0,
    M8010_MODE_FOC,
    M8010_MODE_CALIENCODER,
    
}YS_Motor_Mode_e;

/**
 * @brief 电机模式控制信息
 * 
 */
typedef struct
{
    uint8_t id     :4;      // 电机ID：0-14 ，15表示向所有电机广播数据(此时无返回)
    uint8_t mode   :3;      // 工作模式：0:锁定  1:FOC控制  2:编码器校准 3:保留
    uint8_t reserve:1;      // 保留位

} YS_Motor_Mode_t;          // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 * 
 */
typedef struct
{
    int16_t tor_des;        // 期望关节输出扭矩 (N.m)      q8
    int16_t spd_des;        // 期望关节输出速度 (rad/s)    q8
    int32_t angle_des;      // 期望关节输出位置 (rad)      q15
    int16_t k_pos;          // 期望关节刚度系数 (-1.0~1.0) q15
    int16_t k_spd;          // 期望关节阻尼系数 (-1.0~1.0) q15
    
} YS_Motor_Cmd_t;           // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 * 
 */
typedef struct
{
    int16_t  torque;        // 实际关节输出扭矩 (N.m)      q8
    int16_t  speed;         // 实际关节输出速度 (rad/s)    q8
    int32_t  angle;         // 实际关节输出位置 (rad)      q15
    int8_t   temp;          // 电机温度：-128~127℃
    uint8_t  MError :3;     // 电机错误标识：0.正常 1.过热 2.过流 4.编码器故障 5-7.保留
    uint16_t force  :12;    // 足端气压传感器数据 12bit(0~4096)
    uint8_t  none   :1;     // 保留位

} YS_Motor_Fbk_t;           // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 * 
 */
typedef struct
{
    uint8_t head[2];            // 包头            2Byte
    YS_Motor_Mode_t mode;       // 电机控制模式     1Byte
    YS_Motor_Cmd_t cmd;         // 电机期望数据     12Byte
    uint16_t CRC16;             // CRC校验          2Byte

} YS_Motor_ControlData_t;       //17Byte

/**
 * @brief 电机反馈数据包格式
 * 
 */
typedef struct
{
    uint8_t head[2];            // 包头            2Byte
    YS_Motor_Mode_t mode;       // 电机控制模式     1Byte
    YS_Motor_Fbk_t fbk;         // 电机期望数据     11Byte
    uint16_t CRC16;             // CRC校验          2Byte

} YS_Motor_RecvData_t;          // 16Byte

#pragma pack()


/**
 * @brief M8010电机指令发送结构体
 * 
 */
typedef struct 
{
    unsigned short id;              // 电机ID，15代表全部电机
    unsigned short mode;            // 0:空闲  1:FOC控制  2:电机标定
    float torque;                   // 期望电机输出力矩(电机本身的力矩)(Nm)
    float speed;                    // 期望电机速度(电机本身的速度)(rad/s)
    float angle;                    // 期望电机位置(rad)
    float K_P;                      // 关节刚度系数(0-25.599)
    float K_W;                      // 关节速度系数(0-25.599)

    YS_Motor_ControlData_t motor_send_data;  // 发送数据帧

} YS_Motor_Send_s;

/**
 * @brief M8010电机数据接收结构体
 * 
 */
typedef struct 
{
    uint16_t rxlen;                 // 从串口接收到的字节长度
    uint8_t motor_id;               // 电机ID
    uint8_t mode;                   // 0:空闲  1:FOC控制  2:电机标定
    int temp;                       // 温度
    int MError;           			// 错误码
    float torque;                   // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float speed;                    // 当前实际电机速度(电机本身的速度)(rad/s)
    float angle;                    // 当前电机位置(rad)
    int correct;                    // 接收数据是否完整(1完整,0不完整)
    int footForce;                  // 足端力传感器原始数据

    YS_Motor_RecvData_t motor_recv_data;     // 电机接收数据帧

} YS_Motor_Measure_s;


/**
 * @brief M8010电机实例
 * 
 */
typedef struct 
{
    YS_Motor_Measure_s motor_measure;
    YS_Motor_Send_s motor_send_mailbox;
    Motor_Control_Setting_s motor_settings;
    Motor_Controller_s motor_controller;        // 电机控制器
    Motor_Working_Type_e stop_flag;
    USARTInstance *motor_rs485_ins;             //485总线电机
    DaemonInstance *motor_daemon;
    uint32_t lost_cnt;

    uint8_t debug;
    
} YSMotorInstance;

/**
 * @brief 停止电机,之后电机不会响应任何指令
 *
 * @param motor
 */
void YSMotorStop(YSMotorInstance *motor);

/**
 * @brief 启动电机
 *
 * @param motor
 */
void YSMotorEnable(YSMotorInstance *motor);

/**
 * @brief 设置参考值
 * @attention 注意此函数设定的ref是最外层闭环的输入,若要设定内层闭环的值请通过前馈数据指针设置
 *
 * @param motor 要设置的电机
 * @param ref 设定值
 */
void YSMotorSetRef(YSMotorInstance *motor, float ref);

/**
 * @brief M8010电机初始化
 *
 * @param config 电机初始化配置
 * @return YSMotorInstance* 注册电机实例指针
 */
YSMotorInstance *YSMotorInit(RS485_Motor_Init_Config_s *config);

/**
 * @brief 为所有8010电机计算pid/反转/模式控制,并发送数据帧
 *
 */
//void YSMotorControl();

/**
 * @brief 电机参数配置
 *
 * @param *motor 电机实例
 */
//void YSMotorConfig(YSMotorInstance *motor ,uint8_t _id, uint8_t _mode, float _torque, float _angle, float _K_P, float _K_W);

/**
 * @brief 电机参数设置
 *
 * @param *motor 电机实例
 */
void YSMotorSetTorque(YSMotorInstance *motor, float _torque);
void YSMotorSetSpeed(YSMotorInstance *motor, float _speed);
void YSMotorSetAngle(YSMotorInstance *motor, float _angle);
void YSMotorSetMode(YSMotorInstance *motor, YS_Motor_Mode_e _mode);
void YSMotorSetCoefficient(YSMotorInstance *motor, float _K_P, float _K_W);
void YSMotorControlInit();


#endif //YSMOTOR_H