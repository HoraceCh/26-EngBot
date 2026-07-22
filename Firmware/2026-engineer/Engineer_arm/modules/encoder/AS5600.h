/**
 * @file AS5600.h
 * @author lihao
 * @brief  霍尔编码器AS5600封装
 * @version beta
 * @date 2025-01-03
 *
 */

#ifndef __AS5600_H__
#define __AS5600_H__

#include "bsp_iic.h"
#include "stdint.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"

#define AS5600_DEV_ADDRESS   0x36       //AS5600设备地址，（由于仅高7位存储地址，故左移1位）更正->不需左移
#define AS5600_REG_H_ADDRESS 0x0E       //AS5600寄存器地址，高8位
#define AS5600_REG_L_ADDRESS 0x0F       //AS5600寄存器地址，低8位


/**
 * @brief AS5600工作模式,iic模式或者pwm模式
 *
 */
typedef enum
{
    AS5600_WORK_MODE_IIC = 0,   // iic模式
    AS5600_WORK_MODE_PWM,       // pwm模式
} AS5600_Work_Mode_e;


/**
 * @brief AS5600 实例定义
 *
 */
typedef struct 
{
    AS5600_Work_Mode_e work_mode;   // 工作模式
    TIMInstance *tim;               // 定时器实例
    IICInstance *iic;               // iic实例   
    GPIOInstance *gpio;             // gpio实例
    uint8_t iic_rx_buffer[2];       // iic接收缓冲区
    float angle;                    // 编码器角度数据,弧度rad

}AS5600Instance;

/**
 * @brief AS5600 初始化配置结构体
 *
 */
typedef struct 
{
    IIC_Init_Config_s iic_config;      // iic初始化配置
    TIM_Init_Config_s tim_config;      // 定时器初始化配置
    GPIO_Init_Config_s gpio_config;    // gpio初始化配置

} AS5600_Init_Config_s;


/**
 * @brief AS5600解码函数
 * @note 
 *
 * @param as AS5600实例
 */
void AS5600Decode(AS5600Instance *as);


/**
 * @brief AS5600 初始化.
 * @note  
 *
 */
AS5600Instance *AS5600Init(AS5600_Work_Mode_e work_mode);

#endif