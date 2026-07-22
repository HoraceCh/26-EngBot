/**
 * @file bsp_tim.h
 * @author lh
 * @brief 仅使用TIM定时器的计数或输入捕获功能，pwm功能由bsp_pwm.h实现
 * @version 0.1
 * @date 2025-02-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "tim.h"

#define TIM_DEVICE_CNT 5 

/**
 * @brief TIM定时器实例
 */
typedef struct tim_ins
{
    TIM_HandleTypeDef *htim;                // TIM外设句柄
    uint32_t channel;                       // 通道
    uint32_t tclk;                          // 时钟频率
    float period;                           // 周期
    void (*callback)(struct tim_ins *);     // 定时器计数溢出中断回调函数
    void *id;                               // 用于标识挂载的模块实例
} TIMInstance;

/**
 * @brief TIM实例初始化配置
 */
typedef struct
{
    TIM_HandleTypeDef *htim;                    // TIM句柄
    uint32_t channel;                           // 通道
    float period;                               // 周期
    void (*callback)(TIMInstance *);     // 定时器计数溢出中断回调函数
    void *id;                                   // 实例ID
} TIM_Init_Config_s;

/**
    * @brief TIM注册
    * 
    * @param config 
    * @return TIMInstance* 
    */
TIMInstance *TIMRegister(TIM_Init_Config_s *config);

/**
 * @brief TIM开始计数，启动中断
 * 
 * @param tim 
 */
void TIMStartIT(TIMInstance *tim);

/**
 * @brief TIM停止计数
 * 
 * @param tim 
 */
void TIMStop(TIMInstance *tim);

/**
 * @brief TIM设置计数值
 * 
 * @param tim 
 */
void TIMSetCounter(TIMInstance *tim, uint16_t counter);

/**
 * @brief TIM获取计数值
 * 
 * @param tim 
 * @return uint32_t 
 */
uint16_t TIMGetCounter(TIMInstance *tim);


#endif // !__BSP_TIM_H