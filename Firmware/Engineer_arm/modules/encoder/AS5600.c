/**
 * @file AS5600.c
 * @author lihao
 * @brief  霍尔编码器AS5600封装
 * @version beta
 * @date 2025-01-03
 *
 */

#include "AS5600.h"
#include "general_def.h"
#include <memory.h>
#include <stdlib.h>

#define AS5600_IIC_SIZE 8               //AS5600使用iic发送或接收的数据长度

static uint8_t AS5600_TIMEOUT_FLAG = 0; //AS5600超时标志

/**
 * @brief AS5600寄存器值读取函数,iic读取
 * @note 
 * @param as AS5600实例
 */
static void AS5600TimeoutCallback(TIMInstance *tim)
{
    UNUSED(tim);
    AS5600_TIMEOUT_FLAG = 1;
}

/**
 * @brief AS5600寄存器值读取函数,iic读取
 * @note 
 * @param as AS5600实例
 */
static void AS5600GetDataIIC(AS5600Instance *as)
{
    //直接使用iic内存读取函数
    IICAccessMem(as->iic , AS5600_REG_H_ADDRESS , &as->iic_rx_buffer[0] , AS5600_IIC_SIZE , IIC_READ_MEM , 1);
    IICAccessMem(as->iic , AS5600_REG_L_ADDRESS , &as->iic_rx_buffer[1] , AS5600_IIC_SIZE , IIC_READ_MEM , 1);
}

/**
 * @brief AS5600读取函数,pwm读取
 * @note 读取原理：当AS5600输出高电平时，开始计数，当AS5600输出低电平时，停止计数，计数值即为所需的角度原始值
 * @param as AS5600实例
 */
static uint16_t AS5600GetDataPWM(AS5600Instance *as)
{
    AS5600_TIMEOUT_FLAG = 0;

    TIMSetCounter(as->tim , 0);
    TIMStartIT(as->tim);

    while(GPIORead(as->gpio) && !AS5600_TIMEOUT_FLAG);      // 等待上一次高电平结束

    while(!GPIORead(as->gpio) && !AS5600_TIMEOUT_FLAG);     // 等待低电平结束
    TIMSetCounter(as->tim , 0);                             // 低电平变高电平时计数值置零，从零开始计数

    while(GPIORead(as->gpio) && !AS5600_TIMEOUT_FLAG);      // 等待高电平结束
    TIMStop(as->tim);                                       // 高电平变低电平时停止计数            

    if(AS5600_TIMEOUT_FLAG)
    {   // 超时，返回最大值
        AS5600_TIMEOUT_FLAG = 0;
        return 65535;
    }

    return TIMGetCounter(as->tim);
}



/**
 * @brief AS5600解码函数
 * @note 
 * @param as AS5600实例
 */
void AS5600Decode(AS5600Instance *as)
{
    AS5600Instance *encoder = as;
    uint16_t raw_angle;


    switch (as->work_mode)
    {
        case AS5600_WORK_MODE_IIC:

            AS5600GetDataIIC(encoder);        // 读取寄存器值
            // 拼接数据
            raw_angle = encoder->iic_rx_buffer[0];//high
            raw_angle <<= 8;
            raw_angle |= encoder->iic_rx_buffer[1];//low
            // 解算角度
            encoder->angle = (float)raw_angle / 4096.0f * PI2;
            break;

        case AS5600_WORK_MODE_PWM:

            raw_angle = AS5600GetDataPWM(encoder);        // 读取PWM值
            encoder->angle = (float)raw_angle;

            break;

        default:
            break;
    }
}

AS5600Instance *AS5600Init(AS5600_Work_Mode_e work_mode)
{
    // 分配空间,清除flash防止已经填充的垃圾值
    AS5600Instance *as = (AS5600Instance *)malloc(sizeof(AS5600Instance));
    memset(as , 0 , sizeof(AS5600Instance));

    switch (work_mode)
    {
        case AS5600_WORK_MODE_IIC:
            as->work_mode = AS5600_WORK_MODE_IIC;

            // AS5600霍尔编码器初始化结构体
            AS5600_Init_Config_s as5600_iic_config = {
                .iic_config = {
                    .dev_address = AS5600_DEV_ADDRESS,      // 设备地址
                    .work_mode = IIC_DMA_MODE,              // 工作模式
                    .handle = &hi2c2,                       // 选择的iic外设
                    .id = as,                               // iic实例的父节点位as5600
                    //.callback = AS5600Decode,               // iic接收完成后的回调函数
                },
            };

            // 注册iic
            as->iic = IICRegister(&as5600_iic_config.iic_config);

            break;

        case AS5600_WORK_MODE_PWM:
            as->work_mode = AS5600_WORK_MODE_PWM;

            AS5600_Init_Config_s as5600_pwm_config = {
                .gpio_config = {
                    .GPIOx = GPIOI,
                    .GPIO_Pin = GPIO_PIN_7,
                    .exti_mode = GPIO_EXTI_MODE_NONE,
                    .pin_state = GPIO_PIN_RESET,
                    .gpio_model_callback = NULL,
                    .id = as,
                },
                .tim_config = {
                    .htim = &htim6,
                    .period = 40000,
                    .callback = AS5600TimeoutCallback,
                    .id = as,
                },
            };

            as->gpio = GPIORegister(&as5600_pwm_config.gpio_config);
            as->tim = TIMRegister(&as5600_pwm_config.tim_config);
            break;

        default:
            break;
    }
    
    return as;
}