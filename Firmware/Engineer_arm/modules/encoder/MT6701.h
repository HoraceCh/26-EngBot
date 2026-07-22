/**
 * @file MT6701.h
 * @author lihao
 * @brief  磁编码器MT6701封装
 * @version beta
 * @date 2025-03-01
 *
 */


#ifndef __MT6701_H__
#define __MT6701_H__

#include "bsp_iic.h"
#include "stdint.h"

#define MT6701_DEV_ADDRESS   0x06       //AS5600设备地址，（由于仅高7位存储地址，故左移1位）更正->不需左移
#define MT6701_REG_H_ADDRESS 0x03       //AS5600寄存器地址，高8位
#define MT6701_REG_L_ADDRESS 0x04       //AS5600寄存器地址，低8位

/**
 * @brief MT6701 实例定义
 *
 */
typedef struct 
{
    IICInstance *iic;               // iic实例   
    uint8_t rx_buffer[2];       // iic接收缓冲区
    float angle;                    // 编码器角度数据,弧度rad

}MT6701Instance;

/**
 * @brief MT6701 初始化配置结构体
 *
 */
typedef struct 
{
    IIC_Init_Config_s iic_config;      // iic初始化配置

} MT6701_Init_Config_s;


/**
 * @brief MT6701 初始化.
 * @note  
 *
 */
MT6701Instance *MT6701Init();

/**
 * @brief  读取MT6701角度.
 * @note  
 *
 */
void MT6701GetData(MT6701Instance *instance);


#endif