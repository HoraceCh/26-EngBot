/**
 * @file MT6816.h
 * @author lihao
 * @brief  磁编码器MT6816封装
 * @version beta
 * @date 2025-02-25
 *
 */

#ifndef __MT6816_H__
#define __MT6816_H__

#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "general_def.h"
#include "stdint.h"


typedef struct
{
    SPIInstance *spi;
    GPIOInstance *cs;
    uint8_t rx_buffer[4];
    uint8_t tx_buffer[4];
    uint8_t length;
    float angle;

} MT6816Instance;

typedef struct
{
    SPI_Init_Config_s spi_config;
    GPIO_Init_Config_s cs_config;
} MT6816_Init_Config_s;

/**
 * @brief  初始化MT6816
 * @param  *config: MT6816初始化配置
 * @retval *MT6816: MT6816实例
 */
MT6816Instance *MT6816Init();

/**
 * @brief  读取MT6816角度
 * @param  *MT6816: MT6816实例
 */
void MT6816GetData(MT6816Instance *MT6816);


#endif
