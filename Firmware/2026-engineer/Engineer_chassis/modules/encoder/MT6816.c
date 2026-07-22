/**
 * @file MT6816.c
 * @author lihao
 * @brief  磁编码器MT6816封装
 * @version beta
 * @date 2025-02-25
 *
 */

#include "MT6816.h"
#include <memory.h>
#include <stdlib.h>
#include "spi.h"


MT6816Instance *MT6816Init()
{
    MT6816Instance *instance = (MT6816Instance *)malloc(sizeof(MT6816Instance));
    memset(instance, 0, sizeof(MT6816Instance));

    MT6816_Init_Config_s config = {
        .spi_config ={
            .spi_handle = &hspi2,
            .GPIOx = GPIOB,
            .cs_pin = GPIO_PIN_12,
            .spi_work_mode = SPI_BLOCK_MODE,
            .callback = NULL,
            .id = instance,
        },
        .cs_config ={
            .GPIOx = GPIOB,
            .GPIO_Pin = GPIO_PIN_12,
            .pin_state = GPIO_PIN_SET,
            .exti_mode = GPIO_EXTI_MODE_NONE,
            .id = instance,
        },
    };
    
    instance->length = 2;

    instance->tx_buffer[0] = 0x83;
    instance->tx_buffer[1] = 0xFF;
    instance->tx_buffer[2] = 0x84;
    instance->tx_buffer[3] = 0xFF;

    instance->rx_buffer[0] = 0;

    instance->cs = GPIORegister(&config.cs_config);
    instance->spi = SPIRegister(&config.spi_config);

    GPIOSet(instance->cs);

    return instance;
}

void MT6816GetData(MT6816Instance *instance)
{
    SPITransRecv(instance->spi, &instance->rx_buffer[0], &instance->tx_buffer[0], instance->length);
    SPITransRecv(instance->spi, &instance->rx_buffer[2], &instance->tx_buffer[2], instance->length);

    /*---测试代码---*/
    // HAL_GPIO_WritePin(instance->cs->GPIOx, instance->cs->GPIO_Pin, GPIO_PIN_RESET);
    // HAL_SPI_TransmitReceive(instance->spi->spi_handle, &instance->tx_buffer[0], &instance->rx_buffer[0], 2, HAL_MAX_DELAY);
    // HAL_GPIO_WritePin(instance->cs->GPIOx, instance->cs->GPIO_Pin, GPIO_PIN_SET);

    // HAL_GPIO_WritePin(instance->cs->GPIOx, instance->cs->GPIO_Pin, GPIO_PIN_RESET);
    // HAL_SPI_TransmitReceive(instance->spi->spi_handle, &instance->tx_buffer[2], &instance->rx_buffer[2], 2, HAL_MAX_DELAY);
    // HAL_GPIO_WritePin(instance->cs->GPIOx, instance->cs->GPIO_Pin, GPIO_PIN_SET);
    /*---测试代码---*/

    instance->angle = (float)(((instance->rx_buffer[1] & 0x00ff) << 6) | (instance->rx_buffer[3] >> 2)) * PI2 / 16384.0f ;
}
