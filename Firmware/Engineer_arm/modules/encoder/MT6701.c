/**
 * @file MT6701.c
 * @author lihao
 * @brief  磁编码器MT6701封装
 * @version beta
 * @date 2025-03-01
 *
 */

#include "MT6701.h"
#include "general_def.h"
#include <memory.h>
#include <stdlib.h>

MT6701Instance *MT6701Init()
{
    MT6701Instance *instance = (MT6701Instance *)malloc(sizeof(MT6701Instance));
    memset(instance, 0, sizeof(MT6701Instance));

    MT6701_Init_Config_s config = {
        .iic_config ={
            .handle = &hi2c2,
            .dev_address = MT6701_DEV_ADDRESS,
            .callback = NULL,
            .id = instance,
            .work_mode = IIC_BLOCK_MODE,
        },
    };

    instance->iic = IICRegister(&config.iic_config);

    return instance;
}

void MT6701GetData(MT6701Instance *instance)
{
    IICAccessMem(instance->iic, MT6701_REG_H_ADDRESS, &instance->rx_buffer[0], 1, IIC_READ_MEM, 1);
    IICAccessMem(instance->iic, MT6701_REG_L_ADDRESS, &instance->rx_buffer[1], 1, IIC_READ_MEM, 1);

    instance->angle = (float)(((instance->rx_buffer[0] & 0x00ff) << 6) | (instance->rx_buffer[1] >> 2)) * PI2 / 16384.0f ;
}