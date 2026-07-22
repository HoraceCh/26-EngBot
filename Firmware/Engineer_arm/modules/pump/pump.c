/**
 * @file pump.c
 * @author lh
 * @brief  气泵驱动
 * @version beta
 * @date 2025-3-22
 * @todo 
 * 
 */
#include "pump.h"
#include "gpio.h"

static uint8_t pump_state_ = 0; 

/**
 * @brief 开启气泵
 * @param id 选择要开启气泵的编号
 */
void pump_run(uint8_t id)
{
    switch (id)
    {
    case 1:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_SET);
        pump_state_ |= 0b0001;
        pump_state_ &= ~0b1000;
        break;
    case 2:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_SET);
        pump_state_ |= 0b0010;
        pump_state_ &= ~0b1000;
        break;
    case 3:
        HAL_GPIO_WritePin(PUMP3_GPIO_Port, PUMP3_Pin, GPIO_PIN_SET);
        pump_state_ |= 0b0100;
        pump_state_ &= ~0b1000;
        break;
    default:
        break;
    }
}


/**
 * @brief 关闭气泵
 * @param id 选择要关闭气泵的编号
 */
void pump_stop(uint8_t id)
{
    switch (id)
    {
    case 1:
        HAL_GPIO_WritePin(PUMP1_GPIO_Port, PUMP1_Pin, GPIO_PIN_RESET);
        pump_state_ &= ~0b1001;
        break;
    case 2:
        HAL_GPIO_WritePin(PUMP2_GPIO_Port, PUMP2_Pin, GPIO_PIN_RESET);
        pump_state_ &= ~0b1010;
        break;
    case 3:
        HAL_GPIO_WritePin(PUMP3_GPIO_Port, PUMP3_Pin, GPIO_PIN_RESET);
        pump_state_ &= ~0b1100;
        break;
    default:
        break;
    }
}

/**
 * @brief 返回气泵状态变量指针
 * 
 */
uint8_t* get_pump_state_ptr(void)
{
    return &pump_state_;
}