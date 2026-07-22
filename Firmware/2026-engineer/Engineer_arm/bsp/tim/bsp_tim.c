#include "bsp_tim.h"
#include <memory.h>
#include <stdlib.h>

static uint8_t idx;
static TIMInstance *tim_instance[TIM_DEVICE_CNT] = {NULL}; // 所有的tim instance保存于此,用于callback时判断中断来源

// 设置定时器时钟源频率
// tim2~7,12~14:APB1  tim1,8~11:APB2
static uint32_t TIMSelectTclk(TIM_HandleTypeDef *htim )
{
    uintptr_t tclk_temp  = ((uintptr_t)((htim)->Instance));
    if (
            (tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) &&
            (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL)))
    {
        return (HAL_RCC_GetPCLK1Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    else if (
            ((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
            ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) &&
             (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL))))
    {
        return (HAL_RCC_GetPCLK2Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    return 0;
}

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    {
        if (tim_instance[i]->htim == htim)
        {
            if(tim_instance[i]->callback != NULL)
                tim_instance[i]->callback(tim_instance[i]);
        }
        return;
    }
}


TIMInstance *TIMRegister(TIM_Init_Config_s *config)
{
    TIMInstance *tim = (TIMInstance *)malloc(sizeof(TIMInstance));
    memset(tim, 0, sizeof(TIMInstance));

    tim->htim = config->htim;
    tim->channel = config->channel;
    tim->period = config->period;
    tim->id = config->id;
    tim->callback = config->callback;
    tim->tclk = TIMSelectTclk(tim->htim);

    tim_instance[idx++] = tim;
    return tim;
}

void TIMStartIT(TIMInstance *tim)
{
    HAL_TIM_Base_Start_IT(tim->htim);
}

void TIMStop(TIMInstance *tim)
{
    HAL_TIM_Base_Stop_IT(tim->htim);
}

void TIMSetCounter(TIMInstance *tim, uint16_t counter)
{
    __HAL_TIM_SET_COUNTER(tim->htim, counter);
}

uint16_t TIMGetCounter(TIMInstance *tim)
{
    return __HAL_TIM_GET_COUNTER(tim->htim);
}
    