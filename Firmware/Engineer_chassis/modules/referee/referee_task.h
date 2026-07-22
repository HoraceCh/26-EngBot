#ifndef REFEREE_H
#define REFEREE_H

#include "rm_referee.h"
#include "robot_def.h"

/**
 * @brief 初始化裁判系统交互任务(UI和多机通信)
 *
 */
referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data);

/**
 * @brief 在referee task之前调用,添加在freertos.c中
 * 
 */
void MyUIInit();

/**
 * @brief 裁判系统交互任务(UI和多机通信)
 *
 */
void UITask();

/**
 * @brief 重置UI绘制数据
 * @note 该函数会重置UI绘制数据,包括气泵状态和用户状态等
 *
 * @param _Interactive_data 交互数据指针
 */
extern void UIReset(Referee_Interactive_info_t *_Interactive_data);


#endif // REFEREE_H
