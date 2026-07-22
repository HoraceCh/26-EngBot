#ifndef CUSTOM_CONTROLLER_H__
#define CUSTOM_CONTROLLER_H__

#include <stdint.h>
#include "main.h"
#include "usart.h"

typedef struct 
{
    /* data */
    // uint8_t xyz[3];
    // uint8_t rpy[3];

    float position[6];

} custom_controller_t;

extern custom_controller_t *CustomControllerInit(UART_HandleTypeDef *usart_handle);

extern uint8_t CustomControllerIsOnline();

#endif
