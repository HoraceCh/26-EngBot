/**
 * @file custom_controller.c
 * @author lh
 * @brief  自定义控制器
 * @version beta
 * @date 2025-3-16
 * @todo 
 * 
 */
#include "custom_controller.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "string.h"

#define CUSTOM_CONTROLLER_FRAME_SIZE 26u

static custom_controller_t controller_data;     // 自定义控制器数据
static uint8_t cc_init_flag;                    // 自定义控制器初始化标志位

// 自定义控制器拥有的串口实例,因为是单例,所以这里只有一个,就不封装了
static USARTInstance *cc_usart_instance;
static DaemonInstance *cc_daemon_instance;

/**
 * @brief 自定义控制器数据解析
 *
 * @param sbus_buf 接收buffer
 */
void CustomControllerDecode(USARTInstance *_instance)
{
    DaemonReload(cc_daemon_instance);         // 先喂狗

    if (cc_usart_instance->recv_buff[0] == 0xFF && cc_usart_instance->recv_buff[25] == 0xFF)
    {
        memcpy(&controller_data.position, cc_usart_instance->recv_buff + 1, 24);
        // controller_data.xyzrpy[0] = cc_usart_instance->recv_buff[0];
        // controller_data.xyzrpy[1] = cc_usart_instance->recv_buff[1];
        // controller_data->xyzrpy[2] = cc_usart_instance->recv_buff[2];
        // controller_data->xyzrpy[3] = cc_usart_instance->recv_buff[3];
        // controller_data->xyzrpy[4] = cc_usart_instance->recv_buff[4];
    }
}

/**
 * @brief 自定义控制器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
void CustomControllerLostCallback()
{
    memset(&controller_data, 0, sizeof(controller_data)); // 自定义控制器数据
    USARTServiceInit(cc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[cc] custom controller lost");
}

custom_controller_t *CustomControllerInit(UART_HandleTypeDef *usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = CustomControllerDecode;
    conf.usart_handle = usart_handle;
    conf.recv_buff_size = CUSTOM_CONTROLLER_FRAME_SIZE;
    cc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = CustomControllerLostCallback,
        .owner_id = NULL, // 只有1个自定义控制器,不需要owner_id
    };
    cc_daemon_instance = DaemonRegister(&daemon_conf);

    cc_init_flag = 1;
    return &controller_data;
}

/**
 * @brief 自定义控制器在线检测接口
 * @return uint8_t 1:在线 0:离线
 */
uint8_t CustomControllerIsOnline()
{
    if (cc_init_flag)
        return DaemonIsOnline(cc_daemon_instance);
    return 0;
}