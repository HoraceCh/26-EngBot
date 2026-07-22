/**
 * @file host_communication.c
 * @author lh
 * @brief  上位机通信
 * @version beta
 * @date 2025-2-10
 * @todo 
 */

#include "host_communication.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"
#include "bsp_usb.h"

static uint8_t *vis_recv_buff; //接收上位机的原始数据指针
static Host_Recv_s recv_data;  //解码后的数据指针
static Host_Send_s send_data;
static DaemonInstance *host_daemon_instance;


/**
 * @brief 离线回调函数,将在daemon.c中被daemon task调用
 * @attention 由于HAL库的设计问题,串口开启DMA接收之后同时发送有概率出现__HAL_LOCK()导致的死锁,使得无法
 *            进入接收中断.通过daemon判断数据更新,重新调用服务启动函数以解决此问题.
 *
 * @param id vision_usart_instance的地址,此处没用.
 */
static void HostOfflineCallback(void *id)
{
// #ifdef VISION_USE_UART 
//     USARTServiceInit(vision_usart_instance);
// #endif // !VISION_USE_UART
    LOGWARNING("[vision] vision offline, restart communication.");
}


static void DecodeHost(uint16_t recv_len)
{

}

Host_Recv_s *HostInit()
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeHost};
    vis_recv_buff = USBInit(conf);

    // 为master process注册daemon,用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = HostOfflineCallback, // 离线时调用的回调函数,会重启串口接收
        .owner_id = NULL,
        .reload_count = 5, // 50ms
    };
    host_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}


void HostSend()
{

}


