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
#include "crc16.h"
#include "bsp_dwt.h"

static uint8_t *raw_recv_buff; // 接收上位机的原始数据指针
static Host_Recv_s recv_data;  // 解码后的数据
static Host_Send_s *send_data;
static DaemonInstance *host_daemon_instance;

#define HOST_PACKET_HEADER 0xAB

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
    LOGWARNING("[host] host offline, restart communication.");
}


static void DecodeHost(uint16_t recv_len)
{
    if (raw_recv_buff[0] == HOST_PACKET_HEADER)
    {
        memcpy(recv_data.cmd_arm_angle, raw_recv_buff + 1, sizeof(recv_data.cmd_arm_angle));
        DaemonReload(host_daemon_instance);
        recv_data.feed_dt = DWT_GetDeltaT(&recv_data.feed_dwt_cnt);
        recv_data.IsUpdated = 1;
    }
}

Host_Recv_s *HostInit(Host_Send_s *send_data_ptr)
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeHost};
    raw_recv_buff = USBInit(conf);
    send_data = send_data_ptr;
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
    send_data->header = 0xAC; // 包头
    send_data->CRC16 = crc_16((uint8_t *)send_data->current_arm_angle, sizeof(send_data->current_arm_angle));
    USBTransmit((uint8_t *)send_data, sizeof(Host_Send_s));
}

uint8_t HostIsUpdate()
{
    if (recv_data.IsUpdated)
    {
        recv_data.IsUpdated = 0; // 处理完命令后将更新标志位复位，等待下一次上位机发送命令
        return 1;
    }
    return 0;
}

void HostSetUpdate()
{
    recv_data.IsUpdated = 1;
}


