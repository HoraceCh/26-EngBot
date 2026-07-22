#ifndef __HOST_COMMUNICATION_H__
#define __HOST_COMMUNICATION_H__

#include "main.h"

#define DATA_LEN 27 // 上位机发送数据长度,单位byte

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
typedef struct 
{
    float feed_dt;
    uint32_t feed_dwt_cnt;
    /* data */
    float cmd_arm_angle[6];

    uint8_t IsUpdated;
}Host_Recv_s;

typedef struct
{
    uint8_t header;
    /* data */
    float current_arm_angle[6];
    uint16_t CRC16;	//CRC16整包校验
}Host_Send_s;
#pragma pack()


/**
 * @brief 调用此函数初始化和上位机的USB串口通信
 *
 * @param 
 */
Host_Recv_s *HostInit(Host_Send_s *send_data_ptr);

/**
 * @brief 向上位机发送数据
 *
 */
void HostSend();

uint8_t HostIsUpdate();
void HostSetUpdate();

#endif //__HOST_COMMUNICATION_H__