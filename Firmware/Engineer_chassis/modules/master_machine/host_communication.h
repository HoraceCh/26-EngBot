#ifndef __HOST_COMMUNICATION_H__
#define __HOST_COMMUNICATION_H__

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
typedef struct 
{
    /* data */

}Host_Recv_s;

typedef struct
{
    /* data */
}Host_Send_s;
#pragma pack()


/**
 * @brief 调用此函数初始化和上位机的USB串口通信
 *
 * @param 
 */
Host_Recv_s *HostInit();

/**
 * @brief 向上位机发送数据
 *
 */
void HostSend();


#endif //__HOST_COMMUNICATION_H__