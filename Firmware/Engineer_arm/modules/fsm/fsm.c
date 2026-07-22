/**
 * @file fsm.c/h
 * @author 何清华
 * @brief 有限状态机实现函数
 * @version 1.0
 * @date 2022-03-05
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */

#include "fsm.h"
#include <stddef.h>

/**
 * @brief           状态机注册
 * @param[out]      pFsm:   状态机数据结构体指针
 * @param[in]       pTable: 状态机表指针
 */
 
void FsmRegist(FSM_t *pFsm, FsmTable_t *pTable)
{
    pFsm->pFsmTable = pTable;
}


/**
 * @brief           状态迁移
 * @param[out]      pFsm:   状态机数据结构体指针
 * @param[in]       state:  状态
 */
void FsmStateTransfer(FSM_t *pFsm, int state)
{
    pFsm->transfer_flag = 0;
    pFsm->curState = state;
}

/**
 * @brief           事件处理
 * @param[out]      pFsm:   状态机数据结构体指针
 * @param[in]       event:  事件
 */
void FsmEventHandle(FSM_t *pFsm, int event)
{
    FsmTable_t *pActTable = pFsm->pFsmTable;
    void (*eventActFun)() = NULL;           // 事件动作函数指针初始化为空
    int NextState;
    int CurState = pFsm->curState;
    int g_max_num = pFsm->size;
    int flag = 0;                           // 标识是否满足条件
    int Transfer_flag;
    int i;

    /* 获取当前动作函数 */
    for (i = 0; i < g_max_num; i++)
    {
        // 在当前的状态下，接收到指定的事件后，才获取执行事件动作函数
        if (event == pActTable[i].event && CurState == pActTable[i].CurState)
        {
            flag = 1; // 置位，表示满足条件，已获取新的事件动作函数
            eventActFun = pActTable[i].eventActFun;
            NextState = pActTable[i].NextState;
            break;
        }
    }

    if (flag) // 若获取到了新的事件动作函数
    {
        if (eventActFun)    // 动作执行，判断函数指针是否为空
        {
            eventActFun();
        }

        Transfer_flag = pFsm->transfer_flag; // 若当前状态结束，则在eventActFun()中进行置位

        if (Transfer_flag == 1) // 根据标志位，进行状态转换
        {
            FsmStateTransfer(pFsm, NextState);
        }
    }
}
