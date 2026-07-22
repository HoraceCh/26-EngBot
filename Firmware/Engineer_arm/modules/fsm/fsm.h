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

#ifndef __FSM_H__
#define __FSM_H__


typedef struct 
{
    int CurState;           // 当前状态
    int event;              // 事件
    int NextState;          // 下一个状态
    void (*eventActFun)();  // 函数指针
} FsmTable_t;

/*状态机类型*/
typedef struct
{
    int curState;           // 当前状态
    FsmTable_t *pFsmTable;    // 状态表
    int size;               // 状态表的项数
    int transfer_flag;      // 状态转换标志
} FSM_t;

/**
 * @brief           注册状态机
 *
 * @param[out]      pFsm:   状态机结构数据指针
 * @param[in]       pTable: 状态表指针
 */
extern void FsmRegist(FSM_t *pFsm, FsmTable_t *pTable);

/**
 * @brief           状态迁移
 *
 * @param[out]      pFsm:  状态机结构数据指针
 * @param[in]       state: 状态
 */
extern void FsmStateTransfer(FSM_t *pFsm, int state);

/**
 * @brief           事件处理
 *
 * @param[out]      pFsm:  状态机结构数据指针
 * @param[in]       event: 事件
 */
extern void FsmEventHandle(FSM_t *pFsm, int event);

#endif
