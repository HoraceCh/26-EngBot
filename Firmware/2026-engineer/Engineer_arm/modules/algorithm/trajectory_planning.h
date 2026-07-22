/**
 ******************************************************************************
 * @file	 trajectory_planning.h
 * @author  li hao
 * @version V1.0
 * @date    2026/4/10
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _TRAJECTORY_PLANNING_H
#define _TRAJECTORY_PLANNING_H

#include "main.h"

#define DOF 6 // 需要进行规划的自由度

/**
 * @brief  三次多项式轨迹规划结构体
 * @param  T: 运动总时间，插值周期
 * @param  t: 当前插值分段的运动时间
 * @param  a_0, a_1, a_2, a_3: 多项式系数
 * @param  pos: 当前输出位置 p_des
 * @param  vel: 当前输出速度 v_des
 */
typedef struct {
    uint8_t is_active; // 轨迹是否有效
    uint8_t is_last_point; // 是否为最后一个规划点

    float p_prev[DOF];   // 上次更新的目标位置
    float p_curr[DOF];   // 当前目标位置
    float p_next[DOF];   // 下次目标位置

    float dt;
    uint32_t DWT_CNT;

    float T;        // 运动总时间，插值周期
    float t;        // 当前插值分段的运动时间

    float v0[DOF], vf[DOF];   // 对应速度

    float a_0[DOF], a_1[DOF], a_2[DOF], a_3[DOF]; // 多项式系数

    float pos[DOF];  // 当前输出位置 p_des
    float vel[DOF];  // 当前速度 v_des，不输出
} CubicTrajectory;

void CubicTrajectory_Init(CubicTrajectory *traj,
                          float* p0, float* pf,
                          float T);

void CubicTrajectory_Update(CubicTrajectory *traj);

float compute_segment_time(float *q1, float *q2);

#endif // _TRAJECTORY_PLANNING_H