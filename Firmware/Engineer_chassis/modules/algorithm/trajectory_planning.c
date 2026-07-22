/**
 ******************************************************************************
 * @file	 trajectory_planning.c
 * @author  li hao
 * @version V1.0
 * @date    2026/4/10
 * @brief
 */
#include "trajectory_planning.h"
#include <arm_math.h>
#include "bsp_dwt.h"

float compute_segment_time(float *q1, float *q2)
{
    float max_diff = 0.0f;

    for (int i = 0; i < DOF; i++)
    {
        float diff = fabs(q2[i] - q1[i]);
        if (diff > max_diff)
            max_diff = diff;
    }

    float vmax = 4.0f; // 最大关节速度（rad/s）
    return max_diff / vmax + 0.1f;  // 留一点缓冲
}

/**
 * @brief  初始化三次多项式轨迹规划
 *
 * @param traj: 轨迹规划参数结构体
 * @param p0: 初始位置
 * @param pf: 末端位置
 * @param v0: 初始速度
 * @param vf: 末端速度
 * @param T: 运动总时间
 */
void CubicTrajectory_Init(CubicTrajectory *traj,
                          float* p0, float* pf,
                          float T)
{
    traj->T  = T;
    traj->t  = 0.0f;

    for (uint8_t i = 0; i < DOF; i++)
    {
        // 中心差分计算末端速度，保证连续性
        traj->p_prev[i] = traj->p_curr[i];
        traj->p_curr[i] = p0[i];   // traj->p_next[i];
        traj->p_next[i] = pf[i];
        traj->v0[i] = traj->vel[i];//(traj->p_next - traj->p_prev) / (2.0f * T);
        traj->vf[i] = traj->is_last_point ? 0.0f : ((traj->p_next[i] - traj->p_curr[i]) / T);//traj->v0;

        // 计算多项式系数 
        traj->a_0[i] = p0[i]; 
        traj->a_1[i] = traj->v0[i]; 
        traj->a_2[i] = (3.0f*(pf[i] - p0[i])/(T*T)) - (2.0f*traj->v0[i] + traj->vf[i])/T; 
        traj->a_3[i] = (-2.0f*(pf[i] - p0[i])/(T*T*T)) + (traj->v0[i] + traj->vf[i])/(T*T);
        
        // 初始化当前输出位置和速度
        traj->pos[i] = p0[i];
    }

    traj->is_last_point = 0; 
    traj->is_active = 1; // 轨迹有效
}

void CubicTrajectory_Update(CubicTrajectory *traj)
{
    traj->dt = DWT_GetDeltaT(&traj->DWT_CNT);

    if (!traj->is_active)
        return;

    traj->t += traj->dt;//dt;

    if (traj->t > traj->T)
    {
        traj->t = traj->T;
        traj->is_active = 0; // 轨迹完成，设置为无效
    }
        
    float t = traj->t;
    float t2 = t*t;

    for (uint8_t i = 0; i < DOF; i++)
    {
        traj->pos[i] = traj->a_0[i]
                + traj->a_1[i]*t
                + traj->a_2[i]*t2
                + traj->a_3[i]*t2*t;

        traj->vel[i] = traj->a_1[i] + 2.0f*traj->a_2[i]*t + 3.0f*traj->a_3[i]*t2;
    }
}