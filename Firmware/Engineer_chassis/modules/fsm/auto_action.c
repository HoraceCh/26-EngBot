/**
 * @file engineer_behaviour.c/h
 * @author lihao
 * @brief  自动动作
 * @version 1.0
 * @date 2026-05-14
 *
 * @copyright Copyright (c) 2026 SPR
 *
 */
#include "fsm.h"
#include "auto_action.h"
#include <stddef.h>
#include "pump.h"
#include <string.h>
#include <dmmotor.h>


void SoftReset(void)
{
	if(0) 
	{
		// __set_FAULTMASK(1); 
		// NVIC_SystemReset(); 
	}
}

/* 函数声明 */
static void FsmReset(void);
static void reset_init(void);
static void reset_run(void);
static void climb_init(void);
static void climb_run(void);


/* 变量声明 */
static AutoActionInstance auto_action; 		// 自动动作实例
static FSM_t fsm;
static uint16_t time = 0;

static TrajectorySequence seq;
static CubicTrajectory* traj;
	
static const WayPoint reset_point = {{0.01f, -0.01f, 0.23f, -0.23f}};   	// 点0 底盘复位点;

static const WayPoint climb_point[3] = {
		{{2.5f, -2.5f, 0.23f, -0.23f}},   	// 点1 准备上台点
    {{5.0f, -5.0f, 0.7f, -0.7f}},   	// 点2 上台阶目标点

};
FsmTable_t AutoActionTable[] = {
	{MANUAL_STATE, 				MANUAL_EVENT_NONE, 		MANUAL_STATE, 					NULL},
	// 底盘复位
	{MANUAL_STATE, 				REPOSITION, 					REPOSITION_STATE, 			reset_init},
	{REPOSITION_STATE, 		REPOSITION, 					MANUAL_STATE, 					reset_run},


	{MANUAL_STATE, 				START_CLIMB, 					CLIMB_STATE, 				climb_init},
	{CLIMB_STATE, 				START_CLIMB, 					MANUAL_STATE, 			climb_run},

	{CLIMB_STATE, 				REFORMAT, 						MANUAL_STATE, 			FsmReset},
	{REPOSITION_STATE, 		REFORMAT, 						MANUAL_STATE, 			FsmReset},
};

/**
 * @brief 	自动动作初始化
 * @retval	自动动作实例指针
 */
AutoActionInstance *AutoActionInit(CubicTrajectory* traj_)
{
	FsmRegist(&fsm, AutoActionTable);
	fsm.size = 47;
	FsmStateTransfer(&fsm, MANUAL_STATE);
	auto_action.event = MANUAL_EVENT_NONE;

	traj = traj_;
	
	return &auto_action;
}


static void TrajectorySequence_Init(TrajectorySequence *seq,
                            const WayPoint *points,
                            uint8_t num)
{
  seq->total_points = num;
  seq->current_segment = 0;
  seq->finished = 0;

  for (uint8_t i = 0; i < num; i++)
  {
      seq->points[i] = points[i];
  }
}

static void run_trajectory_sequence(TrajectorySequence *seq,
                             CubicTrajectory *traj)
{
  // 如果已经完成全部轨迹
  if (seq->finished)
      return;

	// 规划第一个点
	if (seq->current_segment == 0)
	{
		float *q_start = traj->pos; 				// 从当前位置开始规划, 需要在开始序列前更新为当前位置
    float *q_end   = seq->points[0].q; 	// 0号点即第一个要到达的目标点

    float T = compute_segment_time(q_start, q_end);

		if (seq->current_segment == (seq->total_points - 1))
			traj->is_last_point = 1;	// 运行到最后一点,置位

    CubicTrajectory_Init(traj, q_start, q_end, T);
		seq->current_segment++;
	}

  // 判断当前段是否完成
  if (traj->t >= traj->T)
  {
    // 判断是否结束
    if (seq->current_segment >= seq->total_points)
    {
      seq->finished = 1;
      return;
    }

    // 初始化规划下一段轨迹
    float *q_start = traj->pos;
    float *q_end   = seq->points[seq->current_segment].q;

    float T = compute_segment_time(q_start, q_end);

		if (seq->current_segment == (seq->total_points - 1))
			traj->is_last_point = 1;	// 运行到最后一点,置位

    CubicTrajectory_Init(traj, q_start, q_end, T);
		seq->current_segment++;
  }
}


/**
 * @brief 	自动动作处理函数
 *
 */
void AutoActionRun(void)
{
	FsmEventHandle(&fsm, auto_action.event);
	
	SoftReset();
}

/**
 * @brief          
 *
 * @param[out]     none
 * @retval         none
 */
void FsmReset(void)
{
	auto_action.event = MANUAL_EVENT_NONE;
	fsm.transfer_flag = 1;
	auto_action.climb_flow = Pre_climb;
}

// static void FsmReset(void)
// {
// 	auto_action.current_step = STARTING;
// 	auto_action.event = MANUAL_EVENT_NONE;
// 	fsm.transfer_flag = 1;
// 	time = 0;
// }


/***********************************************************************************************************************
 * @brief 姿态复位
 **********************************************************************************************************************/
static void reset_init(void)
{
	TrajectorySequence_Init(&seq, &reset_point, 1);
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void reset_run(void)
{
	run_trajectory_sequence(&seq, traj);

	// memcpy(auto_action.target_angle, traj->pos, sizeof(traj->pos));

	if (seq.finished)
	{
		fsm.transfer_flag = 1;
		auto_action.event = MANUAL_EVENT_NONE;
	}
}



static void climb_init(void)
{
	TrajectorySequence_Init(&seq, climb_point, 2);
	auto_action.start_track = 1;
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void climb_run(void)
{	// 状态机中的状态机 :)
	static uint8_t delay_time = 0;

	switch (auto_action.climb_flow)
	{
		case Pre_climb:
			run_trajectory_sequence(&seq, traj);

			if (seq.finished)
			{	// 检测前摇臂的力矩:在摇臂完全放下,撑起前轮后,若其力矩突然减小至小于某个值,判断为前轮已经登上台阶
				float torque1 = DMMotorGetCurrTorque(0);
				float torque2 = DMMotorGetCurrTorque(1);

				if ((torque1 < 0.9f) && (torque2 > -0.9f))
				{	
					delay_time ++;
				}
				
				if (delay_time > 10)
				{
					delay_time = 0;
					auto_action.climb_flow = Climbbing;
					TrajectorySequence_Init(&seq, &reset_point, 1); //初始化下一阶段目标点
				}
			}
			break;

		case Climbbing:	// 前轮登上台阶后,开始收前摇臂和后腿,(适当加速?)
			run_trajectory_sequence(&seq, traj);

			if (seq.finished)
			{
				if (delay_time < 300)
				{
					delay_time++;
					return;
				}
				auto_action.climb_flow = Pre_climb;
				fsm.transfer_flag = 1;
				auto_action.start_track = 0;  // star trek  :)
				auto_action.event = MANUAL_EVENT_NONE;
				delay_time = 0;
			}// 暂时先进行到这一步
			break;

		case Post_climb:
			run_trajectory_sequence(&seq, traj);
			if (seq.finished)
			{
				auto_action.climb_flow = Pre_climb;
				fsm.transfer_flag = 1;
				// auto_action.start_track = 0;  // star trek  :)
				auto_action.event = MANUAL_EVENT_NONE;
			}
	}
}