/**
 * @file auto_action.c/h
 * @author lihao
 * @brief  自动动作,结合有限状态机和三次插值规划
 * @version 1.0
 * @date 2026-04-29
 *
 * @copyright Copyright (c) 2026 SPR
 *
 */
#include "fsm.h"
#include "auto_action.h"
#include <stddef.h>
#include "pump.h"
#include <string.h>


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
static void grab_init(void);
static void grab_run(void);
static void store_init(void);
static void store_run(void);
static void climb_init(void);
static void climb_run(void);

/* 变量声明 */
static AutoActionInstance auto_action; 		// 自动动作实例
static FSM_t fsm;
static uint16_t time = 0;

static TrajectorySequence seq;
static CubicTrajectory* traj;
	
static const WayPoint way_point[28] = {
		// 复位点
    {{0.0f, 0.91f, 2.48f, 0.0f, 0.0f, 0.0f}},   		// 点0
    {{0.0f, 1.16f, 2.73f, -1.571f, 0.0f, 1.571f}},  // 点1
    {{0.0f, 1.0f, 2.4f, 0.0f, 0.0f, 0.0f}},   			// 点2
		// 能量单元1
    {{0.0f, -0.5f, 1.07f, -1.571f, 0.47f, 1.571f}},   // 点3 预抓取点
    {{0.0f, -0.72f, 0.774f, -1.571f, 0.6f, 1.571f}},   	// 点4 抓取点
		{{0.0f, -0.89f, 0.33f, -1.571f, 0.86f, 1.571f}},   	// 点5 取出点
		// 能量单元2
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点6
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点7
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点8
		// 能量单元3
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点9
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点10
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点11
		// 能量单元4
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点12
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点13
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点14
		// 能量单元5
		{{2.656f, -0.353f, 1.777f, -1.271f, 1.254f, 1.025f}},   // 点15
		{{2.692f, -0.558f, 1.969f, -0.99f, 0.803f, 0.67f}},   // 点16
		{{2.79f, -0.29f, 1.83f, -1.07f, 1.27f, 0.932f}},   // 点17
		// 能量单元6
		{{-2.278f, -0.394f, 1.83f, -1.88f, 1.36f, 2.61f}},   // 点18
		{{-2.25f, -0.591f, 2.015f, -1.906f, 1.071f, 2.737f}},   // 点19
		{{-2.25f, -0.281f, 1.823f, -1.928f, 1.49f, 2.62f}},   // 点20
		// 左存放
		{{2.79f, -0.29f, 1.83f, -1.07f, 1.27f, 0.932f}},   // 点21
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点22
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点23
		// 右存放
		{{-2.28f, -0.31f, 1.86f, -1.967f, 1.496f, 2.646f}},   // 点24
		// {{-2.34f, -0.423f, 1.812f, -1.964f, 1.534f, -0.53f}},   // 点24
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点25
		{{0.0f, 1.25f, 2.7f, 0.0f, 0.0f, 0.0f}},   // 点26

		// 上台阶姿态
		{{0.0f, -1.11f, 0.85f, -1.571f, -0.97f, 1.571f}},   // 点27
};

/*	能量单元编号
				1
		6				2
		5				3
				4
*/	

FsmTable_t AutoActionTable[] = {
	{MANUAL_STATE, 			MANUAL_EVENT_NONE, 		MANUAL_STATE, 		NULL},

	{MANUAL_STATE, 			REPOSITION_EVENT, 		RESET_STATE, 			reset_init},
	{RESET_STATE, 			REPOSITION_EVENT, 		MANUAL_STATE, 		reset_run},	

	{MANUAL_STATE, 			START_GRAB_1, 		GRAB_STATE, 		grab_init},
	{MANUAL_STATE, 			START_GRAB_2, 		GRAB_STATE, 		grab_init},
	{MANUAL_STATE, 			START_GRAB_3, 		GRAB_STATE, 		grab_init},
	{MANUAL_STATE, 			START_GRAB_4, 		GRAB_STATE, 		grab_init},
	{MANUAL_STATE, 			START_GRAB_5, 		GRAB_STATE, 		grab_init},
	{MANUAL_STATE, 			START_GRAB_6, 		GRAB_STATE, 		grab_init},

	{GRAB_STATE, 			START_GRAB_1, 		MANUAL_STATE, 		grab_run},
	{GRAB_STATE, 			START_GRAB_2, 		MANUAL_STATE, 		grab_run},
	{GRAB_STATE, 			START_GRAB_3, 		MANUAL_STATE, 		grab_run},
	{GRAB_STATE, 			START_GRAB_4, 		MANUAL_STATE, 		grab_run},
	{GRAB_STATE, 			START_GRAB_5, 		MANUAL_STATE, 		grab_run},
	{GRAB_STATE, 			START_GRAB_6, 		MANUAL_STATE, 		grab_run},

	{MANUAL_STATE, 			START_STORE_L, 		STORE_STATE, 		store_init},
	{MANUAL_STATE, 			START_STORE_R, 		STORE_STATE, 		store_init},

	{STORE_STATE, 			START_STORE_L, 		MANUAL_STATE, 		store_run},
	{STORE_STATE, 			START_STORE_R, 		MANUAL_STATE, 		store_run},

	{MANUAL_STATE, 			CLIMB_EVENT, 		CLIMB_STATE, 			climb_init},
	{CLIMB_STATE, 			CLIMB_EVENT, 		MANUAL_STATE, 		climb_run},	

	{GRAB_STATE, 		REFORMAT, 		MANUAL_STATE, 			FsmReset},
	{STORE_STATE, 	REFORMAT, 		MANUAL_STATE, 			FsmReset},
	{RESET_STATE, 	REFORMAT, 		MANUAL_STATE, 			FsmReset},
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


static void FsmReset(void)
{
	auto_action.grab_flow = Pre_grab;
	auto_action.store_flow = Pre_store;
	auto_action.event = MANUAL_EVENT_NONE;
	fsm.transfer_flag = 1;
}


/***********************************************************************************************************************
 * @brief 姿态复位
 **********************************************************************************************************************/
static void reset_init(void)
{
	TrajectorySequence_Init(&seq, &way_point[0], 2);
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void reset_run(void)
{
	run_trajectory_sequence(&seq, traj);

	if (seq.finished)
	{
		fsm.transfer_flag = 1;
		auto_action.event = MANUAL_EVENT_NONE;
	}
}

static void grab_init(void)
{
	// auto_action.IsGrabbing = 0;
	// switch (auto_action.event)
	// {
	// 	case START_GRAB_1:
	// 		TrajectorySequence_Init(&seq, &way_point[2], 2);
	// 		break;
	// 	case START_GRAB_2:
	// 		TrajectorySequence_Init(&seq, &way_point[4], 2);
	// 		break;
	// 	case START_GRAB_3:
	// 		TrajectorySequence_Init(&seq, &way_point[6], 2);
	// 		break;
	// 	case START_GRAB_4:
	// 		TrajectorySequence_Init(&seq, &way_point[8], 2);
	// 		break;
	// 	case START_GRAB_5:
	// 		TrajectorySequence_Init(&seq, &way_point[10], 2);	
	// 		break;
	// 	case START_GRAB_6:
	// 		TrajectorySequence_Init(&seq, &way_point[12], 2);	
	// 		break;
	// 	default :
	// 		break;
	// }
	uint8_t idx = 3 * auto_action.event;
	TrajectorySequence_Init(&seq, &way_point[idx], 2);	
	auto_action.IsGrabbing = 0;
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void grab_run(void)
{// 状态机中的状态机 :)
	switch (auto_action.grab_flow)
	{
		case Pre_grab:
			run_trajectory_sequence(&seq, traj);
			// memcpy(auto_action.target_angle, traj->pos, sizeof(traj->pos)); // 应该可以注释掉

			if (seq.finished)
			{
				auto_action.grab_flow = Grabbing;
			}
			break;

		case Grabbing:
			uint8_t idx = 3 * auto_action.event + 2;
			TrajectorySequence_Init(&seq, &way_point[idx], 1);
			if (auto_action.IsGrabbing == 1) // 等待夹爪闭合
				auto_action.grab_flow = Post_grab;
			break;

		case Post_grab:
			run_trajectory_sequence(&seq, traj);

			if (seq.finished)
			{
				auto_action.grab_flow = Pre_grab;
				fsm.transfer_flag = 1;
				auto_action.event = REPOSITION_EVENT; // 待修改为存放动作
			}
			break;
		
		default:
			break;
	}
}

static void store_init(void)
{
	uint8_t idx = 3 * auto_action.event;
	TrajectorySequence_Init(&seq, &way_point[idx], 1);	// 2
	auto_action.IsGrabbing = 1;
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void store_run(void)
{// 状态机中的状态机 :)
	switch (auto_action.store_flow)
	{
		case Pre_store:
			run_trajectory_sequence(&seq, traj);

			if (seq.finished)
			{
				auto_action.store_flow = Releasing;
			}
			break;

		case Releasing:
			uint8_t idx = 3 * auto_action.event + 2;
			// TrajectorySequence_Init(&seq, &way_point[idx], 1);
			// if (auto_action.IsGrabbing == 0) // 等待夹爪松开
			// 	auto_action.grab_flow = Post_store;

			if (auto_action.IsGrabbing == 0) // 等待夹爪松开
			{
				auto_action.store_flow = Pre_store;
				fsm.transfer_flag = 1;
				auto_action.event = REPOSITION_EVENT;
			}
			break;

		case Post_store:
			// run_trajectory_sequence(&seq, traj);

			if (seq.finished)
			{
				auto_action.store_flow = Pre_store;
				fsm.transfer_flag = 1;
				auto_action.event = REPOSITION_EVENT;
			}
			break;
		
		default:
			break;
	}
}

// 爬台阶姿态
static void climb_init(void)
{
	TrajectorySequence_Init(&seq, &way_point[27], 1);
	fsm.transfer_flag = 1; // 初始化结束,状态转移
}

static void climb_run(void)
{
	run_trajectory_sequence(&seq, traj);

	if (seq.finished)
	{
		fsm.transfer_flag = 1;
		auto_action.event = MANUAL_EVENT_NONE;
	}
}