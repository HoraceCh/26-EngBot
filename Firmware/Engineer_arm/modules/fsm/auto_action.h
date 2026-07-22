#ifndef __ENGINEER_HEHAVIOR_H__
#define __ENGINEER_HEHAVIOR_H__

#include "main.h"
#include "stdbool.h"
#include "trajectory_planning.h"

#define DEFALT_8010_DELTA_ANGLE		0.00632f
#define DEFALT_3508_DELTA_ANGLE		0.5f
#define DEFALT_4310_DELTA_ANGLE		0.01264f
#define DEFALT_2006_DELTA_ANGLE		0.13f
#define AUTO_increment 				0.00632f  //660/10*0.0002


//4310
#define COMBOS_4310_OFFSET_ANGLE	-0.94834805f

//
#define RATE_8010					6.33f
#define RATE_4310					1.0f

//
#define RISING_DELAY_TIME			1500

//
#define OPEN 						19999
#define CLOSE 						0

#define MAX_POINTS 50

typedef enum{
	/* 手动状态下自动动作启动事件 */
	MANUAL_EVENT_NONE = 0, // 不启动

	START_GRAB_1,
	START_GRAB_2,
	START_GRAB_3,
	START_GRAB_4,
	START_GRAB_5,
	START_GRAB_6,

	START_STORE_L,
	START_STORE_R,

	REPOSITION_EVENT,

	CLIMB_EVENT,

	REFORMAT,		// 重置状态机
}EVENT_ID;

typedef enum{
	/* 手动状态 */
	MANUAL_STATE = 0,
	/* 自动状态 */
	GRAB_STATE,
	STORE_STATE,
	RESET_STATE,
	CLIMB_STATE,
}STATE;

typedef enum
{
	Pre_grab = 0,
	Grabbing,
	Post_grab,
} GrabFlow;

typedef enum
{
	Pre_store = 0,
	Releasing,
	Post_store,
} StoreFlow;

typedef struct {
    float q[DOF];   // 关节角
} WayPoint;

typedef struct {
    WayPoint points[MAX_POINTS];
    uint8_t total_points;

    uint8_t current_segment;   // 当前执行到第几段（i -> i+1）
    uint8_t finished;          // 是否全部完成
} TrajectorySequence;


/**
 * @brief 机械臂自动动作状态机实例
 */
typedef struct 
{
	int event;
	float target_angle[6]; // 目标角度
	uint8_t IsGrabbing;
	GrabFlow grab_flow; 
	StoreFlow store_flow;
} AutoActionInstance;


// void TrajectorySequence_Init(TrajectorySequence *seq,
//                             const WayPoint *points,
//                             uint8_t num);

extern AutoActionInstance *AutoActionInit(CubicTrajectory* traj_);
extern void AutoActionRun(void);

#endif 
