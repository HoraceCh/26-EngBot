#include <stdint.h>
#include "user_lib.h"

// 已知连杆数据
#define a2  0.33f
#define d4  0.39f
#define d6  0.0f

extern uint8_t ResolveInverseKinematics(float *target_pos, float *target_angle, float elbow, float direct);
extern void Robotics_FK(float *current_pos, float *current_angle, float *offset_arm_angle, uint8_t chassis_direct);

