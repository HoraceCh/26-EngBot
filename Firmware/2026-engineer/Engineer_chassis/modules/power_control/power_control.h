#ifndef POWER_CONTROL_H_
#define POWER_CONTROL_H_

#include "controller.h"

#define   MOTOR_3508_R 0.0989f// 
#define   MOTOR_3508_K 0.001709f
#define   MOTOR_3508_P 8.186f// 
#define   RM3508_CURRENT_RATIO 20.0f/16635.0f// 

#define MIN_CAP_VOLTAGE 16.5f// 
#define SUPER_MIN_CAP_VOLTAGE 10.0f// 

typedef struct
{
  PIDInstance *chassis_speed_PID[4];

  float speed_rpm_set[4];
  int16_t speed_rpm_measure[4];

  float SumError_last[4];

  float chassis_power;// 
  float chassis_power_buffer;// 
  float chassis_power_limit;// 
  float chassis_power_set;// 

  float chassis_rpm_double;
  float predict_send_power;// 
  float limit_k;
  float k[4];
  float m[4];
  uint8_t power_management_chassis_output;

  float SumError[4];
  float LastSumError[4];
  float LastError[4];
} PowerControl_t;

typedef struct
{
  PIDInstance *chassis_speed_PID[4];
} PowerControl_Init_Config_t;

PowerControl_t *PowerControlInit(PowerControl_Init_Config_t * config_ );

void calfromPID(PowerControl_t *chassis_power_control);

void chassis_power_control(PowerControl_t *chassis_power_control);


#endif