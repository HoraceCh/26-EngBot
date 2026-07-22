#include "power_control.h"
#include "user_lib.h"

static PowerControl_t power_control;

/**
 * @brief          
 * @param[in]      a+b+c
 * @retval         none
 */
static float LIMIT_MAX_MIN(float x, float max, float min)
{
    if (x > max)
    {
        return max;
    }
    else if (x < min)
    {
        return min;
    }
    else
    {
        return x;
    }
}

PowerControl_t *PowerControlInit(PowerControl_Init_Config_t* config_ )
{
  for (uint8_t i = 0; i < 4; i++)
  {
    power_control.chassis_speed_PID[i] = config_->chassis_speed_PID[i];
  }
  power_control.chassis_power = 0.0f;
  power_control.chassis_power_limit = 105.0f;
  power_control.chassis_power_set = 105.0f;
  return &power_control;
}

/**
 * @brief          功率控制
 * @param[in]      
 * @retval         none
 */
void chassis_power_control(PowerControl_t *chassis_power_control)
{
    // ·¢ËÍÆÚÍû×ªËÙÖ®ºó²úÉúµÄ¹¦ÂÊÔ¤²â
    // abcÎª¶þ´Îº¯ÊýÏµÊý
    float a = 0, b = 0, c = 0;
    for (int i = 0; i < 4; i++)
    {
        a += MOTOR_3508_R * chassis_power_control->k[i] * chassis_power_control->k[i] * chassis_power_control->speed_rpm_set[i] * chassis_power_control->speed_rpm_set[i];
        b += chassis_power_control->speed_rpm_set[i] * (2 * MOTOR_3508_R * chassis_power_control->k[i] * chassis_power_control->m[i] +
                                                                      MOTOR_3508_K * chassis_power_control->k[i] * chassis_power_control->speed_rpm_measure[i]);
        c += (MOTOR_3508_R * chassis_power_control->m[i] * chassis_power_control->m[i] +
              MOTOR_3508_K * chassis_power_control->m[i] * chassis_power_control->speed_rpm_measure[i]);
    }
    c += MOTOR_3508_P;
    // ÕâÀïÏàµ±ÓÚk_c=1Ê±£¬Ô¤²â¹¦ÂÊ ,Ò²¾ÍÊÇµ±×ªËÙ²»Ë¥¼õÊ±£¬Ô¤²â¹¦ÂÊ
    chassis_power_control->predict_send_power = LIMIT_MAX_MIN(a + b + c, 1000, -1000); // ¹ýÂË×ªËÙÔ¤²âÒì³£Êý¾Ý
    // ³¬¹¦ÂÊË¥¼õ
    if (chassis_power_control->predict_send_power > chassis_power_control->chassis_power_set)
    {
        // Ä£ÐÍË¥¼õ¼ÆËã
        if (b * b < 4 * (c - chassis_power_control->chassis_power_set) * a) // b·½Ð¡ÓÚ4ac£ºÃ»ÓÐ½â
        {
            chassis_power_control->limit_k = LIMIT_MAX_MIN(-b / (2 * a), 1.0, 0.0);
        }
        else
        {
            float32_t sqrt_result;
            arm_sqrt_f32(b * b - 4 * (c - chassis_power_control->chassis_power_set) * a, &sqrt_result);
			if(a != 0)
            chassis_power_control->limit_k = LIMIT_MAX_MIN((-b + sqrt_result) / 2 / a, 1.0, 0.0);
			else
			chassis_power_control->limit_k = 0.0f;
        }
        // Ë¥¼õ
        for (int i = 0; i < 4; i++)
        {
            chassis_power_control->speed_rpm_set[i] *= chassis_power_control->limit_k; // Ë¥¼õ
        }
    }
}



/*

 */
void calfromPID(PowerControl_t *chassis_power_control)
{
    float k_s_min;
    k_s_min = 1.0;
    for (int i = 0; i < 4; i++)
    {
        PIDInstance *P = chassis_power_control->chassis_speed_PID[i];
        short speed_now = chassis_power_control->speed_rpm_measure[i];
        float PreError = P->Ref - speed_now;
        float SumError_last = P->Err;
        float SumError = SumError_last;

        SumError += (P->Err + P->Last_Err) / 2; // ÌÝÐÎ»ý·ÖÌá¸ß¾«¶È

        if (fabs(SumError) >= 1000.0) // ²»»ý·ÖµÄÇé¿ö
        {
            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * SumError_last) * RM3508_CURRENT_RATIO;
            chassis_power_control->k[i] = P->Kp * RM3508_CURRENT_RATIO;
        }
        else
        {
            chassis_power_control->m[i] = (-P->Kp * speed_now + P->Ki * (SumError_last + 0.5 * (-speed_now + P->Err))) * RM3508_CURRENT_RATIO;
            chassis_power_control->k[i] = (P->Kp + P->Ki / 2) * RM3508_CURRENT_RATIO;
        }
        double k_s = 1.0;
        double K_W = chassis_power_control->k[i] * P->Ref;
        double current = K_W * k_s + chassis_power_control->m[i];
        if (fabs(current) > P->MaxOut * RM3508_CURRENT_RATIO)
        {
            double l = current > 0 ? 1 : -1;
            k_s = LIMIT_MAX_MIN((l * P->MaxOut * RM3508_CURRENT_RATIO - chassis_power_control->m[i]) /K_W,
                                1.0, 0.0);
            if (k_s < k_s_min)
            {
                k_s_min = k_s;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis_power_control->speed_rpm_set[i] *= k_s_min; // ×î´óÖµÔ½½çË¥¼õ
    }
}