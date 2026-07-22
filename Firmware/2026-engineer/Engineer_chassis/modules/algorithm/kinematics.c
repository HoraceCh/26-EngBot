#include "kinematics.h"

// DH table: a, alpha, d, theta
// static float DH[] = {
//     0.0,    0.0,    0.0,    0.0,
//     0.0,    0.0,    0.0,    0.0,
//     a2,     0.0,    0.0,    PI/2,
//     0.0,    PI/2,   d4,     0.0,
//     0.0,    -PI/2,  0.0,    0.0,
//     0.0,    PI/2,   d6,     0.0
// };

static float R03_inv[9], R0n[9], R3n[9]; // 旋转矩阵

static inline float my_abs(float x)
{
    return x >= 0 ? x : -x;
}

/**
 * @brief 将坐标轴角度变换转换为旋转矩阵,并求逆
 * @param theta2 输入的第二个关节角度
 * @param theta3 输入的第三个关节角度
 * @retval 返回一个3x3的旋转矩阵
 * @note 由于是旋转矩阵，所以逆等于转置
 */
static void AngleToRotateMatrixInv(float theta2, float theta3)
{
    float alpha = 0.0f; // 参数均为0，直接赋值
    float theta = theta2 + theta3 + PI/2;

    float cos_theta = mcos(theta), sin_theta = msin(theta);
    float cos_alpha = mcos(alpha), sin_alpha = msin(alpha);

    R03_inv[0] = cos_theta;
    R03_inv[1] = sin_theta * cos_alpha;
    R03_inv[2] = sin_theta * sin_alpha;

    R03_inv[3] = -sin_theta;
    R03_inv[4] = cos_theta * cos_alpha;
    R03_inv[5] = cos_theta * sin_alpha;

    R03_inv[6] = 0.0f;
    R03_inv[7] = -sin_alpha;
    R03_inv[8] = cos_alpha;
}


/** 
 * @brief ZYX欧拉角转旋转矩阵
 * @param alpha: Z轴旋转(yaw)
 * @param beta: Y轴旋转(pitch)
 * @param gamma: X轴旋转(roll)
 */
static void EulerZYXToMatrix(float alpha, float beta, float gamma)
{
    float ca = mcos(alpha), sa = msin(alpha);
    float cb = mcos(beta),  sb = msin(beta);
    float cg = mcos(gamma), sg = msin(gamma);

    R0n[0] = ca * cb;
    R0n[1] = ca * sb * sg - sa * cg;
    R0n[2] = ca * sb * cg + sa * sg;

    R0n[3] = sa * cb;
    R0n[4] = sa * sb * sg + ca * cg;
    R0n[5] = sa * sb * cg - ca * sg;

    R0n[6] = -sb;
    R0n[7] = cb * sg;
    R0n[8] = cb * cg;
}

/**
 * @brief 机械臂逆运动学求解,使用modified-DH法
 * @param target_pos    输入的目标位姿
 * @param target_angle  输出的目标关节角
 * @retval 0-求解失败 1-求解成功
 * @note 先通过xyz坐标值解出d1 q2 q3,再通过α β γ结合d1 q2 q3解出q4 q5 q6
 */
uint8_t ResolveInverseKinematics(float *target_pos, float *target_angle, float elbow, float direct)
{
    static float target_angle_last[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    // if (switch_is_up(rc_data[TEMP].rc.switch_SD) && switch_is_mid(rc_data[TEMP].rc.switch_SB))   // 
    // {
    //     elbow = 1.0f;
    // }
    // else if (switch_is_down(rc_data[TEMP].rc.switch_SD) && switch_is_mid(rc_data[TEMP].rc.switch_SB))
    // {
    //     elbow = -1.0f;
    // }

    // input
    float x     = target_pos[0];    // x
    float y     = target_pos[1];    // y
    float z     = target_pos[2];    // z
    float alpha = target_pos[3];    // α
    float beta  = target_pos[4];    // β
    float gamma = target_pos[5];    // γ
    // output
    float theta2 = 0.0f, theta3 = 0.0f;
    // float theta4[2] = {0.0f, 0.0f}, theta5[2] = {0.0f, 0.0f}, theta6[2] = {0.0f, 0.0f};  // 处理多解情况
    float theta4 = 0.0f, theta5 = 0.0f, theta6 = 0.0f;
    // 逆运动学计算，使用几何法
    // 计算θ3
    float r = x * x + y * y;
    theta3 = elbow * acosf((r - a2 * a2 - d4 * d4) / (2 * a2 * d4)); // acosf值域为0~pi，theta3可取正负，对应两种手型的解，通过elbow来确定
    // 计算θ2
    float sigma = atan2f(y, x);
    float phi = acosf((r + a2 *a2 - d4 * d4) / (2 * a2 * sqrtf(r)));
    
    if (isnan(sigma) || isnan(phi) || isnan(theta3))    // 判断反三角函数是否可解
    {
        // 不可解，更新为上一次的值
        for (uint8_t i = 0; i < 6; i++)
        {
            target_angle[i] = target_angle_last[i];
        }
        return 0;
    }

    theta2 = sigma + ((theta3 < 0) ? 1.0f : -1.0f) * phi; // θ3<0,取正号，否则取负号

    target_angle[0] = z;
    target_angle[1] = theta2;
    target_angle[2] = theta3;
    
    AngleToRotateMatrixInv(theta2, theta3); // 将θ2和θ3转换为旋转矩阵的逆,得到R03_inv
    // 计算θ4 θ5 θ6
    EulerZYXToMatrix(alpha, beta, gamma); // 将α β γ转换为旋转矩阵,得到R0n

    memset(R3n, 0, sizeof(R3n)); // 清空R3n

    // 由R03_inv*R0n计算得到R(2,1) R(3,1) R(1,1) R(2,3) R(2,2)
    for (uint8_t i = 0; i < 3; i++)
    {
        R3n[3] += R03_inv[3 + i] * R0n[i * 3 + 0];
        R3n[6] += R03_inv[6 + i] * R0n[i * 3 + 0];
        R3n[0] += R03_inv[0 + i] * R0n[i * 3 + 0];
        R3n[5] += R03_inv[3 + i] * R0n[i * 3 + 2];
        R3n[4] += R03_inv[3 + i] * R0n[i * 3 + 1];
    }

    // 计算θ4 θ5 θ6
    // for (uint8_t i = 0; i < 2; i++)
    // {
    //     theta5[i] = ((i == 0) ? 1.0f : -1.0f) * acosf(-R3n[3]); // θ5 = ±acos(-R(2,1))
    //     float sin_theta5 = msin(theta5[i]);
    //     theta4[i] = atan2f(R3n[6] / sin_theta5, R3n[0] / sin_theta5); // θ4 = atan2(R(3,1)/sin(θ5), R(1,1)/sin(θ5))
    //     theta6[i] = atan2f(-R3n[5] / sin_theta5, R3n[4] / sin_theta5); // θ6 = atan2(R(2,3)/sin(θ5), R(2,2)/sin(θ5))
    // }


    // theta5 = direct * acosf(-R3n[3]); // θ5 = ±acos(-R(2,1))
    // float sin_theta5 = msin(theta5);
    // theta4 = atan2f(R3n[6] / sin_theta5, R3n[0] / sin_theta5); // θ4 = atan2(R(3,1)/sin(θ5), R(1,1)/sin(θ5))
    // theta6 = atan2f(-R3n[5] / sin_theta5, R3n[4] / sin_theta5); // θ6 = atan2(R(2,3)/sin(θ5), R(2,2)/sin(θ5))

    if (alpha - theta2 - theta3 > 0.0f) // 选择θ4 θ5 θ6的解
    {
        theta5 = acosf(-R3n[3]); // θ5 = ±acos(-R(2,1))
        float sin_theta5 = msin(theta5);
        theta4 = atan2f(R3n[6] / sin_theta5, R3n[0] / sin_theta5); // θ4 = atan2(R(3,1)/sin(θ5), R(1,1)/sin(θ5))
        theta6 = atan2f(-R3n[5] / sin_theta5, R3n[4] / sin_theta5); // θ6 = atan2(R(2,3)/sin(θ5), R(2,2)/sin(θ5))
    }
    else
    {
        theta5 = -acosf(-R3n[3]); // θ5 = ±acos(-R(2,1))
        float sin_theta5 = msin(theta5);
        theta4 = atan2f(R3n[6] / sin_theta5, R3n[0] / sin_theta5); // θ4 = atan2(R(3,1)/sin(θ5), R(1,1)/sin(θ5))
        theta6 = atan2f(-R3n[5] / sin_theta5, R3n[4] / sin_theta5); // θ6 = atan2(R(2,3)/sin(θ5), R(2,2)/sin(θ5))    
    }

    // if (isnan(theta4[0]) || isnan(theta4[1]) || isnan(theta5[0]) || 
    //     isnan(theta5[1]) || isnan(theta6[0]) || isnan(theta6[1]))
    if (isnan(theta4) || isnan(theta5) || isnan(theta6))
    {
        // 不可解，更新为上一次的值
        for (uint8_t i = 0; i < 6; i++)
        {
            target_angle[i] = target_angle_last[i];
        }
        return 0;
    }

    // 选择θ4 θ5 θ6的解
    // 计算与上一组解的距离
    // float dist0 = my_abs(theta4[0] - target_angle_last[3])+
    //             my_abs(theta5[0] - target_angle_last[4]) +
    //             my_abs(theta6[0] - target_angle_last[5]);

    // float dist1 = my_abs(theta4[1] - target_angle_last[3])+
    //             my_abs(theta5[1] - target_angle_last[4]) +
    //             my_abs(theta6[1] - target_angle_last[5]);

    // uint8_t idx = (dist0 <= dist1) ? 0 : 1;

    // 选取最接近的解
    // target_angle[3] = theta4[idx];
    // target_angle[4] = theta5[idx];
    // target_angle[5] = theta6[idx];
    // if (alpha - theta2 - theta3 > 0.0f) // 选择θ4 θ5 θ6的解
    // {
    //     target_angle[3] = theta4[0];
    //     target_angle[4] = theta5[0];
    //     target_angle[5] = theta6[0];
    // }
    // else
    // {
    //     target_angle[3] = theta4[1];
    //     target_angle[4] = theta5[1];
    //     target_angle[5] = theta6[1];
    // }

    target_angle[3] = theta4;
    target_angle[4] = theta5;
    target_angle[5] = theta6;

    // 可解，更新上一次的值
    for (uint8_t i = 0; i < 6; i++)
    {
        target_angle_last[i] = target_angle[i];
    }
    return 1;
}

/**
 * @brief 机械臂正运动学
 *
 */
void Robotics_FK(float *current_pos, float *current_angle, float *offset_arm_angle, uint8_t chassis_direct)
{
    float temp[3];

    // input
    float d1 = (current_angle[0] - offset_arm_angle[0]) / 25226.28f;
    float theta2;
    if (chassis_direct)
        theta2 = (current_angle[1] - offset_arm_angle[1] +  19.886f) / (-12.66f);
    else
        theta2 = (current_angle[1] - offset_arm_angle[1]) / (-12.66f);
    float theta3 = -(current_angle[2] - offset_arm_angle[2]);

    // output
    temp[0] = a2 * cosf(theta2) + d4 * cosf(theta2 + theta3);
    temp[1] = a2 * sinf(theta2) + d4 * sinf(theta2 + theta3);
    temp[2] = d1;

    if (isnan(temp[0]) || isnan(temp[1]) || isnan(temp[2])) // 判断三角函数是否可解
    {
        return;
    }
    else
    {
        current_pos[0] = temp[0];
        current_pos[1] = temp[1];
        current_pos[2] = temp[2];
    }
}
