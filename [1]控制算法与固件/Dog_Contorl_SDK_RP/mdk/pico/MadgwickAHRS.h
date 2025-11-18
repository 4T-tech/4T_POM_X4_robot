// MadgwickAHRS.h (Optimized for RP2040)

#ifndef __MADGWICKAHRS_H__
#define __MADGWICKAHRS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "leg_ik.h"
#include "reg_protocol.h"

//----------------------------------------------------------------------------------------------------
// 变量声明
extern volatile float beta; // 算法增益
extern volatile float q0, q1, q2, q3; // 姿态四元数

//---------------------------------------------------------------------------------------------------
// 函数声明

/**
 * @brief 使用6轴数据（加速度、陀螺仪）和时间差来更新姿态
 * @param gx, gy, gz  陀螺仪数据 (rad/s)
 * @param ax, ay, az  加速度计数据 (g)
 * @param dt          距离上次调用的时间差 (seconds)
 */
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief 从当前的姿态四元数计算欧拉角（俯仰、滚转、偏航）
 * @param[out] yaw   偏航角 (度)
 * @param[out] pitch 俯仰角 (度)
 * @param[out] roll  横滚角 (度)
 */
void Madgwick_GetYawPitchRoll(float *yaw, float *pitch, float *roll);


#ifdef __cplusplus
}
#endif

#endif // __MADGWICKAHRS_H__