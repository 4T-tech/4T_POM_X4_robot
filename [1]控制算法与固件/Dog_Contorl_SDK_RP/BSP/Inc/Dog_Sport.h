#ifndef DOG_SPORT_H
#define DOG_SPORT_H

#include <stdint.h>
//#include "main.h"
#include <math.h>

/**
 * @brief 四足机器人运动控制结构体
 * @details 存储机器人运动控制的各项参数，包括运动模式、步态参数、姿态数据等
 * 所有参数均经过范围限制，确保运动安全
 */
typedef struct {
    uint8_t Gait_Mode;                     // 模式选择：1=运动模式，0=平衡站立模式
    float Stride_Length;                   // 步长：单步运动的长度(单位mm)，正值=前进，负值=后退，范围[-70,70]
    float Motion_Speed;                    // 运动速度：0=停止，1-5=不同速度等级，等级越高速度越快
    float Turn_Speed;                      // 转向速度：单位mm/s，正值=右转向，负值=左转向，范围[-70,70]
    float Raise_Leg_Height;                // 抬腿高度：运动模式下的抬腿高度(单位mm)，范围[10,50]
    float Side_Length;                     // 侧向移动距离：单位mm，正值=右移，负值=左移，范围[-60,60]
    float Roll_data;                       // 滚转角目标值：单位度，范围[-25,25]（左右倾斜）
    float Pitch_data;                      // 俯仰角目标值：单位度，范围[-20,20]（前后倾斜）
    float Yaw_data;                        // 偏航角目标值：单位度，范围[-20,20]（转向）（通常不用，舵机狗效果很差）
    float X_data;                          // X轴位移：机体坐标系X方向偏移(前后)，单位mm，范围[-50,50]
    float Y_data;                          // Y轴位移：机体坐标系Y方向偏移(左右)，单位mm，范围[-50,50]
    float Z_data;                          // Z轴位移：机体高度，单位mm，范围[40,190]（仅正值有效）
} DogCtrl_t;

extern DogCtrl_t DogCtrl;                  // 运动控制全局结构体实例

/**
 * @brief PID控制器结构体
 * @details 存储PID控制算法的参数和中间变量，用于闭环控制
 */
typedef struct {
    float kp;              // 比例系数
    float ki;              // 积分系数
    float kd;              // 微分系数
    float err;             // 当前误差
    float last_err;        // 上一次误差
    float integral;        // 积分项
    float out_lim;         // 输出限制值
} PID_t;

/**
 * @brief 计算Trot小跑步态
 * @details 实现四足机器人的Trot步态控制，通过插值计算腿部运动轨迹，
 * 实现前进、后退、转向和侧向移动等动作组合
 * 调用时机：运动模式(Gait_Mode=0)且非停止状态(Motion_Speed>0)时周期性调用
 */
void Cal_trot(void);

/**
 * @brief 平滑站立函数
 * @details 控制机器人从初始位置平滑过渡到站立姿态，通过逐步调整Z轴高度和X轴位移实现
 * 执行过程：Z从50mm增至150mm，X从30mm减至0，每步间隔25ms
 * 完成后发送"STAND_UP_DONE"通知，并设置Start_Flag=1
 */
void Stand_Up_Smoothly(void);

/**
 * @brief 打印腿部位置信息
 * @details 格式化输出四条腿（rf=右前、lf=左前、rb=右后、lb=左后）的X/Y/Z坐标，
 * 通过串口和BLE发送位置数据，用于调试
 */
void print_leg_positions(void);

/**
 * @brief PID控制器更新函数
 * @param[in] pid：PID结构体指针
 * @param[in] target：目标值
 * @param[in] actual：实际值
 * @return 计算得到的PID输出（已限制在out_lim范围内）
 * @details 1. 计算当前误差=目标值-实际值
 *          2. 积分项累加（带范围限制防止饱和）
 *          3. 计算微分项=当前误差-上一次误差
 *          4. 计算PID输出=kp*err + ki*integral + kd*derivative
 *          5. 输出限幅处理
 * 注意：当误差绝对值<0.5时，积分项清零，防止静差累积
 */
float pid_update(PID_t *pid, float target, float actual);

/**
 * @brief PID控制主函数
 * @details 1. 计算滚转(roll)和俯仰(pitch)的PID补偿值
 *          2. 通过插值平滑过渡补偿值
 *          3. 应用补偿值计算腿部姿态，实现机器人姿态稳定控制
 * 调用时机：平衡站立模式(Gait_Mode=1)时周期性调用
 */
void PID_Control();

/**
 * @brief 模式控制函数
 * @details 根据Gait_Mode选择执行对应的控制逻辑：
 *          - 模式0：执行Trot步态控制(Cal_trot)
 *          - 模式1：执行PID姿态控制(PID_Control)
 * 注意：仅当Start_Flag=1（站立完成）时有效
 */
void Mode_Control(void);

/**
 * @brief 读取电池电压
 * @details 1. 通过ADC值计算电池电压（公式：(REG_ADDR[43]*3.3/1024)*17/2 + 0.1）
 *          2. 格式化电压信息（保留2位小数）
 *          3. 通过串口和BLE发送电压数据
 */
void Read_Battery_Volt();

/**
 * @brief 读取温度
 * @details 1. 通过ADC值计算NTC电阻（NCP_Ohm = (20*NCP_Volt)/(3.3 - NCP_Volt)）
 *          2. 对比NCP_List查找对应的温度等级（每级5度）
 *          3. 通过串口和BLE发送温度数据
 */
void Read_Temp();

#endif