#ifndef LEG_IK_H
#define LEG_IK_H

#include <stdint.h>
 
#ifndef M_PI
#define M_PI 3.14159265358979323846f  // 圆周率常量，用于角度弧度转换
#endif

/**
 * @brief 腿部位置姿态结构体，存储四条腿的三维坐标
 * @details 坐标单位为毫米(mm)，坐标系以肘髋关节为原点
 * @param rf[3] 右前腿坐标 [x, y, z]
 * @param lf[3] 左前腿坐标 [x, y, z]
 * @param rb[3] 右后腿坐标 [x, y, z]
 * @param lb[3] 左后腿坐标 [x, y, z]
 */
typedef struct{
    float rf[3];                  
    float lf[3];                  
    float rb[3];                  
    float lb[3];                  
}LegPose_t;                      

extern LegPose_t Dog;  // 全局腿部姿态变量，存储四条腿的目标位置

/**-------------@brief 机器人腿部控制主函数----------------------
 * 
 * @details 协调完成腿部逆运动学计算、舵机角度转换、PWM帧发送的完整流程
 * @使用方法：在主控制循环中周期性调用（如10ms一次），确保腿部运动实时更新
 * @注意：需先通过calc_leg_pose设置Dog结构体中的腿部目标位置
 */
void Dog_Control(void);

/**
 * @brief 膝盖舵机角度转换函数（机械结构补偿）
 * @details 将膝盖关节的理论角度转换为实际舵机角度（考虑机械传动结构）
 * @param pulse_angle 膝盖关节理论角度（单位：度，范围需符合机械限制）
 * @return uint16_t 转换后的膝盖舵机实际角度（单位：度，范围35~140）
 * @使用方法：输入腿部逆运动学计算出的膝盖角度，输出用于舵机控制的角度
 * @注意：内部包含角度范围约束，超出范围会被截断
 */
uint16_t Knee_ServoAngle(int16_t pulse_angle);

/**
 * @brief 舵机角度到PWM占空比的转换函数
 * @details 将髋、大腿、膝盖的关节角度转换为对应舵机的PWM占空比（25~125）
 * @使用方法：在逆运动学计算完成（hip/thigh/knee数组更新后）调用，更新Servo_Pwm数组
 * @注意：不同位置的舵机（前后左右）有不同的偏移量补偿（如Hip_Left_Front_Diff等）
 */
void Servo_Angle_Conversion(void);

/**
 * @brief 32路舵机数据帧发送函数（实际使用12路）
 * @details 将12路舵机的PWM值打包为65字节的协议帧，并通过UART2发送
 * @帧结构：前64字节为12路舵机的16位PWM值（高8位+低8位），第65字节为前64字节的校验和
 * @使用方法：在Servo_Angle_Conversion更新Servo_Pwm后调用，发送控制指令到舵机
 * @注意：依赖底层Send_Frame_UART2函数实现UART发送
 */
void Send_32Servo_HEX(void);

/**
 * @brief 3x3矩阵与3维向量的乘法函数（内部辅助）
 * @param R [输入] 3x3旋转矩阵
 * @param x [输入] 3维向量
 * @param y [输出] 结果向量（y = R * x）
 * @使用方法：内部被calc_leg_pose调用，用于姿态旋转计算，无需外部直接调用
 */
static void matrix3x3_mul_vec(const float R[3][3], const float x[3], float y[3]);

/**
 * @brief 计算腿部目标位置（基于机器人整体姿态）
 * @details 根据机器人的滚转、俯仰、偏航角（RPY）和位置偏移，计算四条腿的目标坐标
 * @param Rol 滚转角（单位：度，绕X轴旋转）
 * @param Pitch 俯仰角（单位：度，绕Y轴旋转）
 * @param Yaw 偏航角（单位：度，绕Z轴旋转）
 * @param pos_x_mm X方向位置偏移（单位：mm，前后方向）
 * @param pos_y_mm Y方向位置偏移（单位：mm，左右方向）
 * @param pos_z_mm Z方向位置偏移（单位：mm，上下方向，高度）
 * @使用方法：调用时传入机器人期望姿态和位置，结果会更新到Dog结构体中
 * @注意：坐标转换考虑了左右腿Y轴方向的差异（右前/右后腿Y取负）
 */
void calc_leg_pose(float Rol, float Pitch, float Yaw, float pos_x_mm, float pos_y_mm, float pos_z_mm);

/**
 * @brief 单条腿的逆运动学计算函数
 * @details 将单条腿的目标三维坐标(x,y,z)转换为髋、大腿、膝盖关节的角度
 * @param x 腿部目标X坐标（单位：mm）
 * @param y 腿部目标Y坐标（单位：mm）
 * @param z 腿部目标Z坐标（单位：mm）
 * @param h [输出] 髋关节角度（单位：度，范围-65~65）
 * @param t [输出] 大腿关节角度（单位：度，范围-80~80）
 * @param k [输出] 膝盖关节角度（单位：度，无显式范围，后续会经Knee_ServoAngle处理）
 * @使用方法：对每条腿调用，输入目标坐标，输出关节角度到hip/thigh/knee数组
 * @注意：内部包含角度范围限制，超出机械范围的角度会被截断
 */
void single_ik(float x, float y, float z,int16_t *h, int16_t *t, int16_t *k);


#endif