#include "leg_ik.h"
#include <math.h>
#include "reg_protocol.h"
#include <stdio.h>  
#include <stdlib.h>

/* 结构体物理参数（mm），根据实际机械尺寸修改 */

/* 机器人主体参数 */
#define B  104.0f                            // 机器人左右髋关节间距
#define W  188.0f                            // 机器人前后髋关节间距
#define Len  243.0f                          // 机器人前后轴距离
 
/* 单条腿的连杆物理参数（mm）*/            
static const float H  = 42.0f;               /* 髋关节到大腿关节的垂直距离 */
static const float L1 = 110.0f;              /* 大腿连杆长度 */
static const float L2 = 110.0f;              /* 小腿连杆长度 */
                                           
// 小腿关节机械结构参数（用于膝盖角度转换）     
#define Rs   24.0f                           // 小腿关节旋转轴半径（mm）
#define L   70.0f                            // 小腿连杆长度（mm）
#define LR  27.0f                            // 小腿旋转到水平位置的水平距离（mm）
#define HH 71.6f                             // 髋关节到小腿旋转轴的垂直高度（mm）
#define I 13.0f                              // 小腿旋转轴到连杆端点的垂直距离
#define Gamma_deg  2.8624                    // 机械补偿角度（度）

/* 舵机编号定义（对应32路舵机协议中的编号）*/
#define  Hip_Left_Front      19               // 左前腿髋关节舵机编号
#define  Thigh_Left_Front    18               // 左前腿大腿关节舵机编号
#define  Knee_Left_Front     17               // 左前腿膝盖关节舵机编号
                                                                           
#define  Hip_Right_Front     3                // 右前腿髋关节舵机编号
#define  Thigh_Right_Front   2                // 右前腿大腿关节舵机编号
#define  Knee_Right_Front    1                // 右前腿膝盖关节舵机编号
                                                                           
#define  Hip_Left_Back       30               // 左后腿髋关节舵机编号
#define  Thigh_Left_Back     31               // 左后腿大腿关节舵机编号
#define  Knee_Left_Back      32               // 左后腿膝盖关节舵机编号
                                         
#define  Hip_Right_Back      14               // 右后腿髋关节舵机编号
#define  Thigh_Right_Back    15               // 右后腿大腿关节舵机编号
#define  Knee_Right_Back     16               // 右后腿膝盖关节舵机编号

/* 舵机角度到PWM的转换偏移量（基于舵机中位75的补偿值）*/
#define  Hip_Left_Front_Diff       75-1               //每条腿的误差值不一样，肘关节水平时为90°
#define  Thigh_Left_Front_Diff     75+2                                    // 大腿关节垂直地面为90°
#define  Knee_Left_Front_Diff      125+2                                   // 小腿关节90°，舵机臂水平

#define  Hip_Right_Front_Diff      75+4
#define  Thigh_Right_Front_Diff    75
#define  Knee_Right_Front_Diff     25+4

#define  Hip_Left_Back_Diff        75-3
#define  Thigh_Left_Back_Diff      75+3
#define  Knee_Left_Back_Diff       125

#define  Hip_Right_Back_Diff       75-3
#define  Thigh_Right_Back_Diff     75+3
#define  Knee_Right_Back_Diff      25+3

#define  Coe_ServoAngle             1.8f      // 角度到PWM的转换系数（180度对应100PWM，1.8=180/100）

uint16_t Servo_Pwm[12];                      // 12路舵机PWM值数组（范围25~125，对应0~180度）
                                             // 索引定义：0-2右前腿，3-5左前腿，6-8右后腿，9-11左后腿
int16_t hip[4], thigh[4], knee[4];           // 四条腿的关节角度数组（0右前，1左前，2右后，3左后）

LegPose_t Dog;                               // 腿部位置姿态全局变量
 
char tx[30];                                 // 调试用字符串缓存

/**
 * @brief 机器人腿部控制主函数
 * @details 1. 遍历四条腿，调用single_ik计算关节角度
 *          2. 调用Servo_Angle_Conversion将角度转换为PWM
 *          3. 调用Send_32Servo_HEX发送PWM控制帧
 * @使用流程：
 *          1. 先通过calc_leg_pose设置Dog结构体的目标位置
 *          2. 调用本函数执行控制流程
 * @示例：
 *          calc_leg_pose(0, 0, 0, 0, 0, 150);  // 设置机器人姿态（水平，高度150mm）
 *          Dog_Control();                       // 执行腿部控制
 * 注意：
 *          如果想要持续控制四足机器人，将该控制函数放在最小为20ms的定时器里面运行
 */
void Dog_Control(void)
{
    // 定义四条腿的位置指针数组（右前、左前、右后、左后）
    float* pos[4] = { Dog.rf, Dog.lf, Dog.rb, Dog.lb };                 

    // 遍历四条腿，计算每条腿的关节角度
    for (int i = 0; i < 4; ++i)
    {
        single_ik(pos[i][0], pos[i][1], pos[i][2],&hip[i], &thigh[i], &knee[i]);
    }
    // 角度转PWM
    Servo_Angle_Conversion();
    // 发送PWM控制帧
    Send_32Servo_HEX();
}

/**
 * @brief 单条腿逆运动学计算（几何解析法）
 * @details 基于连杆结构，通过几何关系计算髋、大腿、膝盖关节角度：
 *          1. 计算Y-Z平面投影，得到髋关节角度theta0
 *          2. 计算X-YZ平面投影，得到大腿关节角度theta1和膝盖关节角度theta2
 * @param x 目标位置X坐标（mm）
 * @param y 目标位置Y坐标（mm）
 * @param z 目标位置Z坐标（mm）
 * @param h [输出] 髋关节角度（度），范围-65~65
 * @param t [输出] 大腿关节角度（度），范围-80~80
 * @param k [输出] 膝盖关节角度（度），无显式范围
 * @计算原理：
 *          - 髋关节角度：通过Y-Z平面距离和垂直距离H计算
 *          - 大腿/膝盖角度：通过余弦定理计算连杆角度
 * @注意：输出角度会被限制在机械允许范围内，防止关节超限
 * @示例：
 *          single_ik(pos[i][0], pos[i][1], pos[i][2],&hip[i], &thigh[i], &knee[i]);
 *           
 */
void single_ik(float x, float y, float z,int16_t *h, int16_t *t, int16_t *k)
{
    // 计算Y-Z平面的距离
    float dyz = sqrtf(y*y + z*z);
    // 计算Y-Z平面中去除H后的水平距离
    float lyz = sqrtf(dyz*dyz - H*H);

    // 计算髋关节角度分量（Y-Z平面）
    float gamma_yz  = -atan2f(y, z);
    float gamma_off = -atan2f(H, lyz);
    float theta0    = gamma_yz - gamma_off;  // 最终髋关节角度（弧度）

    // 计算X与lyz构成的平面距离
    float lxzp = sqrtf(lyz*lyz + x*x);

    // 余弦定理计算膝盖关节角度
    float n = (lxzp*lxzp - L1*L1 - L2*L2) / (2.0f*L1);
    float theta2 = -acosf(n / L2);  // 膝盖关节角度（弧度）

    // 计算大腿关节角度
    float alfa_xzp = -atan2f(x, lyz);
    float alfa_off =  acosf((L1 + n) / lxzp);
    float theta1   = alfa_xzp + alfa_off;  // 大腿关节角度（弧度）

    // 弧度转角度，并四舍五入
    *h = (int16_t)(theta0 * 180.0f/M_PI + 0.5f);
    *t = (int16_t)(theta1 * 180.0f/M_PI + 0.5f);
    *k = (int16_t)(theta2 * 180.0f/M_PI + 0.5f);

    // 限制髋关节角度范围
    *h = (*h < -65) ? -65 : ((*h > 65) ? 65 : *h);
    // 限制大腿关节角度范围
    *t = (*t < -80) ? -80 : ((*t > 80) ? 80 : *t);
}

/**
 * @brief 膝盖舵机角度转换（机械结构补偿）
 * @details 由于膝盖关节为连杆传动（非直接旋转），需通过几何关系将理论角度转换为舵机实际角度
 * @param pulse_angle 膝盖关节理论角度（度）
 * @return 转换后的舵机角度（度），范围35~140
 * @转换原理：
 *          1. 基于机械结构参数（Rs、L、LR等）计算传动关系
 *          2. 通过余弦定理计算舵机需要旋转的角度
 *          3. 包含角度范围约束，防止机械超限
 * @注意：输入角度为负时表示膝盖弯曲方向，内部会处理方向转换
 * @示例：
 *          Knee_ServoAngle(knee);                 输入小腿关节角度，即可计算出舵机角度
 */
uint16_t Knee_ServoAngle(int16_t pulse_angle) 
{
    // 限制输入角度范围（机械允许的理论角度）
    int16_t constrained_angle;
    if (pulse_angle < 35) {                
        constrained_angle = 35;
    } else if (pulse_angle > 135) {
        constrained_angle = 135;
    } else {
        constrained_angle = pulse_angle;
    }
    
    // 角度转弧度
    float theta = (float)constrained_angle * (float)M_PI / 180.0f;
    
    // 补偿角度转弧度
    float Gamma_Wucha   = (float)Gamma_deg * (float)M_PI / 180.0f;
    
    // 机械结构几何计算
    float h1 = (LR * cosf(theta)) / cosf(Gamma_Wucha);
    float h2 = HH - h1;
    
    // 避免除以零错误
    if (h2 < 1e-6f) {
        return 0;
    }
    float Lc = sqrtf(h2 * h2 + I*I);
    
    // 余弦定理计算舵机角度
    float numerator = Lc * Lc + Rs * Rs - L * L;
    float denominator = 2.0f * h2 * Rs;
    float cos_phi = numerator / denominator;
    
    // 限制余弦值范围（防止计算误差导致超出[-1,1]）
    if (cos_phi < -1.0f) cos_phi = -1.0f;
    if (cos_phi > 1.0f) cos_phi = 1.0f;
    
    // 弧度转角度
    float knee_rad = acosf(cos_phi) + acosf(h2/Lc);
    float knee_deg = knee_rad * 180.0f / (float)M_PI;
    
    // 限制舵机输出角度范围
    int32_t temp = (int32_t)roundf(knee_deg);
    if (temp < 35) {
        return 35;
    } else if (temp > 140) {
        return 140;
    } else {
        return (uint16_t)temp;
    }
}

/**
 * @brief 32路舵机协议帧发送（实际使用12路）
 * @details 按协议将12路舵机PWM值打包为65字节帧，通过UART2发送
 * @帧结构：
 *          - 字节0~63：12路舵机的16位PWM值（高8位在前，低8位在后）
 *          - 字节64：前64字节的校验和（低8位）
 * @舵机映射关系：
 *          - 左前腿：Hip_Left_Front(19)、Thigh_Left_Front(18)、Knee_Left_Front(17)
 *          - 右前腿：Hip_Right_Front(3)、Thigh_Right_Front(2)、Knee_Right_Front(1)
 *          - 左后腿：Hip_Left_Back(30)、Thigh_Left_Back(31)、Knee_Left_Back(32)
 *          - 右后腿：Hip_Right_Back(14)、Thigh_Right_Back(15)、Knee_Right_Back(16)
 * @使用前提：需先通过Servo_Angle_Conversion更新Servo_Pwm数组
 */
void Send_32Servo_HEX(void) 
{
    // 初始化65字节发送帧（全0）
    uint8_t Servo_32frame[65] = {0};

    // 左前腿舵机数据（索引3-5）
    Servo_32frame[Hip_Left_Front*2-2]    =  (Servo_Pwm[3]  >> 8) & 0xFF;  // 髋关节高8位
    Servo_32frame[Hip_Left_Front*2-1]    =  Servo_Pwm[3]   & 0xFF;        // 髋关节低8位
    Servo_32frame[Thigh_Left_Front *2-2] = (Servo_Pwm[4]   >> 8) & 0xFF;  // 大腿关节高8位
    Servo_32frame[Thigh_Left_Front *2-1] =  Servo_Pwm[4]   & 0xFF;        // 大腿关节低8位
    Servo_32frame[Knee_Left_Front  *2-2] = (Servo_Pwm[5]   >> 8) & 0xFF;  // 膝盖关节高8位
    Servo_32frame[Knee_Left_Front  *2-1] =  Servo_Pwm[5]   & 0xFF;        // 膝盖关节低8位
                                                                          
    // 右前腿舵机数据（索引0-2）
    Servo_32frame[Hip_Right_Front*2-2]   =  (Servo_Pwm[0]   >> 8) & 0xFF; // 髋关节高8位
    Servo_32frame[Hip_Right_Front*2-1]   =  Servo_Pwm[0]    & 0xFF;       // 髋关节低8位
    Servo_32frame[Thigh_Right_Front*2-2] =  (Servo_Pwm[1]   >> 8) & 0xFF; // 大腿关节高8位
    Servo_32frame[Thigh_Right_Front*2-1] =  Servo_Pwm[1]    & 0xFF;       // 大腿关节低8位
    Servo_32frame[Knee_Right_Front*2-2]   =  (Servo_Pwm[2]  >> 8) & 0xFF; // 膝盖关节高8位
    Servo_32frame[Knee_Right_Front*2-1]   =  Servo_Pwm[2]    & 0xFF;      // 膝盖关节低8位
                                                                          
    // 左后腿舵机数据（索引9-11）
    Servo_32frame[Hip_Left_Back  *2-2]   =  (Servo_Pwm[9]  >> 8) & 0xFF;  // 髋关节高8位
    Servo_32frame[Hip_Left_Back  *2-1]   =  Servo_Pwm[9]   & 0xFF;        // 髋关节低8位
    Servo_32frame[Thigh_Left_Back *2-2]  = (Servo_Pwm[10]  >> 8) & 0xFF;  // 大腿关节高8位
    Servo_32frame[Thigh_Left_Back *2-1]  =  Servo_Pwm[10]  & 0xFF;        // 大腿关节低8位
    Servo_32frame[Knee_Left_Back  *2-2]  = (Servo_Pwm[11]  >> 8) & 0xFF;  // 膝盖关节高8位
    Servo_32frame[Knee_Left_Back  *2-1]  =  Servo_Pwm[11]  & 0xFF;        // 膝盖关节低8位
                                                                          
    // 右后腿舵机数据（索引6-8）
    Servo_32frame[Hip_Right_Back*2-2]    =  (Servo_Pwm[6]  >> 8) & 0xFF;  // 髋关节高8位
    Servo_32frame[Hip_Right_Back*2-1]    =  Servo_Pwm[6]   & 0xFF;        // 髋关节低8位
    Servo_32frame[Thigh_Right_Back*2-2]  =  (Servo_Pwm[7]  >> 8) & 0xFF;  // 大腿关节高8位
    Servo_32frame[Thigh_Right_Back*2-1]  =  Servo_Pwm[7]   & 0xFF;        // 大腿关节低8位
    Servo_32frame[Knee_Right_Back *2-2]  =  (Servo_Pwm[8]  >> 8) & 0xFF;  // 膝盖关节高8位
    Servo_32frame[Knee_Right_Back *2-1]  =  Servo_Pwm[8]   & 0xFF;        // 膝盖关节低8位
    
    // 计算前64字节校验和（低8位）
    uint16_t sum = 0;
    for (int i = 0; i < 64; ++i) sum += Servo_32frame[i];
    Servo_32frame[64] = (uint8_t)(sum & 0xFF);
    
    // 通过UART2发送帧
    Send_Frame_UART2(Servo_32frame);
}

/**
 * @brief 关节角度到舵机PWM的转换
 * @details 根据舵机机械特性（中位、方向），将关节角度转换为PWM占空比（25~125）
 * @转换公式：PWM = 偏移量 ± (角度 / 转换系数)，符号由舵机安装方向决定
 * @舵机方向说明：
 *          - 髋关节：左右腿方向相反（左前减角度，右前加角度）
 *          - 大腿关节：左右腿方向相反（左前加角度，右前减角度）
 *          - 膝盖关节：需先通过Knee_ServoAngle转换，再计算PWM
 * @注意：Coe_ServoAngle为1.8（180度对应100PWM），偏移量补偿舵机个体差异
 */
void Servo_Angle_Conversion(void) 
{
    // 左前腿（索引3-5）
    Servo_Pwm[3] = (uint16_t)(Hip_Left_Front_Diff - (hip[1] / Coe_ServoAngle));         // 髋关节
    Servo_Pwm[4] = (uint16_t)(Thigh_Left_Front_Diff + (thigh[1]/ Coe_ServoAngle));      // 大腿关节
    Servo_Pwm[5] = (uint16_t)(Knee_Left_Front_Diff - (Knee_ServoAngle(-knee[1]) / Coe_ServoAngle ));  // 膝盖关节
    
    // 右前腿（索引0-2）
    Servo_Pwm[0] = (uint16_t)(Hip_Right_Front_Diff + (hip[0] / Coe_ServoAngle));        // 髋关节
    Servo_Pwm[1] = (uint16_t)(Thigh_Right_Front_Diff -(thigh[0] / Coe_ServoAngle));     // 大腿关节
    Servo_Pwm[2] =  (uint16_t)(Knee_ServoAngle(-knee[0]) / Coe_ServoAngle + Knee_Right_Front_Diff);  // 膝盖关节
                                                                                                               
    // 左后腿（索引9-11）
    Servo_Pwm[9] = (uint16_t)(Hip_Left_Back_Diff + (hip[3] / Coe_ServoAngle ));         // 髋关节
    Servo_Pwm[10] = (uint16_t)(Thigh_Left_Back_Diff + (thigh[3] / Coe_ServoAngle ));    // 大腿关节
    Servo_Pwm[11] = (uint16_t)(Knee_Left_Back_Diff - (Knee_ServoAngle(-knee[3]) / Coe_ServoAngle ));  // 膝盖关节        
                                                                                                               
    // 右后腿（索引6-8）
    Servo_Pwm[6] = (uint16_t)(Hip_Right_Back_Diff - (hip[2] / Coe_ServoAngle ));        // 髋关节
    Servo_Pwm[7] = (uint16_t)(Thigh_Right_Back_Diff - (thigh[2] / Coe_ServoAngle ));    // 大腿关节
    Servo_Pwm[8] = (uint16_t)(Knee_ServoAngle(-knee[2]) / Coe_ServoAngle+ Knee_Right_Back_Diff);  // 膝盖关节    
}

/**
 * @brief 计算腿部目标位置（基于机器人整体姿态）
 * @details 根据机器人的RPY角（姿态）和位置偏移，通过坐标变换计算四条腿的目标位置
 * @参数说明：
 *          - Rol：滚转角（绕X轴），正方向为机器人右侧抬起
 *          - Pitch：俯仰角（绕Y轴），正方向为机器人前端抬起
 *          - Yaw：偏航角（绕Z轴），正方向为机器人顺时针旋转
 *          - pos_x_mm：X方向偏移（向前为正）
 *          - pos_y_mm：Y方向偏移（向右为正）
 *          - pos_z_mm：Z方向偏移（向下为正，增大机器人高度）
 * @计算流程：
 *          1. 将RPY角转换为旋转矩阵
 *          2. 计算髋关节在旋转后的坐标
 *          3. 结合位置偏移和 foothold 位置，得到腿部目标坐标
 *          4. 修正左右腿Y轴方向（右前/右后腿Y取负）
 * @使用示例：
 *          // 机器人水平放置，高度150mm，无偏移
 *          calc_leg_pose(0, 0, 0, 0, 0, 150);
 */
void calc_leg_pose(float Rol, float Pitch, float Yaw, float pos_x_mm, float pos_y_mm, float pos_z_mm)
{
    // 角度转弧度
    float r = Rol  * (float)M_PI / 180.0f;
    float p = Pitch * (float)M_PI / 180.0f;
    float y = Yaw   * (float)M_PI / 180.0f;

    // 计算旋转矩阵 R = Rz(Yaw) * Ry(Pitch) * Rx(Roll)
    float sr = sinf(r), cr = cosf(r);
    float sp = sinf(p), cp = cosf(p);
    float sy = sinf(y), cy = cosf(y);

    float R[3][3] = {
        { cp*cy,  cp*sy,  -sp },
        { sr*sp*cy - cr*sy,  sr*sp*sy + cr*cy,  sr*cp },
        { cr*sp*cy + sr*sy,  cr*sp*sy - sr*cy,  cr*cp }
    };

    // 机器人坐标系下四条腿髋关节的初始位置（中心为原点）
    const float hip[4][3] = {
        {  Len*0.5f,  B*0.5f, 0.0f },   // 右前腿髋关节
        {  Len*0.5f, -B*0.5f, 0.0f },   // 左前腿髋关节
        { -Len*0.5f,  B*0.5f, 0.0f },   // 右后腿髋关节
        { -Len*0.5f, -B*0.5f, 0.0f }    // 左后腿髋关节
    };

    // 机器人坐标系下四条腿 foothold 的初始位置（落地点参考）
    const float foot[4][3] = {
        {  Len*0.5f,  W*0.5f, 0.0f },   // 右前腿落地点
        {  Len*0.5f, -W*0.5f, 0.0f },   // 左前腿落地点
        { -Len*0.5f,  W*0.5f, 0.0f },   // 右后腿落地点
        { -Len*0.5f, -W*0.5f, 0.0f }    // 左后腿落地点
    };

    // 位置偏移量
    float pos[3] = {pos_x_mm, pos_y_mm, pos_z_mm};
    float tmp[3];  // 临时计算变量

    // 计算四条腿的目标位置
    for(uint8_t i = 0; i < 4; ++i)
    {
        // 旋转髋关节位置：tmp = R * hip[i]
        matrix3x3_mul_vec( R, hip[i], tmp);

        // 计算腿部目标位置：pos + 旋转后的髋关节位置 - 初始落地点位置
        tmp[0] += pos[0] - foot[i][0];
        tmp[1] += pos[1] - foot[i][1];
        tmp[2] += pos[2] - foot[i][2];

        // 赋值到Dog结构体（右前/右后腿Y轴取负，修正坐标系方向）
        if(i==0){ Dog.rf[0]=tmp[0]; Dog.rf[1]=-tmp[1]; Dog.rf[2]=tmp[2]; }  // 右前腿
        if(i==1){ Dog.lf[0]=tmp[0]; Dog.lf[1]=tmp[1]; Dog.lf[2]=tmp[2]; }   // 左前腿
        if(i==2){ Dog.rb[0]=tmp[0]; Dog.rb[1]=-tmp[1]; Dog.rb[2]=tmp[2]; }  // 右后腿
        if(i==3){ Dog.lb[0]=tmp[0]; Dog.lb[1]=tmp[1]; Dog.lb[2]=tmp[2]; }   // 左后腿
    }
}

/**
 * @brief 3x3矩阵与3维向量的乘法（内部辅助函数）
 * @param R [输入] 3x3旋转矩阵
 * @param x [输入] 3维输入向量
 * @param y [输出] 3维输出向量（y = R * x）
 * @计算方式：y[i] = sum(R[i][j] * x[j])，j=0~2
 * @使用场景：仅在calc_leg_pose中用于姿态旋转计算，无需外部调用
 */
static void matrix3x3_mul_vec(const float R[3][3], const float x[3], float y[3])
{
    y[0] = R[0][0]*x[0] + R[0][1]*x[1] + R[0][2]*x[2];
    y[1] = R[1][0]*x[0] + R[1][1]*x[1] + R[1][2]*x[2];
    y[2] = R[2][0]*x[0] + R[2][1]*x[1] + R[2][2]*x[2];
}

