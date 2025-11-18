#include "leg_ik.h"          // 腿部逆运动学计算
#include <stdio.h>           // 标准输入输出
#include <stdlib.h>          // 标准库函数
#include "Dog_Sport.h"       // 运动控制头文件
#include "reg_protocol.h"    // 寄存器协议
#include "esp_log.h"       // ESP32日志

extern float LSM6DSOW_Yaw;    // 外部引用：陀螺仪偏航角（航向角）
extern float LSM6DSOW_Roll;   // 外部引用：陀螺仪滚转角
extern float LSM6DSOW_Pitch;  // 外部引用：陀螺仪俯仰角

void Delay_MS(uint32_t ms); // 延时函数声明

/**
 * @brief 插值浮点型结构体
 * @details 用于实现参数的平滑过渡，避免突变导致的运动冲击
 */
typedef struct {
    float target;   // 目标值：最终需要达到的值
    float current;  // 当前值：当前实际值（逐步逼近目标值）
    float step;     // 步长：每帧允许的最大变化量（控制过渡速度）
} InterpFloat_t;

/**
 * @brief 初始化插值变量
 * @param[in] v：插值结构体指针
 * @param[in] initial：初始值（current和target的初始值）
 * @param[in] _step：步长设置
 * @details 将current和target初始化为initial，设置step为_step
 */
static void Interp_Init(InterpFloat_t *v, float initial, float _step) {
    v->target  = initial;
    v->current = initial;
    v->step    = _step;
}



/**
 * @brief 姿态与位置插值变量
 * @details 用于实现对应参数的平滑过渡，避免突变
 * step值含义：每帧最大变化量（度/帧或mm/帧）
 */
static InterpFloat_t interp_roll  = {.step = 1.0f};   // 滚转角插值（1度/帧）
static InterpFloat_t interp_pitch = {.step = 1.0f};   // 俯仰角插值（1度/帧）
static InterpFloat_t interp_yaw   = {.step = 1.0f};   // 偏航角插值（1度/帧）
static InterpFloat_t interp_x     = {.step = 3.5f};   // X轴位移插值（3.5mm/帧）
static InterpFloat_t interp_y     = {.step = 3.5f};   // Y轴位移插值（3.5mm/帧）
static InterpFloat_t interp_z     = {.step = 3.5f};   // Z轴高度插值（3.5mm/帧）

/**
 * @brief PID补偿值插值变量
 * @details 用于平滑PID输出的补偿值，避免姿态突变
 */
static InterpFloat_t interp_roll_pid  = {.step = 0.35f};   // 滚转PID补偿插值（度/帧）
static InterpFloat_t interp_pitch_pid = {.step = 0.35f};   // 俯仰PID补偿插值（度/帧）

/**
 * @brief 更新插值变量
 * @param[in] v：插值结构体指针
 * @details 使current逐步逼近target：
 *          - 若当前差值<step：直接将current设为target
 *          - 否则：按step向target方向移动（正值加step，负值减step）
 */
static void Interp_Update(InterpFloat_t *v) {
    float diff = v->target - v->current;
    if (fabsf(diff) < v->step) {
        v->current = v->target;
    } else {
        v->current += (diff > 0 ? v->step : -v->step);
    }
}

/**
 * @brief PID控制器实例
 * @details 用于机器人姿态稳定控制的PID参数配置
 */
static PID_t roll_pid = { .kp = 0.6f, .ki = 0.03f, .kd = 0.01f, .out_lim = 25.0f };  // 滚转PID参数
static PID_t pitch_pid = { .kp = 0.6f, .ki = 0.03f, .kd = 0.01f, .out_lim = 20.0f }; // 俯仰PID参数

uint8_t Start_Flag = 0;  // 启动标志：0=未启动（未完成站立），1=已启动（可执行运动控制）

/**
 * @brief 腿部初始位置数组
 * @details 存储四条腿的初始坐标（在机体坐标系下的原点位置）
 * rf0=右前腿，lf0=左前腿，rb0=右后腿，lb0=左后腿
 */
static float rf0[3] = {0};  // 右前腿初始位置
static float lf0[3] = {0};  // 左前腿初始位置
static float rb0[3] = {0};  // 右后腿初始位置
static float lb0[3] = {0};  // 左后腿初始位置

/**
 * @brief NCP温度传感器电阻值列表
 * @details 存储不同温度下的NTC电阻值（单位kΩ），用于温度计算
 * 对应温度：索引0=0℃，索引1=5℃，...，索引23=115℃（每级5℃）
 */
#define NCP_LEN  (sizeof(NCP_List)/sizeof(NCP_List[0]))
static const float NCP_List[] = {
    27.219f, 22.021f, 17.926f, 14.674f, 12.081f, 10.000f, 8.315f, 6.948f,
    5.834f,  4.917f,  4.161f,  3.535f,  3.014f,  2.586f,  2.228f, 1.925f,
    1.669f,  1.452f,  1.268f,  1.110f,  0.974f ,0.858f ,0.758f, 0.672
};

char CDC_TX_BUF[50];  // 串口发送缓冲区（用于调试信息输出）

/**
 * @brief 运动控制全局结构体初始化
 * @details 设置各项参数的默认值，确保机器人初始状态安全
 */
DogCtrl_t DogCtrl = {
    .Gait_Mode = 0 ,                             // 默认运动模式 
    .Stride_Length =50.0f,                       // 默认步长50mm
    .Motion_Speed = 0.0 ,                        // 默认停止状态
    .Turn_Speed = 0.0f,                          // 默认不转向
    .Raise_Leg_Height = 20.0,                    // 默认抬腿高度20mm
    .Side_Length =0.0f,                          // 默认不侧向移动
    .Roll_data = 0.0,                            // 默认滚转角0度
    .Pitch_data = 0.0,                           // 默认俯仰角0度
    .Yaw_data = 0.0,                             // 默认偏航角0度
    .X_data = 0.0,                               // 默认X轴无偏移
    .Y_data =0.0,                                // 默认Y轴无偏移
    .Z_data = 140.0                              // 默认高度140mm
};

static float  Offset_Trun= 0;  // 转向补偿偏移量：用于航向角校正

/**
 * @brief 限制值在范围内
 * @param[in] v：需要限制的值
 * @param[in] min：最小值
 * @param[in] max：最大值
 * @return 限制后的值（若v<min返回min，v>max返回max，否则返回v）
 */
static inline float clamp(float v, float min, float max)
{
    return v < min ? min : (v > max ? max : v);
}

/**
 * @brief 限制运动控制参数范围
 * @details 确保DogCtrl结构体的所有参数在安全范围内，防止因参数异常导致机器人失控
 * 各参数范围参考DogCtrl_t结构体定义
 */
void DogCtrl_Clamp(void)
{
    DogCtrl.Gait_Mode        = (uint8_t)clamp(DogCtrl.Gait_Mode,        0,   1);
    DogCtrl.Stride_Length    = clamp(DogCtrl.Stride_Length,           -70.0f, 70.0f);
    DogCtrl.Motion_Speed     = (uint8_t)clamp(DogCtrl.Motion_Speed,     0,   5);
    DogCtrl.Turn_Speed       = clamp(DogCtrl.Turn_Speed,              -70.0f, 70.0f);
    DogCtrl.Raise_Leg_Height = clamp(DogCtrl.Raise_Leg_Height,         10.0f, 50.0f);
    DogCtrl.Side_Length      = clamp(DogCtrl.Side_Length,             -60.0f, 60.0f);
    DogCtrl.Roll_data        = clamp(DogCtrl.Roll_data,               -25.0f, 25.0f);
    DogCtrl.Pitch_data       = clamp(DogCtrl.Pitch_data,              -20.0f, 20.0f);
    DogCtrl.Yaw_data         = clamp(DogCtrl.Yaw_data,               -20.0f,20.0f);
    DogCtrl.X_data           = clamp(DogCtrl.X_data,                  -50.0f, 50.0f);
    DogCtrl.Y_data           = clamp(DogCtrl.Y_data,                  -50.0f, 50.0f);
    DogCtrl.Z_data           = clamp(DogCtrl.Z_data,                   40.0f,190.0f);
}

/**
 * @brief 计算X方向步长起始和结束值
 * @param[in] v：步长参考值（与Stride_Length、Turn_Speed相关）
 * @param[out] xs：步长起始值
 * @param[out] xf：步长结束值
 * @details 根据运动方向（前进/后退）计算步长的起始和结束点：
 *          - 前进（v>=0）：起始-10mm，结束v-10mm     即机身向后偏移10， 相当于把X_data设置为 -10也可以
 *          - 后退（v<0）：起始-v+10mm，结束+10mm     即机身向前偏移10， 相当于把X_data设置为 +10也可以
 */
static void step_len_X(float v, float *xs, float *xf)
{
    if (v >= 0.0f) {                                /* 前进 */
        *xs = 10.0f;
        *xf = v * 1.0f + 10.0f;
    } else {                                        /* 后退 */
        *xs = -v * 1.0f - 10.0f;
        *xf =  -10.0f;
    }
}

/**
 * @brief 计算Y方向步长起始和结束值
 * @param[in] v：侧向移动距离（Side_Length）
 * @param[out] xs：步长起始值
 * @param[out] xf：步长结束值
 * @details 根据侧向移动方向计算步长的起始和结束点，实现左右对称运动
 */
static void step_len_Y(float v, float *xs, float *xf)
{
    if (v >= 0.0f) {                              
        *xs = v * 0.5f ;
        *xf = -v * 0.5f ;
    } else {                                      
        *xs = v * 0.5f;
        *xf = -v * 0.5f;
    }
}

/**
 * @brief 更新腿部原点位置
 * @details 1. 限制控制参数范围（DogCtrl_Clamp）
 *          2. 设置插值目标值为当前控制参数
 *          3. 更新插值变量（使current逼近target）
 *          4. 根据插值后的姿态参数计算腿部位置（calc_leg_pose）
 *          5. 保存当前腿部位置为原点（rf0/lf0/rb0/lb0）
 * 作用：实现姿态参数的平滑过渡，避免突变
 */
static void update_leg_origins(void)
{
    DogCtrl_Clamp();
    interp_roll.target  = DogCtrl.Roll_data;
    interp_pitch.target = DogCtrl.Pitch_data;
    interp_yaw.target   = DogCtrl.Yaw_data;
    interp_x.target     = DogCtrl.X_data;
    interp_y.target     = DogCtrl.Y_data;
    interp_z.target     = DogCtrl.Z_data;

    Interp_Update(&interp_roll);
    Interp_Update(&interp_pitch);
    Interp_Update(&interp_yaw);
    Interp_Update(&interp_x);
    Interp_Update(&interp_y);
    Interp_Update(&interp_z);
    
    // 使用插值后的值计算姿态
    calc_leg_pose(interp_roll.current,
                  interp_pitch.current,
                  interp_yaw.current,
                  interp_x.current,
                  interp_y.current,
                  interp_z.current);

    // 保存当前腿部位置为原点
    rf0[0] = Dog.rf[0]; rf0[1] = Dog.rf[1]; rf0[2] = Dog.rf[2];
    lf0[0] = Dog.lf[0]; lf0[1] = Dog.lf[1]; lf0[2] = Dog.lf[2];
    rb0[0] = Dog.rb[0]; rb0[1] = Dog.rb[1]; rb0[2] = Dog.rb[2];
    lb0[0] = Dog.lb[0]; lb0[1] = Dog.lb[1]; lb0[2] = Dog.lb[2];
}

/**
 * @brief 航向角校正
 * @param[in] motion_speed：当前运动速度（DogCtrl.Motion_Speed）
 * @details 1. 若正在转向（Turn_Speed≠0），清零补偿偏移
 *          2. 运动开始时记录初始航向角作为参考
 *          3. 运动中计算当前航向角与参考值的误差，转换为补偿偏移（Offset_Trun）
 *          4. 限制补偿偏移在[-30,30]，并发送调试信息
 * 作用：确保机器人直线运动时不偏离航向
 */
void Yaw_Correct(uint8_t motion_speed)
{
    static float yaw_ref = 0.0f;
    static uint8_t moving_last = 0;
    static uint8_t turning_last = 0;

    float yaw_now = -LSM6DSOW_Yaw;
    if (yaw_now < 0) yaw_now += 360.0f;
    if (yaw_now >= 360.0f) yaw_now -= 360.0f;

    // 如果正在转弯，更新参考方向
    if (DogCtrl.Turn_Speed != 0)
    {
        Offset_Trun = 0;
        yaw_ref = yaw_now;  //  实时更新参考方向
        turning_last = 1;
        return;
    }

    // 转弯结束瞬间，更新参考方向
    if (turning_last == 1 && DogCtrl.Turn_Speed == 0)
    {
        yaw_ref = yaw_now;
        turning_last = 0;
    }

    // 直行时的偏航修正
    if (motion_speed > 0)
    {
        if (moving_last == 0) {
            yaw_ref = yaw_now;
        }

        float err = yaw_now - yaw_ref;
        if (err > 180.0f)  err -= 360.0f;
        if (err < -180.0f) err += 360.0f;

        float k_yaw = 1.5f;
        Offset_Trun = clamp(err * k_yaw, -30.0f, 30.0f);

        int len = sprintf(CDC_TX_BUF, "Turn:%lf \r\n", Offset_Trun);
        send_str(CDC_TX_BUF);
    }

    moving_last = (motion_speed > 0);
}

/**
 * @brief 计算Trot步态
 * @details 实现四足机器人Trot步态（对角腿同步运动）：
 *          1. 更新腿部原点位置（update_leg_origins）
 *          2. 航向角校正（Yaw_Correct）
 *          3. 若停止或非运动模式，直接返回
 *          4. 计算步态周期参数（t为当前时刻，Ts为周期1.0f）
 *          5. 计算左右腿速度（vL=右前/左后，vR=左前/右后）
 *          6. 分阶段（0~Ts*faai和Ts*faai~Ts）计算腿部位置：
 *             - 前半周期：右前和左后腿抬起，左前和右后腿落地
 *             - 后半周期：左前和右后腿抬起，右前和左后腿落地
 *          7. 通过正弦曲线计算抬腿高度，实现平滑运动
 */
void Cal_trot(void)
{
    // 更新每条腿的原点（含姿态/高度/平移）
    update_leg_origins();

    Yaw_Correct(DogCtrl.Motion_Speed);
    if (DogCtrl.Gait_Mode == 1 || DogCtrl.Motion_Speed == 0)
        return;
    
    static float t = 0.0f;           // 当前步态周期时刻（0~Ts）
    const float Ts = 1.0f;           // 步态周期（1.0f）
    const float faai = 0.5f;         // 步态相位系数（0.5表示半周期切换）
    const float h = DogCtrl.Raise_Leg_Height;  // 抬腿高度

    // 运动参数
    float fwd = DogCtrl.Stride_Length;  // 前进/后退步长
    float turn = DogCtrl.Turn_Speed;    // 转向速度
    float side = DogCtrl.Side_Length;   // 侧向移动距离

    // 计算左右腿速度（含转向和航向补偿）
    float vL = fwd + turn + Offset_Trun;  // 右前/左后腿速度
    float vR = fwd - turn - Offset_Trun;  // 左前/右后腿速度
      
    vL = clamp(vL , -70 ,70);  // 限制范围
    vR = clamp(vR , -70 ,70);

    // 计算X和Y方向步长
    float xs_L, xf_L, xs_R, xf_R;
    step_len_X(vL, &xs_L, &xf_L);  // 右前/左后腿X步长
    step_len_X(vR, &xs_R, &xf_R);  // 左前/右后腿X步长

    float ys_L, yf_L, ys_R, yf_R;
    step_len_Y(side, &ys_L, &yf_L);  // 左半侧腿Y步长
    step_len_Y(side, &ys_R, &yf_R);  // 右半侧腿Y步长

    // 更新周期时刻（随速度增加而加快）
    t += DogCtrl.Motion_Speed / 100;
    if (t >= Ts) t = 0.0f;

    // 步态轨迹计算变量
    float sigma, zep;  // sigma=相位角，zep=Z方向抬升量
    float xep_b_L, xep_z_L, xep_b_R, xep_z_R;  // X方向轨迹参数
    float side_b_L, side_z_L,side_b_R,side_z_R;  // Y方向轨迹参数

    // 前半周期（0 ~ Ts*faai）：右前+左后腿抬起
    if (t <= Ts * faai)
    {
        sigma = 2.0f * M_PI * t / (faai * Ts);  // 计算相位角（0~2π）
        zep = h * (1.0f - cosf(sigma)) * 0.5f;  // 正弦曲线计算抬升量（0~h）

        // 计算Y方向轨迹
        side_b_L = (yf_L - ys_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + ys_L;
        side_z_L = (ys_L - yf_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + yf_L;
        side_b_R = (yf_R - ys_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + ys_R;
        side_z_R = (ys_R - yf_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + yf_R;

        // 计算X方向轨迹
        xep_b_L = (xf_L - xs_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xs_L;
        xep_z_L = (xs_L - xf_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xf_L;
        xep_b_R = (xf_R - xs_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xs_R;
        xep_z_R = (xs_R - xf_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xf_R;

        // 设置腿部位置（右前和左后腿抬起，其他落地）
        Dog.rf[0] = rf0[0] - xep_z_R;  Dog.rf[1] = rf0[1] - side_b_R;  Dog.rf[2] = rf0[2] - zep;
        Dog.lb[0] = lb0[0] - xep_z_L;  Dog.lb[1] = lb0[1] - side_z_L;  Dog.lb[2] = lb0[2] - zep;
        Dog.lf[0] = lf0[0] - xep_b_L;  Dog.lf[1] = lf0[1] - side_b_L;  Dog.lf[2] = lf0[2];
        Dog.rb[0] = rb0[0] - xep_b_R;  Dog.rb[1] = rb0[1] - side_z_R;  Dog.rb[2] = rb0[2];
    }
    // 后半周期（Ts*faai ~ Ts）：左前+右后腿抬起
    else
    {
        sigma = 2.0f * M_PI * (t - Ts * faai) / (faai * Ts);  // 计算相位角（0~2π）
        zep = h * (1.0f - cosf(sigma)) * 0.5f;  // 正弦曲线计算抬升量（0~h）

        // 计算Y方向轨迹
        side_b_L = (yf_L - ys_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + ys_L;
        side_z_L = (ys_L - yf_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + yf_L;
        side_b_R = (yf_R - ys_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + ys_R;
        side_z_R = (ys_R - yf_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + yf_R;

        // 计算X方向轨迹
        xep_b_L = (xf_L - xs_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xs_L;
        xep_z_L = (xs_L - xf_L) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xf_L;
        xep_b_R = (xf_R - xs_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xs_R;
        xep_z_R = (xs_R - xf_R) * (sigma - sinf(sigma)) / (2.0f * M_PI) + xf_R;

        // 设置腿部位置（左前和右后腿抬起，其他落地）
        Dog.rf[0] = rf0[0] - xep_b_R;  Dog.rf[1] = rf0[1] - side_z_L;  Dog.rf[2] = rf0[2];
        Dog.lb[0] = lb0[0] - xep_b_L;  Dog.lb[1] = lb0[1] - side_b_R;  Dog.lb[2] = lb0[2];
        Dog.lf[0] = lf0[0] - xep_z_L;  Dog.lf[1] = lf0[1] - side_z_R;  Dog.lf[2] = lf0[2] - zep;
        Dog.rb[0] = rb0[0] - xep_z_R;  Dog.rb[1] = rb0[1] - side_b_L;  Dog.rb[2] = rb0[2] - zep;
    }
}

/**
 * @brief 平滑站立函数
 * @details 控制机器人从初始低位平滑起身到站立姿态：
 *          1. 初始化Start_Flag=0（禁止其他控制）
 *          2. 循环递增Z轴高度（50mm→150mm），递减X轴位移（30mm→0）
 *          3. 每步更新腿部位置（update_leg_origins），延迟25ms
 *          4. 完成后发送"STAND_UP_DONE"，设置Start_Flag=1（允许控制）
 */
void Stand_Up_Smoothly(void)
{
    const float step = 1.0f, delay_ms = 10.0f;  // 步长2mm，间隔25ms
    Start_Flag = 0;  // 站立过程中禁止其他控制
    for (float z = 50.0f, x = 30.0f; z <= 140.0f; z += step, x -= step/3)
    {
        DogCtrl.Z_data = z;  // 更新高度
        DogCtrl.X_data = x;  // 更新X位移
        update_leg_origins();  // 刷新腿部位置
        Delay_MS((uint32_t)delay_ms);  // 延迟等待
    }
    send_str("STAND_UP_DONE\r\n");  // 发送完成通知
    Start_Flag = 1;  // 允许控制
}

/**
 * @brief PID控制器更新实现
 * @param[in] pid：PID结构体指针
 * @param[in] target：目标值
 * @param[in] actual：实际测量值
 * @return 经过限幅的PID输出
 * @details 1. 计算当前误差=target-actual
 *          2. 积分项累加（限制在[-50,50]防止饱和）
 *          3. 计算微分项=当前误差-上一次误差
 *          4. 误差小于0.5时清零积分项，避免静差累积
 *          5. 计算PID输出并限幅
 */
float pid_update(PID_t *pid, float target, float actual)
{
    float err = target - actual;
    
    // 积分项限制
    if (pid->integral > 50.0f) pid->integral = 50.0f;
    else if (pid->integral < -50.0f) pid->integral = -50.0f;
    else pid->integral += err;

    // 微分项计算
    float derivative = err - pid->last_err;
    pid->last_err = err;
    // 小误差时清零积分，避免累积
    if (fabsf(err) < 0.5f) pid->integral = 0.0f;
    
    // 计算输出并限幅
    float out = pid->kp * err + pid->ki * pid->integral + pid->kd * derivative;
    if (out >  pid->out_lim) out =  pid->out_lim;
    if (out < -pid->out_lim) out = -pid->out_lim;
    return out;
}

/**
 * @brief PID姿态控制实现
 * @details 1. 计算滚转和俯仰的PID补偿值（基于陀螺仪反馈）
 *          2. 设置补偿值的插值目标，实现平滑过渡
 *          3. 更新插值变量，获取当前补偿值
 *          4. 应用补偿值计算腿部姿态，实现稳定控制
 * 调用场景：平衡站立模式下维持机器人姿态稳定
 */
void PID_Control(void)
{
    /* 1. 计算补偿值 */
    float roll_comp  = pid_update(&roll_pid,  DogCtrl.Roll_data, -LSM6DSOW_Roll);
    float pitch_comp = pid_update(&pitch_pid, DogCtrl.Pitch_data, LSM6DSOW_Pitch);

    /* 2. 设置插值目标 */
    interp_roll_pid.target  = roll_comp;      // 滚转补偿目标
    interp_pitch_pid.target = pitch_comp;     // 俯仰补偿目标

    /* 3. 平滑过渡补偿值 */
    Interp_Update(&interp_roll_pid);
    Interp_Update(&interp_pitch_pid);

    /* 4. 应用补偿计算腿部姿态 */
    calc_leg_pose(interp_roll_pid.current,interp_pitch_pid.current, 0, interp_x.current, interp_y.current, interp_z.current);
}

/**
 * @brief 模式控制主函数
 * @details 根据当前模式和启动状态选择控制逻辑：
 *          - 启动完成（Start_Flag=1）时：
 *            - 模式0：执行Trot步态控制（Cal_trot）
 *            - 模式1：执行PID姿态控制（PID_Control）
 *          - 未启动时不执行任何控制
 */
void Mode_Control(void)
{
    if(Start_Flag)
    {
        switch(DogCtrl.Gait_Mode)
        {
            case 0:
                Cal_trot();
            break;
            case 1:
                PID_Control();
            break;
        }
    }

}

/**
 * @brief 读取电池电压
 * @details 1. 从寄存器REG_ADDR[43]获取ADC值
 *          2. 计算电压：(ADC*3.3/1024)*17/2 + 0.1（考虑分压电路）
 *          3. 格式化电压信息并通过串口和BLE发送
 */
void Read_Battery_Volt()
{
    float Battery_Volt = ((REG_ADDR[43]*3.3f)/1024)* 17/2+0.1;  // 电压计算
    int len = sprintf(CDC_TX_BUF, "Battery_Volt:%.2f V\r\n",Battery_Volt);
    send_str(CDC_TX_BUF);  // 串口发送
}

/**
 * @brief 读取温度
 * @details 1. 从寄存器REG_ADDR[44]获取NTC的ADC值，计算电压NCP_Volt
 *          2. 计算NTC电阻：NCP_Ohm=(20*NCP_Volt)/(3.3-NCP_Volt)
 *          3. 对比NCP_List查找对应的温度等级（索引*5=温度值）
 *          4. 格式化温度信息并通过串口和BLE发送
 */
void Read_Temp()
{
    float NCP_Volt= ((REG_ADDR[44]*3.3f)/1024);  // NTC电压
    float NCP_Ohm = (20 * NCP_Volt)/(3.3f - NCP_Volt);  // NTC电阻
    uint8_t NCP_Num = 0;
    // 查找匹配的电阻等级
    for (size_t i = 0; i < NCP_LEN; ++i)
        if (NCP_List[i] < NCP_Ohm)
        {
            NCP_Num = i;
            break;
        }
    uint8_t Temp = NCP_Num * 5;  // 计算温度（每级5度）
    
    int len = sprintf(CDC_TX_BUF, "Temp:%d \r\n",Temp);
    send_str(CDC_TX_BUF);  // 串口发送
}


// 打印腿部位置（调试用，已注释）
//void print_leg_positions(void)
//{
//    char buf[200];  // 缓冲区存储位置信息
//    
//    // 格式化四条腿的X/Y/Z坐标（保留1位小数）
//    sprintf(buf, 
//        "rf: X=%.1f, Y=%.1f, Z=%.1f\r\n"
//        "lf: X=%.1f, Y=%.1f, Z=%.1f\r\n"
//        "rb: X=%.1f, Y=%.1f, Z=%.1f\r\n"
//        "lb: X=%.1f, Y=%.1f, Z=%.1f\r\n",
//        Dog.rf[0], Dog.rf[1], Dog.rf[2],
//        Dog.lf[0], Dog.lf[1], Dog.lf[2],
//        Dog.rb[0], Dog.rb[1], Dog.rb[2],
//        Dog.lb[0], Dog.lb[1], Dog.lb[2]
//    );
//    
//    send_str(buf);  // 串口发送
//    Send_BLE(buf);  // BLE发送
//}