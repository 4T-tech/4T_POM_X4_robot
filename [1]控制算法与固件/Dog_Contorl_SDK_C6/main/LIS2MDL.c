/* LIS2MDL 地磁计的驱动文件 */
#include "include.h"
#include "driver/i2c_master.h"

/************* LIS2MDL 内部寄存器 *************/
#define LIS2MDL_ADDRESS            0x1E         // LIS2MDL I2C 地址
#define LIS2MDL_Device_ID_Ad       0x4F         // LIS2MDL 设备ID 寄存器地址 
#define LIS2MDL_OUTX_L_REG         0x68         // X轴低字节寄存器起始地址
/*********************************************/

#define LIS2MDL_Device_ID          0x40         // LIS2MDL 设备ID 寄存器值

i2c_master_dev_handle_t            lis2mdl_handle;        // LIS2MDL 设备句柄
extern i2c_master_bus_handle_t     bus_handle;            // I2C 总线句柄

#define MAG_CAL_MIN_SAMPLES 300
typedef enum { MAG_CAL_IDLE, MAG_CAL_RUNNING, MAG_CAL_DONE } mag_cal_state_t;
static mag_cal_state_t mag_cal_state = MAG_CAL_IDLE;
static int16_t  mag_min[3]={32767,32767,32767};
static int16_t  mag_max[3]={-32768,-32768,-32768};
static float    mag_bias[3]={0}, mag_scale[3]={1,1,1};
static uint32_t mag_samples=0;


/**
 * @brief  读取 LIS2MDL 设备 ID 并验证连接状态
 * 
 * 读取 WHO_AM_I 寄存器（0x4F），判断返回值是否为 0x40。
 * 如果返回 0x40，说明 LIS2MDL 连接正常，否则连接失败。
 */
void lis2mdl_read_id(void) {
    uint8_t reg_addr = LIS2MDL_Device_ID_Ad; 
    uint8_t id = 0;

    // 先写寄存器地址，再读1字节
    ESP_ERROR_CHECK(i2c_master_transmit_receive(lis2mdl_handle, &reg_addr, 1, &id, 1, 100));
    if (id == LIS2MDL_Device_ID) {
        printf("LIS2MDL connected successfully!\n");
    } else {
        printf("Failed to connect to LIS2MDL!\n");
    }
}

/**
 * @brief  初始化 LIS2MDL 地磁传感器
 *
 * 添加 I2C 设备并配置各寄存器：
 * - CFG_REG_A：连续测量模式，ODR=100Hz，温度补偿使能
 * - CFG_REG_B：低通滤波使能（增益固定为±4高斯）
 * - CFG_REG_C：Block Data Update（BDU）使能，I2C接口
 * 初始化完成后自动读取并验证设备ID。
 */
void lis2mdl_init(void){
    uint8_t reg[2];

    // 添加一个 LIS2MDL 设备
    i2c_device_config_t dev_conf = {
        .device_address = LIS2MDL_ADDRESS,       // 设备地址
        .dev_addr_length = I2C_ADDR_BIT_7,       // 地址长度
        .scl_speed_hz = 100000,                  // SCL 频率
    };
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_conf, &lis2mdl_handle));

/************************** lis2mdl 初始化 **************************/

    /* 配置A寄存器：连续测量模式，ODR=100Hz，温度补偿使能 */ 
    reg[0] = 0x60;     // CFG_REG_A 寄存器地址
    reg[1] = 0x8C;     // 连续测量模式，ODR=100Hz，温度补偿使能
    ESP_ERROR_CHECK(i2c_master_transmit(lis2mdl_handle, reg, 2, 100));
    
    /* 配置B寄存器：增益设置为±4高斯 */
    reg[0] = 0x61;     // CFG_REG_B 寄存
    reg[1] = 0x01;     // 设置低通滤波
    ESP_ERROR_CHECK(i2c_master_transmit(lis2mdl_handle, reg, 2, 100));
    
    /* 配置C寄存器：I2C接口，连续测量模式 */
    reg[0] = 0x62;     // CFG_REG_C 寄存
    reg[1] = 0x10;     // Block Data Update（BDU）使能，I2C接口
    ESP_ERROR_CHECK(i2c_master_transmit(lis2mdl_handle, reg, 2, 100));
/********************************************************************/
    lis2mdl_read_id();
}

/**
 * @brief  读取 LIS2MDL 三轴磁场原始数据
 * @param  mag 三轴磁场原始数据数组，mag[0]=X，mag[1]=Y，mag[2]=Z
 * @note   结果单位为原始 LSB，需乘以 1.5 得到 mG
 */
void lis2mdl_read_magnet(int16_t mag[3]) {
    uint8_t reg_addr = LIS2MDL_OUTX_L_REG;
    uint8_t buf[6] = {0};

    ESP_ERROR_CHECK(i2c_master_transmit_receive(lis2mdl_handle, &reg_addr, 1, buf, 6, 100));

    mag[0] = (int16_t)(buf[1] << 8 | buf[0]); // X
    mag[1] = (int16_t)(buf[3] << 8 | buf[2]); // Y
    mag[2] = (int16_t)(buf[5] << 8 | buf[4]); // Z
}

void mag_cal_start(void){
    mag_cal_state = MAG_CAL_RUNNING;
    mag_samples = 0;
    for(int i=0;i<3;i++){
        mag_min[i]=32767; mag_max[i]=-32768;
        mag_bias[i]=0; mag_scale[i]=1;
    }
}




