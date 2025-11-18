#include "POM_MotionFX.h"

#include "usart.h"

#define DELATE_TIME     ((double)(0.0022))//0.0025
#define ALGO_FREQ       100U/* Algorithm frequency 100Hz*/
#define ALGO_PERIOD     (1000U / ALGO_FREQ)^/* Algorithm period [ms]*/


#define USE_LSM6DSO_APP_DEBUG                                                                                                                       //使用USE_DEBUG
#ifdef USE_LSM6DSO_APP_DEBUG                                                                                                                        //如果使用USE_DEBUG 则编译以下代码
uint8_t printf_motionfx_Buf[100];                                                                                                                 //定义发送缓冲区 该日志输入 不需要使用重定向 不需要开启微库 且可使用中断方式打印
#define LOG_MOTIONFX_DEBUG(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_motionfx_Buf, sprintf((char*)printf_motionfx_Buf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_MOTIONFX_INFO(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_motionfx_Buf, sprintf((char*)printf_motionfx_Buf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_MOTIONFX_PRINTF(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_motionfx_Buf, sprintf((char*)printf_motionfx_Buf, format "\n", ##__VA_ARGS__), 200)

#else
#define LOG_MOTIONFX_DEBUG(format, ...)
#define LOG_MOTIONFX_INFO(format, ...)
#define LOG_MOTIONFX_PRINTF(format, ...)
#endif





/*************
用法:
uint8_t FIFO_NUM;
uint8_t FIFO_BUF[30][7];
mian()
{
    LSM6DSO_Initialization();
    LSM6DSO_Set_FIFO_Mode();
    MotionFX_6x_init();
    while(1)
    {
        if(FIFO_NUM)
        {
            MotionFX_6x_Pro(FIFO_BUF,FIFO_NUM);
            FIFO_NUM = 0;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_2)
    {
        FIFO_NUM = LSM6DSO_FIFO_Buf_Get(FIFO_BUF);
    }
}
*************/


sensor_hub_data_t sensor_hub_data;
uint32_t deltatime_1,deltatime_2;

static uint8_t mfxstate_6x[FX_STATE_SIZE];



/**
  * @brief  Initialize 6-axis MotionFX sensor fusion algorithm for LSM6DSO
  * @note   Configures algorithm parameters including:
  *         - Sensor bias thresholds
  *         - Axis orientation matrices
  *         - Output data format
  *         - Operational modes
  * @retval None
  * @note   Critical configurations:
  *         - Uses ENU (East-North-Up) coordinate system by default
  *         - Enables 6-axis fusion (accel + gyro)
  *         - Disables 9-axis fusion (accel + gyro + mag)
  *         - Applies device-specific orientation corrections
  */
void MotionFX_6x_init(void)
{
    MFX_knobs_t iKnobs;
    MFX_knobs_t *ipKnobs = &iKnobs;
    
    MotionFX_initialize((MFXState_t *)mfxstate_6x);
    MotionFX_getKnobs(mfxstate_6x,ipKnobs);
    
    ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC_6X;
    ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC_6X;
    ipKnobs->gbias_mag_th_sc = GBIAS_MAG_TH_SC_6X;
    
    ipKnobs->acc_orientation[0] = ACC_ORIENTATION_X;
    ipKnobs->acc_orientation[1] = ACC_ORIENTATION_Y;
    ipKnobs->acc_orientation[2] = ACC_ORIENTATION_Z;
    
    ipKnobs->gyro_orientation[0] = GYR_ORIENTATION_X;
    ipKnobs->gyro_orientation[1] = GYR_ORIENTATION_Y;
    ipKnobs->gyro_orientation[2] = GYR_ORIENTATION_Z;
    
    ipKnobs->mag_orientation[0] = MAG_ORIENTATION_X;
    ipKnobs->mag_orientation[1] = MAG_ORIENTATION_Y;
    ipKnobs->mag_orientation[2] = MAG_ORIENTATION_Z;
    
    ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
    ipKnobs->LMode = 1;
    
    ipKnobs->modx = 1;
    
    MotionFX_setKnobs(mfxstate_6x,ipKnobs);
    
    MotionFX_enable_6X(mfxstate_6x,MFX_ENGINE_ENABLE);
    
    MotionFX_enable_9X(mfxstate_6x,MFX_ENGINE_DISABLE);
}

/**
  * @brief  Execute 6-axis MotionFX sensor fusion algorithm
  * @note   Processes accelerometer and gyroscope data to compute:
  *         - Orientation quaternion/angles
  *         - Corrected sensor data
  *         - Motion states
  * @retval None
  * @note   Critical operations:
  *         - Converts sensor units to MotionFX standard units (g for accel, dps for gyro)
  *         - Handles timestamp overflow (32-bit rollover)
  *         - Skips processing when delta_time=0
  *         - Magnetometer data forced to zero (6-axis mode)
  */
void MotionFX_6x_Determin(void)
{
    sensor_hub_data.acceleration[0] = sensor_hub_data.acc.x;
    sensor_hub_data.acceleration[1] = sensor_hub_data.acc.y;
    sensor_hub_data.acceleration[2] = sensor_hub_data.acc.z;
    
    sensor_hub_data.angular_rate[0] = sensor_hub_data.gyr.x;
    sensor_hub_data.angular_rate[1] = sensor_hub_data.gyr.y;
    sensor_hub_data.angular_rate[2] = sensor_hub_data.gyr.z;
    
    MFX_input_t mfx_data_in;
    
    mfx_data_in.acc[0] = sensor_hub_data.acceleration[0] * FROM_MG_TO_G;
    mfx_data_in.acc[1] = sensor_hub_data.acceleration[1] * FROM_MG_TO_G;
    mfx_data_in.acc[2] = sensor_hub_data.acceleration[2] * FROM_MG_TO_G;
    
    mfx_data_in.gyro[0] = sensor_hub_data.angular_rate[0] * FROM_MDPS_TO_DPS;
    mfx_data_in.gyro[1] = sensor_hub_data.angular_rate[1] * FROM_MDPS_TO_DPS;
    mfx_data_in.gyro[2] = sensor_hub_data.angular_rate[2] * FROM_MDPS_TO_DPS;
    
    mfx_data_in.mag[0] = 0;
    mfx_data_in.mag[1] = 0;
    mfx_data_in.mag[2] = 0;
    
    float delta_time;
    if(deltatime_2 > deltatime_1)
    {
        delta_time = (float)((double)(deltatime_2 - deltatime_1) * 25.0f/1000000);
        MotionFX_propagate(mfxstate_6x,&sensor_hub_data.mfx_6x,&mfx_data_in,&delta_time);
        MotionFX_update(mfxstate_6x,&sensor_hub_data.mfx_6x,&mfx_data_in,&delta_time,NULL);
    }
    else if(deltatime_1 > deltatime_2)
    {
        delta_time = (float)((double)(0xffffffff-deltatime_2+deltatime_1)*25.0f/1000000);
        MotionFX_propagate(mfxstate_6x,&sensor_hub_data.mfx_6x,&mfx_data_in,&delta_time);
        MotionFX_update(mfxstate_6x,&sensor_hub_data.mfx_6x,&mfx_data_in,&delta_time,NULL);
    }
    else if(deltatime_1 == deltatime_2)
    {
        delta_time = 0.0f;
    }
}


/**
  * @brief  Process LSM6DSO FIFO data for MotionFX 6-axis sensor fusion
  * @param  FIFO_data  2D array containing raw FIFO data (each row: [tag, data0..data5])
  * @param  fifo_num   Number of valid samples in FIFO_data
  * @retval int8_t     Execution status:
  *                    - POM_MOTIONFX_OK if processing completes successfully
  *                    - POM_MOTIONFX_ERROR if sensitivity calibration fails
  * @note   - Processes accelerometer, gyroscope and timestamp data from FIFO
  *         - Applies sensitivity calibration to convert raw data to physical units
  *         - Triggers sensor fusion (MotionFX_6x_Determin) when all required data is available
  *         - Timestamp handling includes delta-time calculation between samples
  */
int8_t MotionFX_6x_Pro(uint8_t FIFO_data[][7],uint8_t fifo_num)
{
    uint8_t temp_num;
    float_t sensitivity[2] = {0.0f,0.0f};
    uint32_t timestamp;
    uint8_t first_time_flag = 0;
    uint8_t acc_flag=0,gyr_flag=0,deltatime_flag=0;//加速度 陀螺仪 时间戳 标志位
    int16_t *datax,*datay,*dataz;
    
    if (LSM6DSO_ACC_GetSensitivity(&lsm6dso_obj, &sensitivity[0]) != LSM6DSO_OK)
    {
        return POM_MOTIONFX_ERROR;
    }
    if (LSM6DSO_GYRO_GetSensitivity(&lsm6dso_obj, &sensitivity[1]) != LSM6DSO_OK)
    {
        return POM_MOTIONFX_ERROR;
    }
    
    for(temp_num=0;temp_num<fifo_num;temp_num++)// 遍历 FIFO 数据数组
    {
        
        datax = (int16_t *)&FIFO_data[temp_num][1];
        datay = (int16_t *)&FIFO_data[temp_num][3];
        dataz = (int16_t *)&FIFO_data[temp_num][5];
        switch (FIFO_data[temp_num][0])
        {
            case LSM6DSO_XL_NC_TAG:// 加速度数据
                acc_flag = 1;
                sensor_hub_data.acc.x = *datax*sensitivity[0];
                sensor_hub_data.acc.y = *datay*sensitivity[0];
                sensor_hub_data.acc.z = *dataz*sensitivity[0];
                break;
            case LSM6DSO_GYRO_NC_TAG://陀螺仪数据
                gyr_flag = 1;
                sensor_hub_data.gyr.x = *datax*sensitivity[1];
                sensor_hub_data.gyr.y = *datay*sensitivity[1];
                sensor_hub_data.gyr.z = *dataz*sensitivity[1];
                break;
            case LSM6DSO_TIMESTAMP_TAG://时间戳数据
                deltatime_flag = 1;
                timestamp = 0;
                timestamp = FIFO_data[temp_num][1] + (FIFO_data[temp_num][2]<<8) + (FIFO_data[temp_num][3]<<16) + (FIFO_data[temp_num][4]<<24);
                if(first_time_flag == 0)
                {
                    deltatime_1=timestamp;
                    deltatime_2=deltatime_1;
                    first_time_flag = 1;
                }
                else
                {
                    deltatime_2=timestamp;
                }
                break;
            default:
                break;
        }
        // 数据已经全部获取
        if(acc_flag && gyr_flag && deltatime_flag)
        {
            MotionFX_6x_Determin();
            acc_flag = 0;
            gyr_flag = 0;
            deltatime_flag = 0;
            deltatime_1=deltatime_2;// 更新时间戳
        }
    }
    return POM_MOTIONFX_OK;
}
