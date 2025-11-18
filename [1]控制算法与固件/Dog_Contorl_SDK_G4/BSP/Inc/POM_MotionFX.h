#ifndef _POM_MOTION_FX_H__
#define _POM_MOTION_FX_H__

#include "stdio.h"
#include "motion_fx.h"

#include "POM_lsm6dso.h"
#include "POM_lis2mdl.h"


#define POM_MOTIONFX_OK     0
#define POM_MOTIONFX_ERROR -1

typedef struct{
    float_t x;
    float_t y;
    float_t z;
}Accelerometer;

typedef struct{
    float_t x;
    float_t y;
    float_t z;
}Gyroscope;

typedef struct{
    float_t x;
    float_t y;
    float_t z;
}Magnetometer;


typedef struct{
    Accelerometer acc;
    Gyroscope gyr;
    Magnetometer mag;
    float acceleration[3];
    float angular_rate[3];
    float magnetism[3];
    MFX_output_t mfx_6x ;       //六轴解算输出结果在这里
    MFX_output_t mfx_9x ;       //九轴解算输出结果在这里
}sensor_hub_data_t;

extern sensor_hub_data_t sensor_hub_data;

extern uint32_t deltatime_1,deltatime_2;

/*MFX算法需要的宏，来自AlgoBuilded生成代码*/
#define FX_STATE_SIZE (size_t)(2432)
#define ACC_ORIENTATION_X 'n'
#define ACC_ORIENTATION_Y 'w'
#define ACC_ORIENTATION_Z 'u'

#define GYR_ORIENTATION_X 'n'
#define GYR_ORIENTATION_Y 'w'
#define GYR_ORIENTATION_Z 'u'

#define MAG_ORIENTATION_X 'n'
#define MAG_ORIENTATION_Y 'w'
#define MAG_ORIENTATION_Z 'u'

#define GBIAS_ACC_TH_SC_6X  (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X  (2.0f*0.001500f)

#define GBIAS_ACC_TH_SC_9x  (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X  (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9x  (2.0f*0.001500f)

#define FROM_MG_TO_G        0.001f
#define FROM_MDPS_TO_DPS    0.001f
#define FROM_MAGUSS_TO_UT50 (0.1f/50.0f)


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
void MotionFX_6x_init(void);



///**
//  * @brief  Execute 6-axis MotionFX sensor fusion algorithm
//  * @note   Processes accelerometer and gyroscope data to compute:
//  *         - Orientation quaternion/angles
//  *         - Corrected sensor data
//  *         - Motion states
//  * @retval None
//  * @note   Critical operations:
//  *         - Converts sensor units to MotionFX standard units (g for accel, dps for gyro)
//  *         - Handles timestamp overflow (32-bit rollover)
//  *         - Skips processing when delta_time=0
//  *         - Magnetometer data forced to zero (6-axis mode)
//  */
//void MotionFX_6x_Determin(void);

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
  * @use    Place it in the while for optimal use
  */
int8_t MotionFX_6x_Pro(uint8_t FIFO_data[][7],uint8_t fifo_num);


#endif



