#ifndef _POM_LSM6DSO_H__
#define _POM_LSM6DSO_H__

/*
使用方法 在CubMX中选择 X-CUBE-MEMS1 传感器 选择lsm6dso器件并选择为I2C1通讯
            I2C1总线引脚为   PA15 PB9
            中断1引脚为      PB2
在使用前 需要调用LSM6DSO_Initialization()来初始化器件
之后设置器件的运行模式     - LSM6DSO_Set_Poll_Mode()       //轮询模式
                         -      //FIFO中断模式

在轮询模式下 需要使用LSM6DSO_Poll_ACC_Value_Get和LSM6DSO_Poll_GYRO_Value_Get来得到数据
在FIFO中断模式下 需要在EXIT2中断中调用 和 得到数据
*/


#include "custom_bus.h"
#include "lsm6dso.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"


#define POM_LSM6DSO_OK          0
#define POM_LSM6DSO_ERROR       -1

extern LSM6DSO_Object_t lsm6dso_obj;

/**
  *@brief Initialize the LSM6DSO sensor
  *@retval POM_LSM6DSO_OK in case of success, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Initialization(void);



/**
  * @brief  Configure LSM6DSO sensor in polling mode
  * @details This function configures the accelerometer and gyroscope with:
  *          - Accelerometer full scale ±2g
  *          - Gyroscope full scale ±2000dps  
  *          - ODR 12.5Hz for both sensors
  *          - Basic analog filter configuration
  *          - Enables both sensors
  * @retval POM_LSM6DSO_OK if configuration successful, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Set_Poll_Mode(void);


/**
  * @brief  Configure LSM6DSO sensor in FIFO mode
  * @details This function configures the accelerometer and gyroscope with:
  *          - Accelerometer full scale ±2g
  *          - Gyroscope full scale ±2000dps  
  *          - FIFO Watermark_Level scale 30
  *          - FIFO 417Hz for both sensors
  *          - ODR 417Hz for both sensors
  *          - DRDY Set Mode to LSM6DSO_DRDY_PULSED
  *          - Trigger INT1 jump when FIFO reaches threshold
  *          - Enables timestamp
  *          - Enables both sensors
  * @retval POM_LSM6DSO_OK if configuration successful, POM_LSM6DSO_ERROR otherwise
  */
int8_t LSM6DSO_Set_FIFO_Mode(void);

/**
  * @brief  Get accelerometer data from LSM6DSO sensor (polling mode)
  * @param  ACC_Buf Pointer to structure for storing accelerometer axes data (x/y/z)
  * @retval POM_LSM6DSO_OK New data successfully read
  * @retval POM_LSM6DSO_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Ensures only fresh data is retrieved
  */
int8_t LSM6DSO_Poll_ACC_Value_Get(LSM6DSO_Axes_t *ACC_Buf);




/**
  * @brief  Get gyroscope data from LSM6DSO sensor (polling mode)
  * @param  GYRO_Buf Pointer to structure for storing gyroscope axes data (x/y/z in dps)
  * @retval POM_LSM6DSO_OK New data successfully read
  * @retval POM_LSM6DSO_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Non-blocking function - returns immediately if no new data
  */
int8_t LSM6DSO_Poll_GYRO_Value_Get(LSM6DSO_Axes_t *GYRO_Buf);


/**
  * @brief  Retrieve FIFO data from LSM6DSO sensor when Watermark (WTM) threshold is reached
  * @param  FIFO_data Pointer to a 2D array for storing FIFO data (each row contains [tag, data0..data5])
  * @retval uint8_t Number of samples read if WTM flag is set
  * @retval POM_LSM6DSO_ERROR If WTM flag is not set (no data available)
  * @note   - Checks FIFO Watermark interrupt flag before reading
  *         - Reads all available samples up to the WTM threshold
  *         - Each sample consists of: 
  *             - 1-byte TAG (FIFO_data[i][0]) 
  *             - 6-byte data payload (FIFO_data[i][1..6])
  *         - Typical usage: Call in interrupt handler when WTM event occurs
  *         - FIFO_data array must be pre-allocated with sufficient rows (≥fifo_num)
  * @use    Place it in the interrupt for optimal use
  */
int8_t LSM6DSO_FIFO_Buf_Get(uint8_t FIFO_data[][7]);
#endif


