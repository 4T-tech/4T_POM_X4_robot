#ifndef _POM_LIS2MDL_H__
#define _POM_LIS2MDL_H__

/*
使用方法 在CubMX中选择 X-CUBE-MEMS1 传感器 选择lis2mdl器件并选择为I2C1通讯
            I2C1总线引脚为   PA15 PB9
            中断1引脚为      PA7
在使用前 需要调用LIS2MDL_Initialization()来初始化器件
之后设置器件的运行模式     - LIS2MDL_Set_Poll_Mode()       //轮询模式

在轮询模式下 需要使用LIS2MDL_Poll_MAG_Value_Get来得到数据
*/



#include "custom_bus.h"
#include "lis2mdl.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"

#define POM_LIS2MDL_OK          0
#define POM_LIS2MDL_ERROR       -1

extern LIS2MDL_Object_t lis2mdl_obj;

/**
  * @brief  Initialize the LIS2MDL magnetometer sensor
  * @retval POM_LIS2MDL_OK if initialization succeeds
  * @retval POM_LIS2MDL_ERROR if any initialization step fails
  * @note   Performs the following initialization sequence:
  *         - Clears the device object memory
  *         - Registers I/O interface functions
  *         - Verifies device ID (WHO_AM_I = 0x40)
  *         - Initializes the sensor
  */
int8_t LIS2MDL_Initialization(void);




/**
  * @brief  Configure LIS2MDL magnetometer in polling mode
  * @retval POM_LIS2MDL_OK if configuration succeeds
  * @retval POM_LIS2MDL_ERROR if any configuration step fails
  * @note   Configures the sensor with:
  *         - Output data rate: 10Hz
  *         - Power mode: continuous conversion (mode 0)
  *         - Enables the magnetometer
  */
int8_t LIS2MDL_Set_Poll_Mode(void);



/**
  * @brief  Get magnetometer data from LIS2MDL sensor (polling mode)
  * @param  MAG_Buf Pointer to structure for storing magnetometer axes data (x/y/z in Gauss)
  * @retval POM_LIS2MDL_OK New data successfully read
  * @retval POM_LIS2MDL_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Non-blocking function - returns immediately if no new data
  *         Data units are in Gauss (default sensor configuration)
  */
int8_t LIS2MDL_Poll_MAG_Value_Get(LIS2MDL_Axes_t *MAG_Buf);

#endif

