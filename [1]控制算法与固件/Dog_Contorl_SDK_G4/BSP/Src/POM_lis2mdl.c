#include "POM_lis2mdl.h"


//#define USE_LIS2MDL_DEBUG       
#ifdef USE_LIS2MDL_DEBUG        
uint8_t printf_lis2mdl_Buf[100];
#define LOG_LIS2MDL_DEBUG(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lis2mdl_Buf, sprintf((char*)printf_lis2mdl_Buf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_LIS2MDL_INFO(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lis2mdl_Buf, sprintf((char*)printf_lis2mdl_Buf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__), 200)
#define LOG_LIS2MDL_PRINTF(format, ...) HAL_UART_Transmit((UART_HandleTypeDef *)&huart1, (uint8_t *)printf_lis2mdl_Buf, sprintf((char*)printf_lis2mdl_Buf, format "\n", ##__VA_ARGS__), 200)

#else
#define LOG_LIS2MDL_DEBUG(format, ...)
#define LOG_LIS2MDL_INFO(format, ...)
#define LOG_LIS2MDL_PRINTF(format, ...)
#endif



/*************
用法①:

LIS2MDL_Axes_t LIS2MDL_MAG_Num;
mian()
{
    LIS2MDL_Initialization();
    LIS2MDL_Set_Poll_Mode();
    while(1)
    {
        if(LIS2MDL_Poll_MAG_Value_Get(&LIS2MDL_MAG_Num) == POM_LIS2MDL_OK)
        {
        
        }
    }
}

*/





LIS2MDL_Object_t lis2mdl_obj;


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
int8_t LIS2MDL_Initialization(void)
{
    /* 清零对象内存 */
    memset(&lis2mdl_obj, 0, sizeof(LIS2MDL_Object_t));
    
     /* 注册IO接口 */
    LIS2MDL_IO_t io_ctx = {
        .Init = BSP_I2C1_Init,
        .DeInit = BSP_I2C1_DeInit,
        .BusType = LIS2MDL_I2C_BUS,
        .Address = LIS2MDL_I2C_ADD,
        .WriteReg = BSP_I2C1_WriteReg,
        .ReadReg = BSP_I2C1_ReadReg,
        .GetTick = BSP_GetTick,
        .Delay = HAL_Delay
    };
    if(LIS2MDL_RegisterBusIO(&lis2mdl_obj,&io_ctx) != LIS2MDL_OK)
    {
        LOG_LIS2MDL_INFO("Failed to register bus IO");
        return POM_LIS2MDL_ERROR;
    }
    if(LIS2MDL_Init(&lis2mdl_obj) != LIS2MDL_OK)
    {
        LOG_LIS2MDL_INFO("Failed to initialize LIS2MDL");
        return POM_LIS2MDL_ERROR;
    }
    /* 获取WHO_AM_I 确保传感器运行正常*/
    uint8_t WHO_AM_I_Reg;
    if(LIS2MDL_ReadID(&lis2mdl_obj,&WHO_AM_I_Reg) != LIS2MDL_OK)
    {
        LOG_LIS2MDL_INFO("Failed to LIS2MDL Read WHO_AM_I_Reg");
        return POM_LIS2MDL_ERROR;
    }
    if(WHO_AM_I_Reg != LIS2MDL_ID)
    {
        LOG_LIS2MDL_INFO("Failed to LIS2MDL WHO_AM_I_Reg = 0x%x",WHO_AM_I_Reg);
    }
    //LOG_LIS2MDL_INFO("LIS2MDL initialized successfully");
    return POM_LIS2MDL_OK;
}

/**
  * @brief  Configure LIS2MDL magnetometer in polling mode
  * @retval POM_LIS2MDL_OK if configuration succeeds
  * @retval POM_LIS2MDL_ERROR if any configuration step fails
  * @note   Configures the sensor with:
  *         - Output data rate: 10Hz
  *         - Power mode: continuous conversion (mode 0)
  *         - Enables the magnetometer
  */
int8_t LIS2MDL_Set_Poll_Mode(void)
{
    //设置磁力计测量频率为10Hz
    if(LIS2MDL_MAG_SetOutputDataRate(&lis2mdl_obj,10) != LIS2MDL_OK)
    {
        return POM_LIS2MDL_ERROR;
    }
    //设置测力计电源模式为 LIS2MDL_HIGH_RESOLUTION
    if(LIS2MDL_MAG_Set_Power_Mode(&lis2mdl_obj,0) != LIS2MDL_OK)
    {
        return POM_LIS2MDL_ERROR;
    }
    //使能磁力计
    if(LIS2MDL_MAG_Enable(&lis2mdl_obj) != LIS2MDL_OK)
    {
        return POM_LIS2MDL_ERROR;
    }
    return POM_LIS2MDL_OK;
}


/**
  * @brief  Get magnetometer data from LIS2MDL sensor (polling mode)
  * @param  MAG_Buf Pointer to structure for storing magnetometer axes data (x/y/z in Gauss)
  * @retval POM_LIS2MDL_OK New data successfully read
  * @retval POM_LIS2MDL_ERROR No new data available
  * @note   Uses polling approach - checks DRDY status before reading
  *         Non-blocking function - returns immediately if no new data
  *         Data units are in Gauss (default sensor configuration)
  */
int8_t LIS2MDL_Poll_MAG_Value_Get(LIS2MDL_Axes_t *MAG_Buf)
{
    uint8_t reg;
    //检查数据状态
    LIS2MDL_MAG_Get_DRDY_Status(&lis2mdl_obj,&reg);
    if (reg) 
    {
        //得到数据
        LIS2MDL_MAG_GetAxes(&lis2mdl_obj,MAG_Buf);
        return POM_LIS2MDL_OK;
    }
    else
    {
        return POM_LIS2MDL_ERROR;
    }
}

