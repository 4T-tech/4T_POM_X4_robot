/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "usart.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "G431_Series.h"
#include "reg_protocol.h"

#include "Dog_Sport.h"
#include "leg_ik.h"
#include "usbd_cdc_if.h"
#include "POM_lsm6dso.h"
#include "POM_lis2mdl.h"
#include "POM_MotionFX.h" 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_DEBUG
#define USE_INFO
#define USE_PRINT

#define BUF_SIZE 1024

#if defined(USE_DEBUG) || defined(USE_INFO) || defined(USE_PRINT)
uint8_t printfBuf[100];
#endif
#ifdef USE_DEBUG
#define LOG_DEBUG(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_DEBUG(format, ...)
#endif
#ifdef USE_INFO
#define LOG_INFO(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_INFO(format, ...)
#endif
#ifdef USE_PRINT
#define LOG_PRINT(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[PRINT] %s(): " format "\n", __func__, ##__VA_ARGS__))
#else
#define LOG_PRINT(format, ...)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//---变量定义区:

extern uint16_t Servo_Pwm[12]; // 从 leg_ik.c 外部声明 PWM 数组

#define HAL_MAX_DELAY      0xFFFFFFFFU

uint8_t uart_rx2_buf1[BUF_SIZE], uart_rx2_buf2[BUF_SIZE];

uint8_t UART2_TX_State = 0;

unsigned char CDC_RX_FLAG = 0;

extern uint16_t REG_ADDR[REG_LEN]; 

extern uint16_t ADDR;
extern uint16_t UP;
uint8_t uart_rx4_buf1[BUF_SIZE];
uint8_t *BUF_addr = uart_rx4_buf1;

volatile uint8_t Mode = 0;

unsigned char UART4_RX_Result = 0;

unsigned char UART2_RX_Result = 0;

uint8_t  FIFO_TX_BUF[50];
uint8_t FIFO_NUM = 0;
uint8_t FIFO_BUF[60][7];

uint32_t RISE_TIME = 0;
uint32_t Pos_Stand_Time = 0;

float LSM6DSOW_Yaw;   
float LSM6DSOW_Roll;  
float LSM6DSOW_Pitch; 
 
uint32_t sysTickMs = 0; 

uint8_t TIMER_20MS_FLAG = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CorrectServo_90();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_UARTEx_ReceiveToIdle_IT(&huart2,uart_rx2_buf1,BUF_SIZE);           
  HAL_UARTEx_ReceiveToIdle_IT(&huart4,BUF_addr,BUF_SIZE);
  Reset_ESP32_C3_To_Run();
  HAL_Delay(1000);
  
    if(LSM6DSO_Initialization() != POM_LSM6DSO_OK)
    {
      send_str("LSM6DSO Init Failed\r\n");
    }
    if(LSM6DSO_Set_FIFO_Mode() != POM_LSM6DSO_OK)
    {
      send_str("LSM6DSO Set Failed\r\n");
    }
    MotionFX_6x_init();
    
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim1);

  Stand_Up_Smoothly();
		
		
		uint16_t cnt = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
      //send_str("init");
    if(TIMER_20MS_FLAG)
    { 
				cnt ++;
				if(cnt > 100)
				{
					send_str("running");
					cnt = 0;
				}
         TIMER_20MS_FLAG = 0;
         switch(ADDR)
        {
         case 43:
             Read_Battery_Volt();
         break;
         case 44:
             Read_Temp();
         break;
        }
        ADDR = 0;

    }
    
    
    if(FIFO_NUM)
    {
        MotionFX_6x_Pro(FIFO_BUF,FIFO_NUM);
        LSM6DSOW_Yaw    = sensor_hub_data.mfx_6x.rotation[0];                         // /yaw
        LSM6DSOW_Roll    = sensor_hub_data.mfx_6x.rotation[1];                         // roll
        LSM6DSOW_Pitch   = sensor_hub_data.mfx_6x.rotation[2];                         //  pitch
//        sprintf((char*)FIFO_TX_BUF,"Roll:%.2f , pitch:%.2f , yaw:%.2f",LSM6DSOW_Roll,LSM6DSOW_Pitch,LSM6DSOW_Yaw);
//        send_str((char*)FIFO_TX_BUF);
//        Send_BLE((char*)FIFO_TX_BUF);
        FIFO_NUM = 0;
    }
    

    if(CDC_RX_FLAG == 1)
    {
         switch(ADDR)
        {
            case 43:
                REG_6Byte_Send(79 , 43);             //发送读取电压6字节指令
            break;
            case 44:
                REG_6Byte_Send(79 , 44);             //发送读取温度6字节指令 
            break;
        }
        CDC_RX_FLAG = 0;
    }
            
    if(UART4_RX_Result == 1)
    {
         switch(ADDR)
        {
            case 43:
                REG_6Byte_Send(79 , 43);             //发送读取电压6字节指令
            break;
            case 44:
                REG_6Byte_Send(79 , 44);             //发送读取温度6字节指令
            break;
        }
        UART4_RX_Result = 0;
    }

    if(UART2_RX_Result == 1)
    {

         UART2_RX_Result =0;
    }
		
//	CorrectServo_90();
    

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance == USART2)
    {
        UART2_RX_Result = REG_FeedBytes(uart_rx2_buf1,Size);
        HAL_UARTEx_ReceiveToIdle_IT(&huart2,uart_rx2_buf1,BUF_SIZE);
    }
     if(huart->Instance == UART4)
    {
        UART4_RX_Result = String_parsing(BUF_addr,Size);  
        HAL_UARTEx_ReceiveToIdle_IT(&huart4,BUF_addr,BUF_SIZE);
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    
  if(htim->Instance == htim1.Instance)
  {         
      Dog_Control();
      TIMER_20MS_FLAG  = 1;
 
  }
  if(htim == &htim2)
  {    
      Pos_Stand_Time++;
      sysTickMs++;
      
      if(Pos_Stand_Time >=  20)
      {   
          Pos_Stand_Time -= 20;
          Mode_Control();
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


void Delay_MS(uint32_t ms)
{
    uint32_t start = sysTickMs;
    while ((sysTickMs - start) < ms)
    {
//         __WFI();  //非阻塞式延时函数
    }
}

void Set_All_Servos_To_90_Degrees(void)
{
    const uint16_t pwm_90_degrees = 75;
    
    for (int i = 0; i < 12; ++i)
    {
        Servo_Pwm[i] = 75; // 将所有12个舵机的PWM值都设置为75
    }
}



void CorrectServo_90()
{
	
		Set_All_Servos_To_90_Degrees();

    // 2. 将打包好的PWM值通过HEX指令发送出去
    Send_32Servo_HEX();

    // 3. 延时一段时间后，再次发送，以维持姿态
    HAL_Delay(20); // 保持50Hz的刷新率
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
