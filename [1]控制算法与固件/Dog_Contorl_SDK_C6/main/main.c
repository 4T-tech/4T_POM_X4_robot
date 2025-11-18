#include "include.h"
             
#define BUF_SIZE 1024

float pitch, roll, yaw;
float LSM6DSOW_Yaw, LSM6DSOW_Roll, LSM6DSOW_Pitch;

extern uint16_t ADDR;
volatile uint8_t Mode = 0;

uint32_t RISE_TIME = 0;

extern void mag_cal_start(void);
extern bool mag_cal_is_done(void);

/*************************** 任务参数设置区域 **************************************/
TaskHandle_t usb_rx_task_handle = NULL;
uint32_t     usb_rx_task_stack  = 2048;
uint32_t     usb_rx_task_prio   = 5;

TaskHandle_t uart_task_handle = NULL;
uint32_t     uart_task_stack  = 2048;
uint32_t     uart_task_prio   = 5;

TaskHandle_t dog_control_handle = NULL;
uint32_t     dog_control_stack  = 4096;
uint32_t     dog_control_prio   = 6;

TaskHandle_t stand_up_handle = NULL;
uint32_t     stand_up_stack  = 2048;
uint32_t     stand_up_prio   = 5;

/**********************************************************************************/

/**
 * @brief   一次性任务，用于执行机器人的站立动作。
 * @note    此任务在完成其使命后会自我销毁
 */
void Stand_Up_Task(void *arg){
    Stand_Up_Smoothly();
    vTaskDelete(NULL);
}

/**
 * @brief   机器人核心控制任务，以固定的20ms周期运行。
 * @details 该任务周期性地获取姿态角，执行模式控制和运动控制函数。
 */
void Dog_Control_Task(void *arg){
    ESP_LOGI("Task","Dog_Control_Task Start");
    const TickType_t period = pdMS_TO_TICKS(20);   // 20ms 周期
    TickType_t last_wake = xTaskGetTickCount();    // 记录基准时刻
    while(1){
        // 获取姿态角
        Get_Madgwick_Angle(&pitch,&roll,&yaw);
        LSM6DSOW_Pitch = pitch;
        LSM6DSOW_Roll  = roll;
        LSM6DSOW_Yaw   = -yaw;
        // 核心控制函数
        Mode_Control();                    // 数据计算
        Dog_Control();                     // 发送控制指令
        vTaskDelayUntil(&last_wake, period);
    }
}

void app_main(void) {
    
    uart_init();     // 初始化串口
    lsm6dsow_init(); // 初始化陀螺仪
    lis2mdl_init();  // 初始化磁力计
    BLE_Init();

    // 没有用到定时器故注释掉
    // Timer1_Init();   // 初始化定时器
    // Timer2_Init();   // 初始化定时器

    xTaskCreatePinnedToCore(Dog_Control_Task, "Dog_Control_Task", dog_control_stack, NULL, dog_control_prio, &dog_control_handle,0);    // 机器人核心控制任务
    xTaskCreatePinnedToCore(Stand_Up_Task, "Stand_Up_Task", stand_up_stack, NULL, stand_up_prio, &stand_up_handle,0);                   // 机器人站立任务（只执行一次）
    
    xTaskCreatePinnedToCore(usb_rx_task, "usb_rx_task", usb_rx_task_stack, NULL, usb_rx_task_prio, &usb_rx_task_handle,0);              // 用于CDC接收控制指令
    xTaskCreatePinnedToCore(UART_Rx_Task, "UART_Rx_Task", uart_task_stack, NULL, uart_task_prio, &uart_task_handle,0);                  // 用于串口接收温度和电压数据
    
}

void Delay_MS(uint32_t ms){
    vTaskDelay(pdMS_TO_TICKS(ms));
}