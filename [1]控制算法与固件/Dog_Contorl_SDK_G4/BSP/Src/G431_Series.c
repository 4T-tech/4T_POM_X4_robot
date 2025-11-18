#include "G431_Series.h"

void Reset_ESP32_C3_To_Run(void)
{
    HAL_GPIO_WritePin(ESP32_EN_GPIO_Port,ESP32_EN_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP32_IO9_GPIO_Port,ESP32_IO9_Pin,GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ESP32_EN_GPIO_Port,ESP32_EN_Pin,GPIO_PIN_SET);
}

void Reset_ESP32_C3_To_Download(void)
{
    HAL_GPIO_WritePin(ESP32_EN_GPIO_Port,ESP32_EN_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP32_IO9_GPIO_Port,ESP32_IO9_Pin,GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(ESP32_EN_GPIO_Port,ESP32_EN_Pin,GPIO_PIN_SET);
}

void Disable_ESP32_C3(void)
{
    HAL_GPIO_WritePin(ESP32_EN_GPIO_Port,ESP32_EN_Pin,GPIO_PIN_RESET);
}


