/**
  ******************************************************************************
  * @file    esp32_control.c
  * @brief   Provides functions to control the state of the onboard ESP32-C3.
  ******************************************************************************
  */

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "esp32_control.h"

/* Private Defines -----------------------------------------------------------*/

// --- Pin Mapping based on the provided PDF ---
// G4's ESP32_EN_Pin -> RP GPIO23
#define ESP_EN_PIN          23

// G4's ESP32_IO9_Pin -> RP GPIO25
#define ESP_IO9_PIN         25

/* Public Functions ----------------------------------------------------------*/

/**
 * @brief Resets the ESP32-C3 and puts it into normal run mode.
 * @note  This is the equivalent of the G4 function.
 */
void Reset_ESP32_C3_To_Run(void)
{
    // 1. Pull EN low to reset the ESP32
    // G4: HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP32_EN_Pin, GPIO_PIN_RESET);
    gpio_put(ESP_EN_PIN, 0);

    // 2. Pull IO9 high for normal boot mode
    // G4: HAL_GPIO_WritePin(ESP32_IO9_GPIO_Port, ESP32_IO9_Pin, GPIO_PIN_SET);
    gpio_put(ESP_IO9_PIN, 1);

    // 3. Wait for a moment to ensure pin states are stable
    // G4: HAL_Delay(10);
    sleep_ms(10);

    // 4. Pull EN high to end reset, causing the ESP32 to boot
    // G4: HAL_GPIO_WritePin(ESP32_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    gpio_put(ESP_EN_PIN, 1);
}

/**
 * @brief Resets the ESP32-C3 and puts it into firmware download (bootloader) mode.
 * @note  This is the equivalent of the G4 function.
 */
void Reset_ESP32_C3_To_Download(void)
{
    // 1. Pull EN low to reset the ESP32
    gpio_put(ESP_EN_PIN, 0);

    // 2. Pull IO9 low for download boot mode
    // G4: HAL_GPIO_WritePin(ESP32_IO9_GPIO_Port, ESP32_IO9_Pin, GPIO_PIN_RESET);
    gpio_put(ESP_IO9_PIN, 0);

    // 3. Wait for a moment
    sleep_ms(10);

    // 4. Pull EN high to end reset, causing the ESP32 to enter bootloader
    gpio_put(ESP_EN_PIN, 1);
}

/**
 * @brief Disables the ESP32-C3 by holding it in a reset state.
 * @note  This is the equivalent of the G4 function.
 */
void Disable_ESP32_C3(void)
{
    // Pull EN low and keep it low to disable the chip
    gpio_put(ESP_EN_PIN, 0);
}