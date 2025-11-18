// esp32_control.h

#ifndef __ESP32_CONTROL_H__
#define __ESP32_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

// --- Public Function Prototypes ---

/**
 * @brief Resets the ESP32-C3 and puts it into normal run mode.
 */
void Reset_ESP32_C3_To_Run(void);

/**
 * @brief Resets the ESP32-C3 and puts it into firmware download (bootloader) mode.
 */
void Reset_ESP32_C3_To_Download(void);

/**
 * @brief Disables the ESP32-C3 by holding it in a reset state.
 */
void Disable_ESP32_C3(void);


#ifdef __cplusplus
}
#endif

#endif // __ESP32_CONTROL_H__