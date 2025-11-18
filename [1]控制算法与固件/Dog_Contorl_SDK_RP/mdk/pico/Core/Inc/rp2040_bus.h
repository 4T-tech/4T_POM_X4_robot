// rp2040_bus.h

#ifndef __RP2040_BUS_H__
#define __RP2040_BUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// --- 定义错误码，与G4的BSP保持兼容 ---
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED  -11
#define BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE -12

// --- 函数原型声明 ---
// 这些函数将是我们对 G4 BSP API 的完整实现
int32_t BSP_I2C1_Init(void);
int32_t BSP_I2C1_DeInit(void);
int32_t BSP_I2C1_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_I2C1_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t len);
int32_t BSP_GetTick(void);

// 注意：G4中的其他BSP函数（如 IsReady, Send, Recv, WriteReg16等）在 LSM6DSO 驱动中并未使用。
// 为了保持简洁，我们只实现LSM6DSO驱动实际调用的这几个核心函数。
// 如果将来有其他驱动需要它们，可以再进行添加。

#ifdef __cplusplus
}
#endif

#endif // __RP2040_BUS_H__