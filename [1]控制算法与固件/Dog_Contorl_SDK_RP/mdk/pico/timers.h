// timers.h

#ifndef __TIMERS_H__
#define __TIMERS_H__

#ifdef __cplusplus
extern "C" {
#endif

// 主初始化函数，在 main() 中调用
void RP_Timers_Init(void);
void Delay_MS(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif // __TIMERS_H__