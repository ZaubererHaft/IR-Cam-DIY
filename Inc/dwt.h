#ifndef DWT_H
#define DWT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void DWT_Init(void);

uint32_t DWT_GetTick(void);

void DWT_DelayUs(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif //DWT_H
