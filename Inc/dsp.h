#ifndef CAM_DSP_H
#define CAM_DSP_H

#include "config.h"

typedef void (*ProcessedLineFunc)(uint16_t *, uint32_t);

void DSP_Init(void);

void DSP_ProcessAndDisplay(void);

void DSP_UpdateMinMaxIfAutoscale(void);

void DSP_ProcessAndDo(ProcessedLineFunc func_after_process);

void DSP_ConfigObserver(Config config);

#endif //CAM_DSP_H
