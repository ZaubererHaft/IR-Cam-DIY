#include "dwt.h"
#include "main.h"

static uint32_t cycles_per_us;

void DWT_Init(void)
{
    SystemCoreClockUpdate();
    cycles_per_us = SystemCoreClock / 1000000U;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetTick(void)
{
    return DWT->CYCCNT;
}

void DWT_DelayUs(uint32_t us) {
    uint32_t cycles = us * cycles_per_us;
    uint32_t start = DWT->CYCCNT;
    while (DWT->CYCCNT - start < cycles) {
        __NOP();
    }
}
