#include "frame.h"

#include "dsp.h"
#include "ILI9341_DMA_driver.h"

void Frame_CleanUp(void) {
    ILI9341_Draw_Rectangle(0, 220, 240, 16, BLACK);
    DSP_ProcessAndDisplay();
}

