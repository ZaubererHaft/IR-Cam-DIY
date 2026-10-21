#include "frame.h"

#include "ILI9341_DMA_driver.h"

void Frame_CleanUp(void) {
    ILI9341_Draw_Rectangle(00, 30, 240, 210, RED);

}

