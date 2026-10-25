#include "frame.h"

#include "ILI9341_DMA_driver.h"
#include "MLX90640.h"

void Frame_CleanUp(void) {
    ILI9341_Draw_Rectangle(0, 220, 240, 16, BLACK);
    MLX90640_RedrawIRImage();
}

