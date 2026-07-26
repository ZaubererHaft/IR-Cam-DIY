#ifndef CAM_USER_INTERFACE_H
#define CAM_USER_INTERFACE_H

#include <stdint.h>

uint32_t UserInterface_Init(void);

int32_t UserInterface_ShowingMenu(void);

void UserInterface_Draw(void);

void UserInterface_RedrawIRImageIfNecessary(float *image);

void UserInterface_PutAnalogData(const uint16_t data[3]);

#endif //CAM_USER_INTERFACE_H