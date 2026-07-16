#ifndef CAM_USER_INTERFACE_H
#define CAM_USER_INTERFACE_H

#include <stdint.h>

void UserInterface_Init(void);

int32_t UserInterface_ShowingMenu(void);

void UserInterface_Draw(void);

void UserInterface_RedrawIRImageIfNecessary(float *image);

#endif //CAM_USER_INTERFACE_H