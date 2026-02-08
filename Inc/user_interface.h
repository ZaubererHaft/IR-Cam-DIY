#ifndef CAM_USER_INTERFACE_H
#define CAM_USER_INTERFACE_H

#include <stdint.h>

void UserInterface_Init(void);

int32_t UserInterface_ShowMenu(void);

int32_t UserInterface_NeedsIRImageRedraw(void);

void UserInterface_Draw(void);

#endif //CAM_USER_INTERFACE_H