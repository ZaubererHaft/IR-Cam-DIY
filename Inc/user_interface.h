#ifndef CAM_USER_INTERFACE_H
#define CAM_USER_INTERFACE_H

#include <stdint.h>
#include "stdbool.h"

#include "config.h"

bool UserInterface_Init(void);

bool UserInterface_ShowingAnyMenuOrDialog(void);

void UserInterface_Draw(void);

void UserInterface_RedrawIRImageIfNecessary(float *image);

void UserInterface_PutAnalogData(const uint16_t data[3]);

void UserInterface_ConfigObserver(Config config);

void UserInterface_ShowError(const char *text);

#endif //CAM_USER_INTERFACE_H
