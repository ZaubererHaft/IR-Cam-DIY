#ifndef CAM_INTERFACE_CALLBACK_H
#define CAM_INTERFACE_CALLBACK_H

#include <stdint.h>

#include "config.h"

void InterfaceCallback_RequestSaveImage(void);

void InterfaceCallback_RequestNewSettings(Config index);

#endif //CAM_INTERFACE_CALLBACK_H