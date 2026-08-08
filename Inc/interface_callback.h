#ifndef CAM_INTERFACE_CALLBACK_H
#define CAM_INTERFACE_CALLBACK_H

#include <stdint.h>

void InterfaceCallback_RequestSaveImage(void);

void InterfaceCallback_RequestNewHeatmap(uint32_t index);

#endif //CAM_INTERFACE_CALLBACK_H