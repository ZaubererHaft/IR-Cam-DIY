#ifndef CAM_COMMON_H
#define CAM_COMMON_H

#include "ui_state.h"

void Common_DrawInitial(void);

void Common_StickPressed(void);

void Common_BatteryStateChanged(BatteryState state);

void Common_AnimationHandler(void);

#endif //CAM_COMMON_H
