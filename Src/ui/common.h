#ifndef CAM_COMMON_H
#define CAM_COMMON_H

#include "battery_state.h"
#include "config.h"

void Common_DrawInitial(void);

void Common_StickPressed(void);

void Common_BatteryStateChanged(BatteryState state);

void Common_AnimationHandler(void);

void Common_ConfigChanged(Config config);

void Common_FPS_Changed(uint32_t curr_fps);

#endif //CAM_COMMON_H
