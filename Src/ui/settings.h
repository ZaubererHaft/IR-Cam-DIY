#ifndef CAM_SETTINGS_H
#define CAM_SETTINGS_H

#include "config.h"

void Settings_DrawInitial(void);

void Settings_StickMovedUp(void);

void Settings_StickMovedDown(void);

void Settings_StickPressed(void);

void Settings_ConfigChanged(Config config);

#endif //CAM_SETTINGS_H
