#ifndef CAM_NAVIGATION_H
#define CAM_NAVIGATION_H

#include "frame.h"
#include "ui_state.h"

void Navigation_InitById(UIFrameIdentifier frame_initial);

void Navigation_NavigateToById(UIFrameIdentifier frame);

void Navigation_NavigateBack(void);

UIFrame *Navigation_CurrentFrame(void);

void Navigation_StickLeftMoved(void);

void Navigation_StickRightMoved(void);

void Navigation_StickUpMoved(void);

void Navigation_StickDownMoved(void);

void Navigation_StickPressed(void);

void Navigation_BatteryStateChanged(BatteryState new_state);

void Navigation_ConfigChanged(Config config);

void Navigation_AnimationCallback(void);

void Navigation_FPSChanged(uint32_t fps);


#endif //CAM_NAVIGATION_H
