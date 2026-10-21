#ifndef CAM_NAVIGATION_H
#define CAM_NAVIGATION_H

#include "frame.h"

void Navigation_InitById(UIFrameIdentifier frame_initial);

void Navigation_NavigateToById(UIFrameIdentifier frame);

void Navigation_NavigateBack(void);

UIFrame *Navigation_CurrentFrame(void);

void Navigation_StickLeftMoved(void);

void Navigation_StickRightMoved(void);

void Navigation_StickUpMoved(void);

void Navigation_StickDownMoved(void);

void Navigation_StickPressed(void);

#endif //CAM_NAVIGATION_H
