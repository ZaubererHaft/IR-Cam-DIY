#ifndef CAM_FRAME_H
#define CAM_FRAME_H

#include "ui_state.h"

typedef enum UIFrameIdentifier_ {
    FRAME_DEFAULT,
    FRAME_MENU_OVERVIEW,
    FRAME_GALLERY
} UIFrameIdentifier;

typedef struct UIFrame_ {
    UIFrameIdentifier id;
    void (*init_function)(void);
} UIFrame;

void Frame_CleanUp(void);

#endif //CAM_FRAME_H
