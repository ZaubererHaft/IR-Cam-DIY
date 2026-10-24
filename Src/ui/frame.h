#ifndef CAM_FRAME_H
#define CAM_FRAME_H

#include "battery_state.h"

typedef enum UIFrameIdentifier_ {
    FRAME_DEFAULT,
    FRAME_MENU_OVERVIEW,
    FRAME_GALLERY,
    FRAME_SETTINGS,
} UIFrameIdentifier;

typedef struct UIFrame_ {
    UIFrameIdentifier id;
    void (*init_function)(void);
    void (*stick_left_callback)(void);
    void (*stick_right_callback)(void);
    void (*stick_up_callback)(void);
    void (*stick_down_callback)(void);
    void (*stick_pressed_callback)(void);
} UIFrame;

void Frame_CleanUp(void);

#endif //CAM_FRAME_H
