#include "navigation.h"
#include "common.h"
#include "gallery.h"
#include "menu.h"

#include <stddef.h>
#include <stdint.h>

#include "settings.h"

static UIFrame default_frame;
static UIFrame gallery_frame;
static UIFrame menu_frame;
static UIFrame settings_frame;
static UIFrame settings_frame;

static int32_t stack_ptr = -1;
static UIFrame *nav_stack[5];

static UIFrame *Navigation_FrameById(UIFrameIdentifier frame) {
    if (frame == FRAME_GALLERY) {
        return &gallery_frame;
    }
    if (frame == FRAME_MENU_OVERVIEW) {
        return &menu_frame;
    }
    if (frame == FRAME_SETTINGS) {
        return &settings_frame;
    }
    return &default_frame;
}

void Navigation_InitById(UIFrameIdentifier frame_initial) {
    default_frame.id = FRAME_DEFAULT;
    default_frame.init_function = Common_DrawInitial;
    default_frame.stick_pressed_callback = Common_StickPressed;

    gallery_frame.id = FRAME_GALLERY;
    gallery_frame.init_function = Gallery_DrawInitial;
    gallery_frame.stick_down_callback = Gallery_StickDownMoved;
    gallery_frame.stick_up_callback = Gallery_StickUpMoved;
    gallery_frame.stick_left_callback = Gallery_StickLeftMoved;
    gallery_frame.stick_right_callback = Gallery_StickRightMoved;
    gallery_frame.stick_pressed_callback = Gallery_StickPressed;

    menu_frame.id = FRAME_MENU_OVERVIEW;
    menu_frame.init_function = Menu_DrawInitial;
    menu_frame.stick_down_callback = Menu_StickMovedDown;
    menu_frame.stick_up_callback = Menu_StickMovedUp;
    menu_frame.stick_pressed_callback = Menu_StickPressed;

    settings_frame.id = FRAME_SETTINGS;
    settings_frame.init_function = Settings_DrawInitial;
    settings_frame.stick_down_callback = Settings_StickMovedDown;
    settings_frame.stick_up_callback = Settings_StickMovedUp;
    settings_frame.stick_pressed_callback = Settings_StickPressed;

    Navigation_NavigateToById(frame_initial);
}

void Navigation_NavigateToById(UIFrameIdentifier frame) {
    Frame_CleanUp();

    stack_ptr++;
    nav_stack[stack_ptr] = Navigation_FrameById(frame);
    nav_stack[stack_ptr]->init_function();

    if (frame != FRAME_DEFAULT) {
        default_frame.init_function();
    }
}

void Navigation_NavigateBack() {
    Frame_CleanUp();

    stack_ptr--;
    nav_stack[stack_ptr]->init_function();

    if (Navigation_CurrentFrame()->id != FRAME_DEFAULT) {
        default_frame.init_function();
    }
}

UIFrame *Navigation_CurrentFrame(void) {
    if (stack_ptr < 0) {
        return NULL;
    }
    return nav_stack[stack_ptr];
}

void Navigation_StickLeftMoved(void) {
    if (Navigation_CurrentFrame()->stick_left_callback != NULL) {
        Navigation_CurrentFrame()->stick_left_callback();
    }
}

void Navigation_StickRightMoved(void) {
    if (Navigation_CurrentFrame()->stick_right_callback != NULL) {
        Navigation_CurrentFrame()->stick_right_callback();
    }
}

void Navigation_StickUpMoved(void) {
    if (Navigation_CurrentFrame()->stick_up_callback != NULL) {
        Navigation_CurrentFrame()->stick_up_callback();
    }
}

void Navigation_StickDownMoved(void) {
    if (Navigation_CurrentFrame()->stick_down_callback != NULL) {
        Navigation_CurrentFrame()->stick_down_callback();
    }
}

void Navigation_StickPressed(void) {
    if (Navigation_CurrentFrame()->stick_pressed_callback != NULL) {
        Navigation_CurrentFrame()->stick_pressed_callback();
    }
}

void Navigation_BatteryStateChanged(BatteryState new_state) {
    Common_BatteryStateChanged(new_state);
}

void Navigation_ConfigChanged(Config config) {
    Common_ConfigChanged(config);
    Settings_ConfigChanged(config);
}

void Navigation_AnimationCallback(void) {
    Common_AnimationHandler();
}

void Navigation_FPSChanged(uint32_t fps) {
    Common_FPS_Changed(fps);
}
