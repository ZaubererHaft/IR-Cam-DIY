#include "navigation.h"
#include "common.h"
#include "gallery.h"

static UIFrame default_frame;
static UIFrame gallery_frame;

static UIFrame *current_frame;

static UIFrame *Navigaion_FrameById(UIFrameIdentifier frame) {
    if (frame == FRAME_GALLERY) {
        return &gallery_frame;
    }
    return &default_frame;
}

void Navigation_InitById(UIFrameIdentifier frame_initial) {
    default_frame.id = FRAME_DEFAULT;
    default_frame.init_function = Common_DrawInitial;

    gallery_frame.id = FRAME_GALLERY;
    gallery_frame.init_function = Gallery_DrawInitial;

    Navigation_NavigateToById(frame_initial);
}

void Navigation_NavigateToById(UIFrameIdentifier frame) {
    Frame_CleanUp();

    current_frame = Navigaion_FrameById(frame);
    current_frame->init_function();
}

void Navigation_NavigateBack() {
}

UIFrame *Navigation_CurrentFrame(void) {
    return current_frame;
}
