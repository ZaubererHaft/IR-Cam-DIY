#include "usb.h"

#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

bool USB_IsConnected(void) {
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
}
