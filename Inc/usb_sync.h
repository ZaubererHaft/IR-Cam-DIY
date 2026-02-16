#ifndef CAM_USB_SYNC_H
#define CAM_USB_SYNC_H

#include <stdint.h>

typedef struct USB_SYNC_ {
    uint8_t USB_BlockBuffer[4096];
    uint32_t address;
} USB_SYNC;

#define USB_SYNC_QUEUE_CAPACITY 8

typedef struct USB_SYNC_Queue_ {
    USB_SYNC data[USB_SYNC_QUEUE_CAPACITY];
    int32_t start;
    int32_t end;
    int32_t size;
}USB_SYNC_Queue;

void USB_SYNC_QueueInit(USB_SYNC_Queue *queue);

USB_SYNC *USB_SYNC_AllocateNext(USB_SYNC_Queue *queue);

USB_SYNC *USB_SYNC_Head(USB_SYNC_Queue *queue);

void USB_SYNC_Head_DeallocateHead(USB_SYNC_Queue *queue);

int32_t USB_SYNC_QueueSize(USB_SYNC_Queue *queue);

#endif //CAM_USB_SYNC_H