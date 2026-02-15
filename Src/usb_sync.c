#include "usb_sync.h"

#include <string.h>

void USB_SYNC_QueueInit(USB_SYNC_Queue *queue) {
    for (int i = 0; i < USB_SYNC_QUEUE_CAPACITY; ++i) {
        memset(&queue->data[i].USB_BlockBuffer, 0, sizeof(queue->data[i].USB_BlockBuffer));
        queue->data[i].address = 0;
    }

    queue->start = 0;
    queue->end = 0;
}

USB_SYNC *USB_SYNC_AllocateNext(USB_SYNC_Queue *queue) {
    if (queue->size == USB_SYNC_QUEUE_CAPACITY) {
        return NULL;
    }

    USB_SYNC *data = &queue->data[queue->end];
    queue->end = (queue->end + 1) % USB_SYNC_QUEUE_CAPACITY;
    queue->size++;
    return data;
}

USB_SYNC *USB_SYNC_Head(USB_SYNC_Queue *queue) {
    if (queue->size == 0) {
        return NULL;
    }
    return &queue->data[queue->start];
}

void USB_SYNC_Head_DeallocateHead(USB_SYNC_Queue *queue) {
    queue->start = (queue->start + 1) % USB_SYNC_QUEUE_CAPACITY;
    queue->size--;
}

int32_t USB_SYNC_QueueSize(USB_SYNC_Queue *queue) {
    return queue->size;
}
