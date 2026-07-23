#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include <stdint.h>

typedef struct Config_ {
    uint32_t image_counter;
    uint32_t heatmap_index;
} Config;


#endif //CAM_CONFIG_H