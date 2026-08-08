#ifndef CAM_CONFIG_H
#define CAM_CONFIG_H

#include <stdint.h>

typedef struct Config_ {
    uint32_t image_counter;
    uint32_t heatmap_index;
    float tMin;
    float tMax;
    uint32_t autoscale;
} Config;

typedef void (*Config_Observer)(Config new_config);

void Config_AddObserver(Config_Observer observer);

void Config_InformObservers(Config new_config);

#endif //CAM_CONFIG_H