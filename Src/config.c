#include "config.h"

static uint32_t index = 0;
static Config_Observer observers[5];

void Config_AddObserver(Config_Observer observer) {
    observers[index] = observer;
    index++;
}

void Config_InformObservers(Config new_config) {
    for (int i = 0; i < index; i++) {
        observers[i](new_config);
    }
}
