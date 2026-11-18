#include "ILI9341_DMA_driver.h"
#include "MLX90640_I2C_Driver.h"
#include "application.h"
#include "file_system.h"
#include "flash.h"
#include "MLX90640.h"
#include "usb_device.h"
#include "user_interface.h"
#include "analog.h"
#include "dsp.h"
#include "dwt.h"
#include "heatmap.h"
#include "interface_callback.h"
#include "usb.h"
#include "ui/dialog.h"

/**
 * Prototypes
 */
static void Task_Init(void);

static void Task_ReadIRData(void);

static void Task_ReadAnalogData(void);

static void Task_WriteImage(void);

static void Task_SynchConfig(void);

static void Task_BlinkyDog(void);

static void Task_InformObservers(void);

static void Task_ProcessAndDisplayIRData(void);

/**
 * Local variables
 */
static uint8_t sector_backup[4096] = {0};
static Config config;
static uint32_t time = 0;
static uint32_t time_analog = 0;
static uint16_t adc_data[3] = {0};

static bool save_image = false;
static bool inform_observers = true;
static bool synch_config = false;
static bool menu_fresh_opened = false;
static bool usb_dialog = false;

void application_main(void) {
    Task_Init();

    while (1) {
        Task_ReadIRData();
        Task_ProcessAndDisplayIRData();
        Task_ReadAnalogData();
        Task_UserInterface();

        if (USB_IsConnected()) {
            if (!usb_dialog) {
                UserInterface_ShowDialog(" MSC USB-Mode - Please disconnect");
                usb_dialog = true;
            }
        } else {
            if (usb_dialog) {
                Dialog_Close();
                usb_dialog = false;
            }
            Task_WriteImage();
            Task_SynchConfig();
        }

        Task_InformObservers();
        Task_BlinkyDog();
    }
}

void application_error_handler(void) {
    UserInterface_ShowDialog("Error");
}

void Task_Init(void) {
    HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_SET);

    DWT_Init();

    Config_AddObserver(UserInterface_ConfigObserver);
    Config_AddObserver(Heatmap_ConfigObserver);
    Config_AddObserver(DSP_ConfigObserver);

    if (!Flash_Init()) {
        Error_Handler();
    }

    if (!FileSystem_Init(sector_backup, &config)) {
        Error_Handler();
    }

    Config_InformObservers(config);

    if (!Analog_PollADCData(adc_data)) {
        Error_Handler();
    }

    if (!UserInterface_Init()) {
        Error_Handler();
    }

    if (MLX90640_Init() != MLX90640_NO_ERROR) {
        Error_Handler();
    }

    MX_USB_DEVICE_Init();

    DSP_Init();

    if (MLX90640_Restart() != MLX90640_NO_ERROR) {
        Error_Handler();
    }

    time = HAL_GetTick();
}

void Task_ReadIRData(void) {
    if (UserInterface_ShowingAnyMenuOrDialog()) {
        menu_fresh_opened = true;
        MLX90640_ReadTemperature(false);
    } else {
        if (menu_fresh_opened) {
            // menu closed -> restart
            MLX90640_Restart();
        } else {
            MLX90640_ReadTemperature(true);
        }
        menu_fresh_opened = false;
    }
}

void Task_ReadAnalogData(void) {
    if (UserInterface_ShowingAnyMenuOrDialog() || HAL_GetTick() - time_analog > 5000) {
        if (!Analog_PollADCData(adc_data)) {
            Error_Handler();
        }
        time_analog = HAL_GetTick();
    }

    UserInterface_PutAnalogData(adc_data);
}

void Task_WriteImage(void) {
    if (save_image) {
        bool status = false;
        char buff[32] = {0};

        int32_t written = snprintf(buff, sizeof(buff), "i%u.bmp", config.image_counter);
        if (written > 0) {
            status = FileSystem_WriteImageAsBitmap(buff);
        }

        written = snprintf(buff, sizeof(buff), "i%u.bin", config.image_counter);
        if (written > 0) {
            status = FileSystem_WriteRaw(MLX90640_GetIRImageScaled(), 32 * 24, buff);
        }

        if (status) {
            config.image_counter++;
            synch_config = 1;
        } else {
            Error_Handler();
        }

        save_image = 0;
    }
}

void Task_SynchConfig(void) {
    if (synch_config) {
        if (!FileSystem_WriteConfig(&config)) {
            Error_Handler();
        }

        synch_config = false;
    }
}

void Task_BlinkyDog(void) {
    if (HAL_GetTick() - time > 1000) {
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        time = HAL_GetTick();
    }
}

void InterfaceCallback_RequestSaveImage(void) {
    save_image = true;
}

void InterfaceCallback_RequestNewSettings(Config settings) {
    uint32_t old_img_counter = config.image_counter;
    config = settings;
    config.image_counter = old_img_counter;

    inform_observers = true;
    synch_config = true;
}

void Task_InformObservers(void) {
    if (inform_observers) {
        Config_InformObservers(config);
        inform_observers = false;
    }
}

void Task_ProcessAndDisplayIRData(void) {
    if (!UserInterface_ShowingAnyMenuOrDialog()) {
        DSP_ProcessAndDisplay();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (!USB_IsConnected()) {
        if (GPIO_Pin == USRBUTTON_Pin) {
            InterfaceCallback_RequestSaveImage();
        }
        if (GPIO_Pin == STICK_Z_Pin) {
            UserInterface_StickZPressed();
        }
    }
}
