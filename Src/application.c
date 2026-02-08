#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ui_constants.h"
#include "heatmap.h"
#include "application.h"

#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air

typedef enum MENU_ {
  MAIN,
  SELECT_HEATMAP
} MENU;

MENU cur_entry = MAIN;

static paramsMLX90640 mlxParams;

static uint16_t frame_1[834];
static uint16_t frame_2[834];

static uint16_t *data_frame = frame_2;
static uint16_t *display_frame = frame_1;
volatile int new_data_available = 0;
static uint16_t eeMLX90640[832];

static float image[32 * 24];
static float emissivity = 0.95f;
static int mlx_status;

volatile uint8_t SPI2_TX_completed_flag = 1; //flag indicating finish of SPI transmission
volatile uint8_t show_menu = 0;
volatile uint8_t do_redraw = 0;

uint16_t (*TempConverter)(float) = &TempToMagma565_Fast;

float tMin = 15.0f;
float tMax = 37.0f;
float tMinOld;
float tMaxOld;

int redraw_menu = 1;
int redraw_record = 1;
uint32_t last_stick_pressed = 0;
uint16_t frames = 0;
uint16_t menu_records = 6;
uint16_t cursor_Y = 1;
int stick_recently_updated = 0;
int redraw_heatmap = 0;

extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c3;

void LCD_Init(void) {
  ILI9341_Init();
  ILI9341_Fill_Screen(BLACK);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  ILI9341_Draw_Text(" D-CAM ", 0, 0, WHITE, 2, BLACK);
  ILI9341_Draw_Rectangle(offset_x, offset_y, ir_width * pixel_size, ir_height * pixel_size,DARKGREY);
}

void MLX90640_Init(void) {
  mlx_status |= MLX90640_SetRefreshRate(MLX90640_ADDR, RefreshRate);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_SetChessMode(MLX90640_ADDR);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  if (mlx_status != 0) {
    Error_Handler();
  }
  mlx_status |= MLX90640_ExtractParameters(eeMLX90640, &mlxParams);
  if (mlx_status != 0) {
    Error_Handler();
  }
}

void redraw() {
    for(int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
      int row = pixelNumber >> 5;       // pixelNumber / 32
      int col = pixelNumber & 31;       // pixelNumber % 32

      ILI9341_Draw_Rectangle(col * pixel_size + offset_x, row * pixel_size + offset_y, pixel_size, pixel_size, TempConverter(image[pixelNumber]));
    }
}

void ChangeAndDisplayRecordState(void) {
  ILI9341_Draw_Rectangle(lcd_width - pixel_size * 4, 0, pixel_size * 4, pixel_size * 2 + 4, BLACK);

  if (show_menu) {
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2, 0, 4, pixel_size * 2, LIGHTGREY);
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2 + 6, 0, 4, pixel_size * 2, LIGHTGREY);
  } else {
    ILI9341_Draw_Filled_Circle(lcd_width - pixel_size * 2 + 4, pixel_size, pixel_size, RED);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == STICK_Z_Pin) {

    uint32_t time = HAL_GetTick();

    if (time - last_stick_pressed > 100) {
      if (show_menu) {
        if (cur_entry == MAIN) {
          if (cursor_Y == 1) {
            cur_entry = SELECT_HEATMAP;
          }
          else if (cursor_Y == 5) {
            show_menu = 0;
            do_redraw = 1;
          }
        }
        else if (cur_entry == SELECT_HEATMAP) {
          if (cursor_Y == 1) {
            TempConverter = &TempToMagma565_Fast;
          }
          else if (cursor_Y == 2) {
            TempConverter = &TempToGray565;
          }
          else if (cursor_Y == 3) {
            TempConverter = &TempToGray565_InvertedFast;
          }
          else if (cursor_Y == 4) {
            TempConverter = &TempToRainbow565_Fast;
          }

          cursor_Y = 1;
          redraw_heatmap = 1;
          do_redraw = 1;
          cur_entry = MAIN;
        }
      }
      else {
        cursor_Y = 1;
        show_menu = 1;
      }

      redraw_menu = 1;
      last_stick_pressed = time;
      redraw_record = 1;
    }
  }
}

void DrawHeatmp(void) {
  tMinOld = tMin;
  tMaxOld = tMax;

  int x = (ir_width + 1) * pixel_size + offset_x;
  int y = offset_y;

  y += pixel_size * 2 - 8;

  char buff[10] = {};
  itoa(tMin, buff, 10);
  ILI9341_Draw_Text(buff, x + 20, y, WHITE, 2, BLACK);

  int chunks = 30;
  float step = (tMax - tMin) / chunks;
  for (int i = 0; i < chunks; ++i) {
    float val = tMin + i * step;
    ILI9341_Draw_Rectangle(x, y, 15, 4, TempConverter(val));

    y += 4;
  }

  memset(buff, 0, sizeof(buff));
  float mid = (tMax + fabs(tMin)) / 2;
  itoa(mid, buff, 10);
  ILI9341_Draw_Text(buff, x + 20, y - chunks * 4 / 2 - 8, WHITE, 2, BLACK);

  memset(buff, 0, sizeof(buff));
  itoa(tMax, buff, 10);
  ILI9341_Draw_Text(buff, x + 20, y - 16, WHITE, 2, BLACK);
}


void DrawFPS(void) {
  int x = (ir_width + 1) * pixel_size + offset_x;
  int y = lcd_height - 15;

  char buff[15] = "FPS:    ";
  ILI9341_Draw_Text(buff, x, y, WHITE, 1, BLACK);

  itoa((int) frames, &buff[5], 10);
  ILI9341_Draw_Text(buff, x, y, WHITE, 1, BLACK);
}

void DrawMenuLine(const char *text, uint16_t line) {
  uint16_t color_foreground = WHITE;
  uint16_t color_background = BLACK;

  if (cursor_Y == line) {
    color_foreground = BLACK;
    color_background = WHITE;
  }
  else {
    color_foreground = WHITE;
    color_background = BLACK;
  }

  uint16_t x = lcd_width / 2 - menu_width / 2 - 30;
  uint16_t y = lcd_height / 2 - menu_height / 2 + menu_font_size * line;

  ILI9341_Draw_Rectangle(x, y, menu_width, menu_font_size, color_background);
  ILI9341_Draw_Text(text, x, y, color_foreground, 2, color_background);

}

void DrawMenu(void) {
  if (cur_entry == MAIN) {
    DrawMenuLine(" Menu", 0);
    DrawMenuLine(" Heatmap", 1);
    DrawMenuLine(" Scaling", 2);
    DrawMenuLine(" Save Image", 3);
    DrawMenuLine(" Settings", 4);
    DrawMenuLine(" Exit", 5);
  }
  else if (cur_entry == SELECT_HEATMAP) {
    DrawMenuLine(" Select Heatmap", 0);
    DrawMenuLine(" Magma", 1);
    DrawMenuLine(" Gray", 2);
    DrawMenuLine(" Gray (inv)", 3);
    DrawMenuLine(" Rainbow", 4);
    DrawMenuLine(" Exit", 5);
  }
}

int MLX90640_ReadAndDisplay(void) {
  if (new_data_available) {
    frames++;

    new_data_available = 0;

    MLX90640_CompleteFrameDataAsync(MLX90640_ADDR, data_frame);
    uint16_t *tmp = display_frame;
    display_frame = data_frame;
    data_frame = tmp;

    MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
    float Ta = MLX90640_GetTa(display_frame, &mlxParams);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    MLX90640_CalculateToAndDisplay(display_frame, &mlxParams, emissivity, tr, image, 0);

    return 1;
  }

  return 0;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c3) {
    new_data_available = 1;
  }
}

void application_main(void) {
    LCD_Init();
    MLX90640_Init();
    DrawHeatmp();
    DrawFPS();

    tMinOld = tMin;
    tMaxOld = tMax;
    int did_nothing;
    uint32_t frame_counter = HAL_GetTick();
    int status = MLX90640_GetFrameDataAsync(MLX90640_ADDR, data_frame);
    if (status != 0) {
        Error_Handler();
    }

    while (1) {
        did_nothing = 1;

        if (show_menu && !stick_recently_updated) {
            HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
            status = HAL_ADC_PollForConversion(&hadc1, 10);
            uint16_t new_stick_y = HAL_ADC_GetValue(&hadc1);

            status = HAL_ADC_Start(&hadc1);
            status = HAL_ADC_PollForConversion(&hadc1, 10);
            uint16_t new_stick_x = HAL_ADC_GetValue(&hadc1);

            if (new_stick_y - 2048 > 1500) {
                cursor_Y = (cursor_Y + 1) % menu_records;
                stick_recently_updated = 1;
                if (cursor_Y == 0) {
                    cursor_Y++;
                }
                redraw_menu = 1;
            }
            else if (new_stick_y - 2048 < -1500) {
                cursor_Y = (cursor_Y - 1) % menu_records;
                stick_recently_updated = 1;
                if (cursor_Y == 0) {
                    cursor_Y = menu_records - 1;
                }
                redraw_menu = 1;
            }
        }

        uint32_t count = HAL_GetTick();

        if (count - frame_counter >= 1000) {
            frame_counter = count;
            if (!show_menu) {
                DrawFPS();
            }
            did_nothing = 0;
            frames = 0;
            stick_recently_updated = 0;
        }

        if (do_redraw) {
            redraw();
            do_redraw = 0;
        }

        if (redraw_record) {
            ChangeAndDisplayRecordState();
            redraw_record = 0;
            did_nothing = 0;
        }

        if (show_menu) {
            if (redraw_menu) {
                DrawMenu();
                redraw_menu = 0;
                did_nothing = 0;
            }
        }
        else {
            if (MLX90640_ReadAndDisplay()) {
                did_nothing = 0;
            }
        }


        if (redraw_heatmap || fabs(tMinOld - tMin) > 0.0f || fabs(tMaxOld - tMax) > 0.0f) {
            DrawHeatmp();
            did_nothing = 0;
            redraw_heatmap = 0;
        }

        if (did_nothing) {
            __WFI();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  SPI2_TX_completed_flag = 1;
}