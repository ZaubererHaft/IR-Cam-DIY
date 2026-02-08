#include "user_interface.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "ui_constants.h"
#include "heatmap.h"

typedef enum MENU_ {
  MAIN,
  SELECT_HEATMAP
} MENU;

static MENU menu_cur_entry = MAIN;
static uint8_t menu_show = 0;
static uint16_t menu_cursor_Y = 1;
static uint16_t menu_records = 6;

static uint32_t last_stick_pressed = 0;

static uint16_t frames = 0;
static uint8_t do_redraw_ir_image = 0;

static float tMinOld;
static float tMaxOld;

float tMin = 15.0f;
float tMax = 37.0f;

uint16_t (*TempConverter)(float) = &TempToMagma565_Fast;

extern ADC_HandleTypeDef hadc1;

void UserInterface_Init(void) {
  ILI9341_Init();
  ILI9341_Fill_Screen(BLACK);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_2);
  ILI9341_Draw_Text(" D-CAM ", 0, 0, WHITE, 2, BLACK);
  ILI9341_Draw_Rectangle(offset_x, offset_y, ir_width * pixel_size, ir_height * pixel_size,DARKGREY);
}

void DrawMenuLine(const char *text, uint16_t line) {
  uint16_t color_foreground = WHITE;
  uint16_t color_background = BLACK;

  if (menu_cursor_Y == line) {
    color_foreground = BLACK;
    color_background = WHITE;
  } else {
    color_foreground = WHITE;
    color_background = BLACK;
  }

  uint16_t x = lcd_width / 2 - menu_width / 2 - 30;
  uint16_t y = lcd_height / 2 - menu_height / 2 + menu_font_size * line;

  ILI9341_Draw_Rectangle(x, y, menu_width, menu_font_size, color_background);
  ILI9341_Draw_Text(text, x, y, color_foreground, 2, color_background);
}


void DrawRecordState(void) {
  ILI9341_Draw_Rectangle(lcd_width - pixel_size * 4, 0, pixel_size * 4, pixel_size * 2 + 4, BLACK);

  if (menu_show) {
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2, 0, 4, pixel_size * 2, LIGHTGREY);
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2 + 6, 0, 4, pixel_size * 2, LIGHTGREY);
  } else {
    ILI9341_Draw_Filled_Circle(lcd_width - pixel_size * 2 + 4, pixel_size, pixel_size, RED);
  }
}


void DrawMenu(void) {
  if (menu_cur_entry == MAIN) {
    DrawMenuLine(" Menu", 0);
    DrawMenuLine(" Heatmap", 1);
    DrawMenuLine(" Scaling", 2);
    DrawMenuLine(" Save Image", 3);
    DrawMenuLine(" Settings", 4);
    DrawMenuLine(" Exit", 5);
  } else if (menu_cur_entry == SELECT_HEATMAP) {
    DrawMenuLine(" Select Heatmap", 0);
    DrawMenuLine(" Magma", 1);
    DrawMenuLine(" Gray", 2);
    DrawMenuLine(" Gray (inv)", 3);
    DrawMenuLine(" Rainbow", 4);
    DrawMenuLine(" Exit", 5);
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


int32_t UserInterface_ShowMenu(void) {
  return menu_show;
}

int32_t UserInterface_NeedsIRImageRedraw() {
  return do_redraw_ir_image;
}

void UserInterface_ReadStick() {
  HAL_StatusTypeDef status = HAL_ADC_Start(&hadc1);
  status = HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t new_stick_y = HAL_ADC_GetValue(&hadc1);

  status = HAL_ADC_Start(&hadc1);
  status = HAL_ADC_PollForConversion(&hadc1, 10);
  uint16_t new_stick_x = HAL_ADC_GetValue(&hadc1);

  if (new_stick_y - 2048 > 1500) {
    menu_cursor_Y = (menu_cursor_Y + 1) % menu_records;
    if (menu_cursor_Y == 0) {
      menu_cursor_Y++;
    }
  } else if (new_stick_x - 2048 < -1500) {
    menu_cursor_Y = (menu_cursor_Y - 1) % menu_records;
    if (menu_cursor_Y == 0) {
      menu_cursor_Y = menu_records - 1;
    }
  }
}

void UserInterface_Draw(void) {
  DrawFPS();
  DrawRecordState();
  DrawHeatmp();

  if (menu_show) {
    UserInterface_ReadStick();
    DrawMenu();
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == STICK_Z_Pin) {
    uint32_t time = HAL_GetTick();

    if (time - last_stick_pressed > 100) {
      if (menu_show) {
        if (menu_cur_entry == MAIN) {
          if (menu_cursor_Y == 1) {
            menu_cur_entry = SELECT_HEATMAP;
          } else if (menu_cursor_Y == 5) {
            menu_show = 0;
            do_redraw_ir_image = 1;
          }
        } else if (menu_cur_entry == SELECT_HEATMAP) {
          if (menu_cursor_Y == 1) {
            TempConverter = &TempToMagma565_Fast;
          } else if (menu_cursor_Y == 2) {
            TempConverter = &TempToGray565;
          } else if (menu_cursor_Y == 3) {
            TempConverter = &TempToGray565_InvertedFast;
          } else if (menu_cursor_Y == 4) {
            TempConverter = &TempToRainbow565_Fast;
          }

          menu_cursor_Y = 1;
          do_redraw_ir_image = 1;
          menu_cur_entry = MAIN;
        }
      } else {
        menu_cursor_Y = 1;
        menu_show = 1;
      }

      last_stick_pressed = time;
    }
  }
}
