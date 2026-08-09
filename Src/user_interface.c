#include "user_interface.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "ui_constants.h"
#include "heatmap.h"
#include "interface_callback.h"

typedef enum MENU_ {
  MENU_MAIN,
  MENU_SELECT_HEATMAP,
  MENU_SCALING,
  MENU_SAVE_IMAGE,
  MENU_SETTINGS,
  MENU_EXIT
} MENU;

// variables for drawing the IR image
static uint8_t do_redraw_ir_image = 0;
static HeatmapFunction TemperatureConverter;

// External memories
float tMin = 15.0f;
float tMax = 40.0f;

// Variables for FPS
static uint16_t frames = 0;
static uint16_t last_frames = 0;
static uint32_t frame_counter = 0;

// Analog stick
static uint32_t last_stick_pressed = 0;
static uint16_t new_stick_y = 0;
static uint16_t new_stick_x = 0;

// Variables for menu
static const uint32_t menu_records = MENU_EXIT + 1;
static MENU menu_cur_entry = MENU_MAIN;
static uint32_t menu_show = 0;
static int32_t menu_redraw = 0;
static int32_t redraw_record_state = 1;
static uint16_t menu_cursor_Y = 1;

// Heatmap
static int32_t redraw_heatmap = 0;
static float tMinOld;
static float tMaxOld;

// Variables for battery
static uint32_t last_batt = 0;
static uint32_t last_anim_charge = 1;
static GPIO_PinState old_stat1;
static GPIO_PinState old_stat2;
static GPIO_PinState old_npg;
static uint32_t initial_draw_battery = 1;
static const uint16_t max_lvl = 3583;
static uint16_t battery_level = max_lvl;
static uint16_t old_battery_level = 0;


uint32_t UserInterface_Init(void) {
  ILI9341_Init();
  ILI9341_Fill_Screen(BLACK);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
  ILI9341_Draw_Text(" D-CAM ", 0, 0, WHITE, 2, BLACK);
  ILI9341_Draw_Rectangle(offset_x, offset_y, ir_width * pixel_size, ir_height * pixel_size,DARKGREY);

  return 1;
}


void UserInterface_ConfigObserver(Config config) {
  TemperatureConverter = Heatmap_GetByIndex(config.heatmap_index);
  redraw_heatmap = 1;
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
  if (redraw_record_state) {
    // clear old area
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2 - 4, 0, pixel_size * 4, pixel_size * 2 + 4, BLACK);

    if (menu_show) {
      // two gray bars -> pause sign
      ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2, 0, 4, pixel_size * 2, LIGHTGREY);
      ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2 + 6, 0, 4, pixel_size * 2, LIGHTGREY);
    } else {
      // else red dot -> record sign
      ILI9341_Draw_Filled_Circle(lcd_width - pixel_size * 2 + 4, pixel_size, pixel_size, RED);
    }

    redraw_record_state = 0;
  }
}

void DrawBatteryState(void) {
  if (HAL_GetTick() - last_batt > 1000) {
    // Every second: Translate electrical state to LED state s.t. we can use table 2-1 of the breakout board
    GPIO_PinState stat1 = !HAL_GPIO_ReadPin(BATTCHARGING_GPIO_Port, BATTCHARGING_Pin);
    GPIO_PinState stat2 = !HAL_GPIO_ReadPin(BATTFULL_GPIO_Port, BATTFULL_Pin);
    GPIO_PinState npg = !HAL_GPIO_ReadPin(EXT_POWER_GPIO_Port, EXT_POWER_Pin);

    if (initial_draw_battery || stat1 != old_stat1 || stat2 != old_stat2 || old_npg != npg || abs(
          battery_level - old_battery_level) >= 50 || (
          stat1 && !stat2 && npg)) {
      initial_draw_battery = 0;
      old_stat1 = stat1;
      old_stat2 = stat2;
      old_npg = npg;

      uint32_t x = lcd_width - pixel_size * 6;
      uint32_t y = 3;

      // clean up
      ILI9341_Draw_Rectangle(x - 55, 0, 77, pixel_size * 1.5 + 8, BLACK);

      // Battery frame
      uint16_t battery_bar_color = WHITE;
      if (!stat1 && stat2 && npg) {
        //charging complete -> green
        battery_bar_color = GREEN;
      }

      // ca. 4.2 V Ladeschlusspannung
      // spannungsteiler 100 - 220 -> max 2.88 V Spannung
      // -> 0.875 Maximal von 3.3 V erreichbar
      // -> 0.875 * 4096 = 3583 sind 100 % batteriespannung

      char buff[10] = {};

      // Battery level in numbers
      uint32_t lvl_perc = (uint32_t) ((float) battery_level / (float) max_lvl * 100.0f);
      snprintf(buff, 10, "%lu%%", lvl_perc);
      ILI9341_Draw_Text(buff, x - 55, 0, WHITE, 2, BLACK);
      old_battery_level = battery_level;

      // Battery frame
      ILI9341_Draw_Hollow_Rectangle_Coord(x - 3, 0, x + 18, y + pixel_size * 1.5 + 2, WHITE);
      ILI9341_Draw_Rectangle(x + 18, y + 1, 3, pixel_size * 1.25, WHITE);


      if (stat1 && !stat2 && npg) {
        // Two bars
        ILI9341_Draw_Rectangle(x, y, 4, pixel_size * 1.5, battery_bar_color);
        ILI9341_Draw_Rectangle(x + 6, y, 4, pixel_size * 1.5, battery_bar_color);

        // charge in progress -> blink last bar
        if (last_anim_charge) {
          ILI9341_Draw_Rectangle(x + 12, y, 4, pixel_size * 1.5, battery_bar_color);
          last_anim_charge = 0;
        } else {
          ILI9341_Draw_Rectangle(x + 12, y, 4, pixel_size * 1.5, BLACK);
          last_anim_charge = 1;
        }
      } else if (!(stat1 && !stat2 && !npg)) {
        // all other cases than LBO (low battery) -> add bars

        if (lvl_perc < 25) {
          battery_bar_color = RED;
        }

        ILI9341_Draw_Rectangle(x, y, 4, pixel_size * 1.5, battery_bar_color);

        if (lvl_perc > 50) {
          ILI9341_Draw_Rectangle(x + 6, y, 4, pixel_size * 1.5, battery_bar_color);
        }

        if (lvl_perc > 75) {
          ILI9341_Draw_Rectangle(x + 12, y, 4, pixel_size * 1.5, battery_bar_color);
        }

        if (!stat1 && !stat2 && npg) {
          // No battery or standby -> battery crossed out
          ILI9341_Draw_Rectangle(x - 4, y + 5, 26, 2, RED);
        }
      }

      last_batt = HAL_GetTick();
    }
  }
}


void DrawMenu(void) {
  if (menu_redraw) {
    if (menu_cur_entry == MENU_MAIN) {
      DrawMenuLine(" Menu", 0);
      DrawMenuLine(" Heatmap", MENU_SELECT_HEATMAP);
      DrawMenuLine(" Scaling", MENU_SCALING);
      DrawMenuLine(" Save Image", MENU_SAVE_IMAGE);
      DrawMenuLine(" Settings", MENU_SETTINGS);
      DrawMenuLine(" Exit", MENU_EXIT);
    } else if (menu_cur_entry == MENU_SELECT_HEATMAP) {
      DrawMenuLine(" Select Heatmap", 0);

      uint32_t index = 1;
      for (; index < Heatmap_GetSize(); index++) {
        DrawMenuLine(Heatmap_GetNameByIndex(index - 1), index);
      }
      index++;
      DrawMenuLine(" Exit", index);
    }
    menu_redraw = 0;
  }
}

void DrawHeatmap(void) {
  if (redraw_heatmap || fabs(tMinOld - tMin) > 0.0f || fabs(tMaxOld - tMax) > 0.0f) {
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
      ILI9341_Draw_Rectangle(x, y, 15, 4, TemperatureConverter(val));

      y += 4;
    }

    memset(buff, 0, sizeof(buff));
    float mid = (tMax + fabs(tMin)) / 2;
    itoa(mid, buff, 10);
    ILI9341_Draw_Text(buff, x + 20, y - chunks * 4 / 2 - 8, WHITE, 2, BLACK);

    memset(buff, 0, sizeof(buff));
    itoa(tMax, buff, 10);
    ILI9341_Draw_Text(buff, x + 20, y - 16, WHITE, 2, BLACK);

    redraw_heatmap = 0;
  }
}


void DrawFPS(void) {
  if (!menu_show) {
    uint32_t count = HAL_GetTick();

    if (count - frame_counter >= 1000 && last_frames != frames) {
      int x = ir_width * pixel_size + offset_x;
      int y = lcd_height - 15;

      char buff[15] = "FPS: ";
      ILI9341_Draw_Rectangle(x, y, 60, 20, BLACK);

      itoa((int) frames, &buff[5], 10);
      ILI9341_Draw_Text(buff, x, y, WHITE, 1, BLACK);

      frame_counter = count;

      last_frames = frames;
      frames = 0;
    }
  }
}


int32_t UserInterface_ShowingMenu(void) {
  return menu_show;
}

void UserInterface_RedrawIRImageIfNecessary(float *image) {
  if (do_redraw_ir_image) {
    for (int pixelNumber = 0; pixelNumber < 768; pixelNumber++) {
      int row = pixelNumber >> 5;
      int col = pixelNumber & 31;

      ILI9341_Draw_Rectangle(col * pixel_size + offset_x, row * pixel_size + offset_y, pixel_size, pixel_size,
                             TemperatureConverter(image[pixelNumber]));
    }
    do_redraw_ir_image = 0;
  }
}

void UserInterface_PutAnalogData(const uint16_t data[3]) {
  new_stick_y = data[0];
  new_stick_x = data[1];
  battery_level = data[2];
}

void UserInterface_ReadStick() {
  if (new_stick_y - 2048 > 1500) {
    menu_cursor_Y = (menu_cursor_Y + 1) % menu_records;
    if (menu_cursor_Y == 0) {
      menu_cursor_Y++;
    }
    menu_redraw = 1;
  } else if (new_stick_y - 2048 < -1500) {
    menu_cursor_Y = (menu_cursor_Y - 1) % menu_records;
    if (menu_cursor_Y == 0) {
      menu_cursor_Y = menu_records - 1;
    }
    menu_redraw = 1;
  }
}

void UserInterface_Draw(void) {
  DrawFPS();
  DrawRecordState();
  DrawBatteryState();
  DrawHeatmap();


  if (menu_show) {
    UserInterface_ReadStick();
    DrawMenu();
  } else {
    frames++;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == STICK_Z_Pin) {
    uint32_t time = HAL_GetTick();

    if (time - last_stick_pressed > 100) {
      if (menu_show) {
        if (menu_cur_entry == MENU_MAIN) {
          if (menu_cursor_Y == MENU_SELECT_HEATMAP) {
            menu_cur_entry = MENU_SELECT_HEATMAP;
          } else if (menu_cursor_Y == MENU_SAVE_IMAGE) {
            InterfaceCallback_RequestSaveImage();
          } else if (menu_cursor_Y == MENU_EXIT) {
            menu_show = 0;
            do_redraw_ir_image = 1;
          }
        } else if (menu_cur_entry == MENU_SELECT_HEATMAP) {
          if (menu_cur_entry > 0 && menu_cur_entry <= Heatmap_GetSize()) {
            InterfaceCallback_RequestNewHeatmap(menu_cursor_Y - 1);
          }
          menu_cursor_Y = 1;
          menu_cur_entry = MENU_MAIN;
        }
      } else {
        menu_cursor_Y = 1;
        menu_show = 1;
      }

      menu_redraw = 1;
      last_stick_pressed = time;
    }

    redraw_record_state = 1;
  }
}
