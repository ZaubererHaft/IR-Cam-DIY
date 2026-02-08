/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "ILI9341_DMA_driver.h"
#include "ILI9341_GFX.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "ui_constants.h"
#include "heatmap.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum MENU_ {
  MAIN,
  SELECT_HEATMAP
} MENU;

MENU cur_entry = MAIN;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS16HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  MLX90640_Init();
  DrawHeatmp();
  DrawFPS();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 300-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin|RST_Pin|DC_Pin|CS_Pin
                          |BATTCHARGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EEPROM_NWC_GPIO_Port, EEPROM_NWC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_LED_Pin RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin|RST_Pin|DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STICK_Z_Pin */
  GPIO_InitStruct.Pin = STICK_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(STICK_Z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BATTFULL_Pin BATTCHARGING_Pin EXT_POWER_Pin */
  GPIO_InitStruct.Pin = BATTFULL_Pin|BATTCHARGING_Pin|EXT_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BATTCHARGE_Pin */
  GPIO_InitStruct.Pin = BATTCHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BATTCHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EEPROM_NWC_Pin */
  GPIO_InitStruct.Pin = EEPROM_NWC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EEPROM_NWC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USRBUTTON_Pin */
  GPIO_InitStruct.Pin = USRBUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USRBUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  SPI2_TX_completed_flag = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
