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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  FPS2HZ   0x02
#define  FPS4HZ   0x03
#define  FPS8HZ   0x04
#define  FPS16HZ  0x05
#define  FPS32HZ  0x06

#define  MLX90640_ADDR 0x33
#define	 RefreshRate FPS8HZ
#define  TA_SHIFT 8 //Default shift for MLX90640 in open air
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
static paramsMLX90640 mlxParams;
static uint16_t frame[834];
static uint16_t eeMLX90640[832];
static float image[32 * 24];
static uint16_t pixels[32 * 24];
static float image_upscaled_1[64 * 48];

static float emissivity = 0.95f;
static int mlx_status;

volatile uint8_t SPI2_TX_completed_flag = 1; //flag indicating finish of SPI transmission
volatile uint8_t record = 0;
volatile uint8_t change_record = 1;
volatile uint8_t upscale = 0;

uint16_t (*TempConverter)(float) = &TempToMagma565_Fast;

float tMin = 20.0f;
float tMax = 25.0f;
float tMinOld;
float tMaxOld;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_SPI1_Init(void);

static void MX_TIM3_Init(void);

static void MX_I2C2_Init(void);

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

void ChangeAndDisplayRecordState(void) {
  record = !record;

  ILI9341_Draw_Rectangle(lcd_width - pixel_size * 4, 0, pixel_size * 4, pixel_size * 2 + 4, BLACK);

  if (record) {
    ILI9341_Draw_Filled_Circle(lcd_width - pixel_size * 2 + 4, pixel_size, pixel_size, RED);
  } else {
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2, 0, 4, pixel_size * 2, LIGHTGREY);
    ILI9341_Draw_Rectangle(lcd_width - pixel_size * 2 + 6, 0, 4, pixel_size * 2, LIGHTGREY);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == USER_BUTTON_Pin) {
    change_record = 1;
    upscale = 1;
  }
}

void DrawHeatmp(void) {
  tMinOld = tMin;
  tMaxOld = tMax;

  int x = (ir_width + 1) * pixel_size + offset_x;
  int y = offset_y;

  ILI9341_Draw_Text("Heatmap", x, y, WHITE, 1, BLACK);

  y += pixel_size * 2;

  char buff[10] = {};
  itoa(tMin, buff, 10);
  ILI9341_Draw_Rectangle(x, y, pixel_size, pixel_size, TempConverter(tMin));
  ILI9341_Draw_Text(buff, x + pixel_size * 2, y, WHITE, 1, BLACK);

  memset(buff, 0, sizeof(buff));
  float mid = (tMax + fabs(tMin)) / 2;
  itoa(mid, buff, 10);
  y += pixel_size;
  ILI9341_Draw_Rectangle(x, y, pixel_size, pixel_size, TempConverter(mid));
  ILI9341_Draw_Text(buff, x + pixel_size * 2, y, WHITE, 1, BLACK);

  memset(buff, 0, sizeof(buff));
  itoa(tMax, buff, 10);
  y += pixel_size;
  ILI9341_Draw_Rectangle(x, y, pixel_size, pixel_size, TempConverter(tMax));
  ILI9341_Draw_Text(buff, x + pixel_size * 2, y, WHITE, 1, BLACK);
}

void UpscaleTimesTwo(const float *original_image, float *upscaled_image, const int width_before,
                     const int height_before) {
  int width_new = width_before * 2;
  int height_new = height_before * 2;

  // Copy real data to checkerboard-like structure like this
  // x - x - x .... -     0
  // - - - - - .... -     1
  // x - x - x .... -     2
  // - - - - - .... -     3
  for (int x = 0; x < width_before; ++x) {
    for (int y = 0; y < height_before; ++y) {
      upscaled_image[(y * 2) * width_new + x * 2] = original_image[y * width_before + x];
    }
  }

  for (int y = 0; y < height_new; ++y) {
    for (int x = 0; x < width_new; ++x) {
      // row even -> partially filled -> interpolate only between two neighbors

      if (y % 2 == 0) {
        //skips every second integer (which has a "real" pixel value)
        if (x % 2 != 0) {
          float left;
          float right;

          if (x == width_new - 1) {
            left = upscaled_image[y * width_new + x - 1];
            right = left;
          } else {
            left = upscaled_image[y * width_new + x - 1];
            right = upscaled_image[y * width_new + x + 1];
          }
          upscaled_image[y * width_new + x] = (left + right) / 2.0f;
        }
      }
      //row odd -> empty -> interpolate between the 4 surrounding cells
      else {
        // boundary cells left -> only use top and bottom
        // x x ... (interpolated before)
        // - - ... (value searched)
        // x - ...
        if (x == 0) {
          float top = upscaled_image[(y - 1) * width_new + x];
          float bottom = top;
          // remark: the case y == 0 is handled in the very first if and not relevant, here
          if (y < height_new - 1) {
            bottom = upscaled_image[(y + 1) * width_new + x];
          }
          upscaled_image[y * width_new + x] = (top + bottom) / 2.0f;
        }
        // boundary cells right -> there is only an (already interpolated) top value due to the even structure of the checkerboard.
        // instead, use its left neighbor
        // ... x x (interpolated before)
        // ... x - (value searched)
        // ... x - (next row has checkerboard pattern)
        else if (x == width_new - 1) {
          float top = upscaled_image[(y - 1) * width_new + x];
          float bottom = top;
          if (y < height_new - 1) {
            bottom = upscaled_image[(y + 1) * width_new + x - 1];
          }
          upscaled_image[y * width_new + x] = (top + bottom) / 2.0f;
        } else {
          float left_top = upscaled_image[(y - 1) * width_new + x - 1];
          float right_top = upscaled_image[(y - 1) * width_new + x + 1];
          float left_bottom = left_top;
          float right_bottom = right_top;
          if (y < height_new - 1) {
            left_bottom = upscaled_image[(y + 1) * width_new + x - 1];
            right_bottom = upscaled_image[(y + 1) * width_new + x + 1];
          }
          upscaled_image[y * width_new + x] = (left_top + right_top + left_bottom + right_bottom) / 4.0f;
        }
      }
    }
  }
}

void UpscaleTimesTwoAndDisplayImmediately(const float *original_image, const int width_before,
                                          const int height_before) {
  int width_new = width_before * 2;
  int height_new = height_before * 2;

  for (int y = 0; y < height_new; ++y) {
    for (int x = 0; x < width_new; ++x) {
      float temp;
      int y_old = y / 2;
      int x_old = x / 2;

      // row even -> partially filled -> interpolate only between two neighbors
      if (y % 2 == 0) {
        //skips every second integer (which has a "real" pixel value)
        if (x % 2 != 0) {
          float left;
          float right;

          if (x == width_new - 1) {
            left = original_image[y_old * width_before + x_old];
            right = left;
          } else {
            left = original_image[y_old * width_before + x_old];
            right = original_image[y_old * width_before + x_old + 1];
          }
          temp = (left + right) / 2.0f;
        } else {
          temp = original_image[y_old * width_before + x_old];
        }
      }
      //row odd -> empty -> interpolate between the 4 surrounding cells
      else {
        if (x == 0) {
          float top = original_image[y_old * width_before + x_old];
          float bottom = top;
          // remark: the case y == 0 is handled in the very first if and not relevant, here
          if (y < height_new - 1) {
            bottom = original_image[(y_old + 1) * width_before + x_old];
          }
          temp = (top + bottom) / 2.0f;
        }
        else if (x == width_new - 1) {
          float top = original_image[y_old * width_before + x_old];
          float bottom = top;
          if (y < height_new - 1) {
            bottom = original_image[(y_old + 1) * width_before + x_old];
          }
          temp = (top + bottom) / 2.0f;
        } else {
          float left_top = original_image[y_old * width_before + x_old];
          float right_top = original_image[y_old * width_before + x_old + 1];
          float left_bottom = left_top;
          float right_bottom = right_top;
          if (y < height_new - 1) {
            left_bottom = original_image[(y_old + 1) * width_before + x_old];
            right_bottom = original_image[(y_old + 1) * width_before + x_old + 1];
          }
          temp = (left_top + right_top + left_bottom + right_bottom) / 4.0f;
        }
      }

      ILI9341_Draw_Rectangle(x * pixel_size / 2 + offset_x, y * pixel_size / 2 + offset_y, pixel_size / 2,
                             pixel_size / 2,
                             TempConverter(temp));
    }
  }
}

void MLX90640_ReadUpscaleAndDisplay() {
  for (int i = 0; i < 2; ++i) {
    MLX90640_GetFrameData(MLX90640_ADDR, frame);
    float Ta = MLX90640_GetTa(frame, &mlxParams);
    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    MLX90640_CalculateToAndDisplay(frame, &mlxParams, emissivity, tr, image, 0, 1);
  }
  UpscaleTimesTwo(image, image_upscaled_1, ir_width, ir_height);
  for (int y = 0; y < ir_height * 2; ++y) {
    for (int x = 0; x < ir_width * 2; ++x) {
      ILI9341_Draw_Rectangle(x * pixel_size / 2 + offset_x, y * pixel_size / 2 + offset_y, pixel_size / 2,
                             pixel_size / 2,
                             TempConverter(image_upscaled_1[y * ir_width * 2 + x]));
    }
  }
}


void MLX90640_ReadAndDisplay(void) {
  int a = HAL_GetTick();
  MLX90640_GetFrameData(MLX90640_ADDR, frame);
  int b = HAL_GetTick();
  float Ta = MLX90640_GetTa(frame, &mlxParams);
  float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
  int c = HAL_GetTick();
  MLX90640_CalculateToAndDisplay(frame, &mlxParams, emissivity, tr, image, 1, 1);
  int d = HAL_GetTick();
  int x = d - c;
  int y = d - a;
  int z = b - a;
  int v = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  MLX90640_Init();
  DrawHeatmp();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  tMinOld = tMin;
  tMaxOld = tMax;
  int did_nothing;

  while (1) {
    did_nothing = 1;

    if (change_record) {
      ChangeAndDisplayRecordState();
      change_record = 0;
      did_nothing = 0;
    }

    if (record) {
      if (upscale) {
        MLX90640_ReadUpscaleAndDisplay();
      } else {
        MLX90640_ReadAndDisplay();
      }
      did_nothing = 0;
    }

    if (fabs(tMinOld - tMin) > 0.0f || fabs(tMaxOld - tMax) > 0.0f) {
      DrawHeatmp();
      did_nothing = 0;
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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void) {
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void) {
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 300 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RST_Pin | DC_Pin | CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin | DC_Pin | CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
