/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BMP180_READ 				0xEF
#define BMP180_WRITE 				0xEE

#define BMP180_CTRL_REG				0xF4
#define BMP180_OUT_MSB				0xF6
#define BMP180_OUT_LSB				0xF7
#define BMP180_OUT_XLSB				0xF8
#define TEMP_CTRL_REG 				0x2E
#define PRES_CTRL_REG				0x34
#define CALIBRATION_DATA_BASE_ADDR	0xAA

#define BMP180_PRES_OSS				0x00	// Can be 0, 1, 2 or 3

#define AC1							(int16_t)calibration_data[0]
#define AC2							(int16_t)calibration_data[1]
#define AC3							(int16_t)calibration_data[2]
#define AC4							calibration_data[3]
#define AC5							calibration_data[4]
#define AC6							calibration_data[5]
#define B1							(int16_t)calibration_data[6]
#define B2							(int16_t)calibration_data[7]
#define MB							(int16_t)calibration_data[8]
#define MC							(int16_t)calibration_data[9]
#define MD							(int16_t)calibration_data[10]

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t calibration_data[11];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int32_t bmp180_calpressure(int32_t upres, int32_t temp);
int32_t bmp180_caltemperature(int32_t utemp);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  unsigned char msg[70];
  unsigned char buf[3];
  HAL_StatusTypeDef ret;

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Read calibration values
  for (int i = 0; i < 11; i++) {

	  buf[0] = CALIBRATION_DATA_BASE_ADDR + 2 * i;
	  buf[1] = buf[0] + 1;

	  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, buf, 2, HAL_MAX_DELAY);

	  if (ret == HAL_OK) {
		  ret = HAL_I2C_Master_Receive(&hi2c1, BMP180_READ, buf, 2, HAL_MAX_DELAY);
		  if (ret == HAL_OK) {
			  calibration_data[i] = buf[1] + (buf[0] << 8);
			  sprintf(msg, "Read calibration value %d: %X", i, calibration_data[i]);
			  HAL_UART_Transmit(&huart2, msg, 8, 100);
		  }
	  }
  }

  int32_t utemp;
  int32_t temp;

  int32_t upres;
  int32_t pres;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Read temperature
	  buf[0] = BMP180_CTRL_REG;
	  buf[1] = TEMP_CTRL_REG;

	  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, buf, 2, HAL_MAX_DELAY);
	  HAL_Delay(5);

	  if (ret == HAL_OK) {
		  buf[0] = BMP180_OUT_MSB;
		  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, buf, 1, HAL_MAX_DELAY);
		  if (ret == HAL_OK) {
			  ret = HAL_I2C_Master_Receive(&hi2c1, BMP180_READ, buf, 2, HAL_MAX_DELAY);
			  utemp = (buf[0] << 8) + buf[1];
			  temp = bmp180_caltemperature(utemp);
		  }
	  }

	  // Read pressure
	  buf[0] = BMP180_CTRL_REG;
	  buf[1] = PRES_CTRL_REG + (BMP180_PRES_OSS << 6);

	  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, buf, 2, HAL_MAX_DELAY);
	  HAL_Delay(5);
	  if (ret == HAL_OK) {
		  buf[0] = BMP180_OUT_MSB;
		  ret = HAL_I2C_Master_Transmit(&hi2c1, BMP180_WRITE, buf, 1, HAL_MAX_DELAY);
		  if (ret == HAL_OK) {
			  ret = HAL_I2C_Master_Receive(&hi2c1, BMP180_READ, buf, 2, HAL_MAX_DELAY);
			  upres = ((buf[0] << 16) + (buf[1] << 8)) >> ( 8 - BMP180_PRES_OSS);
			  pres = bmp180_calpressure(upres, temp);
		  }
	  }
	  sprintf(msg, "Temp: %ld, Pres: %ld\r\n", temp, pres);
	  HAL_UART_Transmit(&huart2, msg, strlen(msg), 100);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
int32_t bmp180_calpressure(int32_t upres, int32_t temp) {
	int32_t pres, x1, x2;

	int32_t b5 = temp * (1 << 4) - 8;

	int32_t b6 = b5 - 4000;
	x1 = (B2 * (b6 * b6 / (1 << 12))) / (1 << 11);
	x2 = AC2 * b6 / (1 << 11);
	int32_t x3 = x1 + x2;
	int32_t b3 = (((AC1 * 4 + x3) << BMP180_PRES_OSS) + 2) / 4;
	x1 = AC3 * b6 / (1 << 13);
	x2 = (B1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	x3 = ((x1 + x2) + 2) / 4;
	uint32_t b4 = AC4 * (unsigned long)(x3 + 32768) / (1 << 15);
	uint32_t b7 = ((unsigned long)upres - b3) * (50000 >> BMP180_PRES_OSS);
	if (b7 < 0x80000000) {
	  pres = (b7 * 2) / b4;
	}
	else {
	  pres = (b7 / b4) * 2;
	}
	x1 = (pres / (1 << 8)) * (pres / (1 << 8));
	x1 = (x1 * 3038) / (1 << 16);
	x2 = (-7357 * pres) / (1 << 16);
	pres = pres + (x1 + x2 + 3791) / (1 << 4);

	return pres;
}

int32_t bmp180_caltemperature(int32_t utemp) {
	int32_t x1, x2, b5;

	x1 = (utemp - AC6) * AC5 / (1 << 15);
	x2 = (MC * (1 << 11)) / (x1 + MD);
	b5 = x1 + x2;
	int32_t temp = (b5 + 8) / (1 << 4);

	return temp;
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

