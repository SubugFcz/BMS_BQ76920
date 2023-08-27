/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "BQ76920.h"
/* USER CODE END Includes */
#include <stdio.h>
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Struct
BQ76920_t BMS;
// Important
float PackCurrent = 0, Vcell[4], Vpack, SOC, SOH;
// Alert
uint8_t CC_READY = 0, DEVICE_XREADY = 0, OVRD_ALERT = 0, UV = 0, OV = 0,
		SCD = 0, OCD = 0, CHECK_SOH = 1, UPDATE_SOH = 1;
// For DAQ
uint32_t timeTask_30000ms = 0; // Variable to track time for 30000ms task
int32_t DATAWattUsage, DATACurrentUsage, DATAPackCurrent;
int16_t DATAVcell1, DATAVcell2, DATAVcell3, DATAVcell4, DATAVpack, DATASOHDebug,
		DATASOHEnergy, DATASOHCapacity, DATASOHOCV, DATASOC, DATASOCEnergy,
		DATASOCCapacity;
int8_t buffer[80];
// For Debug
uint8_t CELL_BAL1, SYS_CTRL2_VAL;
float SOHDebug, SOHEnergy, SOHCapacity, SOHOCV, SOCEnergy, SOCCapacity;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_I2C2_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	BMS.i2cHandle = &hi2c2;
	BQ76920_Initialise(&BMS, &hi2c2);						// Init BMS.
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		uint32_t currentTick = HAL_GetTick();

		for (int i = 0; i <= 6; i += 2) { 					// Get V cell.
			Vcell[i / 2] = getCellVoltage(&BMS, 12 + i);
		}

		Vpack = getPackVoltage(&BMS); 						// Get V pack.

		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)) { // Read Alert (every 250ms normally).
			readAlert(&BMS);
			if (getAlert(&BMS, 7)) { 						// CC_READY.
				PackCurrent = getCurrent(&BMS); // in mA
				SOC = SOCPack(&BMS, PackCurrent, Vpack);
				EnableBalanceCell(&BMS, PackCurrent); // Dynamically enable balancing.
				CC_READY += 1;
			}
			if (getAlert(&BMS, 5)) { 						// DEVICE_XREADY.
				DEVICE_XREADY += 1;
				HAL_Delay(3000); // Wait 3s.
			}
			if (getAlert(&BMS, 4)) { 						// OVRD_ALERT.
				OVRD_ALERT += 1;
			}
			if (checkUV(Vcell)) { 							// UV.
				CHECK_SOH = 0;
				UV += 1;
				turnDSGOff(&BMS);
			} else if (checkNotUV(Vcell, UV)) {
				UV = 0;
				turnDSGOn(&BMS);
			}
			if (checkOV(Vcell)) { 							// OV.
				OV += 1;

				if (PackCurrent >= 0 && !CHECK_SOH) {	// Update SOH every UV.
					SOH = SOHPack(&BMS);
					CHECK_SOH++;
					UPDATE_SOH = 0;
				}

				turnCHGOff(&BMS);
			} else if (checkNotOV(Vcell, OV)) {
				OV = 0;
				turnCHGOn(&BMS);
			}
			if (getAlert(&BMS, 1)) { 						// SCD.
				SCD += 1;
				turnDSGOff(&BMS);
			}
			if (getAlert(&BMS, 0)) { 						// OCD.
				OCD += 1;
				turnDSGOff(&BMS);
			}
			if (!(DEVICE_XREADY || OVRD_ALERT || SCD || OCD)) {
				CLEAR_SYS_STAT(&BMS);
			}
		}

		if (PackCurrent < 0.0f && !UV && UPDATE_SOH) {
			CHECK_SOH++;
		}

		// Log Data to serial monitor.
		if (currentTick - timeTask_30000ms >= 30000) 		// Do every 3s.
				{
			logData();
			timeTask_30000ms = currentTick; // Reset the timer
		}

		// Debug.
		CELL_BAL1 = justRead1(&BMS);
		SYS_CTRL2_VAL = justRead2(&BMS);

		SOCEnergy = justGetter1(&BMS);
		SOCCapacity = justGetter2(&BMS);
		SOHDebug = justGetter3(&BMS);
		SOHEnergy = justGetter4(&BMS);
		SOHCapacity = justGetter5(&BMS);
		SOHOCV = justGetter6(&BMS);

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
	hi2c2.Init.Timing = 0x10707DBC;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void logData() {
	DATAWattUsage = (int32_t) (justGetter7(&BMS));
	if (DATAWattUsage & 0x80000000) {
		// Perform 2's complement conversion
		DATAWattUsage = -(~DATAWattUsage + 1);
	}
	DATACurrentUsage = (int32_t) (justGetter8(&BMS));
	if (DATACurrentUsage & 0x80000000) {
		// Perform 2's complement conversion
		DATACurrentUsage = -(~DATACurrentUsage + 1);
	}
	DATAPackCurrent = (int32_t) (PackCurrent * 100);
	if (DATAPackCurrent & 0x80000000) {
		// Perform 2's complement conversion
		DATAPackCurrent = -(~DATAPackCurrent + 1);
	}
	DATAVpack = (int16_t) (Vpack * 100);
	DATAVcell1 = (int16_t) (Vcell[0] * 100);
	DATAVcell2 = (int16_t) (Vcell[1] * 100);
	DATAVcell3 = (int16_t) (Vcell[2] * 100);
	DATAVcell4 = (int16_t) (Vcell[3] * 100);

	DATASOHDebug = (int16_t) (SOHDebug);
	DATASOHEnergy = (int16_t) (SOHEnergy);
	DATASOHCapacity = (int16_t) (SOHCapacity);
	DATASOHOCV = (int16_t) (SOHOCV);
	DATASOC = (int16_t) (SOC);
	DATASOCEnergy = (int16_t) (SOCEnergy);
	DATASOCCapacity = (int16_t) (SOCCapacity);

	sprintf(buffer,
			"%010ld %010ld %07ld %04d %03d %03d %03d %03d %03d %03d %03d %03d %03d %03d %03d\n",
			DATAWattUsage, DATACurrentUsage, DATAPackCurrent, DATAVpack,
			DATAVcell1, DATAVcell2, DATAVcell3, DATAVcell4, DATASOHDebug,
			DATASOHEnergy, DATASOHCapacity, DATASOHOCV, DATASOC, DATASOCEnergy,
			DATASOCCapacity);

	HAL_UART_Transmit(&huart2, buffer, 78, 10);
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
