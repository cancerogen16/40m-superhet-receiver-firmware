/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2020 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "st7735.h"
#include "si5351.h"
/* USER CODE END Includes */

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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// SSB
const int32_t Fbfo = 12000000; // xtal filter highest frequency in passband
const int32_t BfoToneShift = 0; // don't shift actual BFO frequency for SSB

typedef enum {
	ROTARY_ENCODER_MODE_CHANGE_FREQEUNCY = 0,
	ROTARY_ENCODER_MODE_CHANGE_STEP = 1,
} RotaryEncoderMode_t;

extern volatile uint8_t buttonPressed[5];

int32_t prevCounter = 0;
int32_t targetFrequency = 7115000;
int32_t frequencyStep = 1000;
int32_t prevFrequencyStep = 0;
RotaryEncoderMode_t rotaryEncoderMode;

void UART_TransmitString(const char *str) {
	HAL_UART_Transmit(&huart1, (uint8_t*) str, strlen(str), HAL_MAX_DELAY);
}

int32_t frequencyStepToDigitOffset(int32_t step) {
	// returns digit offset from the left depending on frequency step
	// example:
	// 14.000.00
	//      ^
	// step = 1000
	// offset = 4
	switch (step) {
	case 10:
		return 8;
	case 100:
		return 7;
	case 1000:
		return 5;
	case 10000:
		return 4;
	case 100000:
		return 3;
	case 1000000:
		return 1;
	default:
		return 0;
	}
}

void changeFrequencyStep(int32_t delta) {
	int32_t step = frequencyStep;
	while (delta != 0) {
		if (delta > 0) {
			step = step / 10;
			//if(step == 1) {
			//	step = 10000000;
			//}
			if (step < 10) {
				step = 100000;
			}
			delta--;
		} else {
			step = step * 10;
			//if(step > 10000000) {
			//	step = 10;
			//}
			if (step > 100000) {
				step = 10;
			}
			delta++;
		}
	}
	frequencyStep = step;
}

void changeFrequency(int32_t delta) {
	targetFrequency += frequencyStep * delta;
	if (targetFrequency < 7000000) {
		targetFrequency = 7000000;
	} else if (targetFrequency > 7200000) {
		targetFrequency = 7200000;
	}

	int32_t Fvfo;
	if (targetFrequency < 10000000) {
		Fvfo = Fbfo - targetFrequency; // LSB
	} else {
		Fvfo = Fbfo + targetFrequency; // USB
	}
	si5351_EnableOutputs((1 << 0));
	si5351_SetupCLK2(Fvfo, SI5351_DRIVE_STRENGTH_4MA);
	si5351_EnableOutputs((1 << 2) | (1 << 0));
}

void changeRotaryEncoderMode() {
	if (rotaryEncoderMode == ROTARY_ENCODER_MODE_CHANGE_FREQEUNCY) {
		rotaryEncoderMode = ROTARY_ENCODER_MODE_CHANGE_STEP;
	} else {
		rotaryEncoderMode = ROTARY_ENCODER_MODE_CHANGE_FREQEUNCY;
	}
}

void displayFrequency() {
	if (frequencyStep != prevFrequencyStep) {
		// changeFrequencyStep() was called
		int32_t digitOffset = frequencyStepToDigitOffset(prevFrequencyStep);
		ST7735_FillRectangle(16 * digitOffset, 26 * 2, 16, 2, ST7735_BLACK);

		digitOffset = frequencyStepToDigitOffset(frequencyStep);
		ST7735_FillRectangle(16 * digitOffset, 26 * 2, 16, 2, ST7735_GREEN);

		prevFrequencyStep = frequencyStep;
	}

//	char buff[16];
//	snprintf(buff, sizeof(buff), "%02ld.%03ld.%02ld", 
//		targetFrequency / 1000000, (targetFrequency % 1000000) / 1000, (targetFrequency % 1000) / 10);
//	ST7735_WriteString(0, 26*1, buff, Font_16x26, ST7735_GREEN, ST7735_BLACK);

	static bool firstCall = true;
	// A little optimization
	if (firstCall) {
		ST7735_WriteString(16, 26 * 1, "7.", Font_16x26, ST7735_GREEN,
		ST7735_BLACK);
		firstCall = false;
	}

	char buff[16];
	snprintf(buff, sizeof(buff), "%03ld.%02ld",
			(targetFrequency % 1000000) / 1000, (targetFrequency % 1000) / 10);
	ST7735_WriteString(16 * 3, 26 * 1, buff, Font_16x26, ST7735_GREEN,
			ST7735_BLACK);
}

void showMessage(const char *str) {
	ST7735_WriteString(1, 1, str, Font_7x10, ST7735_YELLOW, ST7735_BLACK);
}

void init() {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	// Make sure all other components are powered up and ready
	HAL_Delay(100);

	UART_TransmitString("Starting encoder timer...\r\n");
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	UART_TransmitString("Initializing ST7735...\r\n");
	ST7735_Init();
	ST7735_FillScreen(ST7735_BLACK);
	showMessage("Initialized ST7735");

	UART_TransmitString("Initializing Si5351...\r\n");
	showMessage("Initializing Si5351");

	const int32_t correction = 5810;
	si5351_Init(correction);
	si5351_SetupCLK0(Fbfo + BfoToneShift, SI5351_DRIVE_STRENGTH_4MA);
	changeFrequency(0);
	showMessage("Initialized Si5351");
	HAL_Delay(100);

	displayFrequency();

	UART_TransmitString("Ready!\r\n");
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void loop() {
	int32_t currCounter = __HAL_TIM_GET_COUNTER(&htim1);
	currCounter = 32767 - ((currCounter - 1) & 0xFFFF) / 2;
	if (currCounter > 32768 / 2) {
		// convert ... 32766, 32767, 0, 1, 2 ... into:
		//               ... -2, -1, 0, 1, 2 ...
		// this simplifies `delta` calculation
		currCounter = currCounter - 32768;
	}
	if (currCounter != prevCounter) {
		int32_t delta = currCounter - prevCounter;
		prevCounter = currCounter;
		// debounce, or skip counter overflow
		if ((delta > -10) && (delta < 10)) {
			if (rotaryEncoderMode == ROTARY_ENCODER_MODE_CHANGE_FREQEUNCY) {
				changeFrequency(delta);
			} else {
				changeFrequencyStep(delta);
			}
			displayFrequency();
		}
	}

	uint8_t buttonNumber = 0;
	while (buttonNumber < sizeof(buttonPressed) / sizeof(buttonPressed[0])) {
		if (buttonPressed[buttonNumber]) {
			buttonPressed[buttonNumber] = 0;
			if (buttonNumber == 0) {
				changeRotaryEncoderMode();
			} else {
				// other buttons are not used yet
				// char buff[16];
				// snprintf(buff, sizeof(buff), "BUTTON %d", buttonNumber);
				// ST7735_WriteString(0, 26*0, buff, Font_16x26, ST7735_RED, ST7735_BLACK);
			}
		}
		buttonNumber++;
	}

	HAL_Delay(100);
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
	MX_SPI1_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		loop();
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.Direction = SPI_DIRECTION_1LINE;
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 9600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, ST7735_RST_Pin | ST7735_CS_Pin | ST7735_DC_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : ST7735_RST_Pin ST7735_CS_Pin */
	GPIO_InitStruct.Pin = ST7735_RST_Pin | ST7735_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ST7735_DC_Pin */
	GPIO_InitStruct.Pin = ST7735_DC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ST7735_DC_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
