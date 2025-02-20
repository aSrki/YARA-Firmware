/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define JOINT1_REDUCTION 2.97
#define JOINT2_REDUCTION 36
#define JOINT3_REDUCTION 20.4
#define JOINT4_REDUCTION 1
#define JOINT5_REDUCTION 1.27

#define MICROSTEPS_FOR_DEG 4.44
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct micStepTim {
	int ms1;
	int ms2;
	int ms3;
	int ms4;
	int ms5;
} sMicStepTim;

typedef struct jointAngle {
	float j1;
	float j2;
	float j3;
	float j4;
	float j5;
} sJointAngle;

typedef struct jointDir {
	int8_t d1;
	int8_t d2;
	int8_t d3;
	int8_t d4;
	int8_t d5;
} sJointDir;

typedef struct elaTim {
	int tim1;
	int tim2;
	int tim3;
	int tim4;
	int tim5;
} sElaTim;

typedef struct upDown {
	uint8_t up_dn1;
	uint8_t up_dn2;
	uint8_t up_dn3;
	uint8_t up_dn4;
	uint8_t up_dn5;
} sUpDown;

typedef struct microsteps {
	int ang1, ang2, ang3, ang4, ang5, ang_diff;
} sMicroteps;

sJointDir jointDirections;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
		;  // wait for the counter to reach the us input in the parameter
}

//dir = 0 -> arm rotates clockwise
//dir = 1 -> arm rotates counterclockwise
int rotate_joint_1(float ang, uint8_t dir) {
	HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, dir);
	int microsteps = (int) ang * JOINT1_REDUCTION * MICROSTEPS_FOR_DEG;
	return microsteps;
}

//dir = 1 -> arm goes up
//dir = 0 -> arm goes down
int rotate_joint_2(float ang, uint8_t dir) {
	HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, dir);
	int microsteps = (int) ang * JOINT2_REDUCTION * MICROSTEPS_FOR_DEG;
	return microsteps;
}

//dir = 1 -> arm goes down
//dir = 0 -> arm goes up
int rotate_joint_3(float ang, uint8_t dir) {
	HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, dir == 1 ? 0 : 1);
	int microsteps = (int) ang * JOINT3_REDUCTION * MICROSTEPS_FOR_DEG;
	return microsteps;
}

//dir = 1 -> segment goes up
//dir = 0 -> segment goes down
int rotate_joint_4(float ang, uint8_t dir) {
	if (dir) {
		HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, 1);
		HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, 0);
	} else {
		HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, 0);
		HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, 1);
	}

	int microsteps = (int) (ang * JOINT4_REDUCTION * MICROSTEPS_FOR_DEG);
	return microsteps;
}

int rotate_joint_5(float ang, uint8_t dir) {
	HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, dir);
	HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, dir);

	int microsteps = (int) (ang * JOINT5_REDUCTION * MICROSTEPS_FOR_DEG);
	return microsteps;
}

//dir4 = 0 -> dize se ruka
//dir4 = 1 -> spusta se ruka
void rotate_diff_joint(float ang4, float ang5, uint8_t dir4, uint8_t dir5,
		int *nema_4, int *nema_5) {

	int microsteps4 = (int) (ang4 * JOINT4_REDUCTION * MICROSTEPS_FOR_DEG);
	int microsteps5 = (int) (ang5 * JOINT5_REDUCTION * MICROSTEPS_FOR_DEG);

	int sign4 = (dir4 == 0 ? 1 : -1);
	int sign5 = (dir5 == 0 ? 1 : -1);


	*nema_4 = microsteps4 * sign4 + microsteps5 * sign5;
	*nema_5 = -microsteps4 * sign4 + microsteps5 * sign5;

	jointDirections.d4 = *nema_4 < 0 ? -1 : 1;
	jointDirections.d5 = *nema_5 < 0 ? 1 : -1;

	HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, *nema_4 < 0 ? 1 : 0);
	HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, *nema_5 < 0 ? 1 : 0);

	*nema_4 = abs(*nema_4);
	*nema_5 = abs(*nema_5);
}

void calculate_times(sMicroteps *sMicrosteps, sMicStepTim *sMictoStepsTime) {
	if (sMicrosteps->ang1 != 0) {
		sMictoStepsTime->ms1 = (int)(((float)sMicrosteps->ang2 / (float)sMicrosteps->ang1)
				* (float)sMictoStepsTime->ms2);
	}
	if (sMicrosteps->ang3 != 0) {
		sMictoStepsTime->ms3 = (int)((((float)sMicrosteps->ang2) / ((float)sMicrosteps->ang3))
				* (float)sMictoStepsTime->ms2);
	}
	if (sMicrosteps->ang4 != 0) {
		sMictoStepsTime->ms4 = (int)(((float)sMicrosteps->ang2 / (float)sMicrosteps->ang4)
				* (float)sMictoStepsTime->ms2);
	}
	if (sMicrosteps->ang5 != 0) {
		sMictoStepsTime->ms5 = (int)(((float)sMicrosteps->ang2 / (float)sMicrosteps->ang5)
				* (float)sMictoStepsTime->ms2);
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
sMicStepTim sMictoStepsTime;
sElaTim sElapsedTime;
sUpDown sUpDwn;
sMicroteps sMicrosteps;
sJointAngle jointAngles;
int cnt = 0;
uint8_t RxData[6];
uint8_t TxData[20];

float ang1, ang2, ang3, ang4, ang5;
int8_t ab_cnt = 0;

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
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */

	sElapsedTime.tim1 = 0;
	sElapsedTime.tim2 = 0;
	sElapsedTime.tim3 = 0;
	sElapsedTime.tim4 = 0;
	sElapsedTime.tim5 = 0;

	sUpDwn.up_dn1 = 0;
	sUpDwn.up_dn2 = 0;
	sUpDwn.up_dn3 = 0;
	sUpDwn.up_dn4 = 0;
	sUpDwn.up_dn5 = 0;

	sMictoStepsTime.ms1 = 80;
	sMictoStepsTime.ms2 = 20;
	sMictoStepsTime.ms3 = 20;
	sMictoStepsTime.ms4 = 120;
	sMictoStepsTime.ms5 = 120;

	jointAngles.j1 = 0.1;
	jointAngles.j2 = 1.2;
	jointAngles.j3 = 3.4;
	jointAngles.j4 = 4.5;
	jointAngles.j5 = 6.7;

	jointDirections.d1 = 1;
	jointDirections.d2 = 1;
	jointDirections.d3 = 1;
	jointDirections.d4 = 1;
	jointDirections.d5 = 1;

	HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, 0);
	HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, 0);
	HAL_GPIO_WritePin(STEP3_GPIO_Port, STEP3_Pin, 0);
	HAL_GPIO_WritePin(STEP4_GPIO_Port, STEP4_Pin, 0);
	HAL_GPIO_WritePin(STEP5_GPIO_Port, STEP5_Pin, 0);

//	calculate_times(&sMicrosteps, &sMictoStepsTime);

	HAL_UART_Receive_IT(&huart2, RxData, 6);

	HAL_TIM_Base_Start(&htim1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		sElapsedTime.tim1++;
		sElapsedTime.tim2++;
		sElapsedTime.tim3++;
		sElapsedTime.tim4++;
		sElapsedTime.tim5++;

		if ((sElapsedTime.tim1 == sMictoStepsTime.ms1)
				&& sMicrosteps.ang1 > 0) {
			HAL_GPIO_TogglePin(STEP1_GPIO_Port, STEP1_Pin);
			sElapsedTime.tim1 = 0;

			if (sUpDwn.up_dn1 == 0) {
				sUpDwn.up_dn1 = 1;
			} else {
				sUpDwn.up_dn1 = 0;
			}

			if (sUpDwn.up_dn1 == 0) {
				sMicrosteps.ang1--;
				jointAngles.j1 +=
						jointDirections.d1 * 1.0
								/ ((float) JOINT1_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
			}
		}
		if ((sElapsedTime.tim2 == sMictoStepsTime.ms2)
				&& sMicrosteps.ang2 > 0) {
			HAL_GPIO_TogglePin(STEP2_GPIO_Port, STEP2_Pin);
			sElapsedTime.tim2 = 0;

			if (sUpDwn.up_dn2 == 0) {
				sUpDwn.up_dn2 = 1;
			} else {
				sUpDwn.up_dn2 = 0;
			}

			if (sUpDwn.up_dn2 == 0) {
				sMicrosteps.ang2--;
				jointAngles.j2 +=
						jointDirections.d2 * 1.0
								/ ((float) JOINT2_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
			}
		}
		if ((sElapsedTime.tim3 == sMictoStepsTime.ms3)
				&& sMicrosteps.ang3 > 0) {
			HAL_GPIO_TogglePin(STEP3_GPIO_Port, STEP3_Pin);
			sElapsedTime.tim3 = 0;

			if (sUpDwn.up_dn3 == 0) {
				sUpDwn.up_dn3 = 1;
			} else {
				sUpDwn.up_dn3 = 0;
			}

			if (sUpDwn.up_dn3 == 0) {
				sMicrosteps.ang3--;
				jointAngles.j3 +=
						jointDirections.d3 * 1.0
								/ ((float) JOINT3_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
			}
		}
		if ((sElapsedTime.tim4 == sMictoStepsTime.ms4)
				&& sMicrosteps.ang4 > 0) {
			HAL_GPIO_TogglePin(STEP4_GPIO_Port, STEP4_Pin);
			sElapsedTime.tim4 = 0;

			if (sUpDwn.up_dn4 == 0) {
				sUpDwn.up_dn4 = 1;
			} else {
				sUpDwn.up_dn4 = 0;
			}

			if (sUpDwn.up_dn4 == 0) {
				sMicrosteps.ang4--;

				jointAngles.j4 +=
						jointDirections.d4 * 0.5
								/ ((float) JOINT4_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
				jointAngles.j5 +=
						-jointDirections.d4 * 0.5
								/ ((float) JOINT5_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
			}
		}
		if ((sElapsedTime.tim5 == sMictoStepsTime.ms5)
				&& sMicrosteps.ang5 > 0) {
			HAL_GPIO_TogglePin(STEP5_GPIO_Port, STEP5_Pin);
			sElapsedTime.tim5 = 0;

			if (sUpDwn.up_dn5 == 0) {
				sUpDwn.up_dn5 = 1;
			} else {
				sUpDwn.up_dn5 = 0;
			}

			if (sUpDwn.up_dn5 == 0) {
				sMicrosteps.ang5--;
				jointAngles.j4 +=
						jointDirections.d5 * 0.5
								/ ((float) JOINT4_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
				jointAngles.j5 +=
						jointDirections.d5 * 0.5
								/ ((float) JOINT5_REDUCTION
										* (float) MICROSTEPS_FOR_DEG);
			}
		}

		delay_us(10);
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 36 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 36 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000 - 1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			STEP4_Pin | DIR4_Pin | STEP5_Pin | DIR5_Pin | DIR2_Pin | DIR1_Pin
					| STEP2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, STEP1_Pin | STEP3_Pin | DIR3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : STEP4_Pin STEP5_Pin STEP2_Pin */
	GPIO_InitStruct.Pin = STEP4_Pin | STEP5_Pin | STEP2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR4_Pin DIR5_Pin DIR2_Pin */
	GPIO_InitStruct.Pin = DIR4_Pin | DIR5_Pin | DIR2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : STEP1_Pin */
	GPIO_InitStruct.Pin = STEP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(STEP1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DIR1_Pin */
	GPIO_InitStruct.Pin = DIR1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DIR1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : STEP3_Pin DIR3_Pin */
	GPIO_InitStruct.Pin = STEP3_Pin | DIR3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart2, RxData, 6);
	if (RxData[0] == 0xAA) {
		switch (RxData[1]) {
		float ang;
	case 0xA1:
		ang = *(float*) (&RxData[2]);
		jointDirections.d1 = ang < 0 ? -1 : 1;
		sMicrosteps.ang1 = rotate_joint_1(abs(ang), ang < 0 ? 0 : 1);
		sElapsedTime.tim1 = 0;
		break;
	case 0xA2:
		ang = *(float*) (&RxData[2]);
		jointDirections.d2 = ang < 0 ? -1 : 1;
		sMicrosteps.ang2 = rotate_joint_2(abs(ang), ang < 0 ? 0 : 1);
		sElapsedTime.tim2 = 0;
		break;
	case 0xA3:
		ang = *(float*) (&RxData[2]);
		jointDirections.d3 = ang < 0 ? -1 : 1;
		sMicrosteps.ang3 = rotate_joint_3(abs(ang), ang < 0 ? 0 : 1);
		sElapsedTime.tim3 = 0;
		break;
	case 0xA4:
		ang = *(float*) (&RxData[2]);
		rotate_diff_joint(abs(ang), 0, ang < 0 ? 0 : 1, 0, &sMicrosteps.ang4,
				&sMicrosteps.ang5);
		sElapsedTime.tim4 = 0;
		sElapsedTime.tim5 = 0;
		break;
	case 0xA5:
		ang = *(float*) (&RxData[2]);
		rotate_diff_joint(0, abs(ang), 0, ang < 0 ? 0 : 1, &sMicrosteps.ang4,
				&sMicrosteps.ang5);
		sElapsedTime.tim4 = 0;
		sElapsedTime.tim5 = 0;
		break;
		}
	} else if (RxData[0] == 0xAB) {
		switch (RxData[1]) {
		case 0xA1:
			ang1 = *(float*) (&RxData[2]);
			jointDirections.d1 = ang1 < 0 ? -1 : 1;
			break;
		case 0xA2:
			ang2 = *(float*) (&RxData[2]);
			jointDirections.d2 = ang2 < 0 ? -1 : 1;
			break;
		case 0xA3:
			ang3 = *(float*) (&RxData[2]);
			jointDirections.d3 = ang3 < 0 ? -1 : 1;
			break;
		case 0xA4:
			ang4 = *(float*) (&RxData[2]);
			break;
		case 0xA5:
			ang5 = *(float*) (&RxData[2]);
			sMicrosteps.ang1 = rotate_joint_1(abs(ang1), ang1 < 0 ? 0 : 1);
			sMicrosteps.ang2 = rotate_joint_2(abs(ang2), ang2 < 0 ? 0 : 1);
			sMicrosteps.ang3 = rotate_joint_3(abs(ang3), ang3 < 0 ? 0 : 1);
			rotate_diff_joint(abs(ang4), abs(ang5), ang4 < 0 ? 0 : 1,
					ang5 < 0 ? 0 : 1, &sMicrosteps.ang4, &sMicrosteps.ang5);
			if (sMicrosteps.ang2 != 0) {
				calculate_times(&sMicrosteps, &sMictoStepsTime);
			}
			sElapsedTime.tim1 = 0;
			sElapsedTime.tim2 = 0;
			sElapsedTime.tim3 = 0;
			sElapsedTime.tim4 = 0;
			sElapsedTime.tim5 = 0;
			break;
		}

	} else if (RxData[0] == 0xAC) {
		jointAngles.j1 = 0;
		jointAngles.j2 = 0;
		jointAngles.j3 = 0;
		jointAngles.j4 = 0;
		jointAngles.j5 = 0;
		sMictoStepsTime.ms1 = 80;
		sMictoStepsTime.ms2 = 20;
		sMictoStepsTime.ms3 = 20;
		sMictoStepsTime.ms4 = 120;
		sMictoStepsTime.ms5 = 120;
	} else if (RxData[0] == 0xAD) {
		HAL_UART_Transmit(&huart2, (uint8_t*) &jointAngles, 20, 100);
	}
	return;
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
