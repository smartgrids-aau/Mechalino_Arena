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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// DMUX pins
#define DMUX_EN_IR_PORT GPIOB
#define DMUX_EN_IR_PIN GPIO_PIN_1

#define DMUX_EN_PD_PORT GPIOB
#define DMUX_EN_PD_PIN GPIO_PIN_0

#define DMUX_ADDR_PORT GPIOB
#define DMUX_ADDR_B0 GPIO_PIN_3
#define DMUX_ADDR_B1 GPIO_PIN_4
#define DMUX_ADDR_B2 GPIO_PIN_5

//Parameters
#define CALIB 100

#define OBSTACLE_DETECTION_THLD 0.5


#define READING_DELAY 20
#define LOOP_DELAY 100

#define MOTOR_STOP 3000
#define MOTOR_SLOW_1 2800
#define MOTOR_SLOW_2 3100
#define MOTOR_MAX_1 800
#define MOTOR_MAX_2 5100

#define NUM_SENSORS 8 // 8 IR LED and receiver pairs

#define SESNOR_IN_PORT GPIOA
#define SESNOR_IN_PIN GPIO_PIN_0

// Sensors’ indices names
#define S_FRONT 0
#define S_FRONT_LEFT 7
#define S_LEFT 6
#define S_REAR_LEFT 5
#define S_REAR 4
#define S_REAR_RIGHT 3
#define S_RIGHT 2
#define S_FRONT_RIGHT 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t sensors_calib[NUM_SENSORS];
uint16_t sensors_currrent[NUM_SENSORS];
float_t sensors_delta[NUM_SENSORS];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motors_stop();
void motors_forward();
void motors_turn_cw(uint16_t duration);
void motors_turn_ccw(uint16_t duration);
void select_mux_channel(uint8_t channel);
uint16_t read_IR_sensor(uint8_t channel);
void read_sensors();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motors_stop()
{
	TIM1->CCR1 = MOTOR_STOP;
	TIM2->CCR3 = MOTOR_STOP;
}
void motors_forward()
{
	TIM1->CCR1 = MOTOR_MAX_2;
	TIM2->CCR3 = MOTOR_MAX_1;
}
void motors_turn_cw(uint16_t duration)
{
	TIM1->CCR1 = MOTOR_MAX_2;
	TIM2->CCR3 = MOTOR_MAX_2;
	HAL_Delay(duration);
}
void motors_turn_ccw(uint16_t duration)
{
	TIM1->CCR1 = MOTOR_MAX_1;
	TIM2->CCR3 = MOTOR_MAX_1;
	HAL_Delay(duration);
}
void select_mux_channel(uint8_t channel) {
	HAL_GPIO_WritePin(DMUX_ADDR_PORT, DMUX_ADDR_B0,
			(channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B0
	HAL_GPIO_WritePin(DMUX_ADDR_PORT, DMUX_ADDR_B1,
			(channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B1
	HAL_GPIO_WritePin(DMUX_ADDR_PORT, DMUX_ADDR_B2,
			(channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);  // B2
}

uint16_t read_IR_sensor(uint8_t channel)
{
	select_mux_channel(channel); // set address for both DMUXes
	HAL_Delay(10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint16_t value =  HAL_ADC_GetValue(&hadc1);
	return value;
}
void read_sensors() {
	for (int i = 0; i < NUM_SENSORS; i++)
	{
		sensors_currrent[i] = read_IR_sensor(i);
		HAL_Delay(READING_DELAY); // Wait some time before testing next IR pair
	}
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
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Init(&htim1);
	HAL_TIM_PWM_Init(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	TIM1->CR1 = 0x01;
	TIM2->CR1 = 0x01;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	motors_stop();
	HAL_Delay(5000); // wait for user to place the robot

	HAL_GPIO_WritePin(DMUX_EN_IR_PORT, DMUX_EN_IR_PIN, GPIO_PIN_RESET); //Disable IR DMUX
	HAL_GPIO_WritePin(DMUX_EN_PD_PORT, DMUX_EN_PD_PIN, GPIO_PIN_SET); // Enable DP DMUX

	for (int s = 0; s < NUM_SENSORS; s++)
	{
		uint16_t temp_readings[CALIB];

		for (int c = 0; c < CALIB; c++)
			temp_readings[c] = read_IR_sensor(s);
		HAL_Delay(READING_DELAY);

		// use median
		// Bubble sort implementation
		for (int i = 0; i < CALIB - 1; i++) {
			for (int j = 0; j < CALIB - i - 1; j++) {
				if (temp_readings[j] > temp_readings[j + 1]) {
					// Swap temp_readings[j] and temp_readings[j + 1]
					uint16_t temp = temp_readings[j];
					temp_readings[j] = temp_readings[j + 1];
					temp_readings[j + 1] = temp;
				}
			}
		}
		sensors_calib[s] = temp_readings[CALIB / 2];
	}

	HAL_GPIO_WritePin(DMUX_EN_IR_PORT, DMUX_EN_IR_PIN, GPIO_PIN_SET); //Enable IR DMUX

	while (1)
	{
		read_sensors();

		// Compute sensors_delta
		for (int s = 0; s < NUM_SENSORS; s++)
		{
			sensors_delta[s] = sensors_currrent[s] / sensors_calib[s] - 1.0f;
		}

		uint8_t front_obstacle =   (sensors_delta[S_FRONT] > OBSTACLE_DETECTION_THLD)
								|| (sensors_delta[S_FRONT_RIGHT] > OBSTACLE_DETECTION_THLD)
								|| (sensors_delta[S_FRONT_LEFT] > OBSTACLE_DETECTION_THLD);

		if (front_obstacle)
		{
			motors_stop();
			HAL_Delay(10*LOOP_DELAY);

			if (sensors_delta[S_FRONT_LEFT] > sensors_delta[S_FRONT_RIGHT])
				motors_turn_cw(1200);
			else
				motors_turn_ccw(1200);
		}
		else
		{
			motors_forward();
			HAL_Delay(100);
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 20;
	RCC_OscInitStruct.PLL.PLLN = 128;
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

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
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
