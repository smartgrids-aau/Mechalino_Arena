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
#include <inttypes.h>  // Include for uint16_t format specifiers
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint32_t current_pwm;
	uint32_t target_pwm;
} Servo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BACKWARD_MAX 800 //800 min that works, but not max speed (2000)
#define BACKWARD_SLOW 2800
#define SERVO_STOP 3000
#define FORWARD_MAX 5100 //5100 max that works, but not max speed (4000)
#define FORWARD_SLOW 3100 // 3250 starting actually to move normally, smaller than this, not usefull!

#define ANGLE_THRESHOLD 5.0f  // Degrees
#define DISTANCE_THRESHOLD 0.05f  // Distance threshold for considering the robot at the target
#define ROTATION_TIME_360 3200 // 3.2 seconds for a 360-degree rotation
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Servo servo_left = { &htim1, TIM_CHANNEL_1, SERVO_STOP, SERVO_STOP };
Servo servo_right = { &htim2, TIM_CHANNEL_3, SERVO_STOP, SERVO_STOP };
volatile int update_pwm_flag = 0;
char rx_buffer[100];
uint8_t rx_index = 0;
volatile int rx_complete = 0;
uint8_t UART1_rxBuffer[1] = { 0 };
volatile uint16_t timer_count = 0;
volatile uint16_t increment_speed = 2000;

float current_x = 0.0f;
float current_y = 0.0f;
float current_yaw = 0.0f;

float target_x = 0.0f;
float target_y = 0.0f;
float target_yaw = 0.0f;

float calculated_rotation_time = 0.0f;
int target_set = 0;  // Flag to indicate if a target has been set
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void execute_command(const char *cmd);
void set_servo_pwm(Servo *servo, uint32_t pulse);
void start_pwm_update(uint32_t left_target, uint32_t right_target,
		uint32_t duration_ms);
void navigate_to_target(void);
void adjust_rotation(void);
//void motor(uint16_t MotL, uint16_t MotR);
int main(void);
void Error_Handler(void);
void assert_failed(uint8_t *file, uint32_t line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (UART1_rxBuffer[0] == '\n') { // Assuming commands are newline terminated
			rx_buffer[rx_index] = '\0'; // Null terminate the string
			rx_complete = 1; // Set flag for command complete
			rx_index = 0; // Reset index
		} else {
			rx_buffer[rx_index++] = UART1_rxBuffer[0]; // Store character in buffer
		}
		HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1); // Listen for next character
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		if (timer_count > 0) {
			timer_count--;

			// Update left servo PWM
			if (servo_left.current_pwm != servo_left.target_pwm) {
				if (servo_left.current_pwm < servo_left.target_pwm) {
					servo_left.current_pwm += increment_speed;
					if (servo_left.current_pwm > servo_left.target_pwm) {
						servo_left.current_pwm = servo_left.target_pwm;
					}
				} else {
					servo_left.current_pwm -= increment_speed;
					if (servo_left.current_pwm < servo_left.target_pwm) {
						servo_left.current_pwm = servo_left.target_pwm;
					}
				}
				set_servo_pwm(&servo_left, servo_left.current_pwm);
			}

			// Update right servo PWM
			if (servo_right.current_pwm != servo_right.target_pwm) {
				if (servo_right.current_pwm < servo_right.target_pwm) {
					servo_right.current_pwm += increment_speed;
					if (servo_right.current_pwm > servo_right.target_pwm) {
						servo_right.current_pwm = servo_right.target_pwm;
					}
				} else {
					servo_right.current_pwm -= increment_speed;
					if (servo_right.current_pwm < servo_right.target_pwm) {
						servo_right.current_pwm = servo_right.target_pwm;
					}
				}
				set_servo_pwm(&servo_right, servo_right.current_pwm);
			}

			// Stop servos when duration expires
			if (timer_count == 0) {
				set_servo_pwm(&servo_left, SERVO_STOP); // Stop left servo
				set_servo_pwm(&servo_right, SERVO_STOP); // Stop right servo
				HAL_TIM_Base_Stop_IT(htim); // Stop the timer
			}
		}
	}
}

void execute_command(const char *cmd) {
	if (strncmp(cmd, "STOP", 4) == 0) {
		start_pwm_update(SERVO_STOP, SERVO_STOP, 1);
	} else if (strncmp(cmd, "START_SPINNING", 14) == 0) {
		start_pwm_update(FORWARD_MAX, FORWARD_MAX, 50000); // Example to spin in place
	} else if (strncmp(cmd, "LOCATION_UPDATE", 15) == 0) {
		sscanf(cmd + 16, "%f;%f;%f", &current_x, &current_y, &current_yaw);
		if (target_set) {
			adjust_rotation(); // Adjust rotation based on the latest location only if a target is set
		}
		//printf("Current Location: X=%.4f, Y=%.4f, Yaw=%.4f",current_x, current_y, current_yaw);
	} else if (strncmp(cmd, "TARGET_UPDATE", 13) == 0) {
		sscanf(cmd + 14, "%f;%f;%f", &target_x, &target_y, &target_yaw);
		target_set = 1; // Set the target flag
		navigate_to_target();
		//printf("Target Location: X=%.4f, Y=%.4f, Yaw=%.4f",target_x, target_y, target_yaw);
	} else {
		//printf("Unknown command: %s\n", cmd);
	}
}

void set_servo_pwm(Servo *servo, uint32_t pulse) {
	servo->current_pwm = pulse;
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

void start_pwm_update(uint32_t left_target, uint32_t right_target,
		uint32_t duration_ms) {
	servo_left.target_pwm = left_target;
	servo_right.target_pwm = right_target;
	timer_count = duration_ms;
	HAL_TIM_Base_Start_IT(&htim3); // Start the timer with interrupt
}

void navigate_to_target(void) {
	// Calculate the angle to the target
	float dx = target_x - current_x;
	float dy = target_y - current_y;
	float target_angle = atan2f(dy, dx) * 180.0f / M_PI; // Convert to degrees

	// Ensure target_angle is in the range -180 to 180
	if (target_angle > 180.0f) {
		target_angle -= 360.0f;
	} else if (target_angle < -180.0f) {
		target_angle += 360.0f;
	}

	// Calculate the angle difference
	float angle_difference = target_angle - current_yaw;

	// Normalize angle difference to be within -180 to 180 degrees
	if (angle_difference > 180.0f) {
		angle_difference -= 360.0f;
	} else if (angle_difference < -180.0f) {
		angle_difference += 360.0f;
	}

	// Calculate rotation time based on angle difference
	calculated_rotation_time = fabsf(angle_difference)
			/ 360.0f* ROTATION_TIME_360;

	// Rotate the robot in place
	if (angle_difference > 0) {
		start_pwm_update(FORWARD_SLOW, FORWARD_SLOW,
				(uint32_t) calculated_rotation_time); // Rotate clockwise
	} else {
		start_pwm_update(BACKWARD_SLOW, BACKWARD_SLOW,
				(uint32_t) calculated_rotation_time); // Rotate counterclockwise
	}

	// After rotation, wait for the next location update to adjust further
}

void adjust_rotation(void) {
	// This function will be called upon receiving a LOCATION_UPDATE
	// It checks if further rotation is needed or if the robot can move forward

	// Calculate the angle to the target again
	float dx = target_x - current_x;
	float dy = target_y - current_y;
	float target_angle = atan2f(dy, dx) * 180.0f / M_PI; // Convert to degrees

	if (target_angle > 180.0f) {
		target_angle -= 360.0f;
	} else if (target_angle < -180.0f) {
		target_angle += 360.0f;
	}

	float angle_difference = target_angle - current_yaw;

	if (angle_difference > 180.0f) {
		angle_difference -= 360.0f;
	} else if (angle_difference < -180.0f) {
		angle_difference += 360.0f;
	}

	// If the robot is aligned with the target, move forward
	if (fabsf(angle_difference) <= ANGLE_THRESHOLD) {
		start_pwm_update(FORWARD_SLOW, BACKWARD_SLOW, 500); // Move forward
	} else {
		// Otherwise, adjust rotation
		navigate_to_target();
	}
}

//void motor(uint16_t MotL, uint16_t MotR) {
////	uint32_t cntL, cntR;
////	cntL = (uint32_t) MotL; //value of PWM start at 2000 and finish at 4000
////	cntR = (uint32_t) MotR;
////	TIM2->CCR3 = cntL; //put value on TIMERs registers
////	TIM1->CCR1 = cntR; //put value on TIMERs registers
//	set_servo_pwm(&htim1, TIM_CHANNEL_1, (uint32_t) MotL); // Forward motion
//	set_servo_pwm(&htim2, TIM_CHANNEL_3, (uint32_t) MotR);  // Backward motion
//}

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
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_Delay(2000);

	/*while (MPU6050_Init(&hi2c1) == 1); //Initialise the MPU6050
	 // calibrate MPU6050
	 for (int interations = 0; interations < CALIB; interations++) {
	 MPU6050_Read_All(&hi2c1, &MPU6050);
	 Gz_mean += MPU6050.Gz;
	 Ax_mean += MPU6050.Ax;
	 Ay_mean += MPU6050.Ay;
	 HAL_Delay(50);
	 }
	 Gz_mean /= CALIB;
	 Ax_mean /= CALIB;
	 Ay_mean /= CALIB;*/

	HAL_TIM_PWM_Init(&htim1);
	HAL_TIM_PWM_Init(&htim2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	TIM1->CR1 = 0x01;
	TIM2->CR1 = 0x01;
	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, sizeof(UART1_rxBuffer)); // interrupt based

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (rx_complete) {
			execute_command(rx_buffer); // Process the command
			rx_complete = 0; // Reset the completion flag
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
