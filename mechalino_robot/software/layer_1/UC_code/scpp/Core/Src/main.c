/* USER CODE BEGIN Header */
/**
 ***********************************************
 * @file           : main.c
 * @brief          : Main program body
 ***********************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ************************************************/
/* USER CODE END Header */
/* Includes ----------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes --------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <inttypes.h>  // Include for uint16_t format specifiers
/* USER CODE END Includes */

/* Private typedef ---------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t channel;
	uint32_t current_pwm;
} Servo;

typedef enum {
	IDLE, ROTATING, MOVING, SPINNING
} RobotState;

typedef struct {
	float integral;
	float previous_error;
	float kp;
	float ki;
	float kd;
	float integral_limit;
	float output_limit;
} PIDController;

/* USER CODE END PTD */

/* Private define ----------------------------------------*/
/* USER CODE BEGIN PD */

/* PWM Values for Servo Control */
#define BACKWARD_MAX 800 	// Maximum backward speed
#define BACKWARD_SLOW 2800 	// Minimum backward speed
#define SERVO_STOP 3000
#define FORWARD_MAX 5100 	// Maximum forward speed
#define FORWARD_SLOW 3250 	// Minimum forward speed
/* Base Speeds for Each Motor */
#define BASE_SPEED_FORWARD 4175    // Base PWM for forward speed
#define BASE_SPEED_BACKWARD 1800   // Base PWM for backward speed
/* Maximum Correction Value */
#define MAX_CORRECTION 900      // Maximum correction value for pid

/* Thresholds and PID Gains */
#define ANGLE_THRESHOLD_ROTATE_TO_MOVE  3.0f  	// Degrees
#define ANGLE_THRESHOLD_MOVE_TO_ROTATE 30.0f  	// Degrees
#define DISTANCE_THRESHOLD_MOVE_TO_STOP 0.12f  	// Stop moving when within this distance
#define DISTANCE_THRESHOLD_STOP_TO_MOVE 0.15f	// Start moving again when beyond this distance

#define KP_ROTATION 10.0f 	// Proportional gain for rotation
#define KI_ROTATION 0.0f 	// Integral gain for rotation
#define KD_ROTATION 3.0f 	// Derivative gain for rotation

/* Maximum Integral Term */
#define MAX_INTEGRAL 1000.0f   // Limit for integral term

#define KP_MOVEMENT 15.0f	// Proportional gain for movement
#define KI_MOVEMENT 0.2f	// Integral gain for movement
#define KD_MOVEMENT 3.0f	// Derivative gain for movement

#define MAX_COORDS 100		// Maximum number of coordinates in path

#define SERIAL_SEND_INTERVAL 1000	// Interval in ms to send data over serial
/* USER CODE END PD */

/* Private macro -----------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables -------------------------------------*/

/* USER CODE BEGIN PV */
Servo servo_left = { &htim1, TIM_CHANNEL_1, SERVO_STOP };
Servo servo_right = { &htim2, TIM_CHANNEL_3, SERVO_STOP };

volatile RobotState current_state = IDLE;

PIDController rotation_pid = { .integral = 0.0f, .previous_error = 0.0f, .kp =
KP_ROTATION, .ki = KI_ROTATION, .kd = KD_ROTATION, .integral_limit =
MAX_INTEGRAL, .output_limit = MAX_CORRECTION }; // Tuning for rotation
PIDController movement_pid = { .integral = 0.0f, .previous_error = 0.0f, .kp =
KP_MOVEMENT, .ki = KI_MOVEMENT, .kd = KD_MOVEMENT, .integral_limit =
MAX_INTEGRAL, .output_limit = MAX_CORRECTION }; // Tuning for movement

char rx_buffer[256];
uint8_t rx_index = 0;
volatile int rx_complete = 0;
uint8_t UART1_rxBuffer[1] = { 0 };

float current_x = 0.0f;
float current_y = 0.0f;
float current_yaw = 0.0f;

float target_x = 0.0f;
float target_y = 0.0f;
float xCoords[MAX_COORDS];
float yCoords[MAX_COORDS];
int totalCoords = 0;
int currentTargetIndex = 0;

float angle_error = 0.0f;
float calculated_correction = 0.0f;
float target_distance = 0.0f;

uint32_t last_status_send_time = 0;

int path_set = 0;  // Flag to indicate if a target has been set
int locationReceived = 0;
/* USER CODE END PV */

/* Private function prototypes ---------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void execute_command(const char *cmd);
void set_servo_pwm(Servo *servo, uint32_t pulse);
void handle_movement(void);
void handle_rotation(void);
float calculate_angle(float x, float y, float x_next, float y_next);
float distance_to_target(float current_x, float current_y, float target_x,
		float target_y);
float pid_controller(PIDController *pid, float setpoint, float measured_value);
void send_status_to_esp(void);

/* USER CODE END PFP */

/* Private user code -------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief UART Receive Complete Callback.
 * @param huart Pointer to UART handle.
 */
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

/**
 * @brief PID Controller function.
 * @param pid Pointer to PIDController structure.
 * @param setpoint Desired setpoint.
 * @param measured_value Current measured value.
 * @return Control output.
 */
float pid_controller(PIDController *pid, float setpoint, float measured_value) {
	float error = setpoint - measured_value;
	pid->integral += error;

	// Anti-windup: Limit the integral term
	if (pid->integral > pid->integral_limit)
		pid->integral = pid->integral_limit;
	if (pid->integral < -pid->integral_limit)
		pid->integral = -pid->integral_limit;

	float derivative = error - pid->previous_error;
	pid->previous_error = error;

	float output = pid->kp * error + pid->ki * pid->integral
			+ pid->kd * derivative;

	// Limit the output to the maximum correction factor
	if (output > pid->output_limit)
		output = pid->output_limit;
	if (output < -pid->output_limit)
		output = -pid->output_limit;

	return output;
}

/**
 * @brief Handle the rotation of the robot towards the target angle.
 */
void handle_rotation() {
	float target_angle = calculate_angle(current_x, current_y, target_x,
			target_y);
	angle_error = target_angle - current_yaw;

	// Normalize the angle error to always choose the shortest rotation path
	if (angle_error > 180) {
		angle_error -= 360;
	} else if (angle_error < -180) {
		angle_error += 360;
	}

	if (fabsf(angle_error) > ANGLE_THRESHOLD_ROTATE_TO_MOVE) {
		// Normalize the angle error for rotation
		float normalized_angle_error = angle_error / 180.0f;

		// PID-based correction for rotation
		calculated_correction = pid_controller(&rotation_pid, 0.0f,
				normalized_angle_error);

		uint32_t left_right_pwm;

		if (angle_error > 0) {
			// Rotate right: Both motors move forward speed
			left_right_pwm = FORWARD_SLOW + calculated_correction;

			// Ensure PWM values are within valid ranges
			if (left_right_pwm > FORWARD_MAX)
				left_right_pwm = FORWARD_MAX;
			if (left_right_pwm < FORWARD_SLOW)
				left_right_pwm = FORWARD_SLOW;
		} else {
			// Rotate left: Both motors move backward speed
			left_right_pwm = BACKWARD_SLOW - calculated_correction;

			// Ensure PWM values are within valid ranges
			if (left_right_pwm < BACKWARD_MAX)
				left_right_pwm = BACKWARD_MAX;
			if (left_right_pwm > BACKWARD_SLOW)
				left_right_pwm = BACKWARD_SLOW;
		}

		angle_error = 0.0f; // reset angle error for better performance monitoring over udp as only handle_movement angle_error should be captured

		set_servo_pwm(&servo_left, left_right_pwm);
		set_servo_pwm(&servo_right, left_right_pwm);
	} else {
		// Stop rotating and switch to moving state
		set_servo_pwm(&servo_left, SERVO_STOP);
		set_servo_pwm(&servo_right, SERVO_STOP);
		current_state = MOVING;
	}
}

/**
 * @brief Handle the movement of the robot towards the target position.
 */
void handle_movement() {
	target_distance = distance_to_target(current_x, current_y, target_x,
			target_y);
	float target_angle = calculate_angle(current_x, current_y, target_x,
			target_y);
	angle_error = target_angle - current_yaw;

	// Normalize the angle error to [-180, 180]
	if (angle_error > 180.0f) {
		angle_error -= 360.0f;
	} else if (angle_error < -180.0f) {
		angle_error += 360.0f;
	}

	if (target_distance > DISTANCE_THRESHOLD_MOVE_TO_STOP) {
		if (fabsf(angle_error) > ANGLE_THRESHOLD_MOVE_TO_ROTATE) {
			// If the angle error is too large, switch to rotating state
			set_servo_pwm(&servo_left, SERVO_STOP);
			set_servo_pwm(&servo_right, SERVO_STOP);
			current_state = ROTATING;
			return;
		}

		// Normalize the angle error
		float normalized_angle_error = angle_error
				/ ANGLE_THRESHOLD_MOVE_TO_ROTATE;

		// PID-based correction for forward movement
		calculated_correction = pid_controller(&movement_pid, 0.0f,
				normalized_angle_error);

		// Base speeds for both motors
		uint32_t left_pwm = BASE_SPEED_FORWARD;
		uint32_t right_pwm = BASE_SPEED_BACKWARD;

		// Limit the correction to prevent excessive adjustments
		if (calculated_correction > MAX_CORRECTION)
			calculated_correction = MAX_CORRECTION;
		if (calculated_correction < -MAX_CORRECTION)
			calculated_correction = -MAX_CORRECTION;

		// Adjust PWM values based on the correction
		left_pwm += (int32_t) (calculated_correction); // Left motor PWM increases with speed
		right_pwm -= (int32_t) (calculated_correction); // Right motor PWM decreases with speed

		// Ensure PWM values are within valid ranges
		if (left_pwm > FORWARD_MAX)
			left_pwm = FORWARD_MAX;
		if (left_pwm < FORWARD_SLOW)
			left_pwm = FORWARD_SLOW;

		if (right_pwm < BACKWARD_MAX)
			right_pwm = BACKWARD_MAX;
		if (right_pwm > BACKWARD_SLOW)
			right_pwm = BACKWARD_SLOW;

		set_servo_pwm(&servo_left, left_pwm);
		set_servo_pwm(&servo_right, right_pwm);
	} else {
		// Stop the robot when it reaches the target
		set_servo_pwm(&servo_left, SERVO_STOP);
		set_servo_pwm(&servo_right, SERVO_STOP);

		current_state = ROTATING;

		// Hysteresis for distance threshold
		if (target_distance < DISTANCE_THRESHOLD_STOP_TO_MOVE) {
			currentTargetIndex++;
			if (currentTargetIndex < totalCoords) {
				target_x = xCoords[currentTargetIndex];
				target_y = yCoords[currentTargetIndex];
				float target_angle = calculate_angle(current_x, current_y,
						target_x, target_y);
				angle_error = target_angle - current_yaw;
				if (fabsf(angle_error) < ANGLE_THRESHOLD_MOVE_TO_ROTATE) {
					current_state = MOVING;
					return;
				}
			} else {
				current_state = IDLE;
				path_set = 0;
			}
		}
	}
}

/**
 * @brief Calculate the angle between the robot's orientation and the target position.
 * @param x Current x position.
 * @param y Current y position.
 * @param x_next Target x position.
 * @param y_next Target y position.
 * @return Angle in degrees.
 */
float calculate_angle(float x, float y, float x_next, float y_next) {
	// Calculation of the scalar product
	float dot_product = -y * (y_next - y);

	// Length of vectors
	float length_v1 = fabsf(y); // Since v1 is the length of (0, -y), the length corresponds to |y|
	float length_v2 = (float) sqrt(pow(x_next - x, 2) + pow(y_next - y, 2));

	// Calculating the cosine of the angle
	float cos_theta = dot_product / (length_v1 * length_v2);

	// Clamp cos_theta to [-1, 1] to avoid NaNs due to floating point errors
	if (cos_theta > 1.0f)
		cos_theta = 1.0f;
	if (cos_theta < -1.0f)
		cos_theta = -1.0f;

	// Calculation of the angle in radians and conversion to degrees
	float theta_rad = (float) acos(cos_theta);
	float theta_deg = (float) (theta_rad * (180.0 / M_PI));

	// Determining the direction
	if (x_next < x) {
		theta_deg = -theta_deg; // Negative angle if x_next is to the left of x
	}

	return theta_deg;
}

/**
 * @brief Calculate the distance to the target position.
 * @param current_x Current x position.
 * @param current_y Current y position.
 * @param target_x Target x position.
 * @param target_y Target y position.
 * @return Distance to target.
 */
float distance_to_target(float current_x, float current_y, float target_x,
		float target_y) {
	return sqrtf(
			powf(target_x - current_x, 2.0f) + powf(target_y - current_y, 2.0f));
}

/**
 * @brief Execute received command from ESP8266.
 * @param cmd The command string.
 */
void execute_command(const char *cmd) {
	if (strncmp(cmd, "STOP", 4) == 0) {
		set_servo_pwm(&servo_left, SERVO_STOP);
		set_servo_pwm(&servo_right, SERVO_STOP);
		current_state = IDLE;
	} else if (strncmp(cmd, "START_SPINNING", 14) == 0) {
		set_servo_pwm(&servo_left, FORWARD_MAX);
		set_servo_pwm(&servo_right, FORWARD_MAX);
		path_set = 0; 								// Set the path flag
		current_state = SPINNING;
	} else if (strncmp(cmd, "LOCATION_UPDATE", 15) == 0) {
		sscanf(cmd + 16, "%f;%f;%f", &current_x, &current_y, &current_yaw);
		locationReceived = 1;
		if (path_set && current_state == IDLE) {
			target_x = xCoords[currentTargetIndex];
			target_y = yCoords[currentTargetIndex];
			current_state = ROTATING;
		}
	} else if (strncmp(cmd, "PATH_UPDATE", 11) == 0) {
		memset(xCoords, 0, sizeof(xCoords));
		memset(yCoords, 0, sizeof(yCoords));
		totalCoords = 0;
		currentTargetIndex = 0;

		// Extract x and y coordinates and the amount of coordinates
		char x_values[256];  // Buffer for x values
		char y_values[256];  // Buffer for y values
		int amount_of_coordinates = 0;

		// Extract the x values, y values, and the number of coordinates
		sscanf(cmd + 12, "%[^;];%[^;];%d", x_values, y_values,
				&amount_of_coordinates);

		amount_of_coordinates = amount_of_coordinates / 2;

		// Now, x_values contains "x0:x1:x2:...", y_values contains "y0:y1:y2:...", and amount_of_coordinates is an integer
		// Ensure that the number of coordinates does not exceed MAX_COORDS
		totalCoords =
				(amount_of_coordinates > MAX_COORDS) ?
				MAX_COORDS :
														amount_of_coordinates;

		char *token;
		int index = 0;

		// Split x values by ':' and store them in an array
		token = strtok(x_values, ":");
		while (token != NULL && index < totalCoords) {
			xCoords[index++] = atof(token);
			token = strtok(NULL, ":");
		}
		if (index != totalCoords) {
			return;
		}

		// Split y values by ':' and store them in an array
		index = 0;
		token = strtok(y_values, ":");
		while (token != NULL && index < totalCoords) {
			yCoords[index++] = atof(token);
			token = strtok(NULL, ":");
		}
		if (index != totalCoords) {
			return;
		}

		path_set = 1; 				// Set the target flag
		currentTargetIndex = 0;

		if (totalCoords > 0) {
			target_x = xCoords[currentTargetIndex];
			target_y = yCoords[currentTargetIndex];
		}

		current_state = ROTATING;  	// Start with rotating to face the target
	}
}

/**
 * @brief Set the PWM value for a servo motor.
 * @param servo Pointer to the Servo structure.
 * @param pulse PWM value to set.
 */
void set_servo_pwm(Servo *servo, uint32_t pulse) {
	servo->current_pwm = pulse;
	__HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

/**
 * @brief Send the robot's status to ESP8266.
 */
void send_status_to_esp() {
	// Increase buffer size to accommodate arrays
	char status_message[256];

	// Append basic status information
	int len =
			snprintf(status_message, sizeof(status_message),
					"STATE:%d;CURRENT_TARGET_INDEX:%d;MOTOR_L:%lu;MOTOR_R:%lu;ANGLE_ERROR:%.1f;CALCULATED_PID:%.2f;TARGET_DISTANCE:%.2f\n",
					current_state, currentTargetIndex,
					(unsigned long) servo_left.current_pwm,
					(unsigned long) servo_right.current_pwm, angle_error,
					calculated_correction, target_distance);

	// Ensure null-termination
	status_message[sizeof(status_message) - 1] = '\0';

	// Check for truncation
	if (len >= sizeof(status_message)) {
		// Handle the error: truncate, log, etc.
		char error_message[] = "ERROR: Status message truncated.\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) error_message,
				strlen(error_message), HAL_MAX_DELAY);
	}

	// Transmit the status message over UART
	HAL_UART_Transmit(&huart1, (uint8_t*) status_message,
			strlen(status_message), HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------*/

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
		switch (current_state) {
		case ROTATING:
			if (locationReceived) {
				handle_rotation();
				locationReceived = 0;
			}
			break;
		case MOVING:
			if (locationReceived) {
				handle_movement();
				locationReceived = 0;
			}
			break;
		case SPINNING:
			// Spinning is already handled in execute_command
			break;
		case IDLE:
			break;
		default:
		}

		// Send status to ESP8266 every 1000ms
		uint32_t current_time = HAL_GetTick();
		if (current_time - last_status_send_time >= SERIAL_SEND_INTERVAL) {
			last_status_send_time = current_time;
			send_status_to_esp();
		}

		HAL_Delay(10);

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
