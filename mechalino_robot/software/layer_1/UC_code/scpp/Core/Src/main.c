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
typedef struct {
	float kp;          // Proportional gain
	float ki;          // Integral gain
	float kd;          // Derivative gain
    float setpoint;    // Target value
    float integral;    // Integral sum
    float prev_error;  // Previous error
} PIDController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CALIB 1000
#define LOOP_DELAY 10
#define ACCELERATION 5 // must be a divisor of 100! No check is being done!
#define MOTOR_MAX_1 1400 //800
#define MOTOR_SLOW_1 2800
#define MOTOR_STOP 3000
#define MOTOR_MAX_2 4500 //5100
#define MOTOR_SLOW_2 3100
#define COMMAND_NC 'N' // no command
#define COMMAND_MOVE 'M' // set movement speed (& direction)
#define COMMAND_ROTATE 'R' // set rotation speed (& direction)
#define COMMAND_HALT 'H' // stops robot (both movement and rotation)
#define COMMAND_LOCATION 'L' //give robot its own location
#define COMMAND_GOAL 'G' //give robot a goal to go (no planning needed)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MPU6050_t MPU6050;
double Gz_mean = 0;
double Ax_mean = 0;
double Ay_mean = 0;
PIDController pid_m;
PIDController pid_r;
float kp = 1700.0f;
float ki = 0.0f;
float kd = 0.0f;
float kp2 = 1.0f;
float ki2 = 0.1f;
float kd2 = 0.0f;
float GzMul = 0.0f;
int Gzs[1000];
int Gzs_index = 0;
int Rdelay = 5;
float R_offset_error = 0.0f;
float current_Gz;
float total_Gz = 0;
float control_signal;
float total_x = 0;

int8_t speed = 0;
int8_t current_speed = 0;

char rx_buffer[30];
uint8_t rx_buffer_index = 0;
char USART_recive = 0;
uint8_t UART1_rxBuffer[1] = { 0 };
char command = COMMAND_NC;
int Arg1, Arg2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void speed_ctl(); // gradually sets current speed to the desired one
void apply_speed(); // applies the desired speed from -100 to 100 to robots.
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PID_init(PIDController *pid, float kp, float ki, float kd, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->prev_error = 0;
    total_Gz = 0;
}

float PID_compute(PIDController *pid, float input) {
	float error = pid->setpoint - input;
    pid->integral += error;
    float derivative = error - pid->prev_error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    pid->prev_error = error;
    return output;
}

void apply_speed()
{
	// if current_speed = 0, stop!
	if (current_speed == 0)
	{
		TIM1->CCR1 = MOTOR_STOP;
		TIM2->CCR3 = MOTOR_STOP;
		return;
	}
	// else
	// PID control
	current_Gz = MPU6050.Gz - Gz_mean;
	total_Gz += current_Gz * (LOOP_DELAY / 1000.0f);
	control_signal = PID_compute(&pid_m, total_Gz);

	// core speed is a number in the range MOTOR_SLOW (>0) - MOTOR_MAX (<=100)
	int core_speed =  (MOTOR_SLOW_1 - MOTOR_MAX_1) / 100.0f * abs(current_speed);
	if (current_speed > 0) // forward
	{
		int speedR = MOTOR_MAX_1 - (int)control_signal;
		if (speedR > MOTOR_SLOW_2)
		{
			speedR = MOTOR_SLOW_2;
		}
		else if (speedR < MOTOR_MAX_1)
		{
			speedR = MOTOR_MAX_1;
		}
		int speedL = MOTOR_MAX_2 - (int)control_signal;
		if (speedL < MOTOR_SLOW_1)
		{
			speedL = MOTOR_SLOW_1;
		}
		else if (speedL > MOTOR_MAX_2)
		{
			speedL = MOTOR_MAX_2;
		}
		TIM1->CCR1 = speedR;
		TIM2->CCR3 = speedL;
	}
}

void speed_ctl()
{
	// if current_speed = 0, stop!
	if (current_speed == 0)
	{
		// renew PID controller, no I or D term should come from the previous movement
		// params = Kp, Ki, Kd, setpoint = 0
		PID_init(&pid_m, kp, ki, kd, 0);
	}
	if (current_speed != speed)
	{
		current_speed = speed;
		/*if (current_speed < speed)
		{
			current_speed += ACCELERATION;
			if (current_speed > speed) // if it passed the desired speed
				current_speed = speed;
		}
		else if (current_speed > speed)
		{
			current_speed -= ACCELERATION;
			if (current_speed < speed)  // if it passed the desired speed
				current_speed = speed;
		}*/
	}
	apply_speed();
}

void rotate(float angle)
{
	if (angle > 0)
		angle -= R_offset_error;
	else
		angle += R_offset_error;
	// freeze the robot slowly
	speed = 0;
	while(current_speed != 0)
		speed_ctl();

	total_Gz = 0;
	PID_init(&pid_r, kp2, ki2, kd2, angle);

	if (angle > 0) // cw
	{
		TIM1->CCR1 = MOTOR_MAX_1;
		TIM2->CCR3 = MOTOR_MAX_1;
	}
	else // ccw
	{
		TIM1->CCR1 = MOTOR_MAX_2;
		TIM2->CCR3 = MOTOR_MAX_2;
	}
	int fixStop = 150;
	HAL_Delay(fixStop);
	int uuu = 0;
	while (fabs(total_Gz) < fabs(angle))
	{
		HAL_Delay(Rdelay);
		MPU6050_Read_All(&hi2c1, &MPU6050);
		current_Gz = (MPU6050.Gz - Gz_mean) * (1+GzMul);
		total_Gz += fabs(current_Gz * (Rdelay / 1000.0f));
	}

	TIM1->CCR1 = 0;
	TIM2->CCR3 = 0;
	TIM2->CCR3 = 0;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_Delay(10000);

  while (MPU6050_Init(&hi2c1) == 1); //Initialise the MPU6050
  // calibrate MPU6050
  for(int interations = 0; interations < CALIB; interations++)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  Gz_mean += MPU6050.Gz;
	  Ax_mean += MPU6050.Ax;
	  Ay_mean += MPU6050.Ay;
	  HAL_Delay(10);
  }
  Gz_mean /= CALIB;
  Ax_mean /= CALIB;
  Ay_mean /= CALIB;

  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  TIM1->CR1 = 0x01;
  TIM2->CR1 = 0x01;
  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, sizeof(UART1_rxBuffer));

  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// check if there is any new command
	if (USART_recive == 1)
	{
	  char *delimiter = " ";
	  char *saveptr;
	  char *token;
	  token = strtok_r(rx_buffer, delimiter, &saveptr);
	  if (token != NULL) {
		  command = token[0];
	  }
	  else
	  {
		  command = COMMAND_NC; // failed to read the command, so drop it
	  }
	  // commands with 1 or 2 arguments
	  if (command == COMMAND_MOVE || command == COMMAND_ROTATE || command == COMMAND_LOCATION || command == COMMAND_GOAL)
	  {
		  token = strtok_r(NULL, delimiter, &saveptr);
		  if (token != NULL) {
			  Arg1 = atoi(token);
		  }
		  else
		  {
			  command = COMMAND_NC; // failed to read arguments, so drop the command
		  }
		  // commands with a second argument
		  if (command == COMMAND_MOVE || command == COMMAND_LOCATION || command == COMMAND_GOAL)
		  {
			  token = strtok_r(NULL, delimiter, &saveptr);
			  if (token != NULL) {
				  Arg2 = atoi(token);
			  }
			  else
			  {
				  command = COMMAND_NC; // failed to read arguments, so drop the command
			  }
		  }
	  }
	  // TODO: remove later / for debugging
	  if (command == 'P')
	  {
		  token = strtok_r(NULL, delimiter, &saveptr);
		  kp = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  ki = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  kd = atof(token);
		  PID_init(&pid_m, kp, ki, kd, 0);
		  command = COMMAND_NC;
	  }
	  if (command == 'X')
	  {
		  token = strtok_r(NULL, delimiter, &saveptr);
		  kp2 = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  ki2 = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  kd2 = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  GzMul = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  R_offset_error = atof(token);
		  token = strtok_r(NULL, delimiter, &saveptr);
		  Rdelay = atoi(token);
		  command = COMMAND_NC;
	  }
	  USART_recive = 0;
	  HAL_UART_Transmit(&huart1, (uint8_t *)("OK!"), sizeof("OK!"), 100);
	}
	// handle current command
	switch(command)
	{
	case COMMAND_NC:
		// nothing to do
		break;
	case COMMAND_HALT:
		// stop both movement and rotation
		speed = 0;
		command = COMMAND_NC; // command done
		break;
	case COMMAND_MOVE:
		// set speed
		speed = Arg1;
		if (Arg2 == 0)
			command = COMMAND_NC; // command done
		break;
	case COMMAND_ROTATE:
		rotate(Arg1);
		command = COMMAND_NC; // command done
		break;
	default:
		// TODO: implementation of other commands
		break;
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//robot speed
	MPU6050_Read_All(&hi2c1, &MPU6050);
	speed_ctl();
	HAL_Delay(LOOP_DELAY);
	if (command == COMMAND_MOVE && Arg2 > 0)
	{
		Arg2 -= LOOP_DELAY;
		if (Arg2 <= 0)
		{
			speed = 0;
			command = COMMAND_NC;
		}
	}
	// TODO: robot rotation
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
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 128;
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

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);


  rx_buffer[rx_buffer_index++] = UART1_rxBuffer[0];

  if ((UART1_rxBuffer[0] == '\r') || rx_buffer_index > 30) // end of data
  {
	  rx_buffer_index = 0;
	  USART_recive = 1;
  }
  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, sizeof(UART1_rxBuffer));
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
