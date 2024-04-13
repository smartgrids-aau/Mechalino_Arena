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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CALIB 100
#define ACCELERATION 5 // must be a divisor of 100! No check is being done!
#define MOTOR_MAX_1 800
#define MOTOR_SLOW_1 2800
#define MOTOR_STOP 3000
#define MOTOR_MAX_2 5100
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
float Gz_mean = 0;

int8_t speed = 0;
int8_t current_speed = 0;

char rx_buffer[20];
uint8_t rx_buffer_index = 0;
char USART_recive = 0;
uint8_t UART1_rxBuffer[1] = { 0 };
char command = COMMAND_NC;
int8_t Arg1, Arg2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void speed_ctl(); // gradually sets current speed to the desired one
void apply_speed(); // applies the desired speed from -100 to 100 to robots.
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
	// core speed is a number in the range MOTOR_SLOW (>0) - MOTOR_MAX (<=100)
	int core_speed =  (MOTOR_SLOW_1 - MOTOR_MAX_1) / 100 * abs(current_speed);
	if (current_speed > 0) // forward
	{
		TIM1->CCR1 = MOTOR_SLOW_1 - core_speed;
		TIM2->CCR3 = MOTOR_SLOW_2 + core_speed;
	}
	else // backward
	{
		TIM1->CCR1 = MOTOR_SLOW_2 + core_speed;
		TIM2->CCR3 = MOTOR_SLOW_1 - core_speed;
	}
}
void speed_ctl()
{
	if (current_speed != speed)
	{
		if (current_speed < speed)
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
		}
	}
	apply_speed();
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

  HAL_Delay(1000);
  while (MPU6050_Init(&hi2c1) == 1); //Initialise the MPU6050
  // calibrate MPU6050
  for(uint8_t interations = 0; interations < CALIB; interations++)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  Gz_mean += MPU6050.Gz;
  }
  Gz_mean /= CALIB;

  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  TIM1->CR1 = 0x01;
  TIM2->CR1 = 0x01;
  HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, sizeof(UART1_rxBuffer));

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
			  if (command == COMMAND_LOCATION || command == COMMAND_GOAL)
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
		  USART_recive = 0;
		  HAL_UART_Transmit(&huart1, "OK!", sizeof("OK!"), 100);
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
		  command = COMMAND_NC; // command done
	  default:
		  // TODO: implementation of other commands
		  break;
	  }

	  //robot speed
	  speed_ctl();
	  // TODO: robot rotation

	  MPU6050_Read_All(&hi2c1, &MPU6050);
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

  if ((UART1_rxBuffer[0] == '\r') || rx_buffer_index > 10) // end of data
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
