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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CALIB 50
#define LOOP_DELAY 10
#define ACCELERATION 10 // must be a divisor of 100! No check is being done!
#define MOTOR_MAX_1 800 //800
#define MOTOR_SLOW_1 2800
#define MOTOR_STOP 3000
#define MOTOR_MAX_2 5100 //5100
#define MOTOR_SLOW_2 3100
#define COMMAND_NC 'N' // no command
#define COMMAND_MOVE 'M' // set movement speed (& direction)
#define COMMAND_ROTATE 'R' // set rotation speed (& direction)
#define COMMAND_HALT 'H' // stops robot (both movement and rotation)
#define COMMAND_LOCATION 'L' //give robot its own location
#define COMMAND_GOAL 'G' //give robot a goal to go (no planning needed)

#define NUM_SENSORS 8 // 8 IR LED and receiver pairs

/* Define GPIO pins for multiplexer control */
#define MUX_A_GPIO GPIOB
#define MUX_A_PIN GPIO_PIN_3
#define MUX_B_GPIO GPIOB
#define MUX_B_PIN GPIO_PIN_4
#define MUX_C_GPIO GPIOB
#define MUX_C_PIN GPIO_PIN_5
#define MUX_RC_EN_GPIO GPIOB
#define MUX_RC_EN_PIN GPIO_PIN_0
#define MUX_EN_GPIO GPIOB
#define MUX_EN_PIN GPIO_PIN_1
#define IR_PHOTO_READ_GPIO GPIOA
#define IR_PHOTO_READ_PIN GPIO_PIN_0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t IR_sensor_readings[NUM_SENSORS];
MPU6050_t MPU6050;
double Gz_mean = 0;
float kp = 50.0f;
int correction_speed = 30;
int correction_time = 160;

int Rdelay = 10;
float r_cw = -69;
float r_ccw = 76.5;

float current_Gz;
float total_Gz = 0;
float control_signal;
float total_x = 0;

uint16_t mR, mL;
int8_t speed = 0;
int16_t movementTime = 0;
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
void select_mux_channel(uint8_t channel);
uint8_t read_IR_sensors(uint8_t channel);
void enable_multiplexers(void);
void disable_multiplexers(void);
void motor(int16_t MotL, int16_t MotR);
void move(); // gradually sets current speed to the desired one
void apply_speed(); // applies the desired speed from -100 to 100 to robots.
void rotate(float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void select_mux_channel(uint8_t channel) {
    // Set A, B, C inputs of the multiplexers
    HAL_GPIO_WritePin(MUX_A_GPIO, MUX_A_PIN, (channel & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);  // A
    HAL_GPIO_WritePin(MUX_B_GPIO, MUX_B_PIN, (channel & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);  // B
    HAL_GPIO_WritePin(MUX_C_GPIO, MUX_C_PIN, (channel & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);  // C
}

void enable_multiplexers(void) {
    HAL_GPIO_WritePin(MUX_RC_EN_GPIO, MUX_RC_EN_PIN, GPIO_PIN_RESET); // Enable CD74HC237E (assuming active high)
    HAL_GPIO_WritePin(MUX_EN_GPIO, MUX_EN_PIN, GPIO_PIN_RESET); // Enable SN74HC138N (assuming active high)
}

void disable_multiplexers(void) {
    HAL_GPIO_WritePin(MUX_RC_EN_GPIO, MUX_RC_EN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MUX_EN_GPIO, MUX_EN_PIN, GPIO_PIN_SET);
}

uint8_t read_IR_sensors(uint8_t channel) {
	select_mux_channel(channel); // Select each IR sender and receiver
	HAL_Delay(100); // Allow time for the IR signal to be sent and received
    enable_multiplexers();  // Ensure the multiplexers are enabled before testing
	HAL_Delay(100); // Allow time for the IR signal to be sent and received
	uint8_t received = HAL_GPIO_ReadPin(IR_PHOTO_READ_GPIO, IR_PHOTO_READ_PIN);
	HAL_Delay(100); // Wait some time before testing next IR pair
    disable_multiplexers();  // Disable multiplexers after testing to save power and avoid interference
	HAL_Delay(100); // Allow time for the IR signal to be sent and received
	return received;
}

void motor(int16_t MotL, int16_t MotR)
{
	uint32_t cntL,cntR;
	MotL = MotL + 100;//put an offset (0 to 200)
	MotR = MotR + 100;//put an offset (0 to 200)
	cntL = ((MotL*2000)/200) + 2000;//value of PWM start at 2000 and finish at 4000
	cntR = ((MotR*2000)/200) + 2000;//value of PWM start at 2000 and finish at 4000
	TIM2->CCR3 = cntL;//put value on TIMERs registers
	TIM1->CCR1 = cntR;//put value on TIMERs registers
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
	else
	{
		MPU6050_Read_All(&hi2c1, &MPU6050);
		current_Gz = MPU6050.Gz - Gz_mean;
		total_Gz += current_Gz * (LOOP_DELAY / 1000.0f);
		float control_signal = kp * total_Gz;
		if (current_speed>0) // forward
		{
			if (control_signal < 0) // turned right, must turn left
			{
				motor(current_speed+10+(int8_t)control_signal, -current_speed);
			}
			else if (control_signal > 0)
			{
				motor(current_speed+10, -current_speed+(int8_t)control_signal);
			}
		}
		else
		{
			if (control_signal < 0) // turned right, must turn left
			{
				motor(-current_speed-10, current_speed+(int8_t)control_signal);
			}
			else if (control_signal > 0)
			{
				motor(-current_speed-10+(int8_t)control_signal, current_speed);
			}
		}
	}
}

void move()
{
	total_Gz = 0;

	current_speed = 0;
	motor(0,0);

	int breakTime = (int)(fabs(speed / ACCELERATION))*LOOP_DELAY;
	movementTime = movementTime - breakTime;

	while(movementTime>0)
	{
		if (current_speed < speed)
		{
			current_speed += ACCELERATION;
		}
		else if (current_speed > speed)
		{
			current_speed -= ACCELERATION;
		}
		apply_speed();
		HAL_Delay(LOOP_DELAY);
		movementTime -= LOOP_DELAY;
	}
	while(current_speed>0)
	{
		current_speed -= ACCELERATION;
		apply_speed();
		HAL_Delay(LOOP_DELAY);
	}
	if (total_Gz < 0) // turned right, must turn left
	{
		motor(-correction_speed, -correction_speed);
	}
	else if (total_Gz > 0)
	{
		motor(correction_speed,correction_speed);
	}
	HAL_Delay(correction_time);
	motor(0,0);

}

void rotate(float angle)
{

	for(int interations = 0; interations < CALIB; interations++)
	{
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  Gz_mean += MPU6050.Gz;
	  HAL_Delay(50);
	}
	Gz_mean /= CALIB;

	while(abs(angle)>0.1)
	{
		total_Gz = 0;
		if (angle > 0)
		{
			mR = MOTOR_SLOW_1-150;
			mL = MOTOR_SLOW_1+50;
		}
		else
		{
			mR = MOTOR_SLOW_2+150;
			mL = MOTOR_SLOW_2-50;
		}
		while (fabs(total_Gz) - fabs(angle) < 0)
		{
			TIM1->CCR1 = mR;
			TIM2->CCR3 = mL;
			if (angle>0)
			{
				mR-=5;
				mL-=5;
			}
			else
			{
				mR+=5;
				mL+=5;
			}
			HAL_Delay(Rdelay);
			MPU6050_Read_All(&hi2c1, &MPU6050);
			current_Gz = (MPU6050.Gz - Gz_mean);
			total_Gz += current_Gz * (Rdelay / 1000.0f);
		}
		TIM1->CCR1 = 0;
		TIM2->CCR3 = 0;
		angle = angle - total_Gz;
	}
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
  HAL_Delay(5000);

  while (MPU6050_Init(&hi2c1) == 1); //Initialise the MPU6050
  // calibrate MPU6050
  for(int interations = 0; interations < CALIB; interations++)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  Gz_mean += MPU6050.Gz;
	  HAL_Delay(50);
  }
  Gz_mean /= CALIB;

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

  disable_multiplexers();
  while (1)
  {
	  for (uint8_t i = 0; i < NUM_SENSORS; i++){
		  IR_sensor_readings[i] = read_IR_sensors(i);
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
