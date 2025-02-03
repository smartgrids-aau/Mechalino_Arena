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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
//#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP 0
#define FORWARD 1
#define BACkWARD 2
#define TURN_RIGHT 3
#define TURN_LEFT 4
#define SPEED 75
char rx_buffer[20];
char USART_recive = 0;
uint8_t UART1_rxBuffer[2] = {0};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
MPU6050_t MPU6050;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t length = 0;
    HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
    if(!(((UART1_rxBuffer[0] == '\n')||(UART1_rxBuffer[0] == '\r'))&&(length == 0)))
    {
    	rx_buffer[length] = UART1_rxBuffer[0];
		length++;
		if((UART1_rxBuffer[0] == '\n')||(UART1_rxBuffer[0] == '\r')||(length == 20))
		{
			if(length > 1)
				USART_recive = 1;
			length = 0;
		}
    }

}
void motor(int16_t MotL, int16_t MotR)
{
	uint32_t cntL,cntR;
	MotL = MotL + 100;
	MotR = MotR + 100;
	cntL = ((MotL*0xC80)/200) + 0xC7F;
	cntR = ((MotR*0xC80)/200) + 0xC7F;
	TIM1->CCR1 = cntL;
    TIM2->CCR3 = cntR;
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
    uint8_t action = 0;
    char ReciveOrder = ' ';
    double angle, distance, derived;
    int16_t Argument = 0;
    int16_t delay;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(100);
  while (MPU6050_Init(&hi2c1) == 1);

  HAL_UART_Receive_IT (&huart1, UART1_rxBuffer, 1);
  HAL_TIM_PWM_Init(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  TIM1->CCMR1 = 0x60;
  TIM1->PSC = 0x19;
  TIM1->ARR = 0xF9FF;
  TIM1->CCR1 = 0x12BF;
  TIM1->CR1 = 0x01;

  HAL_TIM_PWM_Init(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  TIM2->CCMR2 = 0x60;
  TIM2->PSC = 0x19;
  TIM2->ARR = 0xF9FF;
  TIM2->CCR3 = 0x12BF;
  TIM2->CR1 = 0x01;

  HAL_Delay(50);
  char data[200];
  int N = 50;
  double sm = 0.0;
  double mean = 0.0;
  double offset = 0.0;
  int8_t turnvalue = 0;
  int i;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //test
	  while (1)
	    {

		  RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
		  TIM11->CR1 |= TIM_CR1_OPM;
		  TIM11->DIER |= TIM_DIER_UIE;
		  TIM11->PSC = 83;//div by 83+1
		  TIM11->ARR = 10000;//1000*10ms -> 10ms

		  NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
//		  NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1)
		  distance = 0;
		  derived = 0;

		  while(1)
		  {
			  TIM11->CR1 |= TIM_CR1_CEN;
			  HAL_Delay(100);
//			  TIM11->CR1 &= ~TIM_CR1_CEN;
			  TIM11->CNT = 0;
		  }
	    }

	  /////////////////////////// start
	  if(USART_recive == 1)
	  {
		  Argument = 0;
		  ReciveOrder = rx_buffer[0];
		  sscanf(&rx_buffer[1], "%d", &Argument);
//		  Argument -= 10;
		  USART_recive = 0;
		  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	  }
	  ReciveOrder = 'R';
	  switch(ReciveOrder)
	  {
	  	  case '0':
		  {
			  action = STOP;
			  break;
		  }
	  	  case 'F':
	  	  {
	  		  action = FORWARD;
			  break;
	  	  }
	  	  case 'B':
		  {
			  action = BACkWARD;
			  break;
		  }
	  		case 'R':
		  {
			  action = TURN_RIGHT;
			  break;
		  }
	  		case 'L':
		  {
			  action = TURN_LEFT;
			  break;
		  }
	  }
	  switch(action)
	  {
	  	  case STOP:
		  {
			  motor(0,0);
			  break;
		  }
	  	  case FORWARD:
	  	  {
	  		  for (i=0;i<200;i++)
	  		  {
	  			  MPU6050_Read_All(&hi2c1, &MPU6050);
	  			  sm = sm + MPU6050.Gz;
	  			  HAL_Delay(1);
	  		  }
	  		  sm /= 200;
	  		  offset = sm;
	  		  motor(100,-100);
	  		  Argument /= 2;
	  		  delay = 0;
	  		  while(1)
	  		  {

	  			  sm = 0.0;
	  			  for (i=0;i<N;i++)
	  			  {
	  				  MPU6050_Read_All(&hi2c1, &MPU6050);
	  				  sm = sm + MPU6050.Gz;
	  				  HAL_Delay(1);
	  				  delay++;

	  			  }
	  			  sm /= N;
	  			  sm -= offset;
	  			  sm /= 20;// 20*50=1s -> deg/sec
	  			  mean += sm;

	  			  if(mean < 0)//robot turn right must turn left
	  			  {
	  				  motor(100+(int8_t)mean*5,-100);
	  				  turnvalue = 100+(int8_t)mean*5;
	  			  }
	  			  if(mean > 0)//turn left
	  			  {
	  				  motor(100,-100+(int8_t)mean*5);
	  				  turnvalue = -100+(int8_t)mean*5;
	  			  }
	  			  if(USART_recive == 1)
	  				  break;
	  			if(delay >= Argument)
					  break;
	  		  }

			  ReciveOrder = '0';
	  		  break;
	  	  }
	  	  case BACkWARD:
		  {
	  		  for (i=0;i<200;i++)
	  		  {
	  			  MPU6050_Read_All(&hi2c1, &MPU6050);
	  			  sm = sm + MPU6050.Gz;
	  			  HAL_Delay(1);
	  		  }
	  		  sm /= 200;
	  		  offset = sm;
	  		 motor(-100,100);
	  		Argument /= 2;
	  		delay = 0;
	  		  while(1)
	  		  {

	  			  sm = 0.0;
	  			  for (int i=0;i<N;i++)
	  			  {
	  				  MPU6050_Read_All(&hi2c1, &MPU6050);
	  				  sm = sm + MPU6050.Gz;
	  				  HAL_Delay(1);
	  				  delay++;
	  			  }
	  			  sm /= N;
	  			  sm -= offset;
	  			  sm /= 20;// 20*50=1s -> deg/sec
	  			  mean += sm;

	  			  if(mean < 0)//robot turn right must turn left
	  			  {
	  				  motor(-100,100+(int8_t)mean*5);
	  			  }
	  			  if(mean > 0)//turn left
	  			  {
	  				  motor(-100+(int8_t)mean*5,100);
	  			  }
	  			  if(USART_recive == 1)
	  				  break;
	  			  if(delay >= Argument)
					  break;
	  		  }

			  ReciveOrder = '0';
			  break;
		  }
	  	  case TURN_RIGHT:
		  {
			  angle = 0;
			  while(angle >= -Argument)
			  {

				  MPU6050_Read_All(&hi2c1, &MPU6050);
				  angle += (MPU6050.Gz/100);
				  motor(100,100);
				  HAL_Delay(8);
				  if(USART_recive == 1)
					  break;
			  }
			  ReciveOrder = '0';
			  break;
		  }
	  	  case TURN_LEFT:
		  {
			  angle = 0;
			  while(angle <= Argument)
			  {

				  MPU6050_Read_All(&hi2c1, &MPU6050);
				  angle += (MPU6050.Gz/100);
				  motor(-100,-100);
				  HAL_Delay(8);
				  if(USART_recive == 1)
					  break;
			  }
			  ReciveOrder = '0';
			  break;
		  }
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
