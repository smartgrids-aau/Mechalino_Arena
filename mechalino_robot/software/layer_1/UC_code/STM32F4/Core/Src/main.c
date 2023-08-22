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

#define COEFF_CORRE 71
#define CALIB_NUNBER 100
#define ACCELERATION 1
char rx_buffer[20];
char USART_recive = 0;
uint8_t UART1_rxBuffer[2] = { 0 };
uint8_t interrupt10ms = 0;
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //go inside each time caractere is send on serial
{
	static uint8_t length = 0;
	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);
	if (!(((UART1_rxBuffer[0] == '\n') || (UART1_rxBuffer[0] == '\r'))
			&& (length == 0))) //if recive real data ( not just "\r\n")
	{
		rx_buffer[length] = UART1_rxBuffer[0]; //copy the recived caractere on buffer
		length++;
		if ((UART1_rxBuffer[0] == '\n') || (UART1_rxBuffer[0] == '\r')
				|| (length == 20)) //if end of sending data, quit
				{
			if (length > 1)
				USART_recive = 1; //message is complete recived
			length = 0;
		}
	}
}
void motor(int16_t MotL, int16_t MotR) //PWM mcontrole (1 ms -> backward / 1.5 ms -> no moove / 2 ms -> forward)
{
	uint32_t cntL, cntR;
	MotL = MotL + 100; //put an offset (0 to 200)
	MotR = MotR + 100; //put an offset (0 to 200)
	cntL = ((MotL * 2000) / 200) + 2000; //value of PWM start at 2000 and finish at 4000
	cntR = ((MotR * 2000) / 200) + 2000; //value of PWM start at 2000 and finish at 4000
	TIM1->CCR1 = cntL; //put value on TIMERs registers
	TIM2->CCR3 = cntR; //put value on TIMERs registers
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

	while (MPU6050_Init(&hi2c1) == 1)
		; //Init Accelerometer

	//Gpio (IR)
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODER0_Pos); //B0 in output
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODER1_Pos); //B1 in output
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODER3_Pos); //B3 in output
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODER4_Pos); //B4 in output
	GPIOB->MODER |= (0x02 << GPIO_MODER_MODER5_Pos); //B5 in output

	GPIOB->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED0_Pos); //B0 in  Very high speed
	GPIOB->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED1_Pos); //B1 in  Very high speed
	GPIOB->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED3_Pos); //B3 in  Very high speed
	GPIOB->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED4_Pos); //B4 in  Very high speed
	GPIOB->OSPEEDR |= (0x03 << GPIO_OSPEEDR_OSPEED5_Pos); //B5 in  Very high speed

	GPIOB->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD3 | GPIO_ODR_OD4
			| GPIO_ODR_OD5); //Put the pins B0,1,3,4,5 to 0

	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 1);

	//init timer 1 (PWM) Left servo
	HAL_TIM_PWM_Init(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	TIM1->CCMR1 = 0x60; //Configure in PWM mode
	TIM1->PSC = 41; //(41+1)divide 84MHz/42 = 2MHz
	TIM1->ARR = 40000; //PWM duty cycle -> 2MHz/40 000 = 50Hz
	TIM1->CCR1 = 3000; //put PWM at 1.5ms (1/2 000 000)*3000 = 1.5ms state do nothing
	TIM1->CR1 = 0x01; // Turn on PWM

	//init timer 2 (PWM) Right servo
	HAL_TIM_PWM_Init(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	TIM2->CCMR2 = 0x60; //Configure in PWM mode
	TIM2->PSC = 41; //(41+1)divide 84MHz/42 = 2MHz
	TIM2->ARR = 40000; //PWM duty cycle -> 2MHz/40 000 = 50Hz
	TIM2->CCR3 = 3000; //put PWM at 1.5ms (1/2 000 000)*3000 = 1.5ms state do nothing
	TIM2->CR1 = 0x01; // Turn on PWM

	//init timer 11 (10ms) sampling accelerometer
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN; //Enable timer 11 clock
	TIM11->CR1 |= TIM_CR1_OPM; //set timer on counter mode
	TIM11->DIER |= TIM_DIER_UIE; //Enable interrupt of timer 11
	TIM11->PSC = 83; //div by 83+1
	TIM11->ARR = 10000; //1000*10ms -> 10ms

	NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn); //enable unterrupt

	//init timer 10 (10ms) sampling accelerometer             /////////////////To be completed
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN; //Enable timer 10 clock
	TIM10->CR1 |= TIM_CR1_OPM; //set timer on counter mode
	TIM10->DIER |= TIM_DIER_UIE; //Enable interrupt of timer 10
	TIM10->PSC = 6; //div by 6+1
	TIM10->ARR = 150; // 12.5us -> 80 KHz
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //enable unterrupt     ////////////////////
	//TIM10->CR1 |= TIM_CR1_CEN;//Start sampling ADC

	//ADC Configurqtion
	//ADC1_0 pin A0
	GPIOA->MODER |= (0x03 << GPIO_MODER_MODE0_Pos);
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;	//Enqble ADC clock source
	ADC1->CR1 |= ADC_CR1_EOCIE;	//Enable End Of Convertion interrupt
	ADC1->CR1 |= ADC_CR1_DISCEN;	//Enable discontinuous mode
	ADC1->CR2 |= ADC_CR2_ADON;	//Enable ADC
	ADC1->SMPR2 = 0x02;
	//ADC1->CR2 |= ADC_CR2_SWSTART;//stqrt convertion
	//while((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC);//wait the end of convertion

	HAL_Delay(50);	//wait ESP8266 starting
	char data[200];
	int N = 50;
	double sm = 0.0;
	double mean = 0.0;
	double current_face = 0.0;
	double offset = 0.0;
	int8_t turnvalue = 0, calib, accel, decel;
	int i;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/////////////////////////// start
		if (USART_recive == 1)	//if complete message is recived
		{
			Argument = 0;
			ReciveOrder = rx_buffer[0];	//take the first argument
			sscanf(&rx_buffer[1], "%d", &Argument);	//put the caractere chaine in number
//		  Argument -= 10;
			USART_recive = 0;
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);	//toggle debug pin
		}

		// current face of robot is captured
		TIM11->CR1 |= TIM_CR1_CEN;
		calib = 0;
		current_face = 0;
		while (calib < CALIB_NUNBER) {
			if (interrupt10ms == 1) {
				interrupt10ms = 0;
				MPU6050_Read_All(&hi2c1, &MPU6050);
				current_face += (MPU6050.Gz / 100);
				calib++;
			}
		}
		current_face = current_face / CALIB_NUNBER;
		TIM11->CR1 &= ~TIM_CR1_CEN;				//Disable Counter
		TIM11->CNT = 0;

		switch (ReciveOrder) {
			case '0': {
				action = STOP;
				break;
			}
			case 'F': {
				action = FORWARD;
				break;
			}
			case 'B': {
				action = BACkWARD;
				break;
			}
			case 'R': {
				action = TURN_RIGHT;
				break;
			}
			case 'L': {
				action = TURN_LEFT;
				break;
			}
		}
		switch (action) {
			case STOP: {
				motor(0, 0);	//motor do nothing
				break;
			}
			case FORWARD: {
				angle = 0;
				TIM11->CR1 |= TIM_CR1_CEN;

				//calibration has been done in advance

				accel = 0;
				decel = 0;
				delay = 0;

				accel = 100;
				while (accel!=0)
				{
					if (interrupt10ms == 1)
					{
						if (decel)//SLOW-DOWN
						{
							accel -= ACCELERATION;
							if (accel < 10)
							{
								accel = 0;
							}
						}

						interrupt10ms = 0;
						MPU6050_Read_All(&hi2c1, &MPU6050);		//Read Accelerometer
						angle += ((MPU6050.Gz / 100) - current_face) * COEFF_CORRE;
						if (angle < 0)			//robot turn right must turn left
								{
							motor(accel + (int8_t) angle, -accel);
						}
						if (angle > 0)							//turn left
								{
							motor(accel, -accel + (int8_t) angle);
						}

						if (accel == 100)
							delay++;
						if(delay * 10 >= Argument)
							decel = 1;
					}

					if (USART_recive == 1)
						decel = 1;
				}
				interrupt10ms = 0;
				TIM11->CR1 &= ~TIM_CR1_CEN;				//Disable Counter
				TIM11->CNT = 0;							//reset Counter
				ReciveOrder = '0';
				break;
			}
			case BACkWARD: {
				angle = 0;
				TIM11->CR1 |= TIM_CR1_CEN;

				//calibration has been done in advance

				accel = 0;
				decel = 0;
				delay = 0;

				accel = 100;
				while (accel!=0)
				{
					if (interrupt10ms == 1)
					{
						if (decel)//SLOW-DOWN
						{
							accel -= ACCELERATION;
							if (accel < 10)
							{
								accel = 0;
							}
						}

						interrupt10ms = 0;
						MPU6050_Read_All(&hi2c1, &MPU6050);		//Read Accelerometer
						angle += ((MPU6050.Gz / 100) - current_face) * COEFF_CORRE;
						if (angle < 0)			//robot turn right must turn left
								{
							motor(-accel, accel  + (int8_t) angle);
						}
						if (angle > 0)							//turn left
								{
							motor(-accel  + (int8_t) angle, accel);
						}

						if (accel == 100)
							delay++;
						if(delay * 10 >= Argument)
							decel = 1;
					}

					if (USART_recive == 1)
						decel = 1;
				}
				interrupt10ms = 0;
				TIM11->CR1 &= ~TIM_CR1_CEN;				//Disable Counter
				TIM11->CNT = 0;							//reset Counter
				ReciveOrder = '0';
				break;
			}
			case TURN_RIGHT: {
				TIM11->CR1 |= TIM_CR1_CEN;
				angle = -1; //-3 is a constant error (why?)
				accel = 100;
				motor(accel, accel);
				while(angle < Argument)
				{
					if (interrupt10ms == 1)
					{
						if ((Argument - angle) < 15)
						{
							accel = 50;
							motor(accel, accel);
						}
						MPU6050_Read_All(&hi2c1, &MPU6050);		//Read Accelerometer
						angle -= (MPU6050.Gz / 100);
						interrupt10ms = 0;
					}
				}
				accel = 0;
				motor(accel, accel);
				interrupt10ms = 0;
				TIM11->CR1 &= ~TIM_CR1_CEN;				//Disable Counter
				TIM11->CNT = 0;							//reset Counter
				ReciveOrder = '0';
				break;
			}
			case TURN_LEFT: {						//reset Counter
				TIM11->CR1 |= TIM_CR1_CEN;
				angle = 6; //6 is a constant error (why?)
				accel = 100;
				motor(-accel, -accel);
				while(angle < Argument)
				{
					if (interrupt10ms == 1)
					{
						if ((Argument - angle) < 15)
						{
							accel = 50;
							motor(-accel, -accel);
						}
						MPU6050_Read_All(&hi2c1, &MPU6050);		//Read Accelerometer
						angle += (MPU6050.Gz / 100);
						interrupt10ms = 0;
					}
				}
				accel = 0;
				motor(-accel, -accel);
				interrupt10ms = 0;
				TIM11->CR1 &= ~TIM_CR1_CEN;				//Disable Counter
				TIM11->CNT = 0;							//reset Counter
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
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 168;
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
	__disable_irq();							//disable all interrupts
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
