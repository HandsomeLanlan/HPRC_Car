/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "HAL_OLED.h"
#include "HRSR04.h"
#include "control.h"
#include "badc.h"
#include "encoder.h"
#include "control.h"
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

/* USER CODE BEGIN PV */
char Serial_RxPacket[100];
uint8_t Serial_RxFlag;
uint8_t RxData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Get_ID(void);
float Data_Cauculate(char *arr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  Control_Init();
  
  /* 启动超声波模块的定时器 */
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);

  /* 启动编码器模块的定时器 */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

  HAL_UART_Receive_IT(&huart3, &RxData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  // OLED_ShowString(1,1,"L:");
  // OLED_ShowString(1,9,"R:");
  // OLED_ShowString(2,1,"Distance:");
  // OLED_ShowString(3,1,"Tracks:");
  // SetSpeed_Left(0);
  // SetSpeed_Right();
  
  // int left_speed  = 0;
  // int right_speed = 0;
  // uint32_t distance;
  
  while (1)
  {
    //left_encoder_speed = get_left_encoder_speed();
    //right_encoder_speed = get_right_encoder_speed();
    // left_speed  = get_left_encoder_speed();
    // right_speed = get_right_encoder_speed();
    // OLED_ShowSignedNum(1, 8, left_speed, 4);
    // OLED_ShowSignedNum(2, 8, right_speed, 4);

    // distance = Ultrasonic_GetDistance();
    // OLED_ShowNum(3, 10, distance, 3);

    //vofa串口显示部分
    /*****************/
    if (Serial_RxFlag == 1)
		{		
			float result = Data_Cauculate(Serial_RxPacket);
			uint8_t command = Get_ID();
			switch(command)
			{
				case 7:target_speed = result;break;
			}
			Serial_RxFlag = 0;
		}
    
    //printf("%d,%d,%d\n",target_speed,left_encoder_speed,-right_encoder_speed);
    
    /*****************/
    // OLED_ShowSignedNum(1, 3, left_encoder_speed, 4);
    // OLED_ShowSignedNum(1, 11, right_encoder_speed, 4);

    run();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint8_t ID_Num = 0;
uint8_t Bit_cnt;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static uint8_t RxState = 0;
	  static uint8_t pRxPacket = 0;
		if (RxState == 0)
		{
			if (RxData == '#' && Serial_RxFlag == 0)
			{
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if(RxState == 1)
		{
			if(RxData == 'P')
			{
				RxState = 2;
			}
		}
		else if(RxState == 2)
		{
			if(RxData == '=')
			{
				RxState = 3;
			}
			else
			{
				ID_Num = RxData - '0';
			}
		}
		else if (RxState == 3)
		{
			if (RxData == '!')
			{
				
				RxState = 0;
				Serial_RxPacket[pRxPacket] = '\0';
				Bit_cnt = pRxPacket;
				Serial_RxFlag = 1;
			}
			else
			{
				Serial_RxPacket[pRxPacket] = RxData;
				pRxPacket++;
			}
		}
    HAL_UART_Receive_IT(&huart3, &RxData, 1);
}

uint8_t Get_ID(void)
{
	return ID_Num;
}

float Pow_Invert(uint8_t X, uint8_t n)
{
	float result = X;
	while(n--)
	{
		result /= 10;
	}
	return result;
}

//已测试cauculate函数计算没问题
float Data_Cauculate(char *arr)
{
	
	float Data = 0;
	uint8_t Dot_Flag = 0;
	float Dot_afternum = 1;
	
	for(uint8_t i = 0; i < Bit_cnt; i++)
	{
		if(Dot_Flag == 0)
		{
			if(arr[i] == '.')//'.'
			{
				Dot_Flag = 1;
			}
			else
			{
				Data = Data * 10 + arr[i] - '0';
			}
		}
		else
		{
			Data = Data + Pow_Invert(arr[i] - '0', Dot_afternum);
			Dot_afternum++;
		}
	}
	return Data;
}

int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
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
#ifdef USE_FULL_ASSERT
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
