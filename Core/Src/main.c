/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * Functions:
  *
  * Program is generating a PWM signal at pin PA5 (A4) with 1kHz and a duty cycle
  * as defined by MAXCURRENT
  * Program is reading 3 ADC signals
  * 	IN5 = A0 = PA0 = Pin 12 = V_BATT
  * 	IN8 = A3 = PA3 = Pin 10 = PROX
  * 	IN11= A6 = PA6 = Pin  7 = CP
  * to check whether connector is pluged, which max current can be charged and
  * whether charging has started
  *
  ******************************************************************************
  * Functions still missing
  * Check of level of PWM to check status of charging process
  * Switching of relais for power supply and communication
  * Change of PWM duty cycle depending on available energy
  ******************************************************************************
  * Known Bugs:
  * ADC zu immernoch langsam um PWM abzutasten
  *
  ******************************************************************************
  * Change Log:
  *
  * 06.04.22	Change ADC rate to 50us ==> 20 values per period of 1 ms
  * 06.04.22	Array of AD values to check level of PWM
  * 06.04.22	Flags für Spannungslevels (Status A-F) werden gesetzt
  * 06.04.22	Prescaler für ADC von 999 auf 99
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
union convert
{
	uint8_t letter[8];
	uint8_t str[8];
} text;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNELS	3		// 3 AD-Channels
#define ADC_SAMP_RATE  20		// 20 Conversions per period of 1 ms
#define ADC_BUF_SIZE ADC_CHANNELS*ADC_SAMP_RATE // ADC Buffer Size
#define MAXCURRENT 5
// 0 = kein Ladestrom
// 1 = max. Ladestrom 4,8  A
// 2 = max. Ladestrom 6    A
// 3 = max. Ladestrom 10   A
// 4 = max. Ladestrom 16   A
// 5 = max. Ladestrom 19   A
// 6 = max. Ladestrom 25,5 A
// 7 = max. Ladestrom 32   A
#define V_2 250					// Level for 2V at CP
#define V_4 750					// Level for 4V at CP
#define V_5 1000				// Level for 5V at CP
#define V_7 1500				// Level for 7V at CP
#define V_8 1750				// Level for 8V at CP
#define V_10 2250				// Level for 10V at CP
#define V_11 2500				// Level for 11V at CP

#define LEVEL_LIMIT 7			// At least 7 samples within allowed window

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
// General Variables
uint16_t	i = 0;
uint16_t	j = 0;
uint16_t	k = 0;

// Variables for AD Conversion
uint16_t 	ADC_buffer[ADC_BUF_SIZE];
uint8_t		ADC_flag = 0;					//Flag for ADC conversion finished
uint8_t		ADC_channel = 0;				//Index for ADC channel
uint16_t	ADC_Voltage[ADC_BUF_SIZE];		//Buffer for ADC result in mV
uint32_t	ADC_cum_Val = 0;				//Storage for ADC filtering
uint32_t	ADC_CP_High = 0;				//High level of CP
uint32_t	ADC_CP_Low = 0;					//Low level of CP
uint16_t	ADC_Vbat = 0;					//Filtered Value of VBATT
uint16_t	ADC_Prox = 0;					//Filtered Value of PROX

// Variables for Control
uint8_t 	CP_Status = 0;					//Status of Control Pilot
uint8_t		PROX_Status = 0;				//Status of Proximity Signal
uint8_t		VBATT_Status = 0;				//Status of VBATT Signal
uint8_t		CP_Stat_A;						//Counter for CP-Status A (+12V)
uint8_t		CP_Stat_B;						//Counter for CP-Status B (+9V)
uint8_t		CP_Stat_C;						//Counter for CP-Status C (+6V)
uint8_t		CP_Stat_D;						//Counter for CP-Status D (+3V)
uint8_t		CP_Stat_E;						//Counter for CP-Status E (+0V)
uint8_t		CP_Stat_F;						//Counter for CP-Status F (-12V)

// Variables for PWM
uint32_t 	CH1_DC 			= 0;			//Variable for DutyCycle of PWM
uint32_t 	duty 			= 50;			//duty cycle = 50%
uint8_t		current_level 	= MAXCURRENT;	//Level of allowed max load current

// Variables for UART
uint8_t 	tbuffer[30];			//Transmit buffer
uint8_t 	*ptbuffer;
uint16_t 	value;

// Variables for Error Handling
uint8_t 	error_code;			//Transmit buffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_OK != HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1))
	  Error_Handler();
  if(HAL_OK != HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED))
	  Error_Handler();
  if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, ADC_buffer, ADC_BUF_SIZE))
	  Error_Handler();
  if(HAL_OK != HAL_TIM_Base_Start(&htim6))
	  Error_Handler();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(HAL_OK != HAL_ADC_Stop_DMA(&hadc1))
		Error_Handler();
	ADC_flag = 1;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  current_level = MAXCURRENT;
	  switch(current_level)
	  {
		  case(0): duty = 0; break;		//kein Ladestrom
		  case(1): duty = 8; break;		//max. Ladestrom 4,8  A
		  case(2): duty = 10; break;	//max. Ladestrom 6    A
		  case(3): duty = 16; break;	//max. Ladestrom 10   A
		  case(4): duty = 25; break;	//max. Ladestrom 16   A
		  case(5): duty = 30; break;	//max. Ladestrom 19   A
		  case(6): duty = 40; break;	//max. Ladestrom 25,5 A
		  case(7): duty = 50; break;	//max. Ladestrom 32   A
		  default: duty = 0; break;		//Fehler Strom = 0
	  }

	  // Reload timer 2 for PWM
      TIM2->CCR1 = CH1_DC;
      CH1_DC = 3926*duty/100;		//duty = Tastverhältnis

	  if(ADC_flag == 1)
	  {
		  // Store values in mV in ADC_Voltage Array
		  // Order of values is:
		  // channel(0).value(0),
		  // channel(1).value(0),
		  // channel(2).value(0),
		  // channel(0).value(1)...
		  for(i=0; i < ADC_BUF_SIZE; i++)
	  	  {
			  ADC_Voltage[i] = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300,ADC_buffer[i],LL_ADC_RESOLUTION_12B);
	  	  }

		  // Filtering of values
		  CP_Stat_A = 0;
		  CP_Stat_B = 0;
		  CP_Stat_C = 0;
		  CP_Stat_D = 0;
		  CP_Stat_E = 0;
		  CP_Stat_F = 0;

		  ADC_CP_Low = 0;
		  ADC_CP_High = 0;

  		  for(i=0; i < ADC_BUF_SIZE; i=i+3)
		  {
  			  if(ADC_Voltage[i]<V_2)
  			  {
  	   	  		ADC_CP_Low = ADC_CP_Low + ADC_Voltage[i];
  				CP_Stat_E++;
  			  }
 			  if((ADC_Voltage[i]>V_2) & (ADC_Voltage[i]<V_4))
  			  {
  				CP_Stat_D++;
  			  }
  			  if((ADC_Voltage[i]>V_5) & (ADC_Voltage[i]<V_7))
  			  {
  				CP_Stat_C++;
  			  }
  			  if((ADC_Voltage[i]>V_8) & (ADC_Voltage[i]<V_10))
  			  {
  				CP_Stat_B++;
  			  }
  			  if(ADC_Voltage[i]>V_11)
  			  {
  				CP_Stat_A++;
  			  }
  			  if(ADC_Voltage[i]>=V_2)
  			  {
  	   	  		ADC_CP_High = ADC_CP_High + ADC_Voltage[i];
  			  }

		  }

  		  ADC_CP_Low = ADC_CP_Low / ADC_SAMP_RATE;
  		  ADC_CP_High = ADC_CP_High / ADC_SAMP_RATE;

  		  if(CP_Stat_A > LEVEL_LIMIT) CP_Status = 0;
  		  if(CP_Stat_B > LEVEL_LIMIT) CP_Status = 1;
  		  if(CP_Stat_C > LEVEL_LIMIT) CP_Status = 2;
  		  if(CP_Stat_D > LEVEL_LIMIT) CP_Status = 3;
  		  if(CP_Stat_E > LEVEL_LIMIT) CP_Status = 4;
  		  if(CP_Stat_F > LEVEL_LIMIT) CP_Status = 5;

		  ADC_cum_Val = 0;

  		  for(i=1; i < ADC_BUF_SIZE; i=i+3)
	  	  {
  			  ADC_cum_Val = ADC_cum_Val + ADC_Voltage[i];
	  	  }
  		  ADC_Prox = ADC_cum_Val / ADC_SAMP_RATE;

  		  ADC_cum_Val = 0;

  		  for(i=2; i < ADC_BUF_SIZE; i=i+3)
	  	  {
  			  ADC_cum_Val = ADC_cum_Val + ADC_Voltage[i];
	  	  }
		  ADC_Vbat = ADC_cum_Val / ADC_SAMP_RATE;

	  	  //Send data to UART
	  	  ptbuffer = &tbuffer[0];
	  	  strcpy((char *)tbuffer,"\r\nMesswerte: ");

	  	  if(HAL_OK != HAL_UART_Transmit(&huart2, tbuffer, 15, 100))
	  	  {
	  		  error_code = 8;
	  		  Error_Handler();
	  	  }
	  	  for(ADC_channel=0; ADC_channel <= (ADC_CHANNELS); ADC_channel++)
	  	  {
	  		  if(ADC_channel == 0) value = ADC_CP_Low;
	  		  if(ADC_channel == 1) value = ADC_CP_High;
	  		  if(ADC_channel == 2) value = ADC_Prox;
	  		  if(ADC_channel == 3) value = ADC_Vbat;
	  		  strcpy((char *)tbuffer,"\n\r");

	  		  text.letter[0] = '0' + (uint8_t)(value/1000);
	     		  text.letter[1] = '0' + (uint8_t)((value%1000)/100);
	     		  text.letter[2] = '0' + (uint8_t)((value%100)/10);
	  		  text.letter[3] = '0' + (uint8_t)(value%10);
	  		  text.letter[4] = '\0';
	  		  strcat((char *)tbuffer,text.str);

	      	  if(HAL_OK != HAL_UART_Transmit(&huart2, tbuffer, 6, 100))
	      	  {
	      		  error_code = 8;
	      		  Error_Handler();
	      	  }
	      	  strcpy((char *)tbuffer,"; ");
	      	  if(HAL_OK != HAL_UART_Transmit(&huart2, tbuffer, 2, 100))
	      	  {
	      		  error_code = 8;
	      		  Error_Handler();
	      	  }
	  	  }
	  	  strcpy((char *)tbuffer,"\r\n");
	  	  if(HAL_OK != HAL_UART_Transmit(&huart2, tbuffer, 2, 100))
	  	  {
	  		  error_code = 8;
	  		  Error_Handler();
	  	  }
//	  	  HAL_Delay(1000);
	  	  //Restart ADC conversion
	  	  ADC_flag = 0;
	  	  if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, ADC_buffer, ADC_BUF_SIZE))
	  	  {
	  		  error_code = 8;
	  		  Error_Handler();
	  	  }

	  }
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
