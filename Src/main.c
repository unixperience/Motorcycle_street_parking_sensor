/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file		   : main.c
  * @brief		  : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *	  this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *	  this list of conditions and the following disclaimer in the documentation
  *	  and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *	  may be used to endorse or promote products derived from this software
  *	  without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include <stdio.h>
#include <stdbool.h>

#include "WaveGeneration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum _eMessagePriority
{
	LOG_EMERG	= 0,	/* system is unusable */
	LOG_ALERT	= 1,	/* action must be taken immediately */
	LOG_CRIT	= 2,	/* critical conditions */
	LOG_ERR		= 3,	/*  error conditions */
	LOG_WARNING	= 4,	/* warning conditions */
	LOG_NOTICE	= 5,	/* normal but significant condition */
	LOG_INFO	= 6,	/* informational */
	LOG_DEBUG	= 7,	/* debug-level messages */
}eMessagePriority;

const int MAX_MESSAGE_LENGTH = 32;
typedef struct _T_DbgMessage
{
	osThreadId calling_thread;
	char message[MAX_MESSAGE_LENGTH];
	TickType_t  tick_count;		//=xTaskGetTickCount();
	eMessagePriority priority;
} T_DbgMessage;

typedef struct _T_LightThreadArgs
{
	uint16_t strobe_pin;
	GPIO_TypeDef *strobe_port;
	uint32_t signal_mask;
} T_LightThreadArgs;

typedef struct _T_SensorThreadArgs
{
	uint16_t pulse_pin;
	GPIO_TypeDef *pulse_port;
	
	uint16_t echo_pin;
	GPIO_TypeDef *echo_port;
	
	uint32_t timer_capture_mask;
	uint32_t signal_mask;
	uint32_t icp_timer_channel;
} T_SensorThreadArgs;

typedef struct _T_SirenThreadArgs
{
	uint16_t siren_pin;
	GPIO_TypeDef *siren_port;
	uint32_t dac_channel;
	
	uint32_t signal_mask;
} T_SirenThreadArgs;

typedef enum _eRTOSSignals
{
	//signal is going to be a bitfield
	eSigBlank = 0x00,
	//lsb = isLeft
	eSigIsRight = 0x00,
	eSigIsLeft  = 0x01,
	
	//next is Light signal
	eSigIsLight = 0x02,
	
	//and then Siren
	eSigIsSiren = 0x04,	
	
	//Sensor Evaluate, the actual sensor readings are done by interrupt
	eSigIsSensor= 0x08,
}eRTOSSignals;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//------------------- ULTRA SONIC SENSOR PARAMETERS ---------------------------
//Maximum range is 4.5 meters. Sound at Standard Atmosphere temperature travels
//346 m/s. so our max ROUND TRIP time is (4.5m*2) * 340m/s = 26.47ms
const float SPEED_OF_SOUND_25C_CMpS = (346*100); //m/s
const uint8_t MAX_SONAR_DELAY_MS = 33;

//These thresholds are used when reading distance values from the ultrasonic
//sensor
const uint32_t MIN_DISTANCE_CLK_CYCLE_CM = 60;
const uint32_t THRESHOLD_VELOCITY_CHANGE = 10;

const uint16_t SIREN_SOUNDING_PERIOD_MS = 3000;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

osThreadId SirenSoundHandle;
osThreadId LightFlashHandle;
osThreadId SensorReadHandle;
osThreadId UartDebugHandle;
/* USER CODE BEGIN PV */
osThreadId RightSirenSoundHandle;
osThreadId RightLightFlashHandle;
osThreadId RightSensorReadHandle;

//creates a mailbox containing up to 16 T_DbgMessages
osMailQId UartMailbox;
osMailQDef(UartMailbox, 16, T_DbgMessage); 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM6_Init(void);
void StartTaskSirenSound(void const * argument);
void StartTaskLightFlash(void const * argument);
void StartTaskSensorRead(void const * argument);
void StartTaskUartDebug(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t  gau8_NUM_TIM3_ICP_READINGS_CHAN[2] = {0,0};
uint16_t gau16_TIM3_ICP_AVERAGE_CHAN[2]	 = {0,0};
uint32_t gu32_TIM3_ICP_SUM_CHAN[2]		  = {0,0};
uint16_t gu16_TIM3_CURR_ICP_CHAN[2]		 = {0,0};
uint16_t gu16_TIM3_PREV_ICP_CHAN[2]		 = {0,0};

WaveGenerator sound_engine;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char str_buffer[22];
	T_LightThreadArgs LeftStrobe;
	T_LightThreadArgs RightStrobe;
	T_SensorThreadArgs LeftSensor;
	T_SensorThreadArgs RightSensor;
	T_SirenThreadArgs LeftSiren;
	T_SirenThreadArgs RightSiren;
	
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
  MX_LPUART1_UART_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	LeftStrobe.strobe_port  = GPIOC;
	LeftStrobe.strobe_pin   = 8;
	
	RightStrobe.strobe_port = GPIOC;
	RightStrobe.strobe_pin  = 9;	
		
	LeftSensor.pulse_port   = GPIOA;
	LeftSensor.pulse_pin	= GPIO_PIN_8;
	LeftSensor.echo_port	= GPIOA;	//input capture unit...
	LeftSensor.echo_pin	 = GPIO_PIN_6;
	LeftSensor.timer_capture_mask = TIM_FLAG_CC1; // TIM_SR_CC1IF_Msk;;
	LeftSensor.signal_mask  = eSigIsLeft;
	LeftSensor.icp_timer_channel=TIM_CHANNEL_1;
	
	RightSensor.pulse_port  = GPIOA;
	RightSensor.pulse_pin   = GPIO_PIN_9;
	RightSensor.echo_port   = GPIOA;	//input capture unit...
	RightSensor.echo_pin	= GPIO_PIN_7;
	RightSensor.timer_capture_mask = TIM_FLAG_CC2; // TIM_SR_CC2IF_Msk;
	RightSensor.signal_mask = eSigIsRight;
	RightSensor.icp_timer_channel = TIM_CHANNEL_2;
	
	LeftSiren.dac_channel   = DAC_CHANNEL_1;
	LeftSiren.siren_port	= GPIOA;
	LeftSiren.siren_pin	 = 4;
	
	RightSiren.dac_channel  = DAC_CHANNEL_2;
	RightSiren.siren_port   = GPIOA;
	RightSiren.siren_pin	= 5;
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

	// NOT THREAD CODE but not sure where else to put it
	strcpy(str_buffer,"starting scheduler..\0");
	// (handle, string string length, timeout)
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)&str_buffer, 18, 1000); 
	
	//starts the timer with interurpts enabled
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);	
	
	sound_engine = WaveGenerator(Waveform::eSinusoid,
								500,	//wave frequency
								1.0,	//amplitude
								-1,
								Resolution::e12Bit_Lower);
	
	//starts 40kHz square wave for sonar trigger
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of SirenSound */
  osThreadDef(SirenSound, StartTaskSirenSound, osPriorityNormal, 0, 128);
  SirenSoundHandle = osThreadCreate(osThread(SirenSound), (void*) &LeftSiren);

  /* definition and creation of LightFlash */
  osThreadDef(LightFlash, StartTaskLightFlash, osPriorityIdle, 0, 128);
  LightFlashHandle = osThreadCreate(osThread(LightFlash), (void*) &LeftStrobe);

  /* definition and creation of SensorRead */
  osThreadDef(SensorRead, StartTaskSensorRead, osPriorityIdle, 0, 128);
  SensorReadHandle = osThreadCreate(osThread(SensorRead), (void*) &LeftSensor);

  /* definition and creation of UartDebug */
  osThreadDef(UartDebug, StartTaskUartDebug, osPriorityIdle, 0, 128);
  UartDebugHandle = osThreadCreate(osThread(UartDebug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	osThreadDef(RightSirenSound, StartTaskSirenSound, osPriorityNormal, 0, 128);
	RightSirenSoundHandle = osThreadCreate(osThread(SirenSound), (void*) &RightSiren);

	/* definition and creation of LightFlash */
	osThreadDef(RightLightFlash, StartTaskLightFlash, osPriorityIdle, 0, 128);
	RightLightFlashHandle = osThreadCreate(osThread(LightFlash), (void*) &RightStrobe);

	/* definition and creation of SensorRead */
	osThreadDef(RightSensorRead, StartTaskSensorRead, osPriorityIdle, 0, 128);
	RightSensorReadHandle = osThreadCreate(osThread(SensorRead), (void*) &RightSensor);	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

	//we want 1cm resolution. We know sound travels 346m/s@25C through air. Thus
	//our minimum time resolution for round trip is (0.01m*2) / (346) = 57.8uS
	//or 1/57.8uS = 17.3kHz which will be out clock speed. Thus every unit of 
	//time is equivalent to 1 cm
	//Since our TimerClock is the internal 16MHz clock we need to get it down to 
	//17.3kHz, so  16MHz / 17.3kHz = 294.8556 ~ 295, so our clock prescaler
	//is 295. This means each increment of the clock is 2cm (round trip) in 
	//air@25C. Since we only want one way trip this is effectively 1cm.
	// A reading of 20 => 20CM, 150 = 1.5m etc.
		
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 925;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 333;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 400;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Left_ultraSonicPulse_Pin|Right_ultraSonicPulse_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Left_Strobe_Pin|Right_Strobe_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Left_ultraSonicPulse_Pin Right_ultraSonicPulse_Pin */
  GPIO_InitStruct.Pin = Left_ultraSonicPulse_Pin|Right_ultraSonicPulse_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_Strobe_Pin Right_Strobe_Pin */
  GPIO_InitStruct.Pin = Left_Strobe_Pin|Right_Strobe_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief Interrupt handler for Input Capture pin. This callback is used for
  * all timers and all channels so you must cheeck the exact caller
  * @param htim the timer device that triggered the innterrupt
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t  my_idx;
	uint32_t my_signal;
	uint32_t current_reading;
	
	//right now we only want timer3 input capture events
	if (htim->Instance==TIM3)
	{	
		if (htim->Channel == TIM_CHANNEL_1)
		{
			my_idx=0;
			my_signal = eSigIsLeft;
		}
		else if (htim->Channel == TIM_CHANNEL_2)
		{
			my_idx=0;
			my_signal = eSigIsRight;
		}
		else
		{
			return;
		}		
		
		gu16_TIM3_CURR_ICP_CHAN[my_idx] = __HAL_TIM_GetCompare(&htim3, TIM_CHANNEL_1);
		
		//calculate distance accounting for case that timer rolls over
		if (gu16_TIM3_CURR_ICP_CHAN[my_idx] < gu16_TIM3_PREV_ICP_CHAN[my_idx])
		{
			//first we need the value till rollover. then add the extra cycles
			current_reading += (0xFFFF - gu16_TIM3_PREV_ICP_CHAN[my_idx])
										+ gu16_TIM3_CURR_ICP_CHAN[my_idx];
		}
		else
		{
			current_reading += 
				(gu16_TIM3_CURR_ICP_CHAN[my_idx] - gu16_TIM3_PREV_ICP_CHAN[my_idx]);
		}
		
		//there are no objects in range, ignore the reading
		if (current_reading > MAX_SONAR_DELAY_MS)
		{
			return;
		}
		gu32_TIM3_ICP_SUM_CHAN[my_idx] += current_reading;
		gau8_NUM_TIM3_ICP_READINGS_CHAN[my_idx]++;
		
		gu16_TIM3_PREV_ICP_CHAN[my_idx] = gu16_TIM3_CURR_ICP_CHAN[my_idx];
		
		//we want to take an average of 2^n readings for better accuracy
		// 8 = 2^N so N=3
		if (gau8_NUM_TIM3_ICP_READINGS_CHAN[my_idx] >= 8)
		{
			//we divide the running sum by num of readings. since it's a 
			//power of two we get the same effect using a shift by N
			gau16_TIM3_ICP_AVERAGE_CHAN[my_idx] = (gu32_TIM3_ICP_SUM_CHAN[my_idx] >> 3);
			
			osSignalSet(SensorReadHandle, my_signal | eSigIsSensor);
			
			//starts next running average
			gau8_NUM_TIM3_ICP_READINGS_CHAN[my_idx] = 0;
			gu32_TIM3_ICP_SUM_CHAN[my_idx] = 0;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskSirenSound */
/**
  * @brief  Function implementing the SirenSound thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTaskSirenSound */
void StartTaskSirenSound(void const * argument)
{

  /* USER CODE BEGIN 5 */
	T_SirenThreadArgs* config;	
	osEvent evt;
	DAC_ChannelConfTypeDef sDacConfig = {0};
	
	config = (T_SirenThreadArgs*) argument;
	
	HAL_DAC_Start(&hdac1, config->dac_channel);	
	
	//enable output
	sDacConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sDacConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sDacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sDacConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	sDacConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
		  
	/* Infinite loop */
	for(;;)
	{
		//you need to turn on the DAC here
		evt = osSignalWait((int) config->signal_mask | eSigIsSiren, 100);
		if (evt.status == osEventSignal)
		{
		  //enable output, the samples for the DAC are set in Timer6 trigger
		  sDacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
		  if (HAL_DAC_ConfigChannel(&hdac1, &sDacConfig,  config->dac_channel) != HAL_OK)
		  {
			Error_Handler();
		  }
					  
		  osDelay(SIREN_SOUNDING_PERIOD_MS);
			
		  //disables output, but leaves peripreal enabled
		  sDacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
		  if (HAL_DAC_ConfigChannel(&hdac1, &sDacConfig,  config->dac_channel) != HAL_OK)
		  {
			Error_Handler();
		  }		  
		}		 
	}
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartTaskLightFlash */
/**
* @brief Function implementing the LightFlash thread.
* @param argument: T_LightThreadArgs* all paraeters needed for light task
* @retval None
*/
/* USER CODE END Header_StartTaskLightFlash */
void StartTaskLightFlash(void const * argument)
{
  /* USER CODE BEGIN StartTaskLightFlash */
	osEvent event;  
	uint8_t num_flashes;
	T_LightThreadArgs *config;

	num_flashes = 0;  
	
	config = (T_LightThreadArgs*)argument;
	/* Infinite loop */
	for(;;)
	{
	  
		if (num_flashes > 0)
		{			
			HAL_GPIO_TogglePin(config->strobe_port, config->strobe_pin);
			osDelay(150);
			
			if (num_flashes-- == 0)
			{
				//ensure it's off
				HAL_GPIO_WritePin(config->strobe_port,
								  config->strobe_pin,
								  GPIO_PIN_RESET);
			}
		}
		else
		{
			//flags are automatically cleared when thread wakes and checks flags
			event = osSignalWait( eSigIsLight | eSigIsLeft, osWaitForever);
			  
			if (event.status == osEventSignal)
			{
				//must be even number
				num_flashes = 10;
			}
		}
	}
  /* USER CODE END StartTaskLightFlash */
}

/* USER CODE BEGIN Header_StartTaskSensorRead */
/**
* @brief Function implementing the SensorRead thread. This thread checks the
* global variables for sensor readings and decides what action to take. If 
* readings are close call siren/light
* @param argument: T_SensorThreadArgs* all aparameters needed for Sensor task
* @retval None
* @note this function doesn't actually do the readings itself, instead sensor
* readings come from the ICP interrupt see HAL_TIM_IC_CaptureCallback
*/
/* USER CODE END Header_StartTaskSensorRead */
void StartTaskSensorRead(void const * argument)
{
  /* USER CODE BEGIN StartTaskSensorRead */
	UBaseType_t uxHighWaterMark;
	osThreadId my_task_handle;
	uint8_t my_idx;
	
	T_DbgMessage *pMsg;	
	T_SensorThreadArgs* config;
	
	uint16_t distance_cm;
	uint16_t prev_distance_cm;
	int32_t change_in_distance;
	
	osEvent event;
	
	//arguments
	config = (T_SensorThreadArgs*)argument;
	
	if (config->icp_timer_channel == TIM_CHANNEL_1)
	{
		my_idx = 0;
	}
	else if (config->icp_timer_channel == TIM_CHANNEL_2)
	{
		my_idx = 1;
	}

	my_task_handle = xTaskGetCurrentTaskHandle();
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	
	//get watermark, how much stack space is left
	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	
	pMsg->tick_count = xTaskGetTickCount();
	pMsg->calling_thread =  my_task_handle;
	sprintf(pMsg->message,"Watermark:%lu words", uxHighWaterMark);
	
	osMailPut(UartMailbox, pMsg);

	/* Infinite loop */
	for(;;)
	{			
		//flags are automatically cleared when thread wakes and checks flags
		event = osSignalWait( eSigIsSensor | config->signal_mask, osWaitForever);
		  
		if (event.status == osEventSignal)
		{
			distance_cm = gau16_TIM3_ICP_AVERAGE_CHAN[my_idx];
			//RECALL: 1 timer3 clock cycle ~1cm using the ultra sonic sensor			
			//is the value to close?
			//what about wrap around, abs value... no that wont work... aia.test
			if (distance_cm < MIN_DISTANCE_CLK_CYCLE_CM)
			{
				osSignalSet(SirenSoundHandle, config->signal_mask | eSigIsSiren);
			}
			
			change_in_distance = (prev_distance_cm - distance_cm);
			//if the value is changing too quickly (means car is coming up quick)
			if ( (change_in_distance >  THRESHOLD_VELOCITY_CHANGE) 
			   ||(change_in_distance < -THRESHOLD_VELOCITY_CHANGE))
			{
				osSignalSet(LightFlashHandle, config->signal_mask | eSigIsLight);
			}
			
			//send mailbox
			pMsg = (T_DbgMessage*) osMailCAlloc(UartMailbox, osWaitForever);
			
			pMsg->tick_count = xTaskGetTickCount();
			pMsg->calling_thread =  my_task_handle;
			sprintf(pMsg->message,"Dist:%d cm", distance_cm);
			
			osMailPut(UartMailbox, pMsg);
		}
	
	}
  /* USER CODE END StartTaskSensorRead */
}

/* USER CODE BEGIN Header_StartTaskUartDebug */
/**
* @brief Function implementing the UartDebug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskUartDebug */
void StartTaskUartDebug(void const * argument)
{
  /* USER CODE BEGIN StartTaskUartDebug */
	T_DbgMessage *pDbgMsg;
	osEvent event;
	char* task_name;
	char mydata[MAX_MESSAGE_LENGTH];
  
	strcpy(mydata,"Lets go!\r\n");
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, 10, 1000);

	sprintf(mydata, ".\r\n>");
	/* Infinite loop */
	for(;;)
	{
		HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, 4, 1000);	
		osDelay(1000);

		event = osMailGet(UartMailbox, osWaitForever);
			
		if (event.status == osEventMail)
		{
			pDbgMsg = (T_DbgMessage*)event.value.p;
			
			//print thread name
			task_name = pcTaskGetName(pDbgMsg->calling_thread);
			if (task_name != NULL)
			{
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)task_name, strlen(task_name), 1000);
			}			
			else
			{
				sprintf(task_name, "unkown");
				HAL_UART_Transmit(&hlpuart1, (uint8_t*)task_name, strlen(task_name), 1000);
			}
			
			//time
			sprintf(mydata, ":%ul:", pDbgMsg->tick_count);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, strlen(mydata), 1000);
			
			strcpy(mydata, pDbgMsg->message);
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, strlen(mydata), 1000);
			
			strcpy(mydata, "\r\n");
			HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, 2, 1000);			
	  
			osMailFree(UartMailbox, pDbgMsg);	
		  
		}
		else
		{
		  sprintf(mydata, "event:%d", event.status);
		  HAL_UART_Transmit(&hlpuart1, (uint8_t*)mydata, 8, 1000);			
		  sprintf(mydata, ".\r\n>");
		}			
	}
  /* USER CODE END StartTaskUartDebug */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	uint16_t dac_output_value;
	
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) 
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  else if (htim->Instance == TIM6)
  {
	  dac_output_value = sound_engine.generateNextSample();

	  //for now set both channels to the same value, we may change this in the future
	  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_output_value);
	  HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_2, DAC_ALIGN_12B_R, dac_output_value);
  }
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
