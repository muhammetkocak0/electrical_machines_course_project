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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Codes for different IR remote control commands
#define MOTOR_CONTROL_CODE 16753245
#define POSITION_CHANGE_CODE 16736925
#define PWM_UP_CODE 16718055
#define PWM_DOWN_CODE 16730805

// Flags and variables for motor control and status
int isChangeFlagSet = 0;
int currentPWMValueIR = 0;
int motorPosition[2] = {TIM_CHANNEL_1, TIM_CHANNEL_2};
int currentPositionIndex = 0;

// ADC values for potentiometer, electric current, and temperature
int potentiometerADCValue = 0;
int electricCurrentADCValue = 0;
int temperatureADCValue = 0;

// Variables for decoding IR remote control signals
uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;

// Character array for OLED screen display
char myCharArray0[40];

// Switch state and counters for encoder
uint8_t switchState = 0;
uint32_t counter = 0;
int16_t count = 0;
int16_t position = 0;
int speed = 0;

// Variables for PWM control of motors
int pwmValuePOT, pwmValuePOT1;
int previousPWMValuePOT = 0;
int previousPWMValuePOT1 = 0;
int engineStatus = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void updateLCDParameters();
int readThermistor();
void selectADCPOTChannel8();
void selectADCECurrentChannel9();
void selectADCNTCTemperatureChannel1();
void changeDirection();
void controlPotentiometer();
void controlPWM();
void displayStaticTextOnScreen();
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  // MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  switchState = HAL_GPIO_ReadPin(GPIOB, controller_switch_Pin) == GPIO_PIN_RESET;
  ssd1306_Init(&hi2c1);
  displayStaticTextOnScreen();
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Determine controller type
  if (!switchState)
  {
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim1);
  }
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, currentPWMValueIR);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, currentPWMValueIR);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);

  while (1)
  {
    updateLCDParameters();
    controlPWM();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_enable_GPIO_Port, motor_enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_READ_NEC_Pin */
  GPIO_InitStruct.Pin = IR_READ_NEC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_READ_NEC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : controller_switch_Pin */
  GPIO_InitStruct.Pin = controller_switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(controller_switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : motor_enable_Pin */
  GPIO_InitStruct.Pin = motor_enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motor_enable_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 3);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 4);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
int code_prev = 0;
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Handle IR remote control signal
  if ((GPIO_Pin == GPIO_PIN_0) && (switchState == 0))
  {
    handleIRSignal();
  }

  // Handle engine control switches
  else if (GPIO_Pin == GPIO_PIN_12)
  {
    handleEngineSwitch(GPIO_PIN_RESET);
  }
  else if (GPIO_Pin == GPIO_PIN_13)
  {
    handleEngineSwitch(GPIO_PIN_SET);
  }
}

// Function to handle IR remote control signal
void handleIRSignal()
{
  if (__HAL_TIM_GET_COUNTER(&htim2) > 8000)
  {
    tempCode = 0;
    bitIndex = 0;
  }
  else if (__HAL_TIM_GET_COUNTER(&htim2) > 1700)
  {
    tempCode |= (1UL << (31 - bitIndex)); // write 1
    bitIndex++;
  }
  else if (__HAL_TIM_GET_COUNTER(&htim2) > 1000)
  {
    tempCode &= ~(1UL << (31 - bitIndex));
    bitIndex++;
  }

  if (bitIndex == 32)
  {
    cmdli = ~tempCode;
    cmd = tempCode >> 8;

    if (cmdli == cmd)
    {
      code = tempCode;
      handleIRCode();
    }
    bitIndex = 0;
  }

  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

// Function to handle different IR remote control codes
void handleIRCode()
{
  if (code == MOTOR_CONTROL_CODE)
  {
    handleMotorControl();
  }
  else if (code == POSITION_CHANGE_CODE)
  {
    isChangeFlagSet = 1;
  }
  else if (code == PWM_UP_CODE)
  {
    handlePWMChange(1);
  }
  else if (code == PWM_DOWN_CODE)
  {
    handlePWMChange(0);
  }
}

// Function to handle motor control based on IR signal
void handleMotorControl()
{
  // Toggle engine state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (engineStatus == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);

  // Update OLED screen based on engine state
  ssd1306_SetCursor(7 * 8, 48);
  ssd1306_WriteString((engineStatus == GPIO_PIN_SET) ? "Status:on    " : "Status:off   ", Font_7x10, White);
  engineStatus = (engineStatus == GPIO_PIN_SET) ? 0 : 1;
  ssd1306_UpdateScreen(&hi2c1);
}

// Function to handle PWM changes based on IR signal
void handlePWMChange(int isUp)
{
  if (isUp)
  {
    if (currentPWMValueIR < 190)
    {
      currentPWMValueIR += 10;
      updatePWMAndScreen();
    }
  }
  else
  {
    currentPWMValueIR -= 10;
    if (currentPWMValueIR < 0)
    {
      currentPWMValueIR = 0;
    }
    updatePWMAndScreen();
  }
}

// Function to update PWM value and OLED screen
void updatePWMAndScreen()
{
  __HAL_TIM_SET_COMPARE(&htim1, motorPosition[currentPositionIndex], currentPWMValueIR);
  sprintf(myCharArray0, "%d ", currentPWMValueIR / 2);
  ssd1306_SetCursor(7 * 5, 48);
  ssd1306_WriteString(myCharArray0, Font_7x10, White);
  ssd1306_UpdateScreen(&hi2c1);
}

// Function to handle engine switch events
void handleEngineSwitch(uint16_t newState)
{
  while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) != newState)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, newState);
  }
  engineStatus = (newState == GPIO_PIN_SET) ? 1 : 0;
  HAL_Delay(1680); // Delay for switch stabilization
}


void updateLCDParameters()
{
  // Display RPM
  displayVariableTextOnScreen("RPM", -speed, 12);

  // Display temperature
  displayVariableTextOnScreen("C", readThermistor(), 24);

  // Display electric current
  selectADCECurrentChannel9();
  int electricCurrent = readElectricCurrent();
  displayVariableTextOnScreen("mA", electricCurrent, 0);

  // Display engine status
  displayEngineStatus();

  ssd1306_UpdateScreen(&hi2c1);
}

void displayVariableTextOnScreen(const char *unit, int value, int yPos)
{
  sprintf(myCharArray0, "%d %s                 jjj", value, unit);
  ssd1306_SetCursor(7 * 8, yPos);
  ssd1306_WriteString(myCharArray0, Font_7x10, White);
}

int readElectricCurrent()
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  electricCurrentADCValue = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  return (2 * 3300 * electricCurrentADCValue) / 4095;
}

void displayEngineStatus()
{
  const char *statusText = (engineStatus == 0) ? "Status:off    " : "Status:on    ";
  ssd1306_SetCursor(7 * 8, 48);
  ssd1306_WriteString(statusText, Font_7x10, White);
}


int readThermistor()
{
  int analogValue;
  selectADCNTCTemperatureChannel1();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 10);
  analogValue = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  double temperature;
  temperature = log((analogValue * 10000) / (4095 - analogValue));
  temperature = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temperature * temperature)) * temperature);
  temperature = temperature - 273.15 + 55;
  return (int)temperature;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  counter = __HAL_TIM_GET_COUNTER(htim);
  count = (int16_t)counter;
  position = count / 4;
}
void selectADCPOTChannel8()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
void selectADCECurrentChannel9()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void selectADCNTCTemperatureChannel1()
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

void changeDirection()
{
  if (isChangeFlagSet == 1)
  {
    for (int i = currentPWMValueIR; i >= 0; i--)
    {
      __HAL_TIM_SET_COMPARE(&htim1, motorPosition[currentPositionIndex], i);
      HAL_Delay(10);
    }
    if (currentPositionIndex == 0)
    {
      currentPositionIndex = 1;
    }
    else
    {

      currentPositionIndex = 0;
    }
    updateLCDParameters();
    HAL_Delay(380);
    for (int i = 0; i <= currentPWMValueIR; i++)
    {
      __HAL_TIM_SET_COMPARE(&htim1, motorPosition[currentPositionIndex], i);
      HAL_Delay(10);
    }

    isChangeFlagSet = 0;
  }
}
void controlPotentiometer()
{
  selectADCPOTChannel8();
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 50);
  potentiometerADCValue = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);
  potentiometerADCValue = 3300 * potentiometerADCValue / 4095;

  int targetPWMChannel1 = 0;
  int targetPWMChannel2 = 0;

  if ((potentiometerADCValue > 0) && (potentiometerADCValue < 1500))
  {
    targetPWMChannel1 = 0;
    targetPWMChannel2 = ((1500 - potentiometerADCValue) / 16) * 2;
  }
  else if ((potentiometerADCValue >= 1500) && (potentiometerADCValue < 1800))
  {
    targetPWMChannel1 = 0;
    targetPWMChannel2 = 0;
  }
  else if ((potentiometerADCValue >= 1800) && (potentiometerADCValue <= 3300))
  {
    targetPWMChannel1 = ((potentiometerADCValue - 1800) / 16) * 2;
    targetPWMChannel2 = 0;
  }

  updatePWMChannels(targetPWMChannel1, targetPWMChannel2);
}

void updatePWMChannels(int targetPWMChannel1, int targetPWMChannel2)
{
  fadeOutPWM(&htim1, TIM_CHANNEL_1, pwmValuePOT1, targetPWMChannel1);
  fadeOutPWM(&htim1, TIM_CHANNEL_2, pwmValuePOT, targetPWMChannel2);

  pwmValuePOT1 = targetPWMChannel1;
  pwmValuePOT = targetPWMChannel2;

  sprintf(myCharArray0, "%d ", (pwmValuePOT + pwmValuePOT1) / 2);
  ssd1306_SetCursor(7 * 5, 48);
  ssd1306_WriteString(myCharArray0, Font_7x10, White);
  ssd1306_UpdateScreen(&hi2c1);
}

void fadeOutPWM(TIM_HandleTypeDef *htim, uint32_t channel, int currentValue, int targetValue)
{
  int step = (currentValue > targetValue) ? -1 : 1;

  for (int i = currentValue; i != targetValue; i += step)
  {
    __HAL_TIM_SET_COMPARE(htim, channel, i);
    HAL_Delay(2);
  }
}


void controlPWM()
{
  if (switchState == 0)
  {
    changeDirection();
  }
  else
  {
    controlPotentiometer();
  }
}
// Display static text on the OLED screen
void displayStaticTextOnScreen(void)
{
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Current:", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  ssd1306_WriteString("Speed:", Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  ssd1306_WriteString("Temp:", Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  ssd1306_WriteString("Control:", Font_7x10, White);

  ssd1306_SetCursor(0, 48);
  ssd1306_WriteString("PWM:", Font_7x10, White);

  ssd1306_SetCursor(7 * 8, 48);
  ssd1306_WriteString("Status:off", Font_7x10, White);

  ssd1306_SetCursor(7 * 8, 36);
  ssd1306_WriteString(switchState ? "POT" : "IR", Font_7x10, White);
  ssd1306_UpdateScreen(&hi2c1);
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
