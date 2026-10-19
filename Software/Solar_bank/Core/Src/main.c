#include  "main.h"
#include  "stm32u031xx.h"
#include  "INA219.h"
#include  "mcp401x.h"

#define PANEL_POWER_THRESHOLD_W       0.5f
#define MPPT_START_RESISTANCE_OHM     5000U
#define MPPT_MIN_RESISTANCE_OHM       5000U
#define MPPT_MAX_RESISTANCE_OHM       10000U
#define MPPT_STEP_OHM                100U
#define MPPT_POWER_CHANGE_THRESHOLD_W 0.01f
#define MAIN_UPDATE_INTERVAL_MS       1000U
#define BATTERY_LOW_ADC_THRESHOLD    2048U
#define BATTERY_HIGH_ADC_THRESHOLD   3072U

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
INA219_t ina219;
volatile uint8_t user_button_pressed = 0;
volatile uint8_t USB_charging_detected = 0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void startup_sequence(void);
static float get_panel_power(void);
static void Battery_Charge_display(void);
static void handle_user_button(void);
static void handle_usb_event(void);
static void update_charging_control(float panelPower);
static bool MPPT(float panelPower, bool panelEnabled, int step);




int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  if (Set_Resistance_I2C(MPPT_START_RESISTANCE_OHM) != HAL_OK)
  {
    Error_Handler();
  }

  if (!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
  {
    Error_Handler();
  }

  startup_sequence();

  uint32_t lastUpdateTime = HAL_GetTick();

  while (1)
  {
    handle_user_button();
    handle_usb_event();

    uint32_t currentTime = HAL_GetTick();
    if ((currentTime - lastUpdateTime) >= MAIN_UPDATE_INTERVAL_MS)
    {
      lastUpdateTime = currentTime;
      update_charging_control(get_panel_power());
    }
  }
}


static float get_panel_power(void)
{
  return INA219_ReadPower(&ina219) / 1000.0f; // Convert to watts
}

static void startup_sequence(void)
{
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_RESET);
  HAL_Delay(1000);
}

static void handle_user_button(void)
{
  if (user_button_pressed)
  {
    user_button_pressed = 0;
    Battery_Charge_display();
  }
}

static void handle_usb_event(void)
{
  if (USB_charging_detected)
  {
    USB_charging_detected = 0;
  }
}

static void update_charging_control(float panelPower)
{
  bool panelEnabled = panelPower >= PANEL_POWER_THRESHOLD_W;

  HAL_GPIO_WritePin(Fotopanel_enable_GPIO_Port, Fotopanel_enable_Pin,
                    panelEnabled ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (!MPPT(panelPower, panelEnabled, MPPT_STEP_OHM))
  {
    Error_Handler();
  }
}

static bool MPPT(float panelPower, bool panelEnabled, int step)
{
  static bool initialized = false;
  static bool previousPowerAvailable = false;
  static float resistance = MPPT_START_RESISTANCE_OHM;
  static float previousPower = 0.0f;
  static int8_t direction = -1;

  if (step <= 0)
  {
    return false;
  }

  if (!panelEnabled)
  {
    initialized = false;
    previousPowerAvailable = false;
    direction = -1;
    return true;
  }

  if (!initialized)
  {
    resistance = MPPT_START_RESISTANCE_OHM;
    if (Set_Resistance_I2C((uint32_t)resistance) != HAL_OK)
    {
      return false;
    }

    initialized = true;
    previousPowerAvailable = false;
    return true;
  }

  if (previousPowerAvailable)
  {
    float powerChange = panelPower - previousPower;
    if (powerChange < -MPPT_POWER_CHANGE_THRESHOLD_W)
    {
      direction = -direction;
    }
  }

  previousPower = panelPower;
  previousPowerAvailable = true;

  float candidateResistance = resistance + (direction * step);
  if (candidateResistance < MPPT_MIN_RESISTANCE_OHM ||
      candidateResistance > MPPT_MAX_RESISTANCE_OHM)
  {
    direction = -direction;
    candidateResistance = resistance + (direction * step);
  }

  if (candidateResistance < MPPT_MIN_RESISTANCE_OHM)
  {
    candidateResistance = MPPT_MIN_RESISTANCE_OHM;
  }
  else if (candidateResistance > MPPT_MAX_RESISTANCE_OHM)
  {
    candidateResistance = MPPT_MAX_RESISTANCE_OHM;
  }

  if (Set_Resistance_I2C((uint32_t)candidateResistance) != HAL_OK)
  {
    return false;
  }

  resistance = candidateResistance;
  return true;
}

static void Battery_Charge_display(void)
{
  uint16_t adcValue;

  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    Error_Handler();
  }

  adcValue = (uint16_t)HAL_ADC_GetValue(&hadc1);
  if (HAL_ADC_Stop(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  if (adcValue < BATTERY_LOW_ADC_THRESHOLD)
  {
    HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);
  }
  else if (adcValue < BATTERY_HIGH_ADC_THRESHOLD)
  {
    HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_SET);
  }
}


void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == User_button_input_Pin)
  {
    user_button_pressed = 1;
  }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USB_input_Pin)
  {
    USB_charging_detected = 1;
  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00303D5B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Fotopanel_enable_Pin|LED_status_red_Pin|LED_status_orange_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB0 PB4 PB5
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA4 PA5
                           PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Fotopanel_enable_Pin LED_status_red_Pin LED_status_orange_Pin */
  GPIO_InitStruct.Pin = Fotopanel_enable_Pin|LED_status_red_Pin|LED_status_orange_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_status_green_Pin */
  GPIO_InitStruct.Pin = LED_status_green_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_status_green_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : User_button_input_Pin */
GPIO_InitStruct.Pin = User_button_input_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(User_button_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_input_Pin */
  GPIO_InitStruct.Pin = USB_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_input_GPIO_Port, &GPIO_InitStruct);
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
