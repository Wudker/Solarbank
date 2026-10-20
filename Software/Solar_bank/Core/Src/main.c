#include "main.h"
#include "stm32u031xx.h"
#include "INA219.h"
#include "mcp401x.h"

// Wartosci orientacyjne. Nalezy je sprawdzic na prawdziwym panelu.
#define PANEL_MIN_VOLTAGE_MV           3000U
#define PANEL_POWER_START_W           0.50f
#define PANEL_POWER_STOP_W            0.20f
#define PANEL_RETRY_INTERVAL_MS       5000U
#define MAIN_UPDATE_INTERVAL_MS       1000U

#define MPPT_START_RESISTANCE_OHM      5000U
#define MPPT_MIN_RESISTANCE_OHM        5000U
#define MPPT_MAX_RESISTANCE_OHM       10000U
#define MPPT_STEP_OHM                  100U
#define MPPT_POWER_CHANGE_THRESHOLD_W 0.01f

// Dzielnik baterii: R12=220k, R13=330k, referencja ADC=3.3V.
// Odpowiednio ok. 3.5V i 3.9V na ogniwie.
#define BATTERY_LOW_ADC_THRESHOLD      2606U
#define BATTERY_HIGH_ADC_THRESHOLD     2904U
#define BATTERY_LED_TIME_MS            3000U
#define BUTTON_DEBOUNCE_MS             200U

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
INA219_t ina219;

volatile uint8_t user_button_pressed = 0;
static bool panel_on = false;
static bool first_panel_measurement = false;
static uint8_t low_power_count = 0;
static uint32_t last_panel_attempt = 0;
static bool battery_led_visible = false;
static uint32_t battery_led_time = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void startup_sequence(void);
static float get_panel_power(void);
static void Battery_Charge_display(void);
static void handle_user_button(void);
static void turn_off_battery_leds(void);
static void set_panel(bool enable);
static void update_charging_control(void);
static bool MPPT(float panelPower, bool panelEnabled, int step);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  // Kalibracja ADC przed pierwszym pomiarem.
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Panel domyslnie odlaczony (PA3=1 oznacza DISABLE).
  set_panel(false);

  if (Set_Resistance_I2C(MPPT_START_RESISTANCE_OHM) != HAL_OK)
  {
    Error_Handler();
  }
  if (!INA219_Init(&ina219, &hi2c1, INA219_ADDRESS))
  {
    Error_Handler();
  }

  startup_sequence();
  uint32_t last_update_time = HAL_GetTick();
  // Pierwsza proba wlaczenia panelu po 5 sekundach od startu.
  last_panel_attempt = last_update_time;

  while (1)
  {
    handle_user_button();

    uint32_t now = HAL_GetTick();
    if (battery_led_visible && now - battery_led_time >= BATTERY_LED_TIME_MS)
    {
      turn_off_battery_leds();
      battery_led_visible = false;
    }

    if (now - last_update_time >= MAIN_UPDATE_INTERVAL_MS)
    {
      last_update_time = now;
      update_charging_control();
    }
  }
}

static void set_panel(bool enable)
{
  // Schemat: PA3 -> Q4 -> Q5 -> Q6. GPIO=0 wlacza panel.
  HAL_GPIO_WritePin(Fotopanel_enable_GPIO_Port, Fotopanel_enable_Pin,
                    enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
  panel_on = enable;
}

static float get_panel_power(void)
{
  return INA219_ReadPower(&ina219) / 1000.0f; // mW -> W
}

static void startup_sequence(void)
{
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_SET);
  HAL_Delay(200);
  turn_off_battery_leds();
}

static void turn_off_battery_leds(void)
{
  HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);
}

static void handle_user_button(void)
{
  static uint32_t last_press_time = 0;
  if (user_button_pressed)
  {
    user_button_pressed = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_press_time >= BUTTON_DEBOUNCE_MS)
    {
      last_press_time = now;
      Battery_Charge_display();
      battery_led_time = now;
      battery_led_visible = true;
    }
  }
}

static void update_charging_control(void)
{
  uint32_t now = HAL_GetTick();
  // Q3 zwiera PC14 do GND po wykryciu USB. Odczytujemy stan,
  // aby rozpoznac takze USB podpiete jeszcze przed uruchomieniem MCU.
  bool usb_connected = HAL_GPIO_ReadPin(USB_input_GPIO_Port, USB_input_Pin)
                       == GPIO_PIN_RESET;

  if (usb_connected)
  {
    set_panel(false); // Priorytet dla ladowania USB.
    low_power_count = 0;
    first_panel_measurement = false;
    last_panel_attempt = now;
    MPPT(0.0f, false, MPPT_STEP_OHM); // Wyzeruj pamiec MPPT.
    return;
  }

  if (!panel_on)
  {
    if (now - last_panel_attempt < PANEL_RETRY_INTERVAL_MS)
    {
      return;
    }
    last_panel_attempt = now;

    // Napiecie panelu mozna zmierzyc takze przy Q6 odlaczonym.
    // Jest to napiecie bez obciazenia, a nie potwierdzenie dostepnej mocy.
    if (INA219_ReadBusVoltage(&ina219) < PANEL_MIN_VOLTAGE_MV)
    {
      return;
    }

    if (Set_Resistance_I2C(MPPT_START_RESISTANCE_OHM) != HAL_OK)
    {
      Error_Handler();
    }
    MPPT(0.0f, false, MPPT_STEP_OHM);
    first_panel_measurement = true;
    low_power_count = 0;
    set_panel(true);
    return; // Czekamy ok. 1 s na ustabilizowanie napiecia.
  }

  float panel_power = get_panel_power();
  float threshold = first_panel_measurement ? PANEL_POWER_START_W
                                            : PANEL_POWER_STOP_W;

  if (panel_power < threshold)
  {
    low_power_count++;
    if (low_power_count >= 2) // Nie odlaczaj po pojedynczym slabym pomiarze.
    {
      set_panel(false);
      low_power_count = 0;
      first_panel_measurement = false;
      last_panel_attempt = now;
      MPPT(0.0f, false, MPPT_STEP_OHM);
    }
    return;
  }

  first_panel_measurement = false;
  low_power_count = 0;
  if (!MPPT(panel_power, true, MPPT_STEP_OHM))
  {
    Error_Handler();
  }
}

static bool MPPT(float panelPower, bool panelEnabled, int step)
{
  // Prosty algorytm perturb & observe - bez dodatkowych bibliotek.
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

  turn_off_battery_leds();
  if (adcValue < BATTERY_LOW_ADC_THRESHOLD)
  {
    HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_SET);
  }
  else if (adcValue < BATTERY_HIGH_ADC_THRESHOLD)
  {
    HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_SET);
  }
  else
  {
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
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_160CYCLES_5;
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

  /* Na PCB PA3=1 odlacza panel. Ustaw przed konfiguracja pinu. */
  HAL_GPIO_WritePin(GPIOA, Fotopanel_enable_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, LED_status_red_Pin|LED_status_orange_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB9 PB0 PB4 PB5 PB6 */
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

  /*Configure GPIO pins : PA1 PA2 PA4 PA5 PA11 PA12 */
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

  /* PC14: podlaczony USB = stan niski, bez przerwania */
  GPIO_InitStruct.Pin = USB_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // R26 juz podciaga do 3.3 V.
  HAL_GPIO_Init(USB_input_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  // Ustaw panel w stan wylaczony i sygnalizuj blad.
  HAL_GPIO_WritePin(Fotopanel_enable_GPIO_Port, Fotopanel_enable_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_status_green_GPIO_Port, LED_status_green_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_orange_GPIO_Port, LED_status_orange_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_status_red_GPIO_Port, LED_status_red_Pin, GPIO_PIN_SET);
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
  * @param  line: assert_param file line number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
