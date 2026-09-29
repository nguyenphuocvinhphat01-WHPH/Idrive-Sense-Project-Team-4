/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Merged Project - iDrive Sense (Wiper, Window, AC, Gas, Light)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// ===================================================
// 1. Biến cho Mưa & Động cơ (Servo/Fan)
// ===================================================
uint16_t rainADCValue = 0;
bool isWindowClosed = false;
uint32_t previousWiperTick = 0;
uint16_t wiperDelay = 1000;
bool wiperDirection = false;

// ===================================================
// 2. Biến cho Nhiệt độ độ ẩm (DHT11) - Đã dời sang PA5
// ===================================================
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM;
uint8_t Presence = 0;
uint8_t Temperature = 0;
uint8_t Humidity = 0;

// ===================================================
// 3. Biến cho Khí thải MQ-135
// ===================================================
uint32_t mq135_adc_value;
#define THRESHOLD_ON 250   // Bật báo động
#define THRESHOLD_OFF 400  // Tắt báo động
#define ADC_FILTER_SIZE 10

uint32_t adc_buffer[ADC_FILTER_SIZE];
uint8_t adc_index = 0;
uint32_t adc_sum = 0;
uint8_t buffer_filled = 0;

uint8_t buzzer_state = 0;  // 0: An toàn, 1: Báo động
uint32_t buzzer_timer = 0;

// ===================================================
// 4. Biến cho Ánh sáng LDR
// ===================================================
uint32_t ldr_adc_value;
#define LDR_THRESHOLD_ON 3000   // Tối -> Bật đèn
#define LDR_THRESHOLD_OFF 2300  // Sáng -> Tắt đèn
uint8_t led_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
uint32_t Read_ADC_Channel(uint32_t channel);
uint32_t Read_ADC_Filtered(uint32_t channel);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// -----------------------------------------------------------
// HÀM ĐỌC ADC CHUNG (Chia sẻ giữa Mưa, Khí và Ánh sáng)
// -----------------------------------------------------------
uint32_t Read_ADC_Channel(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    HAL_ADC_Start(&hadc1);

    uint32_t value = 0;
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        value = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return value;
}

// Bộ lọc trung bình tĩnh cho cảm biến khí MQ-135
uint32_t Read_ADC_Filtered(uint32_t channel) {
    uint32_t raw = Read_ADC_Channel(channel);
    adc_sum -= adc_buffer[adc_index];
    adc_buffer[adc_index] = raw;
    adc_sum += raw;
    adc_index = (adc_index + 1) % ADC_FILTER_SIZE;
    if (adc_index == 0) buffer_filled = 1;

    uint8_t count = buffer_filled ? ADC_FILTER_SIZE : adc_index;
    if (count == 0) return raw;
    return adc_sum / count;
}

// -----------------------------------------------------------
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ (Delay, Servo)
// -----------------------------------------------------------
void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void Set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t angle) {
    uint16_t pulse = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}

// -----------------------------------------------------------
// HÀM GIAO TIẾP DHT11 (Lưu ý: Đã đổi sang chân PA5)
// -----------------------------------------------------------
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT11_Check_Response(void) {
    uint16_t timeout = 0;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { // Đã đổi PA5
        timeout++; delay_us(1); if (timeout > 100) return 0;
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) { // Đã đổi PA5
        timeout++; delay_us(1); if (timeout > 100) return 0;
    }
    timeout = 0;
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { // Đã đổi PA5
        timeout++; delay_us(1); if (timeout > 100) return 0;
    }
    return 1;
}

uint8_t DHT11_Read_Byte(void) {
    uint8_t i = 0, j;
    uint16_t timeout = 0;
    for (j = 0; j < 8; j++) {
        timeout = 0;
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET) { // Đã đổi PA5
            timeout++; delay_us(1); if (timeout > 100) return 0;
        }
        delay_us(35);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { // Đã đổi PA5
            i |= (1 << (7 - j));
            timeout = 0;
            while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) { // Đã đổi PA5
                timeout++; delay_us(1); if (timeout > 100) break;
            }
        } else {
            i &= ~(1 << (7 - j));
        }
    }
    return i;
}

void DHT11_Read_Data(void) {
    Set_Pin_Output(GPIOA, GPIO_PIN_5); // Đã đổi PA5
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); // Đã đổi PA5
    HAL_Delay(18);

    __disable_irq();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); // Đã đổi PA5
    delay_us(30);
    Set_Pin_Input(GPIOA, GPIO_PIN_5); // Đã đổi PA5

    Presence = DHT11_Check_Response();
    if (Presence == 1) {
        Rh_byte1   = DHT11_Read_Byte();
        Rh_byte2   = DHT11_Read_Byte();
        Temp_byte1 = DHT11_Read_Byte();
        Temp_byte2 = DHT11_Read_Byte();
        SUM        = DHT11_Read_Byte();
    }
    __enable_irq();
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n>>> MULTI-SENSOR SYSTEM INIT OK <<<\r\n", 39, 1000);

  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // Gạt mưa (PB15)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // Kính xe (PB6)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  // Quạt (PB0)
  HAL_TIM_Base_Start(&htim1);                // Timer delay_us

  Set_Servo_Angle(&htim12, TIM_CHANNEL_2, 0);
  Set_Servo_Angle(&htim4, TIM_CHANNEL_1, 0);
  HAL_Delay(1000);
  /* USER CODE END 2 */

  uint32_t previousDHT11Tick = 0;

  while (1)
  {
      // =========================================================
      // 1. ĐỌC VÀ XỬ LÝ CẢM BIẾN MƯA (Kênh ADC1_IN1)
      // =========================================================
      rainADCValue = Read_ADC_Channel(ADC_CHANNEL_1);

      if (rainADCValue < 3500) {
            if (!isWindowClosed) {
                Set_Servo_Angle(&htim4, TIM_CHANNEL_1, 180);
                isWindowClosed = true;
            }
            wiperDelay = (rainADCValue > 2000) ? 800 : 300;

            uint32_t currentTick = HAL_GetTick();
            if (currentTick - previousWiperTick >= wiperDelay) {
                previousWiperTick = currentTick;
                if (wiperDirection == false) {
                    Set_Servo_Angle(&htim12, TIM_CHANNEL_2, 180);
                    wiperDirection = true;
                } else {
                    Set_Servo_Angle(&htim12, TIM_CHANNEL_2, 0);
                    wiperDirection = false;
                }
            }
      } else {
            Set_Servo_Angle(&htim12, TIM_CHANNEL_2, 0);
            wiperDirection = false;
            if (isWindowClosed) {
                Set_Servo_Angle(&htim4, TIM_CHANNEL_1, 0);
                isWindowClosed = false;
            }
      }

      // =========================================================
      // 2. ĐỌC VÀ XỬ LÝ KHÍ MQ-135 (Kênh ADC1_IN0)
      // =========================================================
      mq135_adc_value = Read_ADC_Filtered(ADC_CHANNEL_0);

      if (mq135_adc_value < THRESHOLD_ON) {
          buzzer_state = 1;
      } else if (mq135_adc_value > THRESHOLD_OFF) {
          buzzer_state = 0;
      }

      if (buzzer_state == 1) {
          uint32_t elapsed = HAL_GetTick() - buzzer_timer;
          if (elapsed < 2000) {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // PB1 - Đèn báo khí[cite: 3]
          } else if (elapsed < 3000) {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
          } else {
              buzzer_timer = HAL_GetTick();
          }
      } else {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
          buzzer_timer = HAL_GetTick();
      }

      // =========================================================
      // 3. ĐỌC VÀ XỬ LÝ ÁNH SÁNG LDR (Kênh ADC1_IN4)
      // =========================================================
      ldr_adc_value = Read_ADC_Channel(ADC_CHANNEL_4);

      if (ldr_adc_value > LDR_THRESHOLD_ON) {
          led_state = 1;
      } else if (ldr_adc_value < LDR_THRESHOLD_OFF) {
          led_state = 0;
      }
      // PB2 - Đèn LDR sáng cabin (đã đổi chân)
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // =========================================================
      // 4. ĐỌC DHT11 & QUẠT ĐIỀU HÒA (1 giây / lần)
      // =========================================================
      if (HAL_GetTick() - previousDHT11Tick >= 1000)
      {
            previousDHT11Tick = HAL_GetTick();
            DHT11_Read_Data();

            if (Presence == 1 && ((uint8_t)(Rh_byte1 + Rh_byte2 + Temp_byte1 + Temp_byte2) == SUM)) {
                Temperature = Temp_byte1;
                Humidity    = Rh_byte1;

                // In ra terminal để debug toàn bộ hệ thống
                printf("T:%dC H:%d%% | Rain:%d | Gas:%ld | LDR:%ld\r\n",
                        Temperature, Humidity, rainADCValue, mq135_adc_value, ldr_adc_value);

                if (Temperature < 25) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 200);
                } else if (Temperature <= 32) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 500);
                } else {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
                }
            } else {
                printf("DHT11 Error | Rain:%d | Gas:%ld | LDR:%ld\r\n",
                        rainADCValue, mq135_adc_value, ldr_adc_value);
            }
      }
  }
}

/* KHU VỰC KHỞI TẠO BÊN DƯỚI GIỮ NGUYÊN HOÀN TOÀN NHƯ PROJECT CŨ CỦA BẠN */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
//  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 16 - 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 20000 - 1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim12);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
