/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Carro Bluetooth + Servos + Pantalla OLED I2C
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Driver TB6612 ---
#define MOTOR_STBY  GPIO_PIN_1  // PB1
#define MOTOR_AIN1  GPIO_PIN_4  // PA4
#define MOTOR_AIN2  GPIO_PIN_5  // PA5
#define MOTOR_BIN1  GPIO_PIN_14 // PB14
#define MOTOR_BIN2  GPIO_PIN_15 // PB15

// --- Límites PWM ---
#define PWM_MAX          3599
#define VEL_RECTA        3000

// --- Batería ---
#define FACTOR_BATERIA   4.3f
#define VOLTAJE_MINIMO   6.5f

// --- OLED ---
#define OLED_ADDR        0x78
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
volatile uint16_t adc_buffer[1];
float voltaje_bateria = 0.0f;
volatile char rx_data_bt = 0;
char ultimo_comando = 'X';
uint16_t servo1_pulso = 1500;
uint16_t servo2_pulso = 1500;
volatile int32_t enc_izq = 0, enc_der = 0;
/* USER CODE END PV */

/* Prototipos */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_PrintString(char *str);
void Motores_Control(int32_t pwm_izq, int32_t pwm_der);
void Motores_Frenar(void);
void Control_Bluetooth(char comando);
void Mover_Garra(uint16_t p1, uint16_t p2);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// ============================================================================
// MINI DRIVER PARA PANTALLA OLED SSD1306 (I2C)
// ============================================================================
static const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7F, 0x14, 0x7F, 0x14},
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62}, {0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
    {0x00, 0x1C, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1C, 0x00}, {0x14, 0x08, 0x3E, 0x08, 0x14}, {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
    {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
    {0x08, 0x14, 0x22, 0x41, 0x00}, {0x14, 0x14, 0x14, 0x14, 0x14}, {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x51, 0x09, 0x06},
    {0x32, 0x49, 0x79, 0x41, 0x3E}, {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x49, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x07, 0x08, 0x08, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7F, 0x41, 0x41, 0x00},
    {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7F, 0x00}, {0x04, 0x02, 0x01, 0x02, 0x04}, {0x40, 0x40, 0x40, 0x40, 0x40},
    {0x00, 0x01, 0x02, 0x04, 0x00}, {0x20, 0x54, 0x54, 0x54, 0x78}, {0x7F, 0x48, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x20},
    {0x38, 0x44, 0x44, 0x48, 0x7F}, {0x38, 0x54, 0x54, 0x54, 0x18}, {0x08, 0x7E, 0x09, 0x01, 0x02}, {0x0C, 0x52, 0x52, 0x52, 0x3E},
    {0x7F, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7D, 0x40, 0x00}, {0x20, 0x40, 0x44, 0x3D, 0x00}, {0x7F, 0x10, 0x28, 0x44, 0x00},
    {0x00, 0x41, 0x7F, 0x40, 0x00}, {0x7C, 0x04, 0x18, 0x04, 0x78}, {0x7C, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38},
    {0x7C, 0x14, 0x14, 0x14, 0x08}, {0x08, 0x14, 0x14, 0x18, 0x7C}, {0x7C, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x20},
    {0x04, 0x3F, 0x44, 0x40, 0x20}, {0x3C, 0x40, 0x40, 0x20, 0x7C}, {0x1C, 0x20, 0x40, 0x20, 0x1C}, {0x3C, 0x40, 0x30, 0x40, 0x3C},
    {0x44, 0x28, 0x10, 0x28, 0x44}, {0x0C, 0x50, 0x50, 0x50, 0x3C}, {0x44, 0x64, 0x54, 0x4C, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00},
    {0x00, 0x00, 0x7F, 0x00, 0x00}, {0x00, 0x41, 0x36, 0x08, 0x00}, {0x08, 0x08, 0x2A, 0x1C, 0x08}
};

static void OLED_WriteCommand(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, data, 2, 10);
}

static void OLED_WriteData(uint8_t *data, uint16_t size) {
    uint8_t temp[128];
    temp[0] = 0x40;
    memcpy(&temp[1], data, size);
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDR, temp, size + 1, 50);
}

void OLED_Init(void) {
    HAL_Delay(100);
    OLED_WriteCommand(0xAE); OLED_WriteCommand(0x20); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xB0); OLED_WriteCommand(0xC8); OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10); OLED_WriteCommand(0x40); OLED_WriteCommand(0x81);
    OLED_WriteCommand(0xFF); OLED_WriteCommand(0xA1); OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xA8); OLED_WriteCommand(0x3F); OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xD3); OLED_WriteCommand(0x00); OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0xF0); OLED_WriteCommand(0xD9); OLED_WriteCommand(0x22);
    OLED_WriteCommand(0xDA); OLED_WriteCommand(0x12); OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x20); OLED_WriteCommand(0x8D); OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);
    OLED_Clear();
}

void OLED_Clear(void) {
    uint8_t zero[128] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        OLED_WriteCommand(0xB0 + i); OLED_WriteCommand(0x00); OLED_WriteCommand(0x10);
        OLED_WriteData(zero, 128);
    }
}

void OLED_SetCursor(uint8_t x, uint8_t y) {
    OLED_WriteCommand(0xB0 + y);
    OLED_WriteCommand(((x & 0xF0) >> 4) | 0x10);
    OLED_WriteCommand((x & 0x0F));
}

void OLED_PrintString(char *str) {
    while (*str) {
        if (*str >= 32 && *str <= 126) {
            OLED_WriteData((uint8_t*)Font5x7[*str - 32], 5);
            uint8_t space[1] = {0x00};
            OLED_WriteData(space, 1);
        }
        str++;
    }
}

// ============================================================================
// LÓGICA DEL CARRO
// ============================================================================
void Motores_Control(int32_t pwm_izq, int32_t pwm_der) {
    HAL_GPIO_WritePin(GPIOB, MOTOR_STBY, GPIO_PIN_SET);

    // Motor Izquierdo
    if (pwm_izq > 0) {
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2, GPIO_PIN_RESET);
    } else if (pwm_izq < 0) {
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2, GPIO_PIN_SET);
        pwm_izq = -pwm_izq;
    } else {
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2, GPIO_PIN_RESET);
    }

    // Motor Derecho
    if (pwm_der > 0) {
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN2, GPIO_PIN_RESET);
    } else if (pwm_der < 0) {
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN2, GPIO_PIN_SET);
        pwm_der = -pwm_der;
    } else {
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, MOTOR_BIN2, GPIO_PIN_RESET);
    }

    if(pwm_izq > PWM_MAX) pwm_izq = PWM_MAX;
    if(pwm_der > PWM_MAX) pwm_der = PWM_MAX;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_izq);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_der);
}

void Motores_Frenar(void) {
    HAL_GPIO_WritePin(GPIOB, MOTOR_STBY, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_AIN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, MOTOR_AIN2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_BIN1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_BIN2, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

void Control_Bluetooth(char comando) {
    if (comando == 0) return;
    ultimo_comando = comando;

    switch (comando) {
        case 'W': Motores_Control(VEL_RECTA, VEL_RECTA); break;
        case 'S': Motores_Control(-VEL_RECTA, -VEL_RECTA); break;
        case 'A': Motores_Control(-VEL_RECTA, VEL_RECTA); break;
        case 'D': Motores_Control(VEL_RECTA, -VEL_RECTA); break;
        case 'X': Motores_Frenar(); break;
        case 'O': Mover_Garra(1000, 1000); break;
        case 'C': Mover_Garra(2000, 2000); break;
    }
}

void Mover_Garra(uint16_t p1, uint16_t p2) {
    servo1_pulso = (p1 < 500) ? 500 : (p1 > 2500) ? 2500 : p1;
    servo2_pulso = (p2 < 500) ? 500 : (p2 > 2500) ? 2500 : p2;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo1_pulso);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, servo2_pulso);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_data_bt, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    static uint32_t t_izq = 0, t_der = 0;
    uint32_t ahora = HAL_GetTick();
    if (GPIO_Pin == GPIO_PIN_12 && (ahora - t_izq) >= 3U) { enc_izq++; t_izq = ahora; }
    if (GPIO_Pin == GPIO_PIN_13 && (ahora - t_der) >= 3U) { enc_der++; t_der = ahora; }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_IWDG_Init();

  /* USER CODE BEGIN 2 */
  HAL_IWDG_Refresh(&hiwdg);

  __HAL_TIM_SET_AUTORELOAD(&htim1, 3599);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  Mover_Garra(1500, 1500);

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_data_bt, 1);

  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 1);

  OLED_Init();

  Motores_Frenar();
  char buf[20];
  uint32_t ultimo_tiempo_oled = 0;

  for(int i=0; i<20; i++) { HAL_IWDG_Refresh(&hiwdg); HAL_Delay(100); }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_IWDG_Refresh(&hiwdg);
      voltaje_bateria = ((float)adc_buffer[0] * 3.3f / 4095.0f) * FACTOR_BATERIA;

      Control_Bluetooth(rx_data_bt);
      rx_data_bt = 0;

      if ((HAL_GetTick() - ultimo_tiempo_oled) >= 250U) {
          ultimo_tiempo_oled = HAL_GetTick();

          OLED_SetCursor(0, 0);
          snprintf(buf, sizeof(buf), "CMD:%c V:%2d.%dV", ultimo_comando, (int)voltaje_bateria, (int)((voltaje_bateria-(int)voltaje_bateria)*10));
          OLED_PrintString(buf);

          const char* estado_garra = (servo1_pulso < 1250) ? "OPEN " : ((servo1_pulso > 1750) ? "CLOSE" : "MID  ");
          OLED_SetCursor(0, 2);
          snprintf(buf, sizeof(buf), "Garra: %s", estado_garra);
          OLED_PrintString(buf);

          OLED_SetCursor(0, 4);
          snprintf(buf, sizeof(buf), "Esperando App...");
          OLED_PrintString(buf);
      }

      HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

// ============================================================================
// INICIALIZACIÓN DE PERIFÉRICOS (Generado por CubeMX)
// ============================================================================
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 2500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) { Error_Handler(); }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}
