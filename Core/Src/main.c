/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdint.h>
#include <stm32f1xx_hal_tim.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define open_loop_revolution_limit 130000
#define sector_change_interval (2000 / open_loop_velocity_rpm) 
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//Onceki degerler 
uint8_t sector = 0;
double sector_change_interval_counter = 0;
double motor_align_time_counter = 1.0;
uint32_t align_duration = 10000; // duration value which the motor will hold its position for (ms)
//for each 0.1ms activation of HAL_TIM_PeriodElapsedCallback function 10k.0.1ms= 1s  
uint16_t motor_align_ccr_value = 1500; //ccr value when alignment is being made //capture and compare 

float rpm_ramp_slope = 0.1; // open loop open_loop_velocity_rpm increment value
uint32_t ol_sector_counter = 0;
uint32_t ol_revolution_limit = (open_loop_revolution_limit* 30);
const double open_loop_velocity_rpm = 1200;
const double open_loop_start_speed_rpm = 300;
double desired_sector_change_interval = (2000/open_loop_velocity_rpm); // soft start opening value
double initial_sector_change_interval = (2000/open_loop_start_speed_rpm); //(2000/ open_loop_start_speed_rpm); 6.6
//uint32_t initial_sector_change_interval;
//uint32_t initial_sector_change_interval = 5;
uint32_t initial_open_loop_ccr = 50; // soft start opening value
uint32_t desired_open_loop_ccr = 50; // drive ccr value after soft start cycle
uint32_t ccr_ramp_slope = 32; // open loop ccr value increment value
float ms_counter;

volatile uint32_t throttle_pwm_demand = 0;
volatile bool is_closed_loop_control_active = false; // Closed loop control status(first state is open loop)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void); // NEW
static void MX_TIM2_Init(void); // NEW
/* USER CODE BEGIN PFP */
void update_sector_for_open_loop(void); // Function were changed by me
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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_TIM1_Init(); // NEw
  MX_TIM2_Init(); // nEw
  /* USER CODE BEGIN 2 */
  
  // Clear Timer counter
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  
  // 6-Kanal Complementary PWM'i baslat
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // IPM'e giden sinyal cikislarini etkinlestir (Main Output Enable) //death time icin zorunlu
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Komutasyon icin zamanlayici interrupt'ini baslat
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function (not setted yet)
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  // 72MHz / (44 + 1) / (99 + 1) = 16 kHz PWM
  htim1.Init.Prescaler = 44;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0; // Her periyotta update event
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
  sConfigOC.Pulse = 0; // Baslangicta %0 duty
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH; // Complementary pin de HIGH
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  // Deadtime Ayari: 72MHz clock -> 13.88ns per tick.
  // 181 * 13.88ns = ~2.5 microsecond deadtime.
 
  sBreakDeadTimeConfig.DeadTime = 181; 
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE; // ALM pinleri icin Break aktif
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; // ALM pinleri active-low
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; // Break'ten sonra otomatik kurtarma
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  // 72MHz Timer Clock (APB1=36M, Timer x2 = 72M)
  // 10kHz interrupt (0.1ms periyot)
  // 72,000,000 / (719 + 1) / (9 + 1) = 10,000 Hz
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE8 PE9 PE10 PE11
                           PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // TIM1 pinleri (Full remap)
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // HIZLI PWM
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 (ALM PINLERI) */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Active-Low, dusen kenarda kesme
  GPIO_InitStruct.Pull = GPIO_PULLUP; // Pull-up'li giris
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD3 PD4 (HALL SENSOR PINLERI) */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING; // Cikislar hem yukselen hem de dusen kenarda kesme
  GPIO_InitStruct.Pull = GPIO_NOPULL;// harici pull up var devrede
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  //Set hall sensor pins interrupt priority and enable them
  HAL_NVIC_SetPriority(EXTI1_IRQn,1,0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI2_IRQn,1,0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn,1,0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn,1,0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_TIM1_ENABLE(); // TIM1 pinlerini PE'ye yonlendir

  /* EXTI interrupt init*/
  // ALM pinleri (PC10-12) icin EXTI15_10 hattini etkinlestir
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  IPM icin 6-adim Komutasyon fonksiyonu (Tamamlayici PWM)
  * @note   Bu fonksiyon eski koddaki GPIO'lu fonksiyonun yerini alir.
  * Logic: Bir faz PWM (duty cycle), bir faz 0V (GND), bir faz Hi-Z.
  * PWM = CCRx, GND = 0% duty, Hi-Z = Iki cikis da kapali (CCER=0)
  */
void update_sector_for_open_loop(void) {

  switch (sector) {
    //inital open loop ccr help us the set duty cyle

    case 1: // U -> V (U=PWM, V=GND, W=Hi-Z)
      // U fazi: PWM
      TIM1->CCR1 = initial_open_loop_ccr; 
      // V fazi: 0% Duty (%100 Low-side)
      TIM1->CCR2 = 0;
      // W fazi: Cikislar kapali (Hi-Z)
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | // CH1 ve CH1N aktif
                   TIM_CCER_CC2E | TIM_CCER_CC2NE;  // CH2 ve CH2N aktif
      break;
      
    case 2: // U -> W (U=PWM, W=GND, V=Hi-Z)
      TIM1->CCR1 = initial_open_loop_ccr;
      TIM1->CCR3 = 0;
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | // CH1 ve CH1N aktif
                   TIM_CCER_CC3E | TIM_CCER_CC3NE;  // CH3 ve CH3N aktif
      break;

    case 3: // V -> W (V=PWM, W=GND, U=Hi-Z)
      TIM1->CCR2 = initial_open_loop_ccr;
      TIM1->CCR3 = 0;
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | // CH2 ve CH2N aktif
                   TIM_CCER_CC3E | TIM_CCER_CC3NE;  // CH3 ve CH3N aktif
      break;

    case 4: // V -> U (V=PWM, U=GND, W=Hi-Z)
      TIM1->CCR2 = initial_open_loop_ccr;
      TIM1->CCR1 = 0;
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | // CH1 ve CH1N aktif
                   TIM_CCER_CC2E | TIM_CCER_CC2NE;  // CH2 ve CH2N aktif
      break;

    case 5: // W -> U (W=PWM, U=GND, V=Hi-Z)
      // Bu sektor alignment icin kullaniliyor.
      TIM1->CCR3 = initial_open_loop_ccr;
      TIM1->CCR1 = 0;
      TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | // CH1 ve CH1N aktif
                   TIM_CCER_CC3E | TIM_CCER_CC3NE;  // CH3 ve CH3N aktif
      break;

    case 6: // W -> V (W=PWM, V=GND, U=Hi-Z)
      TIM1->CCR3 = initial_open_loop_ccr;
      TIM1->CCR2 = 0;
      TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC2NE | // CH2 ve CH2N aktif
                   TIM_CCER_CC3E | TIM_CCER_CC3NE;  // CH3 ve CH3N aktif
      break;

    default: // Tum cikislari kapat (Hi-Z)
      TIM1->CCER = 0;
      break;
  }
}

/**
  * @brief  TIM2 Periyot Gecis Callback'i (Her 0.1ms'de bir tetiklenir)
  * @note   Eski koddaki ile BIREBIR AYNI ramp-up logigini kullanir.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {

    ms_counter += 0.1;

    sector_change_interval_counter = sector_change_interval_counter + 0.1;
    //time(1->3) // Hizalama asamasi
    if (motor_align_time_counter <= align_duration) {// motor align duration
      sector_change_interval_counter = 0;
      sector = 5; // Alignment sektoru (W -> U)
      
      // Alignment icin CCR'yi ayarla (Eskisindee bu yoktu yeni eklendi)
      initial_open_loop_ccr = motor_align_ccr_value; 
      
      motor_align_time_counter++;//1 -> 2  -> 3(reached the limit)
      if (sector > 6) {
        sector = 1;
      }
      update_sector_for_open_loop();


      } else { //Hizalama bitti 

    // Alignment sonrasi baslangic CCRsini tekrar ayarla
    if (ol_sector_counter == 0) { // Sadece ilk seferde
         initial_open_loop_ccr = 50; // soft start opening value (pwm duty cyle)
    }

    //initial=(2000/ open_loop_start_speed_rpm); 6.6s per sector 
    if ((sector_change_interval_counter)>= (initial_sector_change_interval) && (ol_sector_counter)<= (ol_revolution_limit) )

        {
      sector_change_interval_counter = 0;

      sector++;
      if (sector > 6) {
        sector = 1;
      }
      ol_sector_counter++;
      update_sector_for_open_loop();

      if (initial_open_loop_ccr <= desired_open_loop_ccr) { // duty cycle ramp
        initial_open_loop_ccr += ccr_ramp_slope;

      } else {
        initial_open_loop_ccr = desired_open_loop_ccr; // maks duty cycle
      }
      if (initial_sector_change_interval > desired_sector_change_interval) { // open_loop_velocity_rpm ramp
        initial_sector_change_interval -= rpm_ramp_slope;

      } else {
        initial_sector_change_interval = desired_sector_change_interval; //maks open_loop_velocity_rpm
      }

    }

  }


}
}

/**
  * @brief  Harici Interrupt Callback (ALM pinleri icin)
  * @param  GPIO_Pin: Tetikleyen pin
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10 || GPIO_Pin == GPIO_PIN_11 || GPIO_Pin == GPIO_PIN_12)
  {
    // HATA DURUMU (ALM pinlerinden biri LOW'a cekildi)
    
    // PWM Cikislarini Kapat
     __HAL_TIM_MOE_DISABLE(&htim1); // TUM 6 CIKISI KES
     
     // veya CCR'leri 0'la
     TIM1->CCR1 = 0;
     TIM1->CCR2 = 0;
     TIM1->CCR3 = 0;
     TIM1->CCER = 0; // Tum cikislari Hi-Z yap

    // Eror led 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); 
    
    //inifinty loop
    Error_Handler();
  }
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

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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