/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

#define ADC_SAMPLES_PER_CH 50
#define TOTAL_ADC_BUFFER (ADC_SAMPLES_PER_CH * 4)
#define WAVE_POINTS 100

// 馃煝 淇锛欰DC鏄?2浣嶇殑锛岀‖楂?DMA 鎼亱蹇呴爤鐢?uint16_t 鎺ユ敹锛岄槻姝㈣鎲堕珨韪╄笍婧㈠嚭锛?
uint16_t adc_buffer[TOTAL_ADC_BUFFER];

// PWM DMA 娉㈠舰绶╄鍗€
uint16_t pwm_wave_buffer[WAVE_POINTS];

// USB 鐧奸€佽▕妗嗙珐琛濆崁 (瑷婃闋?浣嶅厓绲?+ 闀峰害2浣嶅厓绲?+ 鏁告摎)
uint8_t usb_tx_buffer[(TOTAL_ADC_BUFFER * 2) + 5];
volatile uint8_t data_ready_flag = 0; // 馃煝 淇锛氬姞涓?volatile 纰轰繚涓柗鑸囦富寰挵鍚屾

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 鈿?鏍稿績鍔熻兘 1锛氭牴鎹笂浣嶆満鎸囦护锛屽姩鎬佽绠楀苟鏇存柊 PWM DMA 鏁扮粍
void Setup_Waveform(uint8_t wave_type, uint16_t freq, uint8_t vpp) {
    float amplitude = vpp / 2.0f; // Vpp 鏄?0-255锛屽搴?0-3.3V
    float offset = 127.5f;        // 1.65V 鐩存祦鍋忕疆涓績鐐?
    
    // 馃煝 鏍稿績淇锛氫慨姝ｉ鐜囧叕寮忓苟闃叉鏁村瀷涓嬫孩锛?
    // 鐞嗚鍊? PSC + 1 = 72MHz / (256 * 100鐐? / freq = 2812.5 / freq
    uint32_t psc = 0;
    uint32_t target_hz = (freq > 0) ? freq : 1; 
    uint32_t calc_psc = 2812 / target_hz; 
    
    if (calc_psc > 0) {
        psc = calc_psc - 1;
    } else {
        psc = 0; // 棰戠巼杩囬珮鏃讹紝灏嗗叾闄愬埗鍦ㄧ‖浠惰兘杈撳嚭鐨勬渶楂橀鐜囷紙绾?.8kHz锛?
    }
    
    if (psc > 65535) psc = 65535; 
    
    __HAL_TIM_SET_PRESCALER(&htim3, psc);
    htim3.Instance->EGR = TIM_EGR_UG; // 馃煝 鍏抽敭锛氬己鍒朵骇鐢熸洿鏂颁簨浠讹紝璁╂柊鐨勫垎棰戠郴鏁扮珛鍒荤敓鏁堬紝閬垮厤鍗℃
    
    for (int i = 0; i < WAVE_POINTS; i++) {
        float t = (float)i / WAVE_POINTS;
        float val = 0;
        
        switch (wave_type) {
            case 0: // 姝ｅ鸡娉?
                val = offset + amplitude * sin(2 * M_PI * t);
                break;
            case 1: // 鏂规尝
                val = offset + amplitude * (t < 0.5f ? 1.0f : -1.0f);
                break;
            case 2: // 涓夎娉?
                val = offset + amplitude * (2.0f * fabs(2.0f * t - 1.0f) - 1.0f);
                break;
            case 3: // 閿娇娉?
                val = offset + amplitude * (2.0f * t - 1.0f);
                break;
            default: // 榛樿鐩存祦
                val = offset;
        }
        
        // 闄愬埗鍦?0-255 (瀵瑰簲 TIM3 鐨?CCR 鍊?
        if(val > 255) val = 255;
        if(val < 0) val = 0;
        pwm_wave_buffer[i] = (uint16_t)val;
    }
}

// 鈿?鏍稿績鍔熻兘 2锛欰DC DMA 閲囬泦瀹屾垚涓柇鍥炶皟 (鎵撳寘鍙戠粰涓婁綅鏈?
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        data_ready_flag = 1; // 鏍囧織浣嶇疆 1锛岃涓诲惊鐜鐞?
    }
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  // 1. 鍟熷嫊闋愯ō娉㈠舰
  Setup_Waveform(0, 1000, 255);
 // 馃煝 鏀逛负锛氬惎鍔ㄦ爣鍑嗙殑纭欢 PWM 杈撳嚭
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  // 馃煝 鏀逛负锛氭墜鍔ㄥ紑鍚?TIM3 鐨勫叏灞€涓柇鍜?NVIC锛堣繖鏍蜂綘涓嶉渶瑕侀噸鏂版墦寮€ CubeMX 鍘诲嬀閫変腑鏂級
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_TIM_Base_Start_IT(&htim3); // 寮€鍚畾鏃跺櫒鏇存柊锛堟孩鍑猴級涓柇
  
  // 2. 寤烘 USB 閫氳▕鍥哄畾褰辨牸闋?
  usb_tx_buffer[0] = 0xAA;
  usb_tx_buffer[1] = 0x55;
  
  // 馃煝 淇锛氱櫦閫佺殑瀵﹂殯浣嶅厓绲勬暩鏄?鎺℃ǎ榛炴暩 * 2 (400 浣嶅厓绲?
  uint16_t tx_bytes_len = TOTAL_ADC_BUFFER;
  usb_tx_buffer[2] = (tx_bytes_len >> 8) & 0xFF; // Len H
  usb_tx_buffer[3] = tx_bytes_len & 0xFF;        // Len L

  // 3. 鍟熷嫊瀹氭檪鍣ㄨ垏 ADC DMA 鎺冩弿锛堝彧鍛煎彨涓€娆★級
  HAL_TIM_Base_Start(&htim3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, TOTAL_ADC_BUFFER);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 馃煝 淇锛氬湪涓诲惊鐠颁腑铏曠悊璩囨枡鐧奸€?
    if (data_ready_flag)
    {
      data_ready_flag = 0; // 娓呴櫎鏃楁
      
      // 馃煝 淇锛氶厤鍚?Python 涓婁綅鏈哄崗璁紝鍙彂閫?8 浣嶆暟鎹紙灏?12 浣?ADC 鍙崇Щ 4 浣嶅彇楂?8 浣嶏級
      int tx_idx = 4; 
      for (int i = 0; i < TOTAL_ADC_BUFFER; i++)
      {
        usb_tx_buffer[tx_idx++] = (adc_buffer[i] >> 4) & 0xFF; 
      }
      
      usb_tx_buffer[tx_idx] = 0x0D; // 鍙€夛細缁撴潫鐮?
      
      // 馃煝 淇锛氬彂閫侀暱搴︽仮澶嶄负鍘熷鐐规暟 + 5 涓崗璁ご灏惧瓧鑺?
      CDC_Transmit_FS(usb_tx_buffer, TOTAL_ADC_BUFFER + 5);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;       // 鍟熺敤鎺冩弿妯″紡
  hadc1.Init.ContinuousConvMode = ENABLE;          // 閫ｇ簩杞夋彌
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;                  // 杞夋彌閫氶亾鏁哥偤 4
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** 馃煝 淇锛氶厤缃?RANK 1 -> Channel 0 (PA0) */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  /** 馃煝 淇锛氶厤缃?RANK 2 -> Channel 1 (PA1) */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  /** 馃煝 淇锛氶厤缃?RANK 3 -> Channel 2 (PA2) */
  sConfig.Channel = ADC_CHANNEL_2; // 杩欓噷鍘熷厛鍐欓敊浜嗭紝鍐欐垚浜?ADC_CHANNEL_3
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

  /** 馃煝 淇锛氶厤缃?RANK 4 -> Channel 3 (PA3) */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 馃煝 淇锛氭墜鍔ㄨˉ鍏?TIM3 涓柇鏈嶅姟鍑芥暟锛岀粫杩?CubeMX 鑷姩鐢熸垚鐨勯檺鍒?
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

// 馃煝 淇锛氬畾鏃跺櫒姣忓畬鎴愪竴娆″畬鏁寸殑 PWM 鍛ㄦ湡锛堟暟瀹?56涓偣婧㈠嚭锛夛紝瑙﹀彂璇ュ嚱鏁颁竴娆?
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        static uint16_t wave_idx = 0;
        
        // 姣忎竴涓?PWM 鍛ㄦ湡锛岀簿鍑嗐€佸畨鍏ㄥ湴灏嗕笅涓€涓尝褰㈢偣鐨勭數鍘嬪€煎啓鍏ユ帶鍒跺瘎瀛樺櫒
        TIM3->CCR1 = pwm_wave_buffer[wave_idx++];
        
        if (wave_idx >= WAVE_POINTS)
        {
            wave_idx = 0; // 鎾斁瀹屼竴杞尝褰紝鍥炲埌璧风偣寰幆
        }
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
