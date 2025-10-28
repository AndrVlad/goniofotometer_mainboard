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
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static FLASH_EraseInitTypeDef EraseInitStruct;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADDR_FLASH_SECTOR_2 ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define ENCODER_RESOLUTION 131072
#define ACCEL_OFFSET 5 // values in ark degrees
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
uint8_t buf[5] = {0x0A,0x0A,0x0A,0x0A,0x0A};
//uint8_t amplifier_val[3] = {0};
uint8_t dma_spi4_buf[5] = {0};
uint8_t dma_spi3_buf[5] = {0};
uint8_t response_buf[33] = {0};
uint8_t ampl_buf[2];
uint8_t start_ending_angle_items[2][8] = {{1,2,3,4,5,6,7,8},{180,150,120,90,60,30,10,5}};
uint16_t measurement_res_items[2][8] = {{1,2,3,4,5,6,7,8},{3600,1800,600,300,60,30,10}}; // The values are set in arc seconds.
uint16_t crc = 0;
uint8_t current_pos = 0;
uint8_t i = 0;
char str[64] = {0,};
uint32_t idata[] = {0x1941, 0x1945};

uint32_t encoder_offset[2] = {0};
uint32_t address = ADDR_FLASH_SECTOR_2;
uint8_t amplifier_val = 0;
uint8_t amplifier_val_saved = 0;
uint8_t save_code = 0;
bool wait_flag = 0;
uint8_t error_code = 0;
bool reach_start_pos = 0;

uint32_t start_position_drv1, start_position_drv2, end_position_drv1, end_position_drv2 = 0;
uint32_t ENCODER_1_OFFSET = 0;
uint32_t ENCODER_2_OFFSET = 0;
uint16_t angle_position_drv1 = 0;
uint16_t meas_res_drv1, meas_res_drv2 = 0;
uint8_t uart3_rx_buffer[6] = {0};
uint8_t uart3_rx_safe_buffer[6] = {0};
uint8_t uart1_rx_buffer[5] = {0};
uint8_t uart1_rx_safe_buffer[5] = {0};
bool uart1_rx_complete = 0;
bool uart3_rx_complete = 0;
bool driver_dir1, driver_dir2, chosen_drv = 1; // 0 - forward, 1 - back

/* Telemetry status values */
enum status { ERROR_, READY_, BUSY_ } ready_status;
enum action { NONE, HORIZONTAL, VERTICAL, HEMISPHERE, LIGHT_POWER, CALIBRATION,
				TEST_TURN, TEST_ROTATION, TEST_ANGLE_OFFSET } cur_action;
enum response_status { ERROR__, ACCEPTED__, ALREADY_EXEC, EXEC_OTHER};
bool trans_states = 0; // 0 - no trans_state, 1 - trans_state
enum data { NONE_, _READY_, SOME_PACKETS} data_status;
enum horiz_platform { HORIZONTAL_, VERTICAL_} current_horiz_platform = HORIZONTAL_;
uint8_t operation_progress = 0;
uint32_t SSI_data, SSI_data_safe, encoder1_data, encoder2_data = 0;
uint32_t adc_value = 0;
uint8_t motor_frequency_1, motor_frequency_2 = 0;

// flash values
uint32_t page_error = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
void parser();
void stepDriver(uint8_t step_num);
void createResponsePacket(uint8_t command_code, uint8_t status_code);
void moveToPosition(uint8_t angle, bool chosen_drv);
void changeMotorDirection(bool chosen_drv, uint32_t target_position);
void changeMotorDirection_(bool chosen_drv, uint32_t target_position);
uint32_t processSSIData(uint8_t *SSI_buffer);
uint32_t calculateEncPosition(uint32_t encoder_position, bool chosen_encoder);
void clearBuffer(uint8_t *buf, uint8_t size);
uint8_t getADCAmplifierVal(uint8_t value);


void FlashInit();
void WriteToFlash_();
void WriteToFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_program);
void ReadFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_read);
void ReadFlash_();
void usDelay(uint16_t useconds);
void completeReceivePhotodetector();
void createErrorResponse();
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
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Stop_IT(&htim2);
  HAL_TIM_Base_Start(&htim6);

  // start receiving of messages from PC
  HAL_Delay(1000);
  HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 6);
  //HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);


  // start receiving of encoder values
  HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
  HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);

  // Read encoder offset values from flash
  FlashInit();
  ReadFlash(encoder_offset,2,address,FLASH_TYPEPROGRAM_WORD);
  ENCODER_1_OFFSET = encoder_offset[0];
  ENCODER_2_OFFSET = encoder_offset[1];

  ready_status = READY_;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // handle of message from PC
	  if (uart3_rx_complete) {
	  	  	parser();
	  }
	  // handle of message from Photodetector
	  if (uart1_rx_complete) {

		  uart1_rx_complete = 0;
          completeReceivePhotodetector();


	  		  //parser_photodetector();
	  }
	  //HAL_Delay(1000);

	 //printf("USART1 Error: 0x%lX\n", huart1.ErrorCode);
	 //printf("USART1 SR: 0x%lX\n", huart1.Instance->ISR);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  htim2.Init.Prescaler = 107;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 7999;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 107;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;
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
  htim6.Init.Prescaler = 107;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 107;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 10799;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 7999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ORE);
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void parser() {
	switch (uart3_rx_buffer[0]) {
	case 0x01:

		HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
		HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
		// poll photodetector
		//HAL_UART_DMAStop(&huart1);
		//HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 3);

		if (wait_flag == 0) {

			HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			//HAL_Delay(1);
			usDelay(100);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

		}

		//clearBuffer(response_buf,33);
		createResponsePacket(0x01,0);
		//HAL_UART_Transmit(&huart3, response_buf,33,100);
		break;
	case 0x02:

		createResponsePacket(0x02,ACCEPTED__);

		// set current action
		cur_action = HORIZONTAL;

		// choosing a platform
		chosen_drv = current_horiz_platform;

		int16_t accel_position;

		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);

		accel_position = start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];

		// calculate acceleration offset position

		/*
		if ((accel_position - ACCEL_OFFSET) < 0) {

		} */


		if (chosen_drv == HORIZONTAL_) {

			// moving to acceleration offset position
			moveToPosition(accel_position-ACCEL_OFFSET, chosen_drv);

			// set start position of measurement
			start_position_drv1 = start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
			start_position_drv1 = (start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1] * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);

			// set end position of measurement
			end_position_drv1 = start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1];
			end_position_drv1 = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1] * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			end_position_drv1 = calculateEncPosition(end_position_drv1,chosen_drv);

			/* choose of measurement resolution  */
			meas_res_drv1 = 183; // 0.5 degree

			//measurement_res_items = uart3_rx_safe_buffer[];

			// reset flag of reaching start position
			reach_start_pos = 0;

			// start measurement
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2); // start first motor moving

		} else {

			// set start position of measurement
			start_position_drv2 = start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
			start_position_drv2 = (start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1] * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv2 = calculateEncPosition(start_position_drv2,chosen_drv);

			// set end position of measurement
			end_position_drv2 = start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1];
			end_position_drv2 = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1] * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			end_position_drv2 = calculateEncPosition(end_position_drv2,chosen_drv);

			/* choose of measurement resolution  */
			meas_res_drv2 = 183; // 0.5 degree

			//measurement_res_items = uart3_rx_safe_buffer[];

			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim3); // start second motor moving
		}


		break;
	case 0x08:
		createResponsePacket(0x08,ACCEPTED__);
		if(uart3_rx_buffer[3] != 0) { // move vertical driver
			chosen_drv = 1;
		    uint32_t angle_position_drv2 = 0;
			angle_position_drv2 = uart3_rx_buffer[1] << 8;
			angle_position_drv2 |= uart3_rx_buffer[2];

			start_position_drv2 = (angle_position_drv2 * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv2 = calculateEncPosition(start_position_drv2,chosen_drv);

			changeMotorDirection(chosen_drv, start_position_drv2);

		} else { // move horizontal driver

			chosen_drv = 0;
			angle_position_drv1 = 0;
			angle_position_drv1 = uart3_rx_buffer[1] << 8;
			angle_position_drv1 |= uart3_rx_buffer[2];

			start_position_drv1 = (angle_position_drv1 * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);

			changeMotorDirection(chosen_drv, start_position_drv1);

		}

		break;
	case 0x09:

		if (!chosen_drv) {
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2); // start horizontal motor moving
		} else {
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim3); // start vertical motor moving
		}

		break;
	case 0x0A:
		createResponsePacket(0xA,ACCEPTED__);
		if(uart3_rx_buffer[3] != 0) {
			current_horiz_platform = VERTICAL_;
		} else {
			current_horiz_platform = HORIZONTAL_;
		}
		break;
	case 0x0C:
		createErrorResponse();
		ready_status = READY_;
		error_code = 0;
		break;
	case 0x0D:

		break;
	case 0x0E:
		if (uart3_rx_buffer[2]) {
			driver_dir1 = 0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		} else {
			driver_dir1 = 1;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		}
		/*
		if (uart3_rx_buffer[3]) {
			chosen_drv = 1;
		} else {
			chosen_drv = 0;
		} */

		stepDriver(uart3_rx_buffer[1]);
		break;
	case 0x0F:

		/*
		start_position_drv1 = 0;
		start_position_drv1 |= uart1_rx_buffer[1] << 16;
		start_position_drv1 |= uart1_rx_buffer[2] << 8;
		start_position_drv1 |= uart1_rx_buffer[3];
		*/

		/*
		if (start_position_drv1 < SSI_data_safe) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // движение назад
			driver_dir1 = 1;
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // движение вперед
			driver_dir1 = 0;
		}
		*/

		if(chosen_drv) {
			//HAL_TIM_Base_Start_IT(&htim3); // start second motor
		} else {
			//changeMotorDirection(chosen_drv, start_position_drv1);
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2); // start first motor
		}

		//HAL_TIM_Base_Start_IT(&htim2);
		break;

	case 0x10: // move to the specified angle
		createResponsePacket(0x10,ACCEPTED__);
		if(chosen_drv) {

			uint32_t angle_position_drv2 = 0;
			angle_position_drv2 = uart3_rx_buffer[1] << 8;
			angle_position_drv2 |= uart3_rx_buffer[2];

			if(!(uart3_rx_buffer[3])) { // absolute moving
				start_position_drv2 = (angle_position_drv2 * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				start_position_drv2 = calculateEncPosition(start_position_drv2,chosen_drv);
			}

			changeMotorDirection(chosen_drv, start_position_drv2);
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim3); // start second motor moving
      
		} else {

			angle_position_drv1 = 0;
			angle_position_drv1 = uart3_rx_buffer[1] << 8;
			angle_position_drv1 |= uart3_rx_buffer[2];

			if(!(uart3_rx_buffer[3])) { // absolute moving
				start_position_drv1 = (angle_position_drv1 * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);
			}

			changeMotorDirection(chosen_drv, start_position_drv1);
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2); // start first motor moving
		}
		break;

	case 0x11: // stop moving
		createResponsePacket(0x11,ACCEPTED__);
		//HAL_UART_Transmit(&huart3, response_buf,33,100);
		if(chosen_drv) {
			HAL_TIM_Base_Stop_IT(&htim3); // stop second motor
			HAL_TIM_Base_Stop_IT(&htim7);
		} else {
			HAL_TIM_Base_Stop_IT(&htim2); // stop first motor
			HAL_TIM_Base_Stop_IT(&htim7);
		}

		break;
	case 0x12:
		createResponsePacket(0x12,ACCEPTED__);
		if(uart3_rx_buffer[1] != 0) {
			chosen_drv = 1; // second motor
		} else {
			chosen_drv = 0; // first motor
		}
		break;
	case 0x13:
		createResponsePacket(0x13,ACCEPTED__);

		/* UNUSED
		// get current first encoder data
		//HAL_SPI_Receive(&hspi4, dma_spi4_buf, 5,100);
		// get current second encoder data
		//HAL_SPI_Receive(&hspi3, dma_spi3_buf, 5,100);
		UNUSED */

		// Write new values of encoder offset
		ENCODER_1_OFFSET = encoder1_data;
		ENCODER_2_OFFSET = encoder2_data;

		// Write new values of encoder offset for saving to Flash
		encoder_offset[0] = ENCODER_1_OFFSET;
		encoder_offset[1] = ENCODER_2_OFFSET;

		// Save data to flash
		WriteToFlash(encoder_offset, 2, address, FLASH_TYPEPROGRAM_WORD);

		break;
	case 0x14:

		// send response packet
		createResponsePacket(0x14,ACCEPTED__);
		wait_flag = 1;

		ampl_buf[0] = getADCAmplifierVal(uart3_rx_buffer[1]);
		if (uart3_rx_buffer[2] == 1) {
			ampl_buf[0] |= 0b10001000;
		}
		ampl_buf[1] = ampl_buf[0];
		//__disable_irq();
		//HAL_DMA_Abort(&hdma_usart1_rx);  // или hdma_usartx_rx
		// Сбрасываем счетчик
		//__HAL_DMA_SET_COUNTER(&hdma_usart1_rx, 0);
		//HAL_UART_DMAStop(&huart1);
		//HAL_UART_Transmit_DMA(&huart1, amplifier_val, 3);
		//__enable_irq();
		//memcpy(uart1_rx_safe_buffer, uart1_rx_buffer, 3);
		//HAL_UART_DMAStop(&huart1);
		HAL_UART_DMAStop(&huart1);
		HAL_UART_Transmit(&huart1, ampl_buf, 1,100);
		/* this string fixed bug early */
		//HAL_UART_Receive_IT(&huart1, buf, 5);
		HAL_UART_Receive_DMA(&huart1, buf, 5);
		//HAL_TIM_Base_Start_IT(&htim10);

		//usDelay(700);
		//wait_flag = 0;
		//HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 3);
		//hdma_usart1_rx.Instance->NDTR = 0;


		/*
		if (!(ampl_buf[0] == (amplifier_val[0] >> 4))) {
			// handle of error;
		} */

		break;
	case 0x15:
		HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 0x17:
		ReadFlash(encoder_offset,2,address,FLASH_TYPEPROGRAM_WORD);
		 HAL_UART_DeInit(&huart1);
		  MX_USART1_UART_Init();
		break;
	case 0x18: // only for test of mainboard

		if(!chosen_drv) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			for (i = 0; i < 400; i++) {

				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
				createResponsePacket(0x01,0);

			}
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
			for (i = 0; i < 400; i++) {

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
				HAL_Delay(100);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
				createResponsePacket(0x01,0);
			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		}

		break;
	case 0x19: // get current value of encoder
		if (!chosen_drv) {
			HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
			HAL_UART_Transmit(&huart3,dma_spi4_buf,5,100);
		} else {
			HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
			HAL_UART_Transmit(&huart3,dma_spi3_buf,5,100);
		}

		break;
}

	uart3_rx_complete = 0;
}

void stepDriver(uint8_t step_num) {

	if(!chosen_drv) {
		for (int i = 0; i < step_num; i++) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			HAL_Delay(2);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_Delay(2);
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			//HAL_Delay(4);
			HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
			//HAL_SPI_Receive(&hspi4, dma_spi4_buf, 5,100);
			//HAL_UART_Transmit(&huart1,buf,5,100);
			//HAL_SPI_TransmitReceive_DMA(&hspi4, buf, buf, 5);
			HAL_UART_Transmit(&huart3,dma_spi4_buf,5,100);

		}
	} else {
		for (int i = 0; i < step_num; i++) {
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
			//HAL_Delay(2);
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
			HAL_Delay(4);
			HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
			//HAL_SPI_Receive(&hspi4, dma_spi4_buf, 5,100);
			//HAL_UART_Transmit(&huart1,buf,5,100);
			//HAL_SPI_TransmitReceive_DMA(&hspi4, buf, buf, 5);
			HAL_UART_Transmit(&huart3,dma_spi3_buf,5,100);
		}
	}

	current_pos = 0;
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

}

void changeMotorDirection_(bool chosen_drv, uint32_t target_position) {

	if (!chosen_drv) { // first motor
		if (target_position < encoder1_data) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // moving back
			driver_dir1 = 1;
		} else {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // moving forward
			driver_dir1 = 0;
		}
	} else { // second motor
		if (target_position < encoder2_data) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // moving back
			driver_dir2 = 1;
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // moving forward
			driver_dir2 = 0;
		}
	}
	/*
	if (start_position_drv1 < SSI_data_safe) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // движение назад
		driver_dir1 = 1;
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // движение вперед
		driver_dir1 = 0;
	} */
}

void changeMotorDirection(bool chosen_drv, uint32_t target_position) {

	if (!chosen_drv) { // first motor

		if (target_position < encoder1_data) {
			if ((encoder1_data - target_position) > (ENCODER_RESOLUTION - encoder1_data + target_position)) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // moving forward
				driver_dir1 = 0;
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // moving back
				driver_dir1 = 1;
			}
		} else {
			if ((ENCODER_RESOLUTION - target_position + encoder1_data) < (target_position - encoder1_data)) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // moving back
				driver_dir1 = 1;
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); // moving forward
				driver_dir1 = 0;
			}
		}

	} else { // second motor

		if (target_position < encoder2_data) {
			if ((encoder2_data - target_position) > (ENCODER_RESOLUTION - encoder2_data + target_position)) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // moving forward
				driver_dir2 = 0;
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // moving back
				driver_dir2 = 1;
			}
		} else {
			if ((ENCODER_RESOLUTION - target_position + encoder2_data) < (target_position - encoder2_data)) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // moving back
				driver_dir2 = 1;
			} else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // moving forward
				driver_dir2 = 0;
			}
		}
	}
}

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	//HAL_UART_Transmit(&huart1, UART1_rxBuffer, 12, 100);
	    //HAL_UART_Transmit(&huart1, uart1_rx_buffer,2,100);
		if (huart->Instance == USART3) {
			uart3_rx_complete = 1;

		}

		if (huart->Instance == USART1) {

			// if the coefficient of amplifier of photodetector was set
			if (wait_flag == 1) {

				// CRC calculation
				uint32_t CRC_Photodetector = 0;
				CRC_Photodetector = buf[0] + buf[1] + buf[2];
				CRC_Photodetector = CRC_Photodetector & 0xFF;

				// Check CRC
				if (!((CRC_Photodetector == buf[3]) && (buf[4] == 0xA5))) {
					ready_status = ERROR_;
				} else {
					// Check response of photodetector
					if (buf[0] != (ampl_buf[0] >> 4)) {
						// handle of error
						ready_status = ERROR_;
						error_code = 0x01;
					}
				}
				/*
				if (buf[0] != (ampl_buf[0] >> 4)) {
					// handle of error
					ready_status = ERROR_;
				}


				if(huart->RxXferCount != 0) {
					HAL_UART_Receive_IT(&huart1, uart1_rx_buffer,5);
					return;
				}
				*/
				wait_flag = 0;
				//HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer,5);
			}

			// other cases
			uart1_rx_complete = 1;
			//createResponsePacket(0x01,0);
		}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// first encoder
	if (hspi->Instance == SPI4) {
		encoder1_data  =  (dma_spi4_buf[2] >> 5) & 0x07;
		encoder1_data |=  ((uint32_t)dma_spi4_buf[1] << 3);
		encoder1_data |=  (((uint32_t)dma_spi4_buf[0] & 0x3F) << 11);

		if (cur_action == HORIZONTAL || cur_action == VERTICAL) {

			if (reach_start_pos == 1) {
				if (encoder1_data >= end_position_drv1) {
					cur_action = NONE;
					reach_start_pos = 0;
					// stop measurement
					HAL_TIM_Base_Stop_IT(&htim2); // stop motor
					HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer
				} else {
					HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
					usDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
				}
			} else {

				if (encoder1_data >= start_position_drv1) {
					reach_start_pos = 1;
					// start poll photodetector
					HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
					usDelay(100);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
				}
			}

		} else {

			if ((encoder1_data >= start_position_drv1 - 1) && (encoder1_data <= start_position_drv1 + 1)) {
				HAL_TIM_Base_Stop_IT(&htim2); // stop motor
				HAL_TIM_Base_Stop_IT(&htim7);
				//HAL_SPI_DMAStop(&hspi4);
				//HAL_UART_Transmit(&huart3,dma_spi4_buf,5,100);
			}
		}
	}

	// second encoder
	if (hspi->Instance == SPI3) {
		encoder2_data  =  (dma_spi3_buf[2] >> 5) & 0x07;
		encoder2_data |=  ((uint32_t)dma_spi3_buf[1] << 3);
		encoder2_data |=  (((uint32_t)dma_spi3_buf[0] & 0x3F) << 11);

			if ((encoder2_data >= start_position_drv2 - 1) && (encoder2_data <= start_position_drv2 + 1)) {
				HAL_TIM_Base_Stop_IT(&htim3);
				HAL_TIM_Base_Stop_IT(&htim7);
			}
		}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// timer for the step
	if(htim->Instance == TIM2) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
	}

	if(htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
	}

	// timer for the delay between SPI request to encoder
	if (htim->Instance == TIM7) {
		if (!chosen_drv) {
			HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
		} else {
			HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
		}

		__HAL_TIM_SET_COUNTER(&htim7, 0);
	}

	// timer for checking response of photodetector
	if (htim->Instance == TIM10) {
			//wait_flag = 0;
			//HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
			//HAL_UART_DMAStop(&huart1);
			//hdma_usart1_rx.Instance->NDTR = 3;

			// ! create handle of error
			HAL_TIM_Base_Stop(&htim10);
			__HAL_TIM_SET_COUNTER(&htim10, 0);
		}
}

void createResponsePacket(uint8_t command_code, uint8_t status_code) {
	crc=0;
	if (command_code == 0x01) {
		response_buf[0] = 0x01;
		response_buf[1] = (uint8_t)ready_status;
		response_buf[2] = (uint8_t)cur_action;
		response_buf[3] = trans_states;
		response_buf[4] = (uint8_t)data_status;
		response_buf[5] = operation_progress;
		response_buf[6] = uart1_rx_safe_buffer[0];//(adc_value >> 16) & 0xFF;
		response_buf[7] = uart1_rx_safe_buffer[1];//(adc_value >> 8) & 0xFF;
		response_buf[8] = uart1_rx_safe_buffer[2];//adc_value & 0x000000FF;
		response_buf[9] = encoder1_data & 0xFF;
		response_buf[10] = encoder1_data >> 8;
		response_buf[11] = encoder1_data >> 16;
		response_buf[15] = encoder2_data & 0xFF;
		response_buf[16] = encoder2_data >> 8;
		response_buf[17] =  encoder2_data >> 16;
		response_buf[31] = 0;
		response_buf[32] = 0;

		// CRC calculation
		for (int i = 0; i < 30; i+=2) {
			crc += (uint16_t)response_buf[i] + ((uint16_t)(response_buf[i+1])<<8);
		}
		crc += response_buf[30];
		*(uint16_t*)(response_buf+31) = crc;

	} else {
		clearBuffer(response_buf,33);

		// CRC calculation
		response_buf[0] = command_code;
		response_buf[1] = status_code;
		response_buf[31] = command_code;
		response_buf[32] = status_code;

	}

	HAL_UART_Transmit(&huart3, response_buf,33,100);

}

uint32_t calculateEncPosition(uint32_t encoder_position, bool chosen_encoder) {
	if (chosen_encoder) { // chosen vertical platform
		if((encoder_position + ENCODER_2_OFFSET) > ENCODER_RESOLUTION) {
			return (encoder_position + ENCODER_2_OFFSET) - ENCODER_RESOLUTION;
		} else {
			return encoder_position + ENCODER_2_OFFSET;
		}
	} else { // chosen horizontal platform
		if((encoder_position + ENCODER_1_OFFSET) > ENCODER_RESOLUTION) {
			return (encoder_position + ENCODER_1_OFFSET) - ENCODER_RESOLUTION;
		} else {
			return encoder_position + ENCODER_1_OFFSET;
		}
	}
}

uint32_t processSSIData(uint8_t *SSI_buffer) {
	uint32_t encoder_data = 0;
	encoder_data  =  (SSI_buffer[2] >> 5) & 0x07;
	encoder_data |=  ((uint32_t)SSI_buffer[1] << 3);
	encoder_data |=  (((uint32_t)SSI_buffer[0] & 0x3F) << 11);
	return encoder_data;
}

void FlashInit() {
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = FLASH_SECTOR_3;
	EraseInitStruct.NbSectors     = 1;
}

void WriteToFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_program) {

	HAL_FLASH_Unlock();

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &page_error) != HAL_OK) {
	      //error handler of erasing flash
	      return;
	  }

	  uint8_t address_inc = 0;

	  if (type_of_program == FLASH_TYPEPROGRAM_WORD) {
		  address_inc = 4;
	  } else if (type_of_program == FLASH_TYPEPROGRAM_HALFWORD) {
		  address_inc = 2;
	  }

	  for(uint8_t i = 0; i < data_size; i++)
	  {
		  if(HAL_FLASH_Program(type_of_program, address, data[i]) != HAL_OK) {
	            // error handler of programming flash
	            return;
	      }
	           address += address_inc;
	   }

	   HAL_FLASH_Lock();
}

void ReadFlash(uint32_t *data, uint8_t data_size, uint32_t address, uint32_t type_of_read) {

	uint8_t address_inc = 0;
	uint32_t dig32 = 0;
	if (type_of_read == FLASH_TYPEPROGRAM_WORD) {
	    address_inc = 4;
	} else if (type_of_read == FLASH_TYPEPROGRAM_HALFWORD) {
		address_inc = 2;
	}

	for(uint16_t i = 0; i < data_size; i++) {

		dig32 = *(uint32_t*)address;
		data[i] = dig32;
	    address += address_inc;
	  }
}

void clearBuffer(uint8_t *buf, uint8_t size){
	for(uint8_t i = 0; i < size; i++) {
		buf[i] = 0;
	}
}

uint8_t getADCAmplifierVal(uint8_t value) {

	switch(value) {
		case 1:
			return 0;
		case 2:
			return 0b00010001;
		case 4:
			return 0b00100010;
		case 8:
			return 0b00110011;
		case 16:
			return 0b01000100;
		case 32:
			return 0b01010101;
		case 64:
			return 0b01100110;
		case 128:
			return 0b01110111;
		default:
			return 1;
	}
}

void usDelay(uint16_t useconds)
{
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  while(__HAL_TIM_GET_COUNTER(&htim6) < useconds);
}

void moveToPosition(uint8_t angle, bool chosen_drv) {
	if(chosen_drv) {

		start_position_drv2 = (angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
		start_position_drv2 = calculateEncPosition(start_position_drv2,chosen_drv);

		changeMotorDirection(chosen_drv, start_position_drv2);
		HAL_TIM_Base_Start_IT(&htim7);
		HAL_TIM_Base_Start_IT(&htim3); // start second motor moving

	} else {

		start_position_drv1 = (angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
		start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);

		changeMotorDirection(chosen_drv, start_position_drv1);
		HAL_TIM_Base_Start_IT(&htim7);
		HAL_TIM_Base_Start_IT(&htim2); // start first motor moving
	}
}


void completeReceivePhotodetector() {
  uint32_t CRC_Photodetector = 0;
  //calculate CRC
  CRC_Photodetector = uart1_rx_buffer[0] + uart1_rx_buffer[1] + uart1_rx_buffer[2];
  CRC_Photodetector = CRC_Photodetector & 0xFF;
  // check CRC
  if ((CRC_Photodetector == uart1_rx_buffer[3]) && (uart1_rx_buffer[4] == 0xA5)) {
    uart1_rx_safe_buffer[0] = uart1_rx_buffer[0];
    uart1_rx_safe_buffer[1] = uart1_rx_buffer[1];
    uart1_rx_safe_buffer[2] = uart1_rx_buffer[2];
  } else {
    // error handler
  }
}

void createErrorResponse() {
	clearBuffer(response_buf,33);
	// CRC calculation
	response_buf[0] = 0x0C;
	response_buf[1] = error_code;

	response_buf[31] = 0x0C;
	response_buf[32] = error_code;

	HAL_UART_Transmit(&huart3, response_buf,33,100);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {

        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
        // clear RXNE flag
        volatile uint8_t data = huart1.Instance->RDR;

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
