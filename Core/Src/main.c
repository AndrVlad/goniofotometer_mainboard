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
#include "FlashUtils.h"
#include "HardwareUtils.h"
#include "Common.h"
#include "Platform.h"
#include "Utils.h"
#include "PlatformMeasurements.h"
#include "PD_Communication.h"
#include "PC_Communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#define ENCODER_RESOLUTION 131072
//#define ACCEL_OFFSET 5 // values in ark degrees
//#define POSITION_ERROR 92
#define VERTICAL_ROTATION_ANGLE 180
//#define ENCODER_TOLERANCE 46
//#define AHB1_TIMER_CLOCK_MHz 108
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
uint8_t buf[5] = {0x0A,0x0A,0x0A,0x0A,0x0A};
//uint8_t amplifier_val[3] = {0};
uint8_t dma_spi4_buf[5] = {0};
uint8_t dma_spi3_buf[5] = {0};
uint8_t response_buf[33] = {0};
//uint8_t adc_data_buf[33] = {0};
//uint8_t data_buf_counter = 0;
//uint8_t data_elem_cnt = 1;
uint8_t data_elem_cnt_calib = 0;
uint16_t tim14_arr_val = 0; 
uint16_t test_counter_adc_data, test_cnt_uart1_rx, uart1_received_cnt, uart1_received_cnt_global, take_data_cnt = 0;
uint16_t test_counter_adc_data2 = 0;
uint16_t busy_cnt, tim13_ovflw = 0;
uint8_t ampl_buf[2];
uint8_t start_ending_angle_items[2][8] = {{1,2,3,4,5,6,7,8},{180,150,120,90,60,30,10,5}};
uint16_t measurement_res_items[2][9] = {{1,2,3,4,5,6,7,8},{365,182,60,29,6,3,1,1820}}; // The values are set in arc seconds.
uint32_t light_pow_period_items[9] = {36001000,18001000,6001000,601000,301000,101000,10000,5000,1000}; // values for TIMER_5 ARR
uint16_t light_pow_res_items[12] = {50000,25000,10000,5000,2500,1000,500,250,100,50,25,10}; // values for TIMER ARR
uint16_t crc, packet_cnt, error_val = 0;
uint8_t current_pos = 0;
uint8_t i = 0;
char str[64] = {0,};
//uint32_t idata[] = {0x1941, 0x1945};
uint32_t CRC_Photodetector, tim13cnt, tim14_arr_val_converted, new_tim_arr_val, new_arr_val = 0;
uint32_t adc_data_cnt, required_data_num = 0;
uint32_t usart3_reg, usart3_error = 0;
uint32_t photodetector_offset_val = 0;
uint32_t control_pos = 0;
uint32_t inv_zero_limb_pos[2] = {0};

uint32_t encoder_offset[2] = {0};
uint32_t zero_limb_pos[2] = {0};
//uint32_t address = ADDR_FLASH_SECTOR_2;
uint8_t amplifier_val = 0;
uint8_t amplifier_val_saved = 0;
uint8_t save_code = 0;
//bool wait_flag = 0;
uint8_t error_code = 0;
bool reach_start_position = 0;
bool reach_end_position = 0;
bool reach_accel_position = 0;
bool step_1_vertical_meas = 0;
bool step_2_vertical_meas = 0;
bool reach_start_position_vertical = 0;
bool end_meas_flag = 0;
//bool wait_adc_data_flag = 0;
uint16_t start_angle_offset_1 = 0, start_angle_offset_2 = 0;
bool reducing_pos_calc = 0;
bool reach_test_turn_pos = 0;
bool start_light_pow_meas = 0;
bool adc_coeff_command_set = 0;
bool adc_coeff_set_complete = 0;
bool tim14_cnt, tim10_cnt = 0;
bool allow = 0;
//bool stop_poll = 0;
bool end_calibration_flag = 0;
bool full_rotation = 0;
uint32_t inv_encoder1_data, inv_encoder2_data, inv_encoder1_offset, inv_encoder2_offset;

uint32_t start_position_drv1, start_position_drv2, end_position_drv1, end_position_drv2, accel_position_drv1, accel_position_drv2 = 0;
int32_t end_position_drv_tmp, encoder1_increment_res, encoder2_increment_res;
uint32_t ENCODER_1_OFFSET = 0;
uint32_t ENCODER_2_OFFSET = 0;
uint32_t angle_position_drv1, angle_position_drv2, begin_pos_drv1, begin_pos_drv2, angle_position_drv1_tmp = 0;
uint32_t test_angle = 0;
uint16_t meas_res_drv1, meas_res_drv2 = 0;
int16_t start_angle, end_angle;
uint16_t accel_angle = 0;
uint16_t test_data_buf_cnt = 0;
uint8_t uart3_rx_buffer[6] = {0};
uint8_t uart3_rx_safe_buffer[6] = {0};
//uint8_t uart1_rx_buffer[5] = {0};
uint8_t uart1_rx_safe_buffer[5] = {0};
uint8_t uart1_rx_safe_buffer_meas[5] = {0};
uint8_t uart1_rx_calibration_buffer[150] = {0};
uint32_t adc_values_buf[50] = {0};
bool uart1_rx_complete = 0;
bool uart3_rx_complete = 0;
bool spi4_rx_complete = 0;
bool spi3_rx_complete = 0;
bool driver_dir1, driver_dir2, chosen_drv = 1; // 0 - forward, 1 - back
bool init_state = 1;
uint8_t next_command = 0xFF;
uint16_t error_cnt = 0;
bool tim4_ovflw, tim12_ovflw = false;
uint32_t test_enc_data = 130000;
uint32_t encoder1_test_data = 98304, encoder2_test_data = 98304;

/* Telemetry status values */
enum status ready_status;
enum action cur_action = NONE;
enum response_status { ERROR__, ACCEPTED__, ALREADY_EXEC, EXEC_OTHER};
bool trans_states = 0; // 0 - no trans_state, 1 - trans_state
//enum data { NONE_, _READY_, SOME_PACKETS} data_status;
enum horiz_platform { HORIZONTAL_, VERTICAL_} current_horiz_platform = HORIZONTAL_;
uint8_t operation_progress = 0;
uint32_t SSI_data, SSI_data_safe, encoder1_data, encoder2_data, encoder2_data_last = 0;
uint32_t adc_value, error_val_sum = 0;
uint16_t motor_frequency_1 = 40, motor_frequency_2 = 1;
uint32_t crc_PC_error_cnt = 0;
uint8_t adc_data_buf_safe[33] = {0};

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
static void MX_TIM5_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void parser();
void stepDriver(uint8_t step_num);
void createResponsePacket(uint8_t command_code, uint8_t status_code);
void moveToPosition(uint8_t angle, bool chosen_drv);
void changeMotorDirection(bool chosen_drv, uint32_t target_position);
uint32_t processSSIData(uint8_t *SSI_buffer);
uint32_t calculateEncPosition(uint32_t encoder_position, bool chosen_encoder);
uint32_t calculateOffsetEncPosition(uint32_t encoder_position, uint32_t offset_pos);
uint8_t getADCAmplifierVal(uint8_t value);
void createDataPacket();
uint8_t getADCAmplifierValInverted(uint8_t value);

/* Measurement functions */
void handleTestAngleOffset();
void handleMovingToStartOffset();
void handleHorizontalMeasurement();
void handleTestTurn();
void handleVerticalMeasurement();
void reverseHorizontalMeasurement();
void handleHorizontalMeasurementVertPlatf();
void setPlatformParam(uint16_t meas_res);
void checkCRCPhotodetectorData();
void createErrorResponse();
uint32_t calculateRequiredDataNum(uint32_t meas_interval, uint32_t meas_resolution);
uint32_t getTimeOffset();

void convertAdcValues(uint8_t *buf, uint16_t size);
void DeviceInit();
void DeviceReset();
bool checkCRC_PCData();
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
  MX_TIM5_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_TIM4_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  __HAL_DBGMCU_FREEZE_IWDG();
  DeviceInit();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_IWDG_Refresh(&hiwdg);
	  // handle of message from PC
	  if (uart3_rx_complete) {
		  __HAL_TIM_SET_COUNTER(&htim11, 0);
		  if (checkCRC_PCData()) {
			  parser();
		  }
	  }
	  // handle of message from Photodetector
	  if (uart1_rx_complete) {
		  // counter of received messages
		  uart1_received_cnt_global++;

          checkCRCPhotodetectorData();

          // processing of received messages in measurement state of device
          if (cur_action == HORIZONTAL || cur_action == VERTICAL || cur_action == HEMISPHERE || cur_action == LIGHT_POWER) {

        	  // checking that the measurement data has been received
        	if (wait_adc_data_flag) {

        		// filling the buffer of measurement data
        		for (uint8_t i = 0; i < 3; i++, data_elem_cnt++) {
        			adc_data_buf[data_elem_cnt] = uart1_rx_safe_buffer_meas[i];
        		}

        		wait_adc_data_flag = 0;

        		data_buf_counter++;
        		test_data_buf_cnt = data_buf_counter;
        		adc_data_cnt++;
        		test_counter_adc_data++;

        		if (data_buf_counter == 10 && data_elem_cnt == 31) {
        			memcpy(adc_data_buf_safe, adc_data_buf, 33);
        			data_buf_counter = 0;
        			data_elem_cnt = 1;
        			data_status = _READY_;
        			packet_cnt++;
        		}
        	}
          }

		  if (cur_action == CALIBRATION) {
			  if (wait_adc_data_flag) {

				  // filling the buffer of measurement data
				  for (uint8_t i = 0; i < 3; i++, data_elem_cnt_calib++) {
					  uart1_rx_calibration_buffer[data_elem_cnt_calib] = uart1_rx_safe_buffer_meas[i];
				  }

				  wait_adc_data_flag = 0;

				  data_buf_counter++;
				  test_data_buf_cnt = data_buf_counter;
				  adc_data_cnt++;
				  test_counter_adc_data++;

				  // the buffer is filled
				  if (data_buf_counter == 50) {
					  HAL_TIM_Base_Stop_IT(&htim10);

					  convertAdcValues(uart1_rx_calibration_buffer, data_buf_counter * 3);
					  bubbleSort(adc_values_buf, data_buf_counter);
					  photodetector_offset_val = calculateMedianVal(adc_values_buf, data_buf_counter, 12);

					  data_buf_counter = 0;
					  data_elem_cnt_calib = 0;
					  wait_flag = 0;
					  
					  clearBuffer(adc_data_buf_safe,33);

					  adc_data_buf_safe[1] = photodetector_offset_val;
					  adc_data_buf_safe[2] = photodetector_offset_val >> 8;
					  adc_data_buf_safe[3] = photodetector_offset_val >> 16;

					  adc_data_buf_safe[4] = adc_data_buf_safe[1];
					  adc_data_buf_safe[5] = adc_data_buf_safe[2];
					  adc_data_buf_safe[6] = adc_data_buf_safe[3];

					  data_status = _READY_;
					  end_calibration_flag = 1;

				  }
			  }

			  if (end_calibration_flag) {
				  if (data_status == NONE_){
					  cur_action = NONE;
					  ready_status = READY_;
					  end_calibration_flag = 0;
				  }
			  }
		  }

          uart1_rx_complete = 0;

	  }

	  // the handler of the received message from the encoder 1
	  if (spi4_rx_complete) {

		  switch(cur_action) {
		  case TEST_ANGLE_OFFSET:
			  handleTestAngleOffset();
			  break;
		  case TEST_ROTATION:
// only for previous desktop app
			 // handleMovingToStartOffset();
			  break;
		  case MOVING:
			  handleMovingToStartOffset(); //right version
			  break;
		  case HORIZONTAL:
			  handleHorizontalMeasurement();
			  break;
		  case VERTICAL:
			  //handleVerticalMeasurement();
			  handleHorizontalMeasurement();
			  break;
		  case HEMISPHERE:
			  break;
		  case CALIBRATION:
			  break;
		  default:
			  break;
		  }
		  spi4_rx_complete = 0;
	  }

	  // the handler of the received message from the encoder 2

	  if (spi3_rx_complete) {

		  switch(cur_action) {
		  case TEST_ANGLE_OFFSET:
			  handleTestAngleOffset();
			  break;
		  case TEST_ROTATION:
// only for previous desktop app
			  //handleMovingToStartOffset();
			  break;
		  case MOVING:
			  handleMovingToStartOffset(); //right version
			  break;
		  case HORIZONTAL:
			  handleHorizontalMeasurementVertPlatf();
			  break;
		  case VERTICAL:
			  handleVerticalMeasurement();
			  break;
		  case HEMISPHERE:
			  break;
		  case CALIBRATION:
			  break;
		  default:
			  break;
		  }
		  spi3_rx_complete = 0;
	  }

	 if (end_meas_flag) {

		 //if (data_buf_counter > 0 && data_status == NONE_) {
		 if (data_buf_counter > 0 && data_status == NONE_) {
			 wait_adc_data_flag = 0;
			// clearing the part of the buffer that does not contain useful data
			clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
			memcpy(adc_data_buf_safe, adc_data_buf,33);
			data_status = _READY_;
			data_buf_counter = 0;
			data_elem_cnt = 1;

		} else if (data_buf_counter == 0 && data_status == NONE_) {
			cur_action = NONE;
			wait_flag = 0;
			ready_status = READY_;
			end_meas_flag = 0;
			wait_adc_data_flag = 0;
			adc_data_cnt = 0;
			resetNVICPriority();
			HAL_TIM_Base_Stop_IT(&htim13);
		}

	 }


	 if (adc_coeff_command_set && adc_coeff_set_complete && next_command == 0x05) {
		adc_coeff_command_set = 0;
		adc_coeff_set_complete = 0;
		next_command = 0xFF;
		cur_action = LIGHT_POWER;
		ready_status = BUSY_;
		start_light_pow_meas = 1;
		//HAL_TIM_Base_Start_IT(&htim5);
		setNVICPriority(LIGHT_POWER);
		HAL_TIM_Base_Start_IT(&htim14);

	 }

	 if (cur_action == CALIBRATION) {
		 // check status of setting adc frequency and start polling of photodetector
		 if (adc_coeff_command_set && adc_coeff_set_complete) {
			 adc_coeff_command_set = 0;
			 adc_coeff_set_complete = 0;

			 setNVICPriority(CALIBRATION);
			 // need to choose timer
			 __HAL_TIM_SET_COUNTER(&htim10, 0);
			 HAL_TIM_Base_Start_IT(&htim10);
			 //break;
		 }

		 if (tim10_cnt) {
			 tim10_cnt = 0;
			 HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			 wait_adc_data_flag = 1;
			 usDelay(10);
			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			 __HAL_TIM_SET_COUNTER(&htim10, 0);
		 }
	 }

	 // the handler of resolution timer overflow (active only at the light power measurement)
	 if (tim14_cnt) {
		 tim14_cnt = 0;
		 if (start_light_pow_meas) {
			uart1_received_cnt = 0;
			uart1_received_cnt_global = 0;
			//HAL_TIM_Base_Start(&htim13);
			HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			wait_adc_data_flag = 1;
			usDelay(10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			//HAL_TIM_Base_Stop(&htim13);
			__HAL_TIM_SET_COUNTER(&htim14, 0);
			HAL_TIM_Base_Start_IT(&htim5);
			start_light_pow_meas = 0;
			//HAL_TIM_Base_Start(&htim13);
			HAL_TIM_Base_Start_IT(&htim13);

		} else {

			HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			wait_adc_data_flag = 1;
			usDelay(10);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			//HAL_TIM_Base_Stop(&htim13);

			/*

			if (htim13.Instance->CNT >= 200) {
				htim14.Instance->ARR = tim14_arr_val - 10;
				htim13.Instance->CNT = 0;
			} else {
				htim14.Instance->ARR = tim14_arr_val;
			} */
			HAL_TIM_Base_Stop_IT(&htim13);
			tim13cnt = 65535 * tim13_ovflw;
			tim13cnt += htim13.Instance->CNT;
			tim13_ovflw = 0;
			__HAL_TIM_SET_COUNTER(&htim13, 0);
			HAL_TIM_Base_Start_IT(&htim13);
			new_tim_arr_val = getTimeOffset();
			HAL_TIM_Base_Stop(&htim14);
			htim14.Instance->ARR = new_tim_arr_val;
			__HAL_TIM_SET_COUNTER(&htim14, 0);
			HAL_TIM_Base_Start_IT(&htim14);


		}

		test_counter_adc_data2++;

	 }

	 // check backlash of reductor
	 // comment this 'if-block' if checking of the backlash is no need
	 /*
	 if (is_motor_moving) {
		 if (!is_backlash_passed) {
			 if (chosen_drv) {
				 checkBacklash(encoder2_data);
			 } else {
				 checkBacklash(encoder1_data);
			 }

		 }
	 }
	 */
	 // uncomment this if checking of the backlash is no need

	  if (is_motor_moving && !is_req_freq_reach) {
	  	  HAL_TIM_Base_Start_IT(&htim4);
	  }

	 // increase motor frequency

	 if (tim4_ovflw) {
		 if (target_motor_freq > (current_motor_freq + motor_freq_inc_hz)) {
			 setMotorFrequency(chosen_drv,current_motor_freq + motor_freq_inc_hz);
			 tim4_ovflw = false;
			 __HAL_TIM_SET_COUNTER(&htim4, 0);
			 HAL_TIM_Base_Start_IT(&htim4);
		 } else { // the required frequency has been reached
			 setMotorFrequency(chosen_drv,target_motor_freq);
			 HAL_TIM_Base_Stop_IT(&htim4);
			 tim4_ovflw = false;
			 is_req_freq_reach = true;
		 }
	 }


	 if (tim12_ovflw) {
		 if (current_motor_freq <= 50) {
			stopMotorRotation(chosen_drv);
			HAL_TIM_Base_Stop_IT(&htim12);
			tim12_ovflw = false;
		 } else {
			setMotorFrequency(chosen_drv,current_motor_freq - 50);
			__HAL_TIM_SET_COUNTER(&htim12, 0);
			tim12_ovflw = false;
		 }
	 }

  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  htim2.Init.Period = 24999;
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
  htim3.Init.Prescaler = 1079;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 10799;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim10.Init.Prescaler = 21599;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
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
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 10799;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 30000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 10799;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 500;
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
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 1079;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 21599;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
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
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
	case 0x00:
    	HAL_UART_DeInit(&huart3);
    	MX_USART3_UART_Init();
    	break;
	case 0x01:

		HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
		HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);
		// poll photodetector
		//HAL_UART_DMAStop(&huart1);
		//HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 3);

		if (wait_flag == 0 && stop_poll == 0) {

			if(huart1.hdmarx->State == HAL_DMA_STATE_READY) {
				HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
				//HAL_Delay(1);
				//usDelay(100);
				usDelay(10);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			}
		}

		//clearBuffer(response_buf,33);
		//HAL_TIM_Base_Start(&htim10);
		createResponsePacket(0x01,0);
		//HAL_TIM_Base_Stop(&htim10);
		//__HAL_TIM_SET_COUNTER(&htim10, 0);

		break;
	case 0x03:

		createResponsePacket(0x03,ACCEPTED__);

		// only for debug
		test_counter_adc_data = 0;
		test_counter_adc_data2 = 0;
		stop_poll = 1;
		full_rotation = 0;
		//enc_cnt_trg = 0;

		// stop sending photodetector data to telemetry packet (if wait_flag == 1)
		wait_flag = 0;

		// choosing a platform
		chosen_drv = HORIZONTAL_;

		// set data availability status
		data_status = NONE_;

		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);

		// set start angle of measurement
		//start_angle = abs(start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1] - 180);
		//start_angle = 360 - start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
		start_angle = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1]);

		// calculate offset of the measurement from specified start position
		/*
		if ((start_angle + start_angle_offset_1) > 360) {
			start_angle = (start_angle + start_angle_offset_1) - 360;
		} else {
			start_angle = start_angle + start_angle_offset_1;
		} */

		// set end angle of measurement
		//end_angle = start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1] + 180;
		//end_angle = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1]);
		end_angle = 360 - start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];

		if (start_angle == 180 && end_angle == 180) {
					full_rotation = 1;
		}

		/*
		if ((end_angle + start_angle_offset_1) > 360) {
			end_angle = (end_angle + start_angle_offset_1) - 360;
		} else {
			end_angle = end_angle + start_angle_offset_1;
		}
		*/

		meas_res_drv1 = measurement_res_items[1][uart3_rx_safe_buffer[3]-1];

		setPlatformParam(meas_res_drv1);

		// calculate acceleration offset position

		/*
		if ((start_angle - ACCEL_OFFSET) < 0) {
			accel_angle = abs(start_angle - ACCEL_OFFSET);
			accel_angle = 360 - accel_angle;
		} else {
			accel_angle = start_angle - ACCEL_OFFSET;
		}
		*/

		accel_angle = start_angle + ACCEL_OFFSET;

		// reset flag of reaching start position
		reach_start_position = 0;
		reach_accel_position = 0;
		reach_end_position = 0;
		step_1_vertical_meas = 0;
		step_2_vertical_meas = 0;
		reach_start_position_vertical = 0;

		// set acceleration offset position
		accel_position_drv1 = (accel_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
		accel_position_drv1 = calculateEncPosition(accel_position_drv1,chosen_drv);
		changeMotorDirection(chosen_drv, accel_position_drv1);

		// set start position of measurement
		start_position_drv1 = (start_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
		start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);

		// set encoder1_increment_res
		/* for normal rotation */
		if ((start_position_drv1 + meas_res_drv1) > ENCODER_RESOLUTION) {
			encoder1_increment_res = (start_position_drv1 + meas_res_drv1) - ENCODER_RESOLUTION;
		} else {
			encoder1_increment_res = start_position_drv1 + meas_res_drv1;
		}
		/* for normal rotation end */

		/* for reverse rotation */
		if ((start_position_drv1 - meas_res_drv1) < 0) {
			encoder1_increment_res = ENCODER_RESOLUTION - abs(start_position_drv1 - meas_res_drv1);
		} else {
			encoder1_increment_res = start_position_drv1 - meas_res_drv1;
		}
		/* for reverse rotation end */

		// set end position of measurement

		if (full_rotation) {
			end_position_drv_tmp = (end_angle * ENCODER_RESOLUTION) / 360;
			end_position_drv_tmp = calculateEncPosition(end_position_drv_tmp,chosen_drv);
			/* for normal rotation
			if ((end_position_drv_tmp + 720) >= ENCODER_RESOLUTION) {
				control_pos = (end_position_drv_tmp + 720) - ENCODER_RESOLUTION;
			} else {
				control_pos = end_position_drv_tmp + 720;
			}
			*/
			/* for normal rotation end */
			/* for reverse rotation */
			if ((end_position_drv_tmp - 720) < 0) {
				control_pos = ENCODER_RESOLUTION - abs(end_position_drv_tmp - 720);
			} else {
				control_pos = end_position_drv_tmp - 720;
			}
			/* for reverse rotation end */
			end_position_drv1 = ((end_angle+4) * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			end_position_drv1 = calculateEncPosition(end_position_drv1,chosen_drv);
		} else {
			end_position_drv1 = (end_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			end_position_drv1 = calculateEncPosition(end_position_drv1,chosen_drv);
		}



		/* might be useful
		if (end_position_drv1 < start_position_drv1) {
			reducing_pos_calc = 1;
		} */

		// choose of measurement resolution
		//meas_res_drv1 = measurement_res_items[1][uart3_rx_safe_buffer[3]-1];

		// set current action
		cur_action = VERTICAL;
		// set status
		ready_status = BUSY_;

		//move vertical platform to start position and start measurement
		trans_states = 1;

		// start measurement

		target_motor_freq = DEFAULT_MOTOR_FREQUENCY_HZ;
		startMotorRotation(chosen_drv,encoder1_data);
		/*
		HAL_TIM_Base_Start_IT(&htim7);	// start poll encoder
		HAL_TIM_Base_Start_IT(&htim2); // start first motor moving
		*/
		/* work version
		moveToPosition(0, VERTICAL_); */

		/* new version
		 *
		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
		data_status = NONE_;
		stop_poll = 1;
		createResponsePacket(0x03,ACCEPTED__);
		InitPlatformMeasurement(&horizontal, VERTICAL, uart3_rx_safe_buffer[1], uart3_rx_safe_buffer[2], uart3_rx_safe_buffer[3]);
	*/
		break;
	case 0x02:

		createResponsePacket(0x02,ACCEPTED__);

		start_angle = 0;
		end_angle = 0;
		accel_angle = 0;
		stop_poll = 1;
		full_rotation = 0;

		// only for debug
		test_counter_adc_data = 0;
		test_counter_adc_data2 = 0;
		//enc_cnt_trg = 0;

		// stop sending photodetector data to telemetry packet (if wait_flag == 1)
		wait_flag = 0;

		// choosing a platform
		chosen_drv = current_horiz_platform;

		// set data availability status
		data_status = NONE_;

		//setNVICPriority(HORIZONTAL);

		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);

		// set start angle of measurement
		start_angle = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1]);
		//start_angle = abs(start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1] - 180);
		//start_angle = 360 - start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];

		//start_angle = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1]);
		//start_angle = start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
		// calculate offset of the measurement from specified start position
		/*
		if ((start_angle + start_angle_offset_1) > 360) {
			start_angle = (start_angle + start_angle_offset_1) - 360;
		} else {
			start_angle = start_angle + start_angle_offset_1;
		} */

		// set end angle of measurement
		//end_angle = start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1] + 180;
		//end_angle = (start_ending_angle_items[1][uart3_rx_safe_buffer[2]-1]);
		end_angle = 360 - start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
		//end_angle = 360 - start_ending_angle_items[1][uart3_rx_safe_buffer[1]-1];
		/*
		if (start_angle == 0 && end_angle == 360) {
			full_rotation = 1;
		} */

		if (start_angle == 180 && end_angle == 180) {
					full_rotation = 1;
				}

		/*
		if ((end_angle + start_angle_offset_1) > 360) {
			end_angle = (end_angle + start_angle_offset_1) - 360;
		} else {
			end_angle = end_angle + start_angle_offset_1;
		}
		*/

		// calculate acceleration offset position
		/*
		if ((start_angle - ACCEL_OFFSET) < 0) {
			accel_angle = abs(start_angle - ACCEL_OFFSET);
			accel_angle = 360 - accel_angle;
		} else {
			accel_angle = start_angle - ACCEL_OFFSET;
		}
		*/

		accel_angle = start_angle + ACCEL_OFFSET;

		// reset flags
		reach_start_position = 0;
		reach_accel_position = 0;
		reach_end_position = 0;

		if (chosen_drv == HORIZONTAL_) {

			// choose of measurement resolution
			meas_res_drv1 = measurement_res_items[1][uart3_rx_safe_buffer[3]-1];

			//setMotorFrequency(chosen_drv, motor_frequency_1);
			setPlatformParam(meas_res_drv1);

			// set acceleration offset position
			accel_position_drv1 = (accel_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			accel_position_drv1 = calculateEncPosition(accel_position_drv1,chosen_drv);
			changeMotorDirection(chosen_drv, accel_position_drv1);

			// set start position of measurement
			start_position_drv1 = (start_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv1 = calculateEncPosition(start_position_drv1,chosen_drv);

			// set encoder1_increment_res
			// for normal rotation
			/*
			if ((start_position_drv1 + meas_res_drv1) > ENCODER_RESOLUTION) {
				encoder1_increment_res = (start_position_drv1 + meas_res_drv1) - ENCODER_RESOLUTION;
			} else {
				encoder1_increment_res = start_position_drv1 + meas_res_drv1;
			}
			for normal rotation end */

			/* for reverse rotation */
			if ((start_position_drv1 - meas_res_drv1) < 0) {
				encoder1_increment_res = ENCODER_RESOLUTION - abs(start_position_drv1 - meas_res_drv1);
			} else {
				encoder1_increment_res = start_position_drv1 - meas_res_drv1;
			}
			/* for reverse rotation end */

			if (full_rotation) {
				// set end position of measurement
				end_position_drv_tmp = (end_angle * ENCODER_RESOLUTION) / 360;
				end_position_drv_tmp = calculateEncPosition(end_position_drv_tmp,chosen_drv);

				/* for normal rotation
				if ((end_position_drv_tmp + 720) >= ENCODER_RESOLUTION) {
					control_pos = (end_position_drv_tmp + 720) - ENCODER_RESOLUTION;
				} else {
					control_pos = end_position_drv_tmp + 720;
				}
				for normal rotation end */


				/* for reverse rotation */
				if ((end_position_drv_tmp - 720) < 0) {
					control_pos = ENCODER_RESOLUTION - abs(end_position_drv_tmp - 720);
				} else {
					control_pos = end_position_drv_tmp - 720;
				}
				/* for reverse rotation end */

				//end_position_drv1 = ((end_angle-4) * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				end_position_drv1 = ((end_angle+4) * ENCODER_RESOLUTION) / 360; // get absolute encoder position

				end_position_drv1 = calculateEncPosition(end_position_drv1,chosen_drv);
			} else {
				end_position_drv1 = (end_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				end_position_drv1 = calculateEncPosition(end_position_drv1,chosen_drv);
			}

			/* not used
			if (end_position_drv1 < start_position_drv1) {
				reducing_pos_calc = 1;
			} */

			// set current action
			cur_action = HORIZONTAL;

			// set status
			ready_status = BUSY_;

			//setMotorFrequency(0,400);
			target_motor_freq = DEFAULT_MOTOR_FREQUENCY_HZ;
			startMotorRotation(chosen_drv,encoder1_data);

			// start measurement
			/*
			HAL_TIM_Base_Start_IT(&htim7);	// start poll encoder
			HAL_TIM_Base_Start_IT(&htim2); // start first motor moving
			*/

		} else if (chosen_drv == VERTICAL_) {

			// choose of measurement resolution
			meas_res_drv2 = measurement_res_items[1][uart3_rx_safe_buffer[3]-1];

			setPlatformParam(meas_res_drv2);

			// set acceleration offset position
			accel_position_drv2 = (accel_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			accel_position_drv2 = calculateEncPosition(accel_position_drv2,chosen_drv);
			changeMotorDirection(chosen_drv, accel_position_drv2);

			// set start position of measurement
			start_position_drv2 = (start_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
			start_position_drv2 = calculateEncPosition(start_position_drv2,chosen_drv);

			// set encoder1_increment_res
			/* for normal rotation
			if ((start_position_drv2 + meas_res_drv2) > ENCODER_RESOLUTION) {
				encoder2_increment_res = (start_position_drv2 + meas_res_drv2) - ENCODER_RESOLUTION;
			} else {
				encoder2_increment_res = start_position_drv2 + meas_res_drv2;
			} normal rotation end */

			/* for reverse rotation */
			if ((start_position_drv2 - meas_res_drv2) < 0) {
				encoder2_increment_res = ENCODER_RESOLUTION - abs(start_position_drv2 - meas_res_drv2);
			} else {
				encoder2_increment_res = start_position_drv2 - meas_res_drv2;
			}
			/* for reverse rotation end */

			if (full_rotation) {
				// set end position of measurement
				end_position_drv_tmp = (end_angle * ENCODER_RESOLUTION) / 360;
				end_position_drv_tmp = calculateEncPosition(end_position_drv_tmp,chosen_drv);
				/* for normal rotation
				if ((end_position_drv_tmp + 720) >= ENCODER_RESOLUTION) {
					control_pos = (end_position_drv_tmp + 720) - ENCODER_RESOLUTION;
				} else {
					control_pos = end_position_drv_tmp + 720;
				} normal rotation end */


				/* for reverse rotation */
				if ((end_position_drv_tmp - 720) < 0) {
					control_pos = ENCODER_RESOLUTION - abs(end_position_drv_tmp - 720);
				} else {
					control_pos = end_position_drv_tmp - 720;
				}
				/* for reverse rotation end */
				end_position_drv2 = ((end_angle+4) * ENCODER_RESOLUTION) / 360; // get absolute encoder position

				//end_position_drv2 = ((end_angle - 4) * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				end_position_drv2 = calculateEncPosition(end_position_drv2,chosen_drv);
			} else {
				end_position_drv2 = (end_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				end_position_drv2 = calculateEncPosition(end_position_drv2,chosen_drv);
			}

			if (end_position_drv2 < start_position_drv2) {
				reducing_pos_calc = 1;
			}

			// set current action
			cur_action = HORIZONTAL;

			// set status
			ready_status = BUSY_;

			target_motor_freq = DEFAULT_MOTOR_FREQUENCY_HZ;
			startMotorRotation(chosen_drv,encoder2_data); // need to replace test_enc_data on encoder2_data

			//setMotorFrequency(1,400);

			// start measurement
			/*
			HAL_TIM_Base_Start_IT(&htim7);	// start poll encoder
			HAL_TIM_Base_Start_IT(&htim3); // start first motor moving
			*/
		}

	break;

	case 0x05:

		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
		clearBuffer(adc_data_buf,33);
		__HAL_TIM_SET_COUNTER(&htim13, 0);
		allow = 1;
		// check of setting adc coefficient command
		if (adc_coeff_command_set) {
			next_command = 0x05;
		} else {
			next_command = 0;
		}

		end_meas_flag = 0;

		required_data_num = calculateRequiredDataNum(light_pow_period_items[uart3_rx_safe_buffer[1]-1],light_pow_res_items[uart3_rx_safe_buffer[2]-1]);

		// Init of timers
		HAL_TIM_Base_Stop(&htim5);
		htim5.Instance->ARR = light_pow_period_items[uart3_rx_safe_buffer[1]-1];
		//htim5.Instance->ARR = calculateTimeIntervalError(light_pow_period_items[uart3_rx_safe_buffer[1]-1],light_pow_res_items[uart3_rx_safe_buffer[2]-1]);
		__HAL_TIM_SET_COUNTER(&htim5, 0);

		HAL_TIM_Base_Stop(&htim14);
		tim14_arr_val = light_pow_res_items[uart3_rx_safe_buffer[2]-1];
		htim14.Instance->ARR = tim14_arr_val;
		tim14_arr_val_converted = tim14_arr_val * 20;
		//htim14.Instance->ARR = light_pow_res_items[uart3_rx_safe_buffer[2]-1];
		__HAL_TIM_SET_COUNTER(&htim14, 0);

		createResponsePacket(0x05,ACCEPTED__);

		// then wait adc_coeff installation in the while ...
	break;

	case 0x06:
		createResponsePacket(0x06,ACCEPTED__);
		// установка направления вращения
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

		cur_action = TEST_TURN;
		ready_status = READY_;

		// старт вращения платформы
		startMotorRotation(HORIZONTAL_, encoder1_data);

		break;

	case 0x07:
		createResponsePacket(0x07,ACCEPTED__);

		switch(cur_action) {
		case HORIZONTAL:

			stopMotorRotation(chosen_drv);

			if (data_buf_counter > 0 && data_status == NONE_) {
				// clearing the part of the buffer that does not include useful data
				clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
				data_status = _READY_;
			}

			// reset flags and state
			data_buf_counter = 0;
			data_elem_cnt = 1;
			wait_flag = 0;
			wait_adc_data_flag = 0;
			trans_states = 0;
			break;

		case VERTICAL:

			stopMotorRotation(chosen_drv);

			if (data_buf_counter > 0 && data_status == NONE_) {
				// clearing the part of the buffer that does not include useful data
				clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
				data_status = _READY_;
			}

			// reset flags and state
			data_buf_counter = 0;
			data_elem_cnt = 1;
			wait_flag = 0;
			wait_adc_data_flag = 0;
			trans_states = 0;
			break;

		case LIGHT_POWER:

			HAL_TIM_Base_Stop(&htim14);
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Stop(&htim13);

			// checking for remaining data packets
			if (data_buf_counter > 0 && data_status == NONE_) {
				// clearing the part of the buffer that does not include useful data
				clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
				data_status = _READY_;
			}

			// reset flags and state
			data_buf_counter = 0;
			data_elem_cnt = 1;
			wait_flag = 0;
			wait_adc_data_flag = 0;
			error_val = 0;

			break;
		case TEST_TURN:
			stopMotorRotation(HORIZONTAL_);
			stopMotorRotation(VERTICAL_);
			break;
		case TEST_ANGLE_OFFSET:
			stopMotorRotation(HORIZONTAL_);
			stopMotorRotation(VERTICAL_);
			trans_states = 0;
			break;
		default:
			break;
		}

		cur_action = NONE;
		ready_status = READY_;

		break;
	case 0x08:
		createResponsePacket(0x08,ACCEPTED__);
		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer,6);
		uint16_t offset_angle = 0;
		offset_angle = uart3_rx_safe_buffer[2] << 8;
		offset_angle |= uart3_rx_safe_buffer[1];
		offset_angle = 360 - offset_angle;
		ENCODER_1_OFFSET = ENCODER_2_OFFSET = (offset_angle * ENCODER_RESOLUTION) / 360;

		break;
	case 0x09:
		chosen_drv = uart3_rx_buffer[1];
		createResponsePacket(0x09,ACCEPTED__);

		ready_status = READY_;
		cur_action = MOVING;

		trans_states = 1;

		if (uart3_rx_buffer[1] == 0) {
			setMotorFrequency(0,400);
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2); // start horizontal motor moving

		} else {
			setMotorFrequency(1,400);
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim3); // start vertical motor moving

		}

		break;
	case 0x0A:
		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
		createResponsePacket(0xA,ACCEPTED__);
		if(uart3_rx_safe_buffer[1] != 0) {
			current_horiz_platform = VERTICAL_;
		} else {
			current_horiz_platform = HORIZONTAL_;
		}
		break;

	case 0x0B:
		createDataPacket();
		data_status = NONE_;
		take_data_cnt++;
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
		stepDriver(uart3_rx_buffer[1]);
		break;
	case 0x0F: // тестовый проворот с выбором платформ и направлением вращения

		createResponsePacket(0x0F,ACCEPTED__);
		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
		if (uart3_rx_safe_buffer[2] == 0xFF) { 		// выбрана вертикальная платформа
			if (uart3_rx_safe_buffer[1] == 0xFF) { 	// выбрано направление вращения назад
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
				driver_dir2 = false;
			} else {								// выбрано направление вращения вперед
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				driver_dir2 = true;
			}
			startMotorRotation(VERTICAL_, encoder1_data);
		} else {									// выбрана горизонтальная платформа
			if (uart3_rx_safe_buffer[1] == 0xFF) { 	// выбрано направление вращения назад
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
				driver_dir1 = false;
			} else {								// выбрано направление вращения вперед
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				driver_dir1 = true;
			}
			startMotorRotation(HORIZONTAL_, encoder2_data);
		}

		cur_action = TEST_TURN;
		ready_status = READY_;

		break;

	case 0x10: // Вращение на заданный угол
		memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
		createResponsePacket(0x10,ACCEPTED__);
		cur_action = TEST_ANGLE_OFFSET;
		uint32_t temp_pos; // временная переменная для расчета позиции на которую произойдет смещение
		// получение заданного угла из команды
		test_angle = 0;
		test_angle = uart3_rx_safe_buffer[2] << 8;
		test_angle |= uart3_rx_safe_buffer[1];
		test_angle = 360 - test_angle;
		temp_pos = (test_angle * ENCODER_RESOLUTION) / 360;

		// определение платформы для вращения
		if(chosen_drv) { // вертикальная платформа

			angle_position_drv2 = 0;

			if(uart3_rx_safe_buffer[3] == 0) { // вращение на абсолютный угол по лимбу
				angle_position_drv2 = calculateOffsetEncPosition(temp_pos,zero_limb_pos[1]);
			} else {						// вращение на угол относительно текущей позиции
				angle_position_drv2 = calculateOffsetEncPosition(temp_pos,encoder2_data);
			}

			changeMotorDirection(chosen_drv, angle_position_drv2);
			startMotorRotation(chosen_drv,encoder2_data);

		} else { // горизонтальная платформа

			angle_position_drv1 = 0;

			if(uart3_rx_safe_buffer[3] == 0) { // вращение на абсолютный угол по лимбу
				angle_position_drv1 = calculateOffsetEncPosition(temp_pos,zero_limb_pos[0]);
			} else {						// вращение на угол относительно текущей позиции
				angle_position_drv1 = calculateOffsetEncPosition(temp_pos,encoder1_data);
			}

			changeMotorDirection(chosen_drv, angle_position_drv1);
			startMotorRotation(chosen_drv,encoder1_data);
		}
		break;
/*
	case 0x11: // stop moving
		createResponsePacket(0x11,ACCEPTED__);
		stopMotorRotation(HORIZONTAL_);
		stopMotorRotation(VERTICAL_);
		cur_action = NONE;
		trans_states = 0;
		break; */
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

		// Write new values of encoder offset
		ENCODER_1_OFFSET = encoder1_data;
		ENCODER_2_OFFSET = encoder2_data;

		// Write new values of encoder offset for saving to Flash
		encoder_offset[0] = ENCODER_1_OFFSET;
		encoder_offset[1] = ENCODER_2_OFFSET;

		inv_encoder1_offset = getInvertedEncoderVal(encoder_offset[0]);
		inv_encoder2_offset = getInvertedEncoderVal(encoder_offset[1]);

		// Save data to flash
		WriteToFlash(encoder_offset, 2, ADDR_FLASH_SECTOR_4, FLASH_TYPEPROGRAM_WORD, FLASH_SECTOR_4);

		break;
	case 0x14:

		// send response packet
		createResponsePacket(0x14,ACCEPTED__);
		wait_flag = 1;

		adc_coeff_command_set = 1;
		adc_coeff_set_complete = 0;

		// версия без инверсии второго полубайта
		//ampl_buf[0] = getADCAmplifierVal(uart3_rx_buffer[1]);

		// определение содержимого команды для установки частоты дискретизации без инверсии второго полубайта
		/*
		if (uart3_rx_buffer[2] == 1) {
			ampl_buf[0] |= 0b10001000;
		}
	*/
		// версия с инверсией второго полубайта
		ampl_buf[0] = getADCAmplifierValInverted(uart3_rx_buffer[1]);

		// определение содержимого команды для установки частоты дискретизации c инверсией второго полубайта
		if (uart3_rx_buffer[2] == 1) { 	// частота дискретизации 242Гц
			ampl_buf[0] |= 0b10000000;
		} else {						// частота дискретизации 16Гц
			ampl_buf[0] |= 0b00001000;
		}

		ampl_buf[1] = ampl_buf[0];

		HAL_UART_DMAStop(&huart1);
		HAL_UART_Transmit(&huart1, ampl_buf, 1,100);
		/* this string fixed bug early */
		//HAL_UART_Receive_IT(&huart1, buf, 5);
		HAL_UART_Receive_DMA(&huart1, buf, 5);

		break;
	case 0x15:
		HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		break;
	case 0x17:
		createResponsePacket(0x17,ACCEPTED__);
		// подоготовка абсолютных значений энкодера для хранения во флеш
		zero_limb_pos[0] = encoder1_data;
		zero_limb_pos[1] = encoder2_data;
		WriteToFlash(zero_limb_pos, 2, ADDR_FLASH_SECTOR_5, FLASH_TYPEPROGRAM_WORD, FLASH_SECTOR_5);
		// подготовка инверсных значений энкодера для отправки на ПК
		inv_zero_limb_pos[0] = inv_encoder1_data;
		inv_zero_limb_pos[1] = inv_encoder2_data;

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
	case 0x20:

			for (int i = 0; i < 130; i++) {
				//printf("%lu,\r\n",encoder_data_buf_trg[i]);

			}
			printf("'\r\n");

			break;
	case 0x1A:
		
		createResponsePacket(0x1A, ACCEPTED__);
		
		// set timer for polling photodetector every 100ms
		htim10.Instance->ARR = 500;

		// set adc frequency = 16Hz
		wait_flag = 1;
		data_elem_cnt_calib = 0;
		adc_coeff_command_set = 1;
		adc_coeff_set_complete = 0;
		ampl_buf[0] = getADCAmplifierVal(1);
		
		// set status 
		cur_action = CALIBRATION;
		ready_status = BUSY_;

		//ampl_buf[1] = ampl_buf[0];
		HAL_UART_DMAStop(&huart1);
		HAL_UART_Transmit(&huart1, ampl_buf, 1, 100);
		HAL_UART_Receive_DMA(&huart1, buf, 5);
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

	if (huart->Instance == USART3) {
		uart3_rx_complete = 1;
	}

	if (huart->Instance == USART1) {

		// обработка ответа после отправки команды на настройку АЦП
		if (wait_flag == 1) {

			// расчет CRC для принятого ответа
			uint32_t CRC_Photodetector = 0;
			CRC_Photodetector = buf[0] + buf[1] + buf[2];
			CRC_Photodetector = CRC_Photodetector & 0xFF;

			// сравнение CRC с принятым значением
			if (!((CRC_Photodetector == buf[3]) && (buf[4] == 0xA5))) {
				ready_status = ERROR_; // установка состояния ошибки
				error_code = 0x02;
			} else {
				// проверка на совпадение второго полубайта ответа с первым полубайтом запроса
				if (buf[0] != (ampl_buf[0] >> 4)) {
					ready_status = ERROR_;
					error_code = 0x01;
				}
			}
			// смена внутренних состояний при установке параметров АЦП по умолчанию
			if (init_state) { 	//смена состояний после первого подключения управляющего ПО к прибору
				init_state = 0;	// сброс состояния инициализации
				// сброс флагов состояний
				adc_coeff_set_complete = 0;
				adc_coeff_command_set = 0;
			} else { // остальные случаи изменения параметров АЦП
				adc_coeff_set_complete = 1; // коэффициент установлен
			}

			wait_flag = 0; // сброс флага прекращения отправки данных АЦП в запросах телеметрии
		}

		// обработка ответа с данными АЦП
		uart1_rx_complete = 1;
	}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	// first encoder
	if (hspi->Instance == SPI4) {
		encoder1_data  =  (dma_spi4_buf[2] >> 5) & 0x07;
		encoder1_data |=  ((uint32_t)dma_spi4_buf[1] << 3);
		encoder1_data |=  (((uint32_t)dma_spi4_buf[0] & 0x3F) << 11);
		//encoder1_data = encoder1_test_data;
		horizontal.encoder.current_pos = encoder1_data;
		inv_encoder1_data =  ENCODER_RESOLUTION - encoder1_data;
		spi4_rx_complete = 1;
	}

	// second encoder
	if (hspi->Instance == SPI3) {
		encoder2_data  =  (dma_spi3_buf[2] >> 5) & 0x07;
		encoder2_data |=  ((uint32_t)dma_spi3_buf[1] << 3);
		encoder2_data |=  (((uint32_t)dma_spi3_buf[0] & 0x3F) << 11);
		//encoder2_data = encoder2_test_data;
		vertical.encoder.current_pos = encoder2_data;
		inv_encoder2_data = ENCODER_RESOLUTION - encoder2_data;
		spi3_rx_complete = 1;
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
		response_buf[9] = inv_encoder1_data & 0xFF;//encoder1_data & 0xFF; inv_encoder1_data
		response_buf[10] = inv_encoder1_data >> 8;//encoder1_data >> 8;
		response_buf[11] = inv_encoder1_data >> 16;//encoder1_data >> 16;
		response_buf[12] = inv_zero_limb_pos[0] & 0xFF;//encoder1_data & 0xFF; inv_encoder1_data
		response_buf[13] = inv_zero_limb_pos[0] >> 8;//encoder1_data >> 8;
		response_buf[14] = inv_zero_limb_pos[0] >> 16;//encoder1_data >> 16;
		response_buf[15] = inv_encoder2_data & 0xFF;
		response_buf[16] = inv_encoder2_data >> 8;
		response_buf[17] = inv_encoder2_data >> 16;
		response_buf[18] = inv_zero_limb_pos[1] & 0xFF;//encoder1_data & 0xFF; inv_encoder1_data
		response_buf[19] = inv_zero_limb_pos[1] >> 8;//encoder1_data >> 8;
		response_buf[20] = inv_zero_limb_pos[1] >> 16;//encoder1_data >> 16;
		response_buf[21] = 0;//inv_encoder1_offset & 0xFF;
		response_buf[22] = 0;//inv_encoder1_offset >> 8;
		response_buf[23] = 0;//inv_encoder1_offset >> 16;
		response_buf[24] = 0;//inv_encoder2_offset & 0xFF;
		response_buf[25] = 0;//inv_encoder2_offset >> 8;
		response_buf[26] = 0;//inv_encoder2_offset >> 16;
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

void createDataPacket() {
	uint16_t crc=0;
	adc_data_buf_safe[0] = 0x0B;
	adc_data_buf_safe[31] = 0;
	adc_data_buf_safe[32] = 0;

	// CRC calculation
	for (int i = 0; i < 30; i+=2) {
		crc += (uint16_t)adc_data_buf_safe[i] + ((uint16_t)(adc_data_buf_safe[i+1])<<8);
	}
	crc += adc_data_buf_safe[30];
	*(uint16_t*)(adc_data_buf_safe+31) = crc;
	HAL_UART_DMAStop(&huart3);
	HAL_UART_Transmit(&huart3, adc_data_buf_safe,33,100);
	HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer,6);
	//memcpy(adc_data_buf, 0, 33);
	//clearBuffer(adc_data_buf,33);

}

uint32_t calculateEncPosition(uint32_t encoder_position, bool chosen_encoder) {

	uint32_t encoder_pos_ret = 0;

	if (chosen_encoder) { // chosen vertical platform
		if((encoder_position + ENCODER_2_OFFSET) > ENCODER_RESOLUTION) {
			//return (encoder_position + ENCODER_2_OFFSET) - ENCODER_RESOLUTION;
			encoder_pos_ret = (encoder_position + ENCODER_2_OFFSET) - ENCODER_RESOLUTION;
		} else {
			//return encoder_position + ENCODER_2_OFFSET;
			encoder_pos_ret = encoder_position + ENCODER_2_OFFSET;
		}
	} else { // chosen horizontal platform
		if((encoder_position + ENCODER_1_OFFSET) > ENCODER_RESOLUTION) {
			//return (encoder_position + ENCODER_1_OFFSET) - ENCODER_RESOLUTION;
			encoder_pos_ret = (encoder_position + ENCODER_1_OFFSET) - ENCODER_RESOLUTION;
		} else {
			//return encoder_position + ENCODER_1_OFFSET;
			encoder_pos_ret = encoder_position + ENCODER_1_OFFSET;
		}
	}

	if (encoder_pos_ret == 0) {
		encoder_pos_ret = POSITION_ERROR;
	}

	if (encoder_pos_ret == ENCODER_RESOLUTION) {
		encoder_pos_ret -= POSITION_ERROR;
	}

	return encoder_pos_ret;

}

uint32_t calculateOffsetEncPosition(uint32_t encoder_position, uint32_t offset_pos) {

	uint32_t encoder_pos_ret = 0;

	if((encoder_position + offset_pos) > ENCODER_RESOLUTION) {
		encoder_pos_ret = (encoder_position + offset_pos) - ENCODER_RESOLUTION;
	} else {
		encoder_pos_ret = encoder_position + offset_pos;
	}

	if (encoder_pos_ret == 0) {
		encoder_pos_ret = POSITION_ERROR;
	}

	if (encoder_pos_ret == ENCODER_RESOLUTION) {
		encoder_pos_ret -= POSITION_ERROR;
	}

	return encoder_pos_ret;
}

uint32_t processSSIData(uint8_t *SSI_buffer) {
	uint32_t encoder_data = 0;
	encoder_data  =  (SSI_buffer[2] >> 5) & 0x07;
	encoder_data |=  ((uint32_t)SSI_buffer[1] << 3);
	encoder_data |=  (((uint32_t)SSI_buffer[0] & 0x3F) << 11);
	return encoder_data;
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

uint8_t getADCAmplifierValInverted(uint8_t value) {

	switch(value) {
		case 1:
			return 0b00000111;
		case 2:
			return 0b00010110;
		case 4:
			return 0b00100101;
		case 8:
			return 0b00110100;
		case 16:
			return 0b01000011;
		case 32:
			return 0b01010010;
		case 64:
			return 0b01100001;
		case 128:
			return 0b01110000;
		default:
			return 1;
	}
}

void moveToPosition(uint8_t angle, bool chosen_drv) {
	trans_states = 1;
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

void checkCRCPhotodetectorData() {
  // check CRC
  if ((((uart1_rx_buffer[0] + uart1_rx_buffer[1] + uart1_rx_buffer[2]) & 0xFF) == uart1_rx_buffer[3]) && (uart1_rx_buffer[4] == 0xA5)) {
	  if (wait_adc_data_flag) {
		  uart1_rx_safe_buffer_meas[0] = uart1_rx_buffer[0];
		  uart1_rx_safe_buffer_meas[1] = uart1_rx_buffer[1];
		  uart1_rx_safe_buffer_meas[2] = uart1_rx_buffer[2];
		  test_cnt_uart1_rx++;
	  } else {
		  uart1_rx_safe_buffer[0] = uart1_rx_buffer[0];
		  uart1_rx_safe_buffer[1] = uart1_rx_buffer[1];
		  uart1_rx_safe_buffer[2] = uart1_rx_buffer[2];
	  }
	  uart1_received_cnt++;


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

void handleTestAngleOffset() {

	if (chosen_drv) {
		if ((encoder2_data >= angle_position_drv2 - 4) && (encoder2_data <= angle_position_drv2 + 4)) {
			stopMotorRotation(chosen_drv);
			cur_action = NONE;
		}
	} else {
		if ((encoder1_data >= angle_position_drv1 - 4) && (encoder1_data <= angle_position_drv1 + 4)) {
			stopMotorRotation(chosen_drv);
			cur_action = NONE;
		}
	}
}

void handleMovingToStartOffset() {
  setMotorFrequency(chosen_drv,75);
	if (chosen_drv) {
	  if ((encoder2_data >= begin_pos_drv2 - ENCODER_TOLERANCE) && (encoder2_data <= begin_pos_drv2 + ENCODER_TOLERANCE)) {
		  HAL_TIM_Base_Stop_IT(&htim3); // stop motor
		  HAL_TIM_Base_Stop_IT(&htim7); // stop encoder poll
		  cur_action = NONE;
		  trans_states = 0;
	  }
  } else {
	  if ((encoder1_data >= begin_pos_drv1 - ENCODER_TOLERANCE) && (encoder1_data <= begin_pos_drv1 + ENCODER_TOLERANCE)) {
		  HAL_TIM_Base_Stop_IT(&htim2); // stop motor
		  HAL_TIM_Base_Stop_IT(&htim7); // stop encoder poll
		  cur_action = NONE;
		  trans_states = 0;
	  }
  }
}

void handleTestTurn() {

	if (!reach_test_turn_pos) {
		if ((encoder1_data >= angle_position_drv1_tmp - ENCODER_TOLERANCE)
				&& (encoder1_data <= angle_position_drv1_tmp + ENCODER_TOLERANCE)) {
			  reach_test_turn_pos = 1;
		  }
	} else {
		if ((encoder1_data >= angle_position_drv1 - ENCODER_TOLERANCE)
				&& (encoder1_data <= angle_position_drv1 + ENCODER_TOLERANCE)) {
			stopMotorRotation(HORIZONTAL_);
			cur_action = NONE;
			ready_status = READY_;
		  }
	}
}

void handleHorizontalMeasurement() {
	  // State - move to acceleration position
	  if (!reach_accel_position) {
		  trans_states = 1;
		  if ((encoder1_data >= accel_position_drv1 - 5) && (encoder1_data <= accel_position_drv1 + 5)) {

			  stopMotorRotation(chosen_drv);
			  changeMotorDirection(chosen_drv, start_position_drv1);
			  reach_accel_position = 1;
			  target_motor_freq = motor_frequency_1; // set measurement motor frequency
			  startMotorRotation(chosen_drv, encoder1_data);
			  /*
			  HAL_TIM_Base_Stop_IT(&htim2);
			  changeMotorDirection(chosen_drv, start_position_drv1);
			  setMotorFrequency(chosen_drv, motor_frequency_1);
			  reach_accel_position = 1;
			  HAL_TIM_Base_Start_IT(&htim2); */
		  }
	  }

	  // State - move to start position
	  if (reach_accel_position) {

		  if (!reach_start_position) {
			  trans_states = 0;

			  if ((encoder1_data >= start_position_drv1 - 5) && (encoder1_data <= start_position_drv1 + 5)) {
			  	reach_start_position = 1;
			  	// start photodetector polling

			  	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

			 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
			 	//enc_cnt_trg++;

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			  	wait_adc_data_flag = 1;
			  	usDelay(30);

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			  }
		  }

		  // State - move to end position

		  if (reach_start_position) {

			  if (!reach_end_position) {

				  // if reached end position
				 //if ((encoder1_data >= end_position_drv1)) {
				  if ((encoder1_data >= end_position_drv1) && (encoder1_data <= end_position_drv1 + ENCODER_TOLERANCE)) {

					  HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
					  wait_adc_data_flag = 1;
					  //encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
					  //enc_cnt_trg++;
					  usDelay(30);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
					  test_counter_adc_data2++;
					  reach_end_position = 1;
					  return;

					  /* was here
					  // measurement end but adc buffer is not empty
					  if (data_buf_counter > 0) {
						  // clearing the part of the buffer that does not include useful data
						  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
						  data_status = _READY_;
					  }

					  // reset flags and state
					  data_buf_counter = 0;
					  data_elem_cnt = 1;
					  cur_action = NONE;
					  wait_flag = 0;
					  ready_status = READY_;
					  reach_end_position = 1;
					  wait_adc_data_flag = 0;

					  // stop measurement
					  HAL_TIM_Base_Stop_IT(&htim2); // stop motor
					  HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer */

				  } else {
					  // while not reached end_position
					  /* normal rotation */
					  /*
					  if ((encoder1_data >= (encoder1_increment_res - 4)) && (encoder1_data <= (encoder1_increment_res + 4))) {
					 		encoder1_increment_res += meas_res_drv1;
						  if (encoder1_increment_res >= ENCODER_RESOLUTION) {
							  encoder1_increment_res -= ENCODER_RESOLUTION;
						  }
						  */
					/* normal rotation end */
					  /* reverse rotation */
					  if ((encoder1_data >= (encoder1_increment_res - 4)) && (encoder1_data <= (encoder1_increment_res + 4))) {

						  if ((encoder1_increment_res - meas_res_drv1) < 0) {
							  encoder1_increment_res = ENCODER_RESOLUTION - abs(encoder1_increment_res - meas_res_drv1);
						  } else {
							  encoder1_increment_res -= meas_res_drv1;
						  }
						/* reverse rotation end */

						HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
						wait_adc_data_flag = 1;
						//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
						//enc_cnt_trg++;
						usDelay(30);

						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
						test_counter_adc_data2++;

					  }

					  if (full_rotation) {
						if ((encoder1_data >= control_pos - 60) && (encoder1_data <= control_pos + 60)) {
							end_position_drv1 = start_position_drv1;
						}
					  }

				  }
			  }
		  }
	  }

	  if (reach_end_position && !wait_adc_data_flag && data_status == NONE_) {
		  if (data_buf_counter > 0) {
			  // clearing the part of the buffer that does not include useful data
			  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
			  memcpy(adc_data_buf_safe, adc_data_buf,33);
			  data_status = _READY_;
		  }

		  // reset flags and state
		  data_buf_counter = 0;
		  data_elem_cnt = 1;
		  cur_action = NONE;
		  wait_flag = 0;
		  ready_status = READY_;
		  reach_end_position = 1;
		  wait_adc_data_flag = 0;

		  stop_poll = 0;

		  //resetNVICPriority();

		  // stop measurement
		  stopMotorRotationReq(chosen_drv);
		  //stopMotorRotation(chosen_drv);
		  //HAL_TIM_Base_Stop_IT(&htim2); // stop motor
		  //HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer
	  }
}

void handleHorizontalMeasurementVertPlatf() {
	  // State - move to acceleration position
	  if (!reach_accel_position) {
		  trans_states = 1;
		  if ((encoder2_data >= accel_position_drv2 - 5) && (encoder2_data <= accel_position_drv2 + 5)) {

			  stopMotorRotation(chosen_drv);
			  changeMotorDirection(chosen_drv, start_position_drv2);
			  reach_accel_position = 1;
			  target_motor_freq = motor_frequency_2; // set measurement motor frequency
			  startMotorRotation(chosen_drv, encoder2_data);

			  /*
			  HAL_TIM_Base_Stop_IT(&htim3);
			  changeMotorDirection(chosen_drv, start_position_drv2);
			  setMotorFrequency(chosen_drv, motor_frequency_1);
			  reach_accel_position = 1;
			  HAL_TIM_Base_Start_IT(&htim3);
			  */
		  }
	  }

	  // State - move to start position
	  if (reach_accel_position) {

		  if (!reach_start_position) {
			  trans_states = 0;

			  if ((encoder2_data >= start_position_drv2 - 5) && (encoder2_data <= start_position_drv2 + 5)) {
			  	reach_start_position = 1;
			  	// start photodetector polling

			  	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

			 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
			 	//enc_cnt_trg++;

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			  	wait_adc_data_flag = 1;
			  	usDelay(10);

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			  }
		  }

		  // State - move to end position

		  if (reach_start_position) {

			  if (!reach_end_position) {

				  // if reached end position
				 //if ((encoder1_data >= end_position_drv1)) {
				  if ((encoder2_data >= end_position_drv2 - 8) && (encoder2_data <= end_position_drv2 + 8)) {

					  HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
					  wait_adc_data_flag = 1;
					  //encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
					  //enc_cnt_trg++;
					  usDelay(10);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
					  test_counter_adc_data2++;
					  reach_end_position = 1;
					  return;

					  /* was here
					  // measurement end but adc buffer is not empty
					  if (data_buf_counter > 0) {
						  // clearing the part of the buffer that does not include useful data
						  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
						  data_status = _READY_;
					  }

					  // reset flags and state
					  data_buf_counter = 0;
					  data_elem_cnt = 1;
					  cur_action = NONE;
					  wait_flag = 0;
					  ready_status = READY_;
					  reach_end_position = 1;
					  wait_adc_data_flag = 0;

					  // stop measurement
					  HAL_TIM_Base_Stop_IT(&htim2); // stop motor
					  HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer */

				  } else {
					  // while not reached end_position
					  /* normal rotation
					  if ((encoder2_data >= (encoder2_increment_res - 4)) && (encoder2_data <= (encoder2_increment_res + 4))) {
						  encoder2_increment_res += meas_res_drv2;
						  if (encoder2_increment_res >= ENCODER_RESOLUTION) {
							  encoder2_increment_res -= ENCODER_RESOLUTION;
						  } end normal rotation */


					  /* reverse rotation */
					  if ((encoder2_data >= (encoder2_increment_res - 4)) && (encoder2_data <= (encoder2_increment_res + 4))) {

						  if ((encoder2_increment_res - meas_res_drv2) < 0) {
							  encoder2_increment_res = ENCODER_RESOLUTION - abs(encoder2_increment_res - meas_res_drv2);
						  } else {
							  encoder2_increment_res -= meas_res_drv2;
						  }
						/* reverse rotation end */

							HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

						 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
						 	wait_adc_data_flag = 1;
						 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
						 	//enc_cnt_trg++;
						 	usDelay(10);

						 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
						 	test_counter_adc_data2++;
					  }

					  if (full_rotation) {
						if ((encoder2_data >= control_pos - 60) && (encoder2_data <= control_pos + 60)) {
							end_position_drv2 = start_position_drv2;
						}
					  }
				  }
			  }
		  }
	  }

	  if (reach_end_position && !wait_adc_data_flag && data_status == NONE_) {
		  if (data_buf_counter > 0) {
			  // clearing the part of the buffer that does not include useful data
			  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
			  memcpy(adc_data_buf_safe, adc_data_buf,33);
			  data_status = _READY_;
		  }

		  // reset flags and state
		  data_buf_counter = 0;
		  data_elem_cnt = 1;
		  cur_action = NONE;
		  wait_flag = 0;
		  ready_status = READY_;
		  reach_end_position = 1;
		  wait_adc_data_flag = 0;
		  stop_poll = 0;

		  stopMotorRotationReq(chosen_drv);
		  //stopMotorRotation(chosen_drv);

	  }
}

void handleVerticalMeasurement() {

	if (!reach_start_position_vertical) {

		reach_start_position_vertical = 1;

		/* work version
		if ((encoder2_data >= start_position_drv2 - 5) && (encoder2_data <= start_position_drv2 + 5)) {

			// stop move vertical platform
			HAL_TIM_Base_Stop_IT(&htim7);
			HAL_TIM_Base_Stop_IT(&htim3);

			// start move horizontal platform
			reach_start_position_vertical = 1;
			HAL_TIM_Base_Start_IT(&htim7);
			HAL_TIM_Base_Start_IT(&htim2);

		} */
	}

	if (reach_start_position_vertical && !step_1_vertical_meas) {

		  // State - move to acceleration position
		  if (!reach_accel_position) {
			  trans_states = 1;
			  if ((encoder1_data >= accel_position_drv1 - 5) && (encoder1_data <= accel_position_drv1 + 5)) {
				  HAL_TIM_Base_Stop_IT(&htim2);
				  changeMotorDirection(chosen_drv, start_position_drv1);
				  reach_accel_position = 1;
				  HAL_TIM_Base_Start_IT(&htim2);
			  }
		  }

		  // State - move to start position
		  if (reach_accel_position) {

			  if (!reach_start_position) {
				  trans_states = 0;

				  if ((encoder1_data >= start_position_drv1 - 5) && (encoder1_data <= start_position_drv1 + 5)) {
				  	reach_start_position = 1;
				  	// start photodetector polling

				  	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

				 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
				 	//enc_cnt_trg++;

				  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
				  	wait_adc_data_flag = 1;
				  	usDelay(2);

				  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
				  }
			  }

			  // State - move to end position

			  if (reach_start_position) {

				  if (!reach_end_position) {

					  // if reached end position
					 //if ((encoder1_data >= end_position_drv1)) {
					  if ((encoder1_data >= end_position_drv1 - 8) && (encoder1_data <= end_position_drv1 + 8)) {
						  // measurement end but adc buffer is not empty
						  if (data_buf_counter > 0) {
							  // clearing the part of the buffer that does not include useful data
							  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
							  data_status = _READY_;
						  }

						  // reset flags and state
						  data_buf_counter = 0;
						  data_elem_cnt = 1;
						  //cur_action = NONE;
						  wait_flag = 0;
						  //ready_status = READY_;
						  //reach_end_position = 1;
						  wait_adc_data_flag = 0;

						  // stop measurement
						  HAL_TIM_Base_Stop_IT(&htim2); // stop motor
						  HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer

						  // go to the next step
						  step_1_vertical_meas = 1;
						  // start rotation of vertical position;
						  trans_states = 1;
						  /* work version
						  moveToPosition(VERTICAL_ROTATION_ANGLE,VERTICAL); */

					  } else {
						  // while not reached end_position
						  if ((encoder1_data >= (encoder1_increment_res - 4)) && (encoder1_data <= (encoder1_increment_res + 4))) {
						 		encoder1_increment_res += meas_res_drv1;
								HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

							 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
							 	wait_adc_data_flag = 1;
							 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
							 	//enc_cnt_trg++;
							 	usDelay(2);

							 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
							 	test_counter_adc_data2++;
						  }
					  }
				  }
			  }
		  }
	}

	if (step_1_vertical_meas) {

		if (step_2_vertical_meas) {
			reverseHorizontalMeasurement();
		} else {
			// work version
			//if ((encoder2_data >= start_position_drv2 - 5) && (encoder2_data <= start_position_drv2 + 5)) {
			if (step_1_vertical_meas) {
				// stop move vertical platform
				HAL_TIM_Base_Stop_IT(&htim7);
				HAL_TIM_Base_Stop_IT(&htim3);

				// preparation of horizontal platform

				reach_start_position = 0;
				reach_accel_position = 0;
				reach_end_position = 0;

				uint32_t temp_pos;
				temp_pos = start_position_drv1;
				end_position_drv1 = temp_pos;
				start_position_drv1 = end_position_drv1;

				if ((end_angle + ACCEL_OFFSET) > 360) {
					accel_angle = abs(end_angle + ACCEL_OFFSET);
					accel_angle =- 360;
				} else {
					accel_angle = end_angle + ACCEL_OFFSET;
				}

				accel_position_drv1 = (accel_angle * ENCODER_RESOLUTION) / 360; // get absolute encoder position
				accel_position_drv1 = calculateEncPosition(accel_position_drv1,chosen_drv);
				changeMotorDirection(chosen_drv, accel_position_drv1);
				// start move horizontal platform (go to the next step)
				step_2_vertical_meas = 1;

				HAL_TIM_Base_Start_IT(&htim7);
				HAL_TIM_Base_Start_IT(&htim2);
			}
		}
	}

}

void reverseHorizontalMeasurement() {
	  // State - move to acceleration position
	  if (!reach_accel_position) {
		  trans_states = 1;
		  if ((encoder1_data >= accel_position_drv1 - 5) && (encoder1_data <= accel_position_drv1 + 5)) {
			  HAL_TIM_Base_Stop_IT(&htim2);
			  changeMotorDirection(chosen_drv, start_position_drv1);
			  reach_accel_position = 1;
			  HAL_TIM_Base_Start_IT(&htim2);
		  }
	  }

	  // State - move to start position
	  if (reach_accel_position) {

		  if (!reach_start_position) {
			  trans_states = 0;

			  if ((encoder1_data >= start_position_drv1 - 5) && (encoder1_data <= start_position_drv1 + 5)) {
			  	reach_start_position = 1;
			  	// start photodetector polling

			  	HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

			 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
			 	//enc_cnt_trg++;

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
			  	wait_adc_data_flag = 1;
			  	usDelay(2);

			  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
			  }
		  }

		  // State - move to end position

		  if (reach_start_position) {

			  if (!reach_end_position) {

				  // if reached end position
				 //if ((encoder1_data >= end_position_drv1)) {
				  if ((encoder1_data >= end_position_drv1 - 4) && (encoder1_data <= end_position_drv1 + 4)) {
					  // measurement end but adc buffer is not empty
					  if (data_buf_counter > 0) {
						  // clearing the part of the buffer that does not include useful data
						  clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
						  data_status = _READY_;
					  }

					  // reset flags and state
					  data_buf_counter = 0;
					  data_elem_cnt = 1;
					  cur_action = NONE;
					  wait_flag = 0;
					  ready_status = READY_;
					  reach_end_position = 1;
					  wait_adc_data_flag = 0;

					  // stop measurement
					  HAL_TIM_Base_Stop_IT(&htim2); // stop motor
					  HAL_TIM_Base_Stop_IT(&htim7); // stop SPI timer

				  } else {
					  // while not reached end_position
					  if ((encoder1_data >= (encoder1_increment_res - 4)) && (encoder1_data <= (encoder1_increment_res + 4))) {
					 		encoder1_increment_res += meas_res_drv1;
							HAL_UART_Receive_DMA(&huart1, uart1_rx_buffer, 5);

						 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
						 	wait_adc_data_flag = 1;
						 	//encoder_data_buf_trg[enc_cnt_trg] = encoder1_data;
						 	//enc_cnt_trg++;
						 	usDelay(2);

						 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
						 	test_counter_adc_data2++;
					  }
				  }
			  }
		  }
	  }
}

uint32_t calculateRequiredDataNum(uint32_t meas_interval, uint32_t meas_resolution) {
	uint32_t meas_interval_ms, meas_resolution_ms;
	uint16_t data_num;

	meas_interval_ms = meas_interval / 10;
	meas_resolution_ms = (meas_resolution * 2)/10;
	data_num = (meas_interval_ms / meas_resolution_ms)+1;
	return data_num;
}

uint32_t getTimeOffset() {

	if (tim13cnt >= tim14_arr_val_converted) {
		error_val += tim13cnt - tim14_arr_val_converted;
		error_val_sum += error_val;
		if (error_val >= 200) {
			new_arr_val = tim14_arr_val - (error_val / 200)*2;
			error_val = error_val % 200;
			return new_arr_val;
		}
	}
	return tim14_arr_val;
}

void convertAdcValues(uint8_t* buf, uint16_t size) {
	for (uint16_t i = 0, j = 0; i < size; i += 3, j++) {
		adc_values_buf[j] = buf[i];
		adc_values_buf[j] |= buf[i+1] << 8;
		adc_values_buf[j] |= buf[i+2] << 16;
	}
}

void setPlatformParam(uint16_t meas_res) {
	switch(meas_res) {
	case 1820: // разрешение 5 градусов
		motor_frequency_1 = motor_frequency_2 = 860;
		setEncoderPollFrequency(200);
	case 365: // разрешение 1 градус
		motor_frequency_1 = motor_frequency_2 = 860;
		setEncoderPollFrequency(200);
		break;
	case 182: // разрешение 30 минут
		motor_frequency_1 = motor_frequency_2 = 860;
		setEncoderPollFrequency(200);
		break;
	case 60: // разрешение 10 минут
		motor_frequency_1 = motor_frequency_2 = 366;
		setEncoderPollFrequency(1000);
		break;
	case 29: // разрешение 5 минут
		motor_frequency_1 = motor_frequency_2 = 244;
		setEncoderPollFrequency(1000);
		break;
	case 6: // разрешение 1 минута
		motor_frequency_1 = motor_frequency_2 = 50;
		setEncoderPollFrequency(1000);
		break;
	case 3: // разрешение 30 секунд
		motor_frequency_1 = motor_frequency_2 = 20;
		setEncoderPollFrequency(1000);
		break;
	case 1: // разрешение 10 секунд
		motor_frequency_1 = motor_frequency_2 = 10;
		setEncoderPollFrequency(1000);
		break;
	}
}

void DeviceInit() {

	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Start(&htim6);

	HAL_Delay(100);
	// start receiving of messages from PC
	HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 6);

	// start receiving of encoder values
	HAL_SPI_Receive_DMA(&hspi4, dma_spi4_buf, 5);
	HAL_SPI_Receive_DMA(&hspi3, dma_spi3_buf, 5);

	/* begin �?спользовалось ранее при надобности хранить начало координат в ПЗУ

	// Чтение сохраненных значений энкодера для начала координат
	ReadFlash(encoder_offset,2,ADDR_FLASH_SECTOR_4,FLASH_TYPEPROGRAM_WORD);
	// проверка наличия прежней записи во флеш памяти
	if (encoder_offset[0] > 131072 && encoder_offset[1] > 131072) {
		encoder_offset[0] = 0;
		encoder_offset[1] = 0;
		// запись 0 во флеш память
		WriteToFlash(encoder_offset, 2, ADDR_FLASH_SECTOR_4, FLASH_TYPEPROGRAM_WORD, FLASH_SECTOR_4);
	}

	inv_encoder1_offset = getInvertedEncoderVal(encoder_offset[0]);
	inv_encoder2_offset = getInvertedEncoderVal(encoder_offset[1]);

	horizontal.encoder.ENCODER_OFFSET = encoder_offset[0];
	vertical.encoder.ENCODER_OFFSET = encoder_offset[1];

	ENCODER_1_OFFSET = encoder_offset[0];
	ENCODER_2_OFFSET = encoder_offset[1];
	end */

	ENCODER_1_OFFSET = 0;
	ENCODER_2_OFFSET = 0;

	FlashEraseInit(FLASH_SECTOR_5);
	// Чтение сохраненных значений энкодера нуля лимба
	ReadFlash(zero_limb_pos,2,ADDR_FLASH_SECTOR_5,FLASH_TYPEPROGRAM_WORD);
	// проверка наличия прежней записи во флеш памяти

	if (zero_limb_pos[0] > 131072 && zero_limb_pos[1] > 131072) {
		zero_limb_pos[0] = 1;
		zero_limb_pos[1] = 1;
		// запись 0 во флеш память
		WriteToFlash(zero_limb_pos, 2, ADDR_FLASH_SECTOR_5, FLASH_TYPEPROGRAM_WORD, FLASH_SECTOR_5);
	}

	inv_zero_limb_pos[0] = getInvertedEncoderVal(zero_limb_pos[0]);
	inv_zero_limb_pos[1] = getInvertedEncoderVal(zero_limb_pos[1]);

	// set motor timers for pull
	horizontal.motor_tim = &htim2;
	vertical.motor_tim = &htim3;

	// set default motor frequency
	horizontal.motor_freq_def_Hz = DEFAULT_MOTOR_FREQUENCY_HZ;
	vertical.motor_freq_def_Hz = DEFAULT_MOTOR_FREQUENCY_HZ;

	setMotorFrequency__(&horizontal, horizontal.motor_freq_def_Hz);
	setMotorFrequency__(&vertical, vertical.motor_freq_def_Hz);

	target_motor_freq = DEFAULT_MOTOR_FREQUENCY_HZ;

	// set status of device
	ready_status = READY_;

	HAL_TIM_Base_Start_IT(&htim11);
}

void DeviceReset() {

	switch(cur_action) {
	case HORIZONTAL:

		stopMotorRotationReq(HORIZONTAL_);

		if (data_buf_counter > 0 && data_status == NONE_) {
			// clearing the part of the buffer that does not include useful data
			clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
			data_status = _READY_;
		}

		// reset flags and state
		data_buf_counter = 0;
		data_elem_cnt = 1;
		wait_flag = 0;
		wait_adc_data_flag = 0;
		trans_states = 0;
		break;

	case VERTICAL:

		stopMotorRotationReq(VERTICAL_);

		if (data_buf_counter > 0 && data_status == NONE_) {
			// clearing the part of the buffer that does not include useful data
			clearSpecifiedElemOfBuffer(adc_data_buf,33,data_buf_counter*3+1);
			data_status = _READY_;
		}

		// reset flags and state
		data_buf_counter = 0;
		data_elem_cnt = 1;
		wait_flag = 0;
		wait_adc_data_flag = 0;
		trans_states = 0;
		break;

	case LIGHT_POWER:

		HAL_TIM_Base_Stop(&htim14);
		HAL_TIM_Base_Stop(&htim5);
		HAL_TIM_Base_Stop(&htim13);

		// checking for remaining data packets
		clearBuffer(adc_data_buf,33);
		clearBuffer(adc_data_buf_safe,33);
		data_status = NONE_;

		// reset flags and state
		data_buf_counter = 0;
		data_elem_cnt = 1;
		wait_flag = 0;
		wait_adc_data_flag = 0;
		error_val = 0;
		break;

	case CALIBRATION:

		break;

	case MOVING:
	case TEST_TURN:
	case TEST_ANGLE_OFFSET:
	case TEST_ROTATION:
		stopMotorRotation(VERTICAL_);
		stopMotorRotation(HORIZONTAL_);
		break;

	default:
		break;
	}

	cur_action = NONE;
	ready_status = READY_;
	__HAL_TIM_SET_COUNTER(&htim11, 0);
}

bool checkCRC_PCData() {
	memcpy(uart3_rx_safe_buffer, uart3_rx_buffer, 6);
	uint16_t crc = 0, crc_received = 0;
	for (int i = 0; i < 4; i+=2) {
		crc += (uint16_t)uart3_rx_safe_buffer[i] + ((uint16_t)(uart3_rx_safe_buffer[i+1])<<8);
	}
	crc_received = (uint16_t)uart3_rx_safe_buffer[4] + ((uint16_t)(uart3_rx_safe_buffer[5])<<8);
	if (crc_received != crc) {
      	HAL_UART_DeInit(&huart3);
        MX_USART3_UART_Init();
        uart3_rx_complete = 0;
        HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 6);
        crc_PC_error_cnt++;
        return false;
	}
	return true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
        // clear RXNE flag
        volatile uint8_t data = huart1.Instance->RDR;
    }
    if (huart == &huart3) {

    	if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
    		__HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF);
    		// clear RXNE flag
    		volatile uint8_t data = huart3.Instance->RDR;
    		HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 6);
    	} else {
        	// write status of ISR and error_code for unhandled cases
        	usart3_reg = huart->Instance->ISR;
        	usart3_error = huart->ErrorCode;
        	HAL_UART_DeInit(&huart3);
        	MX_USART3_UART_Init();
        	HAL_UART_Receive_DMA(&huart3, uart3_rx_buffer, 6);
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
