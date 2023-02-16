/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

#include "ina229.h"
#include "VT_INA229.h"
#include <stdio.h>
#include "pulsating_sine.h"
#include "stm32g4xx_hal_dma.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_PWM_COUNT 50000
#define MAX_PWM_FB_COUNT 50000
#define MIN_PWM_COUNT 35
#define MIN_PWM_FB_COUNT 700
#define SAT_LIMIT 1000
#define Kp 128
#define Ki 32
#define Kd 1

#define SINE_NB_STEPS  60

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

FDCAN_HandleTypeDef hfdcan2;

HRTIM_HandleTypeDef hhrtim1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* Buffer used for transmission */
uint8_t aTxBuffer[21];

/* Buffer used for reception */
uint8_t aRxBuffer[21];

// for 0V - 160V variable output PWM
volatile uint16_t duty_cycle = 0;
uint16_t duty_cycle_increase = 0;

// push button
uint8_t button_pressed = 0;

// SPI transmit data INA239/229
extern uint8_t INA229_msg_lenght_cntr, INA229_msg_lenght_cntr_old, INA229_msg_lenght_cntr_repeat_previous;
extern uint8_t INA229_send_packet[70], INA229_send_packet_decoder[70];
extern uint8_t INA229_recv_packet[70], INA229_recv_packet_decoder[70];

uint8_t INA229_decodedAddress, INA229_decoded_Reg, INA229_decoded_Command;

extern uint16_t INA229_REG_CONFIG_val;
extern uint16_t INA229_REG_ADC_CONFIG_val;
extern uint16_t INA229_REG_SHUNT_CAL_val;
extern uint16_t INA229_REG_SHUNT_TEMPCO_val;
extern uint32_t INA229_REG_VSHUNT_val;
extern uint32_t INA229_REG_VBUS_val;
extern uint16_t INA229_REG_DIETEMP_val;
extern uint32_t INA229_REG_CURRENT_val;
extern uint32_t INA229_REG_POWER_val;
extern uint64_t INA229_REG_ENERGY_val;
extern uint64_t INA229_REG_CHARGE_val;
extern uint16_t INA229_REG_DIAG_ALRT_val;
extern uint16_t INA229_REG_SOVL_val;
extern uint16_t INA229_REG_SUVL_val;
extern uint16_t INA229_REG_BOVL_val;
extern uint16_t INA229_REG_BUVL_val;
extern uint16_t INA229_REG_TEMP_LIMIT_val;
extern uint16_t INA229_REG_PWR_LIMIT_val;
extern uint16_t INA229_REG_MANUFACTURER_ID_val;
extern uint16_t INA229_REG_DEVICE_ID_val;

volatile uint8_t HAL_SPI_TxRxCpltCallback_finished = 1, HAL_SPI_TxRxCpltCallback_with_printf = 0, HAL_SPI_TxCpltCallback_repeat_previous = 0;

uint8_t INA239decodedAddress, INA239decodedReg, INA239decodedCommand;

int32_t INA229_REG_VSHUNT_val_total = 0, INA229_REG_VSHUNT_val_ave = 0;
int32_t INA229_REG_VSHUNT_val_data[1024] = {};
uint16_t cntr_ave = 0;

uint16_t TargetCurrent_mA = 0, TargetCurrent_mA_old, TargetCurrent_mA_updated = 0;
uint32_t TargetCurrent_mA_calc = 0;

int32_t PWM_184nS_count_total = 0, PWM_184nS_count_ave = 0;
int32_t PWM_184nS_count_data[1024] = {};
uint16_t PWM_184nS_cntr_ave = 0;

uint16_t PWM_184nS_count = 600, PWM_184nS_count_openloop = 600, PWM_184nS_count_new = 0, PWM_184nS_count_updated = 0;

uint8_t close_loop = 0;

// UART Variables
uint8_t UartRxBuffer[1];

// PID
volatile int32_t seterr, pid_out, pid_out_last, pid_out_diff;
volatile int32_t error, errorProp, errorIntgr, errorDiff, error_cntr;
volatile int32_t last_error;
uint8_t control_loop_ver = 1;

/* Sine wave values for a complete symbol */
uint16_t sinewave[60] = {
0x07ff,0x08cb,0x0994,0x0a5a,0x0b18,0x0bce,0x0c79,0x0d18,0x0da8,0x0e29,0x0e98,0x0ef4,0x0f3e,0x0f72,0x0f92,0x0f9d,
0x0f92,0x0f72,0x0f3e,0x0ef4,0x0e98,0x0e29,0x0da8,0x0d18,0x0c79,0x0bce,0x0b18,0x0a5a,0x0994,0x08cb,0x07ff,0x0733,
0x066a,0x05a4,0x04e6,0x0430,0x0385,0x02e6,0x0256,0x01d5,0x0166,0x010a,0x00c0,0x008c,0x006c,0x0061,0x006c,0x008c,
0x00c0,0x010a,0x0166,0x01d5,0x0256,0x02e6,0x0385,0x0430,0x04e6,0x05a4,0x066a,0x0733};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI2_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

void VT_SEND_SPI_VSHUNT(void);
void SPI_DMA_TX(void);
void SPI_DMA_TX_printf(void);
void SPI_DMA_TX_repeat_previous(void);
void SPI_DMA_TX_repeat_previous_printf(void);
void HAL_SPI_RxCpltCallback_printf(void);
void HAL_SPI_RxCpltCallback_no_printf(void);
void HAL_SPI_TxCpltCallback_printf(void);
void VT_PID_Controller_V1(void);
void VT_PID_Controller_V2(void);
extern void VT_INA229_ReadAllReg(void);
extern void VT_INA229_ReadRegPartial1(void);
extern void VT_INA229_ResetVars(void);
extern uint16_t combine_2_bytes(uint16_t high_byte, uint16_t low_byte);
extern uint32_t combine_3_bytes(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
extern int32_t combine_3_bytes_to_signed_20bits(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
extern uint64_t combine_5_bytes(uint64_t highhigh_byte, uint64_t high_byte, uint64_t mid_byte, uint64_t low_byte, uint64_t lowlow_byte);

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
  MX_SPI3_Init();
  MX_LPUART1_UART_Init();
  MX_TIM6_Init();
  MX_HRTIM1_Init();
  MX_TIM2_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SPI2_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

	printf("Starting >> NUCLEO-G474RE_INA239-INA229_DMA-DAC_HRTIM-SyncBuck_DAC-In-VoutVar \n");

	//	for(uint16_t cntr = 0; cntr < 10000; cntr++)
	//	{
	//		//		pulsating_sine[cntr] = pulsating_sine[cntr] - 4095;
	//		pulsating_sine[cntr] = 1000;
	//	}

	//	for(uint16_t cntr = 0; cntr < 10000; cntr++)
	//	{
	//		printf("pulsating_sine [%d] = %d \n", cntr, pulsating_sine[cntr]);
	//	}

	//	if(HAL_OK != HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
	//			(uint32_t*)pulsating_sine, 10000, DAC_ALIGN_12B_R))
	//	{
	//		/* Start DMA Error */
	//		Error_Handler();
	//	}

	/*##- Enable TIM peripheral counter ######################################*/
	//	if(HAL_OK != HAL_TIM_Base_Start(&htim6))
	//	{
	//		Error_Handler();
	//	}

    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sinewave, SINE_NB_STEPS, DAC_ALIGN_12B_R) != HAL_OK)
    {
      /* DAC conversion start error */
      Error_Handler();
    }

	if (HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)UartRxBuffer, 1) != HAL_OK)
	{
		Error_Handler();
	}

	VT_INA229_ResetVars();

	PWM_184nS_count_new = PWM_184nS_count;

	//	HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1);
	//	HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);

	printf("Ending   >> NUCLEO-G474RE_INA239-INA229_DMA-DAC_HRTIM-SyncBuck_DAC-In-VoutVar \n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//		GPIOB->BSRR = (1<<9); // Set
		HAL_Delay(50);

		if(TargetCurrent_mA <= 40)
			TargetCurrent_mA = 40;

		TargetCurrent_mA_calc = TargetCurrent_mA << 4;

		//		GPIOB->BRR = (1<<9); // Reset

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_HRTIM_TRG01;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 10;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 54400;
  pTimeBaseCfg.RepetitionCounter = 3;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_MREP;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_DACTRIGOUT_2;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.RepetitionCounter = 0x00;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DUAL;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = 711;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 80;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 80;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 51200;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xFFDF;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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
  hi2c1.Init.Timing = 0x00802172;
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

  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30A0A7FB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim2.Init.Prescaler = 16-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim6.Init.Prescaler = 16-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ToResetMCU_GPIO_Port, ToResetMCU_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ToResetMCU_Pin */
  GPIO_InitStruct.Pin = ToResetMCU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ToResetMCU_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

__INLINE void VT_SEND_SPI_VSHUNT(void) // called by void HRTIM1_Master_IRQHandler(void)
{
	GPIOB->BSRR = (1<<9); // Set
	VT_INA229_ReadReg(INA229_REG_VSHUNT);
	SPI_DMA_TX();

	// averaging
	INA229_REG_VSHUNT_val_total -= INA229_REG_VSHUNT_val_data[cntr_ave]; // very old data, previoud data need to be removed
	INA229_REG_VSHUNT_val_data[cntr_ave] = INA229_REG_VSHUNT_val;
	INA229_REG_VSHUNT_val_total += INA229_REG_VSHUNT_val;
	INA229_REG_VSHUNT_val_ave = INA229_REG_VSHUNT_val_total >> 10;
	cntr_ave = (cntr_ave + 1) & (1024 - 1); // counting from 0 to (1024 - 1)

	if (control_loop_ver == 1)
		VT_PID_Controller_V1();
	if (control_loop_ver == 2)
		VT_PID_Controller_V2();

	GPIOB->BRR = (1<<9); // Reset
}

__INLINE void VT_PID_Controller_V1(void)
{

	last_error = error;
	error = TargetCurrent_mA_calc - INA229_REG_VSHUNT_val_ave;

	// from fw
	errorProp = (-Kp * error) >> 3;
	errorIntgr = errorIntgr + ((Ki * error) >> 3);
	errorDiff = error / Kd;

	//		// used in ASM Phase Shift
	//		errorProp = (Kp * error) >> 3;
	//		errorIntgr = errorIntgr + ((Ki * error) >> 3);
	//		errorDiff = error / Kd;

	//	// check if the change is so much to prevent oscillations, see below "pid_out_diff"
	//	pid_out_last = pid_out;
	//
	//	if (errorIntgr > SAT_LIMIT)
	//	{
	//		errorIntgr = SAT_LIMIT;
	//	}
	//	if (errorIntgr < -(SAT_LIMIT))
	//	{
	//		errorIntgr = -(SAT_LIMIT);
	//	}

	// from FW
	//		seterr = (-Kp * error) / 200;
	//		Int_term_Buck = Int_term_Buck + ((-Ki * error) / 200);
	//		pid_out = seterr + Int_term_Buck;


	// My PID
	pid_out =  errorProp;// + errorIntgr; // + errorDiff;

	// check if the change is so much to prevent oscillations, see above "pid_out_last"
	// pid_out_diff = pid_out - pid_out_last;

	// this supposedly prevent sudden change in duty/phase
	//	    if (pid_out_diff > MAX_DUTY_INC)
	//	    {
	//	    	pid_out += MAX_DUTY_INC;
	//	    } else if (pid_out_diff > MAX_DUTY_INC)
	//	    {
	//	    	pid_out -= MAX_DUTY_INC;
	//	    }



	//	// averaging of PID/PWM
	//	PWM_184nS_count_total -= PWM_184nS_count_data[PWM_184nS_cntr_ave]; // very old data, previoud data need to be removed
	//	PWM_184nS_count_data[PWM_184nS_cntr_ave] = PWM_184nS_count;
	//	PWM_184nS_count_total += PWM_184nS_count;
	//	PWM_184nS_count_ave = PWM_184nS_count_total >> 10;
	//	PWM_184nS_cntr_ave = (PWM_184nS_cntr_ave + 1) & (1024 - 1); // counting from 0 - 511
	//
	//	if (PWM_184nS_count_ave >= MAX_PWM_FB_COUNT)
	//	{
	//		PWM_184nS_count_ave = MAX_PWM_FB_COUNT;
	//	} else if (PWM_184nS_count_ave < MIN_PWM_FB_COUNT)
	//	{
	//		PWM_184nS_count_ave = MIN_PWM_FB_COUNT;
	//	}
	//



	if (close_loop) //calculated pwm;
	{
		if (error > 30)
			error_cntr++;
		if (error < -30)
			error_cntr--;

		if (error_cntr > 90)
		{
			PWM_184nS_count++;
			error_cntr = 0;
		}
		if (error_cntr < -90)
		{
			PWM_184nS_count--;
			error_cntr = 0;
		}

		if (PWM_184nS_count >= MAX_PWM_FB_COUNT)
		{
			PWM_184nS_count = MAX_PWM_FB_COUNT;
		} else if (PWM_184nS_count < MIN_PWM_FB_COUNT)
		{
			PWM_184nS_count = MIN_PWM_FB_COUNT;
		}

		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PWM_184nS_count; // TimerA Compare 1
		// equate the close loop (pwm) variable with the open loop variable
		// so that the open loop will start where close loop stopped
		PWM_184nS_count_openloop = PWM_184nS_count;
	}
	else // change PWM
	{
		if (PWM_184nS_count_openloop != PWM_184nS_count_new)
		{
			if (PWM_184nS_count_new >= 54400)
				PWM_184nS_count_new = 50000;
			if (PWM_184nS_count_new <= 32)
				PWM_184nS_count_new = 32;

			if (PWM_184nS_count_new > PWM_184nS_count_openloop)
				PWM_184nS_count_openloop++;
			if (PWM_184nS_count_new < PWM_184nS_count_openloop)
				PWM_184nS_count_openloop--;
		}
		error_cntr = 0;
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PWM_184nS_count_openloop; // TimerA Compare 1
		// equate the open loop variables with the close loop variables
		// so that the close loop will start where open loop stopped
		PWM_184nS_count = PWM_184nS_count_openloop;
		TargetCurrent_mA = INA229_REG_VSHUNT_val_ave >> 4;
	}

}

__INLINE void VT_PID_Controller_V2(void)
{

	if (close_loop) //calculated pwm;
	{
		last_error = error;
		error = TargetCurrent_mA_calc - INA229_REG_VSHUNT_val_ave;

		// from fw
		errorProp = error >> 2;
		errorIntgr = errorIntgr + ((Ki * error) >> 3);
		errorDiff = error / Kd;

		// My PID
		pid_out =  errorProp;// + errorIntgr; // + errorDiff;

		if ((error > 10000) || (error < -10000)) // error very big so the step can be faster
		{
			if (error > 10000)
				PWM_184nS_count = PWM_184nS_count + 3;
			if (error < -10000)
				PWM_184nS_count = PWM_184nS_count - 3;

			if (PWM_184nS_count >= MAX_PWM_FB_COUNT)
			{
				PWM_184nS_count = MAX_PWM_FB_COUNT;
			} else if (PWM_184nS_count < MIN_PWM_FB_COUNT)
			{
				PWM_184nS_count = MIN_PWM_FB_COUNT;
			}

			HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PWM_184nS_count; // TimerA Compare 1
			// equate the close loop (pwm) variable with the open loop variable
			// so that the open loop will start where close loop stopped
			PWM_184nS_count_openloop = PWM_184nS_count;
		}
		else // when error almost reach the set point
		{
			if (error > 30)
				error_cntr++;
			if (error < -30)
				error_cntr--;

			if (error_cntr > 90)
			{
				PWM_184nS_count = PWM_184nS_count + 1;
				error_cntr = 0;
			}
			if (error_cntr < -90)
			{
				PWM_184nS_count = PWM_184nS_count - 1;
				error_cntr = 0;
			}

			if (PWM_184nS_count >= MAX_PWM_FB_COUNT)
			{
				PWM_184nS_count = MAX_PWM_FB_COUNT;
			} else if (PWM_184nS_count < MIN_PWM_FB_COUNT)
			{
				PWM_184nS_count = MIN_PWM_FB_COUNT;
			}

			HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PWM_184nS_count; // TimerA Compare 1
			// equate the close loop (pwm) variable with the open loop variable
			// so that the open loop will start where close loop stopped
			PWM_184nS_count_openloop = PWM_184nS_count;
		}


	}
	else // openloop, change PWM
	{
		if (PWM_184nS_count_openloop != PWM_184nS_count_new)
		{
			if (PWM_184nS_count_new >= 54400)
				PWM_184nS_count_new = 50000;
			if (PWM_184nS_count_new <= 32)
				PWM_184nS_count_new = 32;

			if (PWM_184nS_count_new > PWM_184nS_count_openloop)
				PWM_184nS_count_openloop++;
			if (PWM_184nS_count_new < PWM_184nS_count_openloop)
				PWM_184nS_count_openloop--;
		}
		error_cntr = 0;
		HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PWM_184nS_count_openloop; // TimerA Compare 1
		// equate the open loop variables with the close loop variables
		// so that the close loop will start where open loop stopped
		PWM_184nS_count = PWM_184nS_count_openloop;
		TargetCurrent_mA = INA229_REG_VSHUNT_val_ave >> 4;
		error = 0;
	}

}

__INLINE void SPI_DMA_TX(void)
{
	// do not print
	HAL_SPI_TxRxCpltCallback_with_printf = 0;

	// do not repeat previous SPI commands
	HAL_SPI_TxCpltCallback_repeat_previous = 0;

	// flag to wait until recive SPI
	HAL_SPI_TxRxCpltCallback_finished = 0;

	// SPI DMA TX-RX
	switch (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) INA229_send_packet, (uint8_t*) INA229_recv_packet, INA229_msg_lenght_cntr))
	{
	case HAL_OK:
		/* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT:
		/* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:
		/* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!HAL_SPI_TxRxCpltCallback_finished)
	{
	}

}

__INLINE void SPI_DMA_TX_repeat_previous(void)
{
	// do not print
	HAL_SPI_TxRxCpltCallback_with_printf = 0;

	// repeat previous SPI commands
	HAL_SPI_TxCpltCallback_repeat_previous = 1;

	// flag to wait until recive SPI
	HAL_SPI_TxRxCpltCallback_finished = 0;

	// SPI DMA TX-RX
	switch (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) INA229_send_packet, (uint8_t*) INA229_recv_packet, INA229_msg_lenght_cntr_repeat_previous))
	{
	case HAL_OK:
		/* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT:
		/* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:
		/* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	while(!HAL_SPI_TxRxCpltCallback_finished)
	{
	}
}

void SPI_DMA_TX_repeat_previous_printf(void)
{

	GPIOC->BSRR = (1<<8); // Set

	printf("SPI_DMA_TXRX_repeat_previous_printf \n");

	// pls print
	HAL_SPI_TxRxCpltCallback_with_printf = 1;

	// repeat previous SPI commands
	HAL_SPI_TxCpltCallback_repeat_previous = 1;

	// flag to wait until recive SPI
	HAL_SPI_TxRxCpltCallback_finished = 0;

	//	for(uint16_t cntr = 0; cntr < INA229_msg_lenght_cntr ; cntr++)
	//	{
	//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
	//	}

	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr_repeat_previous; cntr++)
	{

		if (INA229_send_packet_decoder[cntr] != 0) // decode
		{
			INA229_decodedAddress = INA229_send_packet_decoder[cntr] - 1;
			printf("INA229_send_packet_decoder[%d] = %x \n", cntr , INA229_decodedAddress);
			switch(INA229_decodedAddress)
			{
			case INA229_REG_CONFIG: // 00h, 2 bytes
				INA229_REG_CONFIG_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_CONFIG_val = %x \n", INA229_REG_CONFIG_val);
				break;
			case INA229_REG_ADC_CONFIG: // 01h, 2 bytes
				INA229_REG_ADC_CONFIG_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_ADC_CONFIG_val = %x \n", INA229_REG_ADC_CONFIG_val);
				break;
			case INA229_REG_SHUNT_CAL: // 02h, 2 bytes
				INA229_REG_SHUNT_CAL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_CAL_val = %x \n", INA229_REG_SHUNT_CAL_val);
				break;
			case INA229_REG_SHUNT_TEMPCO: // 03h, 2 bytes
				INA229_REG_SHUNT_TEMPCO_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_TEMPCO_val = %x \n", INA229_REG_SHUNT_TEMPCO_val);
				break;
			case INA229_REG_VSHUNT: // 04h, 3 bytes
				INA229_REG_VSHUNT_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_VSHUNT_val = %x \n", INA229_REG_VSHUNT_val);
				break;
			case INA229_REG_VBUS: // 05h, 3 bytes
				INA229_REG_VBUS_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_VBUS_val = %x \n", INA229_REG_VBUS_val);
				break;
			case INA229_REG_DIETEMP: //06h, 2 bytes
				INA229_REG_DIETEMP_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DIETEMP_val = %x \n", INA229_REG_DIETEMP_val);
				break;
			case INA229_REG_CURRENT: // 07h, 3 bytes
				INA229_REG_CURRENT_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_CURRENT_val = %x \n", INA229_REG_CURRENT_val);
				break;
			case INA229_REG_POWER: // 08h, 3 bytes
				INA229_REG_POWER_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_POWER_val = %x \n", INA229_REG_POWER_val);
				break;
			case INA229_REG_ENERGY: // 09h, 5 bytes
				INA229_REG_ENERGY_val = combine_5_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3], INA229_send_packet[cntr + 4], INA229_send_packet[cntr + 5]);
				printf("INA229_REG_ENERGY_val = %x%08x \n", INA229_REG_ENERGY_val);
				break;
			case INA229_REG_CHARGE: // 0ah, 5 bytes
				INA229_REG_CHARGE_val = combine_5_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3], INA229_send_packet[cntr + 4], INA229_send_packet[cntr + 5]);
				printf("INA229_REG_CHARGE_val = %x%08x \n", INA229_REG_CHARGE_val);
				break;
			case INA229_REG_DIAG_ALRT: // 0bh
				INA229_REG_DIAG_ALRT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DIAG_ALRT_val = %x \n", INA229_REG_DIAG_ALRT_val);
				break;
			case INA229_REG_SOVL: // 0ch
				INA229_REG_SOVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SOVL_val = %x \n", INA229_REG_SOVL_val);
				break;
			case INA229_REG_SUVL:
				INA229_REG_SUVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SUVL_val = %x \n", INA229_REG_SUVL_val);
				break;
			case INA229_REG_BOVL:
				INA229_REG_BOVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_BOVL_val = %x \n", INA229_REG_BOVL_val);
				break;
			case INA229_REG_BUVL:
				INA229_REG_BUVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_BUVL_val = %x \n", INA229_REG_BUVL_val);
				break;
			case INA229_REG_TEMP_LIMIT:
				INA229_REG_TEMP_LIMIT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_TEMP_LIMIT_val = %x \n", INA229_REG_TEMP_LIMIT_val);
				break;
			case INA229_REG_PWR_LIMIT:
				INA229_REG_PWR_LIMIT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_PWR_LIMIT_val = %x \n", INA229_REG_PWR_LIMIT_val);
				break;
			case INA229_REG_MANUFACTURER_ID:
				INA229_REG_MANUFACTURER_ID_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_MANUFACTURER_ID_val = %x \n", INA229_REG_MANUFACTURER_ID_val);
				break;
			case INA229_REG_DEVICE_ID:
				INA229_REG_DEVICE_ID_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DEVICE_ID_val = %x \n", INA229_REG_DEVICE_ID_val);
				break;
			default:
				break;

			}

		} else
		{
			printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
		}
	}

	// SPI DMA TX-RX
	switch (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) INA229_send_packet, (uint8_t*) INA229_recv_packet, INA229_msg_lenght_cntr_repeat_previous))
	{
	case HAL_OK:
		/* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT:
		/* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:
		/* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	GPIOC->BRR = (1<<8); // Reset

	while(!HAL_SPI_TxRxCpltCallback_finished)
	{
	}

}

__INLINE void SPI_DMA_TX_printf(void)
{

	GPIOC->BSRR = (1<<8); // Set


	printf("********** SPI_DMA_TXRX_printf **********\n");

	// pls print
	HAL_SPI_TxRxCpltCallback_with_printf = 1;

	// do not repeat previous SPI commands
	HAL_SPI_TxCpltCallback_repeat_previous = 0;

	// flag to wait until recive SPI
	HAL_SPI_TxRxCpltCallback_finished = 0;

	//	for(uint16_t cntr = 0; cntr < INA229_msg_lenght_cntr ; cntr++)
	//	{
	//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
	//	}

	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
	{

		if (INA229_send_packet_decoder[cntr] != 0) // decode
		{
			INA229_decodedAddress = INA229_send_packet_decoder[cntr] - 1;
			printf("INA229_send_packet_decoder[%d] = %x \n", cntr , INA229_decodedAddress);
			switch(INA229_decodedAddress)
			{
			case INA229_REG_CONFIG: // 00h, 2 bytes
				INA229_REG_CONFIG_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_CONFIG_val = %x \n", INA229_REG_CONFIG_val);
				break;
			case INA229_REG_ADC_CONFIG: // 01h, 2 bytes
				INA229_REG_ADC_CONFIG_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_ADC_CONFIG_val = %x \n", INA229_REG_ADC_CONFIG_val);
				break;
			case INA229_REG_SHUNT_CAL: // 02h, 2 bytes
				INA229_REG_SHUNT_CAL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_CAL_val = %x \n", INA229_REG_SHUNT_CAL_val);
				break;
			case INA229_REG_SHUNT_TEMPCO: // 03h, 2 bytes
				INA229_REG_SHUNT_TEMPCO_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_TEMPCO_val = %x \n", INA229_REG_SHUNT_TEMPCO_val);
				break;
			case INA229_REG_VSHUNT: // 04h, 3 bytes
				INA229_REG_VSHUNT_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_VSHUNT_val = %x \n", INA229_REG_VSHUNT_val);
				break;
			case INA229_REG_VBUS: // 05h, 3 bytes
				INA229_REG_VBUS_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_VBUS_val = %x \n", INA229_REG_VBUS_val);
				break;
			case INA229_REG_DIETEMP: //06h, 2 bytes
				INA229_REG_DIETEMP_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DIETEMP_val = %x \n", INA229_REG_DIETEMP_val);
				break;
			case INA229_REG_CURRENT: // 07h, 3 bytes
				INA229_REG_CURRENT_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_CURRENT_val = %x \n", INA229_REG_CURRENT_val);
				break;
			case INA229_REG_POWER: // 08h, 3 bytes
				INA229_REG_POWER_val = combine_3_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3]);
				printf("INA229_REG_POWER_val = %x \n", INA229_REG_POWER_val);
				break;
			case INA229_REG_ENERGY: // 09h, 5 bytes
				INA229_REG_ENERGY_val = combine_5_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3], INA229_send_packet[cntr + 4], INA229_send_packet[cntr + 5]);
				printf("INA229_REG_ENERGY_val = %x%08x \n", INA229_REG_ENERGY_val);
				break;
			case INA229_REG_CHARGE: // 0ah, 5 bytes
				INA229_REG_CHARGE_val = combine_5_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2], INA229_send_packet[cntr + 3], INA229_send_packet[cntr + 4], INA229_send_packet[cntr + 5]);
				printf("INA229_REG_CHARGE_val = %x%08x \n", INA229_REG_CHARGE_val);
				break;
			case INA229_REG_DIAG_ALRT: // 0bh
				INA229_REG_DIAG_ALRT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DIAG_ALRT_val = %x \n", INA229_REG_DIAG_ALRT_val);
				break;
			case INA229_REG_SOVL: // 0ch
				INA229_REG_SOVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SOVL_val = %x \n", INA229_REG_SOVL_val);
				break;
			case INA229_REG_SUVL:
				INA229_REG_SUVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_SUVL_val = %x \n", INA229_REG_SUVL_val);
				break;
			case INA229_REG_BOVL:
				INA229_REG_BOVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_BOVL_val = %x \n", INA229_REG_BOVL_val);
				break;
			case INA229_REG_BUVL:
				INA229_REG_BUVL_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_BUVL_val = %x \n", INA229_REG_BUVL_val);
				break;
			case INA229_REG_TEMP_LIMIT:
				INA229_REG_TEMP_LIMIT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_TEMP_LIMIT_val = %x \n", INA229_REG_TEMP_LIMIT_val);
				break;
			case INA229_REG_PWR_LIMIT:
				INA229_REG_PWR_LIMIT_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_PWR_LIMIT_val = %x \n", INA229_REG_PWR_LIMIT_val);
				break;
			case INA229_REG_MANUFACTURER_ID:
				INA229_REG_MANUFACTURER_ID_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_MANUFACTURER_ID_val = %x \n", INA229_REG_MANUFACTURER_ID_val);
				break;
			case INA229_REG_DEVICE_ID:
				INA229_REG_DEVICE_ID_val = combine_2_bytes(INA229_send_packet[cntr + 1], INA229_send_packet[cntr + 2]);
				printf("INA229_REG_DEVICE_ID_val = %x \n", INA229_REG_DEVICE_ID_val);
				break;
			default:
				break;

			}

		} else
		{
			printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
		}
	}

	// SPI DMA TX-RX
	switch (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) INA229_send_packet, (uint8_t*) INA229_recv_packet, INA229_msg_lenght_cntr))
	{
	case HAL_OK:
		/* Communication is completed ___________________________________________ */
		break;
	case HAL_TIMEOUT:
		/* An Error Occur ______________________________________________________ */
		printf("SPI HAL_TIMEOUT \n");
	case HAL_ERROR:
		/* Call Timeout Handler */
		printf("SPI HAL_ERROR \n");
		Error_Handler();
		break;
	default:
		break;
	}

	GPIOC->BRR = (1<<8); // Reset

	while(!HAL_SPI_TxRxCpltCallback_finished)
	{
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);

	/* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
	 */

	GPIOB->BSRR = (1<<9); // Set
	//	VT_INA229_ReadReg(INA229_REG_VSHUNT);
	//	SPI_DMA_TXRX();
	GPIOB->BRR = (1<<9); // Reset

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */

	if (HAL_SPI_TxRxCpltCallback_with_printf)
		HAL_SPI_RxCpltCallback_printf();
	else
		HAL_SPI_RxCpltCallback_no_printf();

	if(!HAL_SPI_TxCpltCallback_repeat_previous)
		VT_INA229_ResetVars();

}

void HAL_SPI_RxCpltCallback_printf()
{
	GPIOC->BSRR = (1<<6); // Set

	printf("********** HAL_SPI_TxRxCpltCallback_printf ***********\n");

	if(HAL_SPI_TxCpltCallback_repeat_previous)
		INA229_msg_lenght_cntr = INA229_msg_lenght_cntr_repeat_previous;

	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
	{

		if (INA229_send_packet_decoder[cntr] != 0) // decode
		{
			INA229_decodedAddress = INA229_send_packet_decoder[cntr] - 1;
			printf("INA229_send_packet_decoder[%d] = %x \n", cntr , INA229_decodedAddress);
			switch(INA229_decodedAddress)
			{
			case INA229_REG_CONFIG: // 00h, 2 bytes
				INA229_REG_CONFIG_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_CONFIG_val = %x \n", INA229_REG_CONFIG_val);
				break;
			case INA229_REG_ADC_CONFIG: // 01h, 2 bytes
				INA229_REG_ADC_CONFIG_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_ADC_CONFIG_val = %x \n", INA229_REG_ADC_CONFIG_val);
				break;
			case INA229_REG_SHUNT_CAL: // 02h, 2 bytes
				INA229_REG_SHUNT_CAL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_CAL_val = %x \n", INA229_REG_SHUNT_CAL_val);
				break;
			case INA229_REG_SHUNT_TEMPCO: // 03h, 2 bytes
				INA229_REG_SHUNT_TEMPCO_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_SHUNT_TEMPCO_val = %x \n", INA229_REG_SHUNT_TEMPCO_val);
				break;
			case INA229_REG_VSHUNT: // 04h, 3 bytes
				INA229_REG_VSHUNT_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_VSHUNT_val = %x \n", INA229_REG_VSHUNT_val);
				break;
			case INA229_REG_VBUS: // 05h, 3 bytes
				INA229_REG_VBUS_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_VBUS_val = %x \n", INA229_REG_VBUS_val);
				break;
			case INA229_REG_DIETEMP: //06h, 2 bytes
				INA229_REG_DIETEMP_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_DIETEMP_val = %x \n", INA229_REG_DIETEMP_val);
				break;
			case INA229_REG_CURRENT: // 07h, 3 bytes
				INA229_REG_CURRENT_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_CURRENT_val = %x \n", INA229_REG_CURRENT_val);
				break;
			case INA229_REG_POWER: // 08h, 3 bytes
				INA229_REG_POWER_val = combine_3_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_POWER_val = %x \n", INA229_REG_POWER_val);
				break;
			case INA229_REG_ENERGY: // 09h, 5 bytes
				INA229_REG_ENERGY_val = combine_5_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3], INA229_recv_packet[cntr + 4], INA229_recv_packet[cntr + 5]);
				printf("INA229_REG_ENERGY_val = %x%08x \n", INA229_REG_ENERGY_val);
				break;
			case INA229_REG_CHARGE: // 0ah, 5 bytes
				INA229_REG_CHARGE_val = combine_5_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3], INA229_recv_packet[cntr + 4], INA229_recv_packet[cntr + 5]);
				printf("INA229_REG_CHARGE_val = %x%08x \n", INA229_REG_CHARGE_val);
				break;
			case INA229_REG_DIAG_ALRT: // 0bh
				INA229_REG_DIAG_ALRT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_DIAG_ALRT_val = %x \n", INA229_REG_DIAG_ALRT_val);
				break;
			case INA229_REG_SOVL: // 0ch
				INA229_REG_SOVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_SOVL_val = %x \n", INA229_REG_SOVL_val);
				break;
			case INA229_REG_SUVL:
				INA229_REG_SUVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_SUVL_val = %x \n", INA229_REG_SUVL_val);
				break;
			case INA229_REG_BOVL:
				INA229_REG_BOVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_BOVL_val = %x \n", INA229_REG_BOVL_val);
				break;
			case INA229_REG_BUVL:
				INA229_REG_BUVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_BUVL_val = %x \n", INA229_REG_BUVL_val);
				break;
			case INA229_REG_TEMP_LIMIT:
				INA229_REG_TEMP_LIMIT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_TEMP_LIMIT_val = %x \n", INA229_REG_TEMP_LIMIT_val);
				break;
			case INA229_REG_PWR_LIMIT:
				INA229_REG_PWR_LIMIT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_PWR_LIMIT_val = %x \n", INA229_REG_PWR_LIMIT_val);
				break;
			case INA229_REG_MANUFACTURER_ID:
				INA229_REG_MANUFACTURER_ID_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_MANUFACTURER_ID_val = %x \n", INA229_REG_MANUFACTURER_ID_val);
				break;
			case INA229_REG_DEVICE_ID:
				INA229_REG_DEVICE_ID_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_DEVICE_ID_val = %x \n", INA229_REG_DEVICE_ID_val);
				break;
			default:
				break;

			}

		} else
		{
			printf("INA229_recv_packet[%d] = %x \n", cntr , INA229_recv_packet[cntr]);
		}
	}

	HAL_SPI_TxRxCpltCallback_finished = 1;

	// VT_INA229_ResetVars(); // transfer to HAL_SPI_TxRxCpltCallback()

	GPIOC->BRR = (1<<6); // Reset
}

__INLINE void HAL_SPI_RxCpltCallback_no_printf()
{
	GPIOC->BSRR = (1<<6); // Set

	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
	{

		if (INA229_send_packet_decoder[cntr] != 0) // decode
		{
			INA229_decodedAddress = INA229_send_packet_decoder[cntr] - 1;
			//			printf("INA229_send_packet_decoder[%d] = %x \n", cntr , INA229_decodedAddress);
			switch(INA229_decodedAddress)
			{
			case INA229_REG_CONFIG: // 00h, 2 bytes
				INA229_REG_CONFIG_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_CONFIG_val = %x \n", INA229_REG_CONFIG_val);
				break;
			case INA229_REG_ADC_CONFIG: // 01h, 2 bytes
				INA229_REG_ADC_CONFIG_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_ADC_CONFIG_val = %x \n", INA229_REG_ADC_CONFIG_val);
				break;
			case INA229_REG_SHUNT_CAL: // 02h, 2 bytes
				INA229_REG_SHUNT_CAL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_SHUNT_CAL_val = %x \n", INA229_REG_SHUNT_CAL_val);
				break;
			case INA229_REG_SHUNT_TEMPCO: // 03h, 2 bytes
				INA229_REG_SHUNT_TEMPCO_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_SHUNT_TEMPCO_val = %x \n", INA229_REG_SHUNT_TEMPCO_val);
				break;
			case INA229_REG_VSHUNT: // 04h, 3 bytes
				INA229_REG_VSHUNT_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				//				printf("INA229_REG_VSHUNT_val = %x \n", INA229_REG_VSHUNT_val);
				break;
			case INA229_REG_VBUS: // 05h, 3 bytes
				INA229_REG_VBUS_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				//				printf("INA229_REG_VBUS_val = %x \n", INA229_REG_VBUS_val);
				break;
			case INA229_REG_DIETEMP: //06h, 2 bytes
				INA229_REG_DIETEMP_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_DIETEMP_val = %x \n", INA229_REG_DIETEMP_val);
				break;
			case INA229_REG_CURRENT: // 07h, 3 bytes
				INA229_REG_CURRENT_val = combine_3_bytes_to_signed_20bits(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				//				printf("INA229_REG_CURRENT_val = %x \n", INA229_REG_CURRENT_val);
				break;
			case INA229_REG_POWER: // 08h, 3 bytes
				INA229_REG_POWER_val = combine_3_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				//				printf("INA229_REG_POWER_val = %x \n", INA229_REG_POWER_val);
				break;
			case INA229_REG_ENERGY: // 09h, 5 bytes
				INA229_REG_ENERGY_val = combine_5_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3], INA229_recv_packet[cntr + 4], INA229_recv_packet[cntr + 5]);
				//				printf("INA229_REG_ENERGY_val = %x%08x \n", INA229_REG_ENERGY_val);
				break;
			case INA229_REG_CHARGE: // 0ah, 5 bytes
				INA229_REG_CHARGE_val = combine_5_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3], INA229_recv_packet[cntr + 4], INA229_recv_packet[cntr + 5]);
				//				printf("INA229_REG_CHARGE_val = %x%08x \n", INA229_REG_CHARGE_val);
				break;
			case INA229_REG_DIAG_ALRT: // 0bh
				INA229_REG_DIAG_ALRT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_DIAG_ALRT_val = %x \n", INA229_REG_DIAG_ALRT_val);
				break;
			case INA229_REG_SOVL: // 0ch
				INA229_REG_SOVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_SOVL_val = %x \n", INA229_REG_SOVL_val);
				break;
			case INA229_REG_SUVL:
				INA229_REG_SUVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_SUVL_val = %x \n", INA229_REG_SUVL_val);
				break;
			case INA229_REG_BOVL:
				INA229_REG_BOVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_BOVL_val = %x \n", INA229_REG_BOVL_val);
				break;
			case INA229_REG_BUVL:
				INA229_REG_BUVL_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_BUVL_val = %x \n", INA229_REG_BUVL_val);
				break;
			case INA229_REG_TEMP_LIMIT:
				INA229_REG_TEMP_LIMIT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_TEMP_LIMIT_val = %x \n", INA229_REG_TEMP_LIMIT_val);
				break;
			case INA229_REG_PWR_LIMIT:
				INA229_REG_PWR_LIMIT_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_PWR_LIMIT_val = %x \n", INA229_REG_PWR_LIMIT_val);
				break;
			case INA229_REG_MANUFACTURER_ID:
				INA229_REG_MANUFACTURER_ID_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_MANUFACTURER_ID_val = %x \n", INA229_REG_MANUFACTURER_ID_val);
				break;
			case INA229_REG_DEVICE_ID:
				INA229_REG_DEVICE_ID_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				//				printf("INA229_REG_DEVICE_ID_val = %x \n", INA229_REG_DEVICE_ID_val);
				break;
			default:
				break;

			}

		} else
		{
			//			printf("INA229_recv_packet[%d] = %x \n", cntr , INA229_recv_packet[cntr]);
		}
	}

	HAL_SPI_TxRxCpltCallback_finished = 1;

	// VT_INA229_ResetVars(); // transfer to HAL_SPI_TxRxCpltCallback()

	GPIOC->BRR = (1<<6); // Reset
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
	 */
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	//	printf("HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);

	switch(UartRxBuffer[0])
	{
	case 0x30: // 0
		// MCU Reset
		printf("MCU Reset \n");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		break;
	case 0x31: // 1
		// INA229 Reset
		printf("INA229 Reset \n");
		INA229_Write_CONFIG(INA229_REG_CFG_RESET);
		SPI_DMA_TX();
		break;
	case 0x32: // 2
		// INA229 Read All
		VT_INA229_ReadAllReg();
		SPI_DMA_TX();
		break;
	case 0x33: // 3
		// Continous Vshunt 50uS
		INA229_Write_ADC_CONFIG(INA229_MODE_CONTINOUS_VSHUNT | INA229_VBUSCT_50 | INA229_VSHCT_50 | INA229_VTCT_50 | INA229_AVG_1);
		SPI_DMA_TX();
		break;
	case 0x34: // 4
		// INA229_REG_VSHUNT
		VT_INA229_ReadReg(INA229_REG_VSHUNT);
		SPI_DMA_TX();
		break;
	case 0x35: // 5
		// HRTIM on
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1);
		HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
		break;
	case 0x36: // #2 & 3
		INA229_Write_ADC_CONFIG(INA229_MODE_CONTINOUS_VSHUNT | INA229_VBUSCT_50 | INA229_VSHCT_50 | INA229_VTCT_50 | INA229_AVG_1);
		INA229_Write_DIAG_ALERT(INA229_REG_DIAG_ALERT_CNVR);
		SPI_DMA_TX();
		break;
	case 0x37:
		// repeat previous SPI command
		SPI_DMA_TX_repeat_previous();
		break;
	case 0x38:
		// INA229 Write Diag alert
		INA229_Write_DIAG_ALERT(INA229_REG_DIAG_ALERT_CNVR);
		SPI_DMA_TX();
		break;
	case 0x39: // 9
		// INA229_REG_DIAG_ALRT
		VT_INA229_ReadReg(INA229_REG_DIAG_ALRT);
		SPI_DMA_TX();
		break;
	case 0x61: // a
		// INA229 Reset
		printf("INA229 Reset \n");
		INA229_Write_CONFIG(INA229_REG_CFG_RESET);
		SPI_DMA_TX();
		break;
	case 0x62: // b
		// Read All w/ print
		VT_INA229_ReadAllReg();
		SPI_DMA_TX_printf();
		break;
	case 0x63: // c
		// Continous Vshunt 50uS w/ print
		INA229_Write_ADC_CONFIG(INA229_MODE_CONTINOUS_VSHUNT | INA229_VBUSCT_50 | INA229_VSHCT_50 | INA229_VTCT_50 | INA229_AVG_1);
		SPI_DMA_TX_printf();
		break;
	case 0x64: // d
		// INA229_REG_VSHUNT w/ print
		VT_INA229_ReadReg(INA229_REG_VSHUNT);
		SPI_DMA_TX_printf();
		break;
	case 0x65: // e
		// HRTIM on
		printf("HRTIM on \n");
		HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1);
		HAL_HRTIM_WaveformCountStart_IT(&hhrtim1, HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B);
		break;
	case 0x66: // f
		// INA229_REG_VSHUNT INA229_REG_VBUS INA229_REG_DIETEMP w/ print
		VT_INA229_ReadReg(INA229_REG_VSHUNT);
		VT_INA229_ReadReg(INA229_REG_VBUS);
		VT_INA229_ReadReg(INA229_REG_DIETEMP);
		SPI_DMA_TX_printf();
		break;
	case 0x67: // g
		VT_SEND_SPI_VSHUNT();
		break;
	case 0x7a: // z
		// MCU Reset
		printf("MCU Reset \n");
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
		break;
	default: // None
		printf("No data found! \n");
		break;
	}

}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	/* Prevent unused argument(s) compilation warning */
//	UNUSED(hspi);
//
//	/* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_SPI_RxCpltCallback should be implemented in the user file
//	 */
//
//	printf("HAL_SPI_RxCpltCallback \n");
//
//}


//void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi)
//{
//	/* Prevent unused argument(s) compilation warning */
//	UNUSED(hspi);
//
//	/* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_SPI_RxCpltCallback should be implemented in the user file
//	 */
//
//	printf("HAL_SPI_RxHalfCpltCallback \n");
//
//}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error_Handler \n");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	__disable_irq();
	while (1) {
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
