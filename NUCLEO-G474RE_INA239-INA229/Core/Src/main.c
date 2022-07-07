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
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

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
extern uint8_t INA229_msg_lenght_cntr;
extern uint8_t INA229_send_packet[100], INA229_send_packet_decoder[100];
extern uint8_t INA229_recv_packet[100], INA229_recv_packet_decoder[100];

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

extern uint8_t INA239_msg_lenght_cntr;
extern uint8_t INA239_send_packet[100], INA239_send_packet_decoder[100];
extern uint8_t INA239_recv_packet[100], INA239_recv_packet_decoder[100];

uint8_t INA239decodedAddress, INA239decodedReg, INA239decodedCommand;

// UART Variables
uint8_t UartRxBuffer[1];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void SPI_DMA_TXRX(void);
extern void VT_INA229_ReadAllReg(void);
extern void VT_INA229_ReadRegPartial1(void);
extern uint16_t combine_2_bytes(uint16_t high_byte, uint16_t low_byte);
extern uint32_t combine_3_bytes(uint32_t high_byte, uint32_t mid_byte, uint32_t low_byte);
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
  /* USER CODE BEGIN 2 */

	printf("Starting >> NUCLEO-G474RE_INA239-INA229 \n");

	if (HAL_UART_Receive_DMA(&hlpuart1, (uint8_t *)UartRxBuffer, 1) != HAL_OK)
	{
		Error_Handler();
	}

	// reset everything
	for (uint8_t cntr = 0; cntr < sizeof(INA229_send_packet); cntr++)
	{
		INA229_send_packet[cntr] = 0xff;
		INA229_recv_packet[cntr] = 0xff;
		INA229_send_packet_decoder[cntr] = 0;
		INA229_recv_packet_decoder[cntr] = 0;
		//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
	}

//	/* Wait for User push-button press before starting the Communication */
//	while (!button_pressed)
//	{
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
//		HAL_Delay(100);
//	}
//	button_pressed = 0; // reset button variable so can be used again

//	VT_INA229_ReadAllReg();
//
//	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
//	{
//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
//	}

	// SPI DMA TX-RX
//	SPI_DMA_TXRX();

	//	HAL_SPI_DMAStop(&hspi3);

	//	VT_INA229_ReadRegPartial1();
	//
	//	for (uint8_t cntr = 0; cntr < INA229_msg_lenght_cntr; cntr++)
	//	{
	//		printf("INA229_send_packet[%d] = %x \n", cntr , INA229_send_packet[cntr]);
	//	}

	// SPI DMA TX-RX
//	SPI_DMA_TXRX();


	printf("Ending   >> NUCLEO-G474RE_INA239-INA229 \n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		HAL_Delay(500);
		if (duty_cycle == 0)
		{
			duty_cycle_increase = 1; //duty_cycle + 10; // div by 10; // div by 10
			HAL_Delay(4000);
		}
		if (duty_cycle == 1000)
		{
			duty_cycle_increase = 0;
			HAL_Delay(4000);
		}
		if (duty_cycle_increase)
			duty_cycle = duty_cycle + 10;
		else
			duty_cycle = duty_cycle - 10;

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
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  hlpuart1.Init.BaudRate = 57600;
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void SPI_DMA_TXRX(void)
{
	printf("SPI_DMA_TXRX \n");
	// SPI DMA TX-RX
	switch (HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) INA229_send_packet, (uint8_t*) INA229_recv_packet, INA229_msg_lenght_cntr)) {
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
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);

	/* NOTE: This function should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
	 */

	button_pressed = 1;

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hspi);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */

	printf("HAL_SPI_TxRxCpltCallback \n");

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
				INA229_REG_VSHUNT_val = combine_3_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_VSHUNT_val = %x \n", INA229_REG_VSHUNT_val);
				break;
			case INA229_REG_VBUS: // 05h, 3 bytes
				INA229_REG_VBUS_val = combine_3_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
				printf("INA229_REG_VBUS_val = %x \n", INA229_REG_VBUS_val);
				break;
			case INA229_REG_DIETEMP: //06h, 2 bytes
				INA229_REG_DIETEMP_val = combine_2_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2]);
				printf("INA229_REG_DIETEMP_val = %x \n", INA229_REG_DIETEMP_val);
				break;
			case INA229_REG_CURRENT: // 07h, 3 bytes
				INA229_REG_CURRENT_val = combine_3_bytes(INA229_recv_packet[cntr + 1], INA229_recv_packet[cntr + 2], INA229_recv_packet[cntr + 3]);
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

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(huart);

	/* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
	 */
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);

	switch(UartRxBuffer[0])
	{
	case 0x30: // 0
		printf(" INA229 Reset - HAL_UART_RxCpltCallback \n");

		break;
	case 0x31: // 1
		printf(" INA229 Read All - HAL_UART_RxCpltCallback \n");
		VT_INA229_ReadAllReg();
		break;
	case 0x32:
		printf(" VT_INA229_ReadRegPartial1 - HAL_UART_RxCpltCallback \n");
		VT_INA229_ReadRegPartial1();
		break;
	case 0x33:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x34:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x35:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x36:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x37:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x38:
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x39: // 9
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x61: // a
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x62: // b
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	case 0x63: // c
		printf(" HAL_UART_RxCpltCallback = %x \n", UartRxBuffer[0]);
		break;
	default: // None
		printf(" No data found! \n");
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
