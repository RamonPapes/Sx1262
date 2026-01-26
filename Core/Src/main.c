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

//#include "sx126x_hal.h"
//#include "dev_sx126x/Sx126x.h"
#include "../Libs/dev_sx1262/sx1262.h"
#include "../Libs/dev_sx1262/sx1262_hal.h"

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

SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */
//sx1262_handle_t sx;
volatile uint8_t Rec_int_flg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
static void config_lora(sx1262_t *dev);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//extern SX1262 SX_stc;
void sx1262_receive_callback(uint8_t *payload, uint8_t payload_len) {
	asm volatile("nop");
	// Handle received payload
	// For example, you can process or store the received data here
}

//uint8_t recbuf[20];
//uint8_t Len=0;
//HAL_StatusTypeDef err;
//volatile uint8_t Rec_int_flg;
//uint8_t counter = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_SPI4_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	sx1262_t sx1262_dev;
	sx1262_hal_t sx1262_hal;

	sx1262_hal.spi_transmit_receive = sx1262_hal_spi_transmit_receive;
	sx1262_hal.spi_cs_low = sx1262_hal_spi_cs_low;
	sx1262_hal.spi_cs_high = sx1262_hal_spi_cs_high;
	sx1262_hal.gpio_read_busy = sx1262_hal_gpio_read_busy;
	sx1262_hal.gpio_write_reset = sx1262_hal_gpio_write_reset;
	sx1262_hal.delay_ms = sx1262_hal_delay_ms;
	sx1262_hal.get_tick = sx1262_hal_get_tick;
	sx1262_hal.rx_callback = sx1262_receive_callback;
	sx1262_dev.initialized = SX1262_BOOL_FALSE;

	sx1262_status_t err = sx1262_init(&sx1262_dev, &sx1262_hal);

	if (err != SX1262_OK) {
		// Handle initialization error
		while (1)
			;
	}

	config_lora(&sx1262_dev);

	err = sx1262_lora_transmit_it(&sx1262_dev, (uint8_t *) "send_test", 10);

//	uint8_t buf[256];
//	uint8_t len = 0;
//	uint32_t timeout = 10000; // 10 seconds timeout
//	err = sx1262_receive_single_packet(&sx1262_dev, buf, len, timeout);



//	err = sx1262_set_mode_continuous_receive(&sx1262_dev);
//
	if (err != SX1262_OK) {
		// Handle initialization error
		while (1)
			;
	}



	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (Rec_int_flg) {
			Rec_int_flg = 0;
			sx1262_irq_handler(&sx1262_dev);
		}

//			SX1262_HandleCallback(recbuf, &Len);
//			SX1262_setModeReceive();

		sx1262_lora_transmit(&sx1262_dev, (uint8_t *)"send_test", 10, 1000);
		HAL_Delay(3000);

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 15;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 0x0;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi4.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi4.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : SPI4_CS_Pin */
	GPIO_InitStruct.Pin = SPI4_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_1262_DIO1_Pin */
	GPIO_InitStruct.Pin = LORA_1262_DIO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(LORA_1262_DIO1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_1262_RST_Pin */
	GPIO_InitStruct.Pin = LORA_1262_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LORA_1262_RST_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_1262_BUSY_Pin */
	GPIO_InitStruct.Pin = LORA_1262_BUSY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_1262_BUSY_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(LORA_1262_DIO1_EXTI_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(LORA_1262_DIO1_EXTI_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void config_lora(sx1262_t *dev) {
	sx1262_status_t ret = sx1262_set_dio2_as_rf_switch_crtl(dev);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_packet_type(dev, PACKET_TYPE_LORA);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_frequency(dev, 433000000);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_stop_timer_on_sync_word(dev);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_lora_modulation_params(dev, SX1262_LORA_SF_10,
			SX1262_LORA_BANDWIDTH_125_KHZ, SX1262_LORA_CR_4_6,
			SX1262_BOOL_TRUE);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_pa_config(dev, 0x04, 0x07, 0, 0x01);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_tx_params(dev, 15, SX1262_RAMP_TIME_200US);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_lora_symbol_timeout(dev, 0x00);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = sx1262_set_lora_packet_params(dev, 10,
			SX1262_LORA_HEADER_EXPLICIT, 0xFF, 0, 0);

	if(ret != SX1262_OK) {
		while(1);
	}

	uint16_t irq_mask = SX1262_IRQ_TX_DONE | SX1262_IRQ_RX_DONE
			| SX1262_IRQ_CRC_ERR | SX1262_IRQ_HEADER_ERR | SX1262_IRQ_TIMEOUT;

	uint16_t dio1_mask = SX1262_IRQ_RX_DONE | SX1262_IRQ_TX_DONE
			| SX1262_IRQ_TIMEOUT;

	ret = sx1262_set_dio_irq_params(dev, irq_mask, dio1_mask);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

	ret = SX1262_set_fallback_mode(dev, SX1262_RX_TX_FALLBACK_MODE_STDBY_XOSC);

	if (ret != SX1262_OK) {
		while (1)
			;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == LORA_1262_DIO1_Pin) {
		Rec_int_flg = 1;
	}
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
