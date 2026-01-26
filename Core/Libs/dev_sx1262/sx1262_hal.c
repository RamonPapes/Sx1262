#include "sx126x_hal.h"


void sx1262_hal_spi_cs_low(void){
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
}

void sx1262_hal_spi_cs_high(void){
	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
}

uint8_t sx1262_hal_gpio_read_busy(void){
	return HAL_GPIO_ReadPin(LORA_1262_BUSY_GPIO_Port, LORA_1262_BUSY_Pin);
}

void sx1262_hal_gpio_write_reset(uint8_t state){
	if(state == 1){
		HAL_GPIO_WritePin(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin, GPIO_PIN_RESET);
	}
}

void sx1262_hal_spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout){
	HAL_SPI_TransmitReceive(&hspi4, tx_data, rx_data, size, timeout);
}

void sx1262_hal_delay_ms(uint32_t ms){
	HAL_Delay(ms);
}

uint32_t sx1262_hal_get_tick(void){
	return HAL_GetTick();
}

void rx_callback(uint8_t *data, uint8_t length){
	return;
}


