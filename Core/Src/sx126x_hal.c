#include "sx126x_hal.h"

#include "usbd_cdc_if.h"
#include "string.h"
#include "stdarg.h"

uint8_t sx1262_reset_gpio_init(void) {
//	HAL_GPIO_Init(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin);
	return 0;
}

uint8_t sx1262_reset_gpio_deinit(void) {
//	HAL_GPIO_DeInit(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin);
	return 0;
}

uint8_t sx1262_reset_gpio_write(uint8_t value) {
	HAL_GPIO_WritePin(LORA_1262_RST_GPIO_Port, LORA_1262_RST_Pin,
			(GPIO_PinState) value);
	return 0;
}

uint8_t sx1262_busy_gpio_init(void) {
//	HAL_GPIO_Init(LORA_1262_BUSY_GPIO_Port, LORA_1262_BUSY_Pin);
	return 0;
}

uint8_t sx1262_busy_gpio_deinit(void) {
//	HAL_GPIO_DeInit(LORA_1262_BUSY_GPIO_Port, LORA_1262_BUSY_Pin);
	return 0;
}

uint8_t sx1262_busy_gpio_read(uint8_t *value) {
	if (value == NULL)
		return 1;

	*value = (uint8_t) HAL_GPIO_ReadPin(LORA_1262_BUSY_GPIO_Port,
	LORA_1262_BUSY_Pin);
	return 0;
}

uint8_t sx1262_spi_init(void) {
	HAL_StatusTypeDef ret = HAL_SPI_Init(&hspi4);
	if (ret != HAL_OK) {
		return 1;
	}
	return 0;

	return ret;
}

uint8_t sx1262_spi_deinit(void) {
	HAL_StatusTypeDef ret = HAL_SPI_DeInit(&hspi4);

	if (ret != HAL_OK) {
		return 1;
	}
	return 0;

}

uint8_t sx1262_spi_write_read(uint8_t *tx_buf, uint32_t tx_len, uint8_t *rx_buf,
		uint32_t rx_len) {

	uint8_t err = 0;

	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);

	uint8_t dummy = 0x00;

	HAL_StatusTypeDef ret = HAL_OK;
	if (tx_buf != NULL && tx_len > 0) {
		ret = HAL_SPI_Transmit(&hspi4, tx_buf, tx_len, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
			err = 1;
			return err;
		}
	}

	if (rx_buf != NULL && rx_len > 0) {
		ret = HAL_SPI_Receive(&hspi4, rx_buf, rx_len, HAL_MAX_DELAY);
		if (ret != HAL_OK) {
			err = 1;
			HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
			return err;
		}
	}

	HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);

	return err;
}

void sx1262_delay_ms(uint32_t ms) {
	HAL_Delay(ms);
}

void sx1262_debug_print(const char *const fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char buffer[256];
	int len = vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	CDC_Transmit_HS((uint8_t*) buffer, len);
}

void sx1262_receive_callback(uint16_t type, uint8_t *buf, uint16_t len) {
	uint8_t *rx = NULL;
	if (type & SX1262_IRQ_RX_DONE) {
		memcpy(rx, buf, len);
	}

	if (type & SX1262_IRQ_CRC_ERR) {
		return;
	}
}
