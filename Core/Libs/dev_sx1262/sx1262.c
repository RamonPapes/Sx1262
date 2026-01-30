/*
 * sx1262.c
 *
 *  Created on: Jan 15, 2026
 *      Author: ramon.papes
 */

#include "sx1262.h"
#include "string.h"

/* ===== Private function prototypes ================================ */

static void sx1262_get_image_calib_params(uint32_t freq_hz, uint8_t *freq1,
		uint8_t *freq2);

/**
 * @brief Set CS pin low
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
static sx1262_status_t sx1262_cs_low(const sx1262_t *dev);

/**
 * @brief Set CS pin high
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
static sx1262_status_t sx1262_cs_high(const sx1262_t *dev);

/**
 * @brief Wait until device is not busy or timeout occurs
 * @param[in] dev Pointer to device handle
 * @param[in] timeout Timeout in milliseconds
 * @return SX1262_OK if device is ready | SX1262_TIMEOUT if timeout occurs | error code
 */
static sx1262_status_t sx1262_busy_wait(const sx1262_t *dev, uint32_t timeout);

/**
 * @brief Check if device is busy
 * @param[in] dev Pointer to device handle
 * @return 1 if busy, 0 if not busy | error code
 */
static uint8_t sx1262_is_busy(const sx1262_t *dev);

/**
 * @brief Send SPI command to device
 * @param[in] dev Pointer to device handle
 * @param[in] tx_data Data to transmit
 * @param[out] rx_data Buffer for received data
 * @param[in] tx_len Length of data to transmit
 * @param[in] rx_len Expected response length
 * @param[in] timeout_ms Timeout in milliseconds
 */
static sx1262_status_t sx1262_send_command(sx1262_t *dev, uint8_t *cmd, uint8_t *res,
		uint16_t len, uint32_t timeout, uint16_t delay);



/* ===== Private function definitions ================================ */
static void sx1262_get_image_calib_params(uint32_t freq_hz, uint8_t *freq1,
		uint8_t *freq2) {
	// Calculate frequency parameters based on the input frequency

	if (freq_hz >= 430000000 && freq_hz <= 440000000) {
		*freq1 = 0x6B;
		*freq2 = 0x6F;
	}
	else if (freq_hz >= 470000000 && freq_hz <= 510000000) {
		*freq1 = 0x75;
		*freq2 = 0x81;
	}
	else if (freq_hz >= 779000000 && freq_hz <= 787000000) {
		*freq1 = 0xC1;
		*freq2 = 0xC5;
	}
	else if (freq_hz >= 863000000 && freq_hz <= 870000000) {
		*freq1 = 0xD7;
		*freq2 = 0xDB;
	}
	else if (freq_hz >= 902000000 && freq_hz <= 928000000) {
		*freq1 = 0xE1;
		*freq2 = 0xE9;
	}
	else {
		/* Fallback seguro */
		*freq1 = 0xE1;
		*freq2 = 0xE9;
	}
}

/**
 * @brief Set CS pin low
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
static sx1262_status_t sx1262_cs_low(const sx1262_t *dev) {
	if (dev->hal.spi_cs_low == NULL) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	dev->hal.spi_cs_low();
	return SX1262_OK;;
}

/**
 * @brief Set CS pin high
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
static sx1262_status_t sx1262_cs_high(const sx1262_t *dev) {
	if (dev->hal.spi_cs_high == NULL) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	dev->hal.spi_cs_high();
	return SX1262_OK;
}

/**
 * @brief Wait until device is not busy or timeout occurs
 * @param[in] dev Pointer to device handle
 * @param[in] timeout Timeout in milliseconds
 * @return SX1262_OK if device is ready | SX1262_TIMEOUT if timeout occurs | error code
 */
static sx1262_status_t sx1262_busy_wait(const sx1262_t *dev, uint32_t timeout) {
	if (dev->hal.gpio_read_busy == NULL) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint32_t start = dev->hal.get_tick();
	while (sx1262_is_busy(dev)) {
		if ((dev->hal.get_tick() - start) >= timeout) {
			return SX1262_TIMEOUT;
		}
	}

	return SX1262_OK;
}

/**
 * @brief Check if device is busy
 * @param[in] dev Pointer to device handle
 * @return 1 if busy, 0 if not busy | error code
 */
static uint8_t sx1262_is_busy(const sx1262_t *dev) {
	if (dev->hal.gpio_read_busy == NULL) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	return dev->hal.gpio_read_busy();
}

/**
 * @brief Send command to device over SPI
 * @param[in] dev Pointer to device handle
 * @param[in] cmd Command buffer to send
 * @param[out] res Response buffer to receive data
 * @param[in] len Length of command and response
 * @param[in] timeout Timeout in milliseconds
 * @param[in] delay Delay after command execution in milliseconds
 * @return SX1262_OK on success | error code
 */
static sx1262_status_t sx1262_send_command(sx1262_t *dev, uint8_t *cmd, uint8_t *res,
		uint16_t len, uint32_t timeout, uint16_t delay) {
	if (dev->hal.spi_transmit_receive == NULL) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	if (cmd == NULL || res == NULL || len == 0) {
		return SX1262_INVALID_PARAM;
	}

	sx1262_status_t ret;

	ret = sx1262_busy_wait(dev, timeout);

	if (ret != SX1262_OK)
		return ret;

	ret = sx1262_cs_low(dev);
	if (ret != SX1262_OK)
		return ret;

	dev->hal.spi_transmit_receive(cmd, res, len, timeout);

	ret = sx1262_cs_high(dev);

	if (ret != SX1262_OK)
		return ret;

	if (delay > 0) {
		dev->hal.delay_ms(delay);
	}

	return SX1262_OK;
}

/* ===== Public function definitions ================================ */

/**
 * @brief Check device communication
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK if device responds correctly
 */
sx1262_status_t sx1262_get_lora_sync_word(sx1262_t *dev, uint16_t *sync_word) {

	if(!dev || !sync_word) {
		return SX1262_INVALID_PARAM;
	}

	if(dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}


	uint8_t cmd_buf[6] = { 0 };
	uint8_t res_buf[6] = { 0 };

	cmd_buf[0] = SX126X_CMD_READ_REGISTER;
	cmd_buf[1] = (SX126X_REG_LORA_SYNC_WORD_MSB >> 8) & 0xFF; // MSB of register address
	cmd_buf[2] = SX126X_REG_LORA_SYNC_WORD_MSB & 0xFF; // LSB of register address
	cmd_buf[3] = 0x00; // Dummy byte

	/* We expect 2 bytes response for sync word (MSB and LSB) */
	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 6, 100, 0);

	if (ret != SX1262_OK)
		return ret;

	*sync_word = ((uint16_t)res_buf[4] << 8) | res_buf[5];

	return SX1262_OK;
}

/**
 * @brief sx1262 init function
 * @param[in] dev pointer to an sx1262 handle structure
 * @param[in] hal pointer to an sx1262 hal structure
 * @note    none
 */
sx1262_status_t sx1262_init(sx1262_t *dev, const sx1262_hal_t *hal) {
	if (dev == NULL || hal == NULL)
		return SX1262_INVALID_PARAM;

	if (dev->initialized == SX1262_BOOL_TRUE)
		return SX1262_OK;

	if (hal->spi_transmit_receive == NULL)
		return SX1262_INVALID_PARAM;

	if (hal->spi_cs_high == NULL || hal->spi_cs_low == NULL)
		return SX1262_INVALID_PARAM;

	if (hal->gpio_read_busy == NULL || hal->gpio_write_reset == NULL)
		return SX1262_INVALID_PARAM;

	if (hal->delay_ms == NULL || hal->get_tick == NULL)
		return SX1262_INVALID_PARAM;

	uint8_t cmd_buf[4];
	uint8_t res_buf[4];

	memset(dev, 0, sizeof(sx1262_t));

	memcpy(&dev->hal, hal, sizeof(sx1262_hal_t));

	//Reset the device
	sx1262_cs_high(dev);
	hal->gpio_write_reset(0);
	hal->delay_ms(50);
	hal->gpio_write_reset(1);
	hal->delay_ms(100);

	// SetDIO3asTCXOCtrl SPI Transaction
	cmd_buf[0] = SX126X_CMD_SET_DIO3_AS_TCXO_CTRL;
	cmd_buf[1] = SX126X_DIO3_OUTPUT_1_6; //  DIO3 outputs 1.6 V to supply the TCXO
	cmd_buf[2] = SX126X_DIO3_OUTPUT_1_6;
	cmd_buf[3] = SX126X_DIO3_OUTPUT_1_6;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);

	if (ret != SX1262_OK){
		dev->initialized = SX1262_BOOL_FALSE;
		return ret;
	}

	dev->initialized = SX1262_BOOL_TRUE;
	dev->in_receive_mode = SX1262_BOOL_FALSE;
	return SX1262_OK;
}

///**
// * @brief Set device to sleep mode
// * @param[in] dev Pointer to device handle
// * @return SX1262_OK on success | error code
// */
//
//sx1262_status_t sx1262_set_mode_sleep(sx1262_t *dev, bool warm_start,
//bool wake_on_rtc) {
//	if (dev->initialized != SX1262_INITIALIZED) {
//		return SX1262_ERR_NOT_INITIALIZED;
//	}
//
//	sx1262_status_t ret;
//	chip_mode_t mode;
//	sx1262_get_chip_status(dev, &mode, NULL);
//
//	if (mode != CHIP_MODE_STDBY_RC && chip_mode != CHIP_MODE_STDBY_XOSC) {
//		ret = sx1262_set_mode_standby(dev);
//		if (ret != SX1262_OK) {
//			return SX1262_ERROR;
//		}
//		dev->hal.delay_ms(10);
//	}
//
//	uint8_t cmd_buf[2] = { 0 };
//	uint8_t res_buf[2] = { 0 };
//
//	// Build sleepConfig byte
//	uint8_t sleep_config = 0x00;
//
//	// Bit 2: Warm/Cold start
//	if (warm_start) {
//		sleep_config |= (1 << 2); //warm start
//	} else {
//		sleep_config &= ~(1 << 2); // Cold start (default)
//	}
//
//	// Bit 1: RFU (Reserved For Future Use) - always 0
//
//	sleep_config &= ~(1 << 1);
//
//	// Bit 0: RTC timeout enable
//
//	if (wake_on_rtc) {
//		sleep_config |= (1 << 0); //Wake on RTC timeout
//	} else {
//		sleep_config &= ~(1 << 0); // Wake only on NSS
//	}
//
//	//Bits 7:3: Reserved - always 0
//	sleep_config &= 0x07;
//
//	cmd_buf[0] = SX126X_CMD_SET_SLEEP;
//	cmd_buf[1] = sleep_config;
//
//	ret = sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 500);
//
//	if (ret == SX1262_OK) {
//		dev->in_receive_mode = false;
//		dev->mode = CHIP_MODE_SLEEP;
//
//		// According to datasheet: device will be unresponsive for ~500µs
//		// We'll wait a bit more for safety
//		dev->hal.delay_ms(1);
//
//		// Note: After this point, the device will only respond to:
//		// 1. Falling edge on NSS (wakes device)
//		// 2. RTC timeout (if enabled)
//	}
//
//	return ret;
//}

/**
 * @brief Set device to standby mode
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_mode_standby(sx1262_t *dev) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	sx1262_status_t ret;

	uint8_t cmd_buf[2] = { 0 };
	uint8_t res_buf[2] = { 0 };

	uint8_t dummy_byte = 0x01;

	cmd_buf[0] = SX126X_CMD_SET_STANDBY;
	cmd_buf[1] = dummy_byte; //Dummy byte, status will overwrite this byte (STDBY_XOSC 1)

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	chip_mode_t mode;
	command_status_t cmd_status;

	ret = sx1262_get_chip_status(dev, &mode, &cmd_status);

	if (mode != CHIP_MODE_STDBY_XOSC && mode != CHIP_MODE_STDBY_RC) {
		return SX1262_ERROR;
	}

	return ret;
}

/**
 * @brief Set device to receive mode
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_mode_continuous_receive(sx1262_t *dev) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	sx1262_status_t ret;

	uint8_t cmd_buf[4] = { 0 };
	uint8_t res_buf[4] = { 0 };

//	uint8_t dummy_byte = 0xFF;

	chip_mode_t mode;
	command_status_t cmd_status;

//	ret = sx1262_get_chip_status(dev, &mode, &cmd_status);
//
//	if (mode != CHIP_MODE_STDBY_RC && mode != CHIP_MODE_STDBY_XOSC) {
//		// Tentar colocar em standby primeiro
//		ret = sx1262_set_mode_standby(dev);
//		if (ret != SX1262_OK) {
//			return ret;
//		}
//		dev->hal.delay_ms(10);
//	}

	cmd_buf[0] = SX126X_CMD_SET_RX;
	cmd_buf[1] = 0xFF; //Dummy byte, status will overwrite this byte
	cmd_buf[2] = 0xFF;
	cmd_buf[3] = 0xFF;

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	ret = sx1262_get_chip_status(dev, &mode, &cmd_status);

	if (mode != CHIP_MODE_RX) {
		return SX1262_ERROR;
	}

	dev->in_receive_mode = true;

	return ret;
}

/**
 * @brief Set packet type
 * @param[in] dev Pointer to device handle
 * @param[in] packet_type Packet type to set
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_packet_type(sx1262_t *dev, packet_type_t pkt_type) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[2] = { 0 };
	uint8_t res_buf[2] = { 0 };

	sx1262_status_t ret;

	cmd_buf[0] = SX126X_CMD_SET_PACKET_TYPE;
	cmd_buf[1] = (uint8_t) pkt_type;

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	return SX1262_OK;
}
/**
 * @brief Get packet type
 * @param[in] dev Pointer to device handle
 * @param[out] pkt_type Pointer to packet type variable
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_get_packet_type(sx1262_t *dev, packet_type_t *pkt_type) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[3] = { 0 };
	uint8_t res_buf[3] = { 0 };

	sx1262_status_t ret;

	cmd_buf[0] = SX126X_CMD_GET_PACKET_TYPE;
	cmd_buf[1] = 0x00; //Dummy byte, status will overwrite this
	cmd_buf[2] = 0x00; //Dummy byte, status will overwrite this

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 3, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	*pkt_type = (packet_type_t) (res_buf[2] & 0x0F); //Packet type is bits [3:0] (4-bits)

	return SX1262_OK;
}

/**
 * @brief Get chip status
 * @param[in] dev Pointer to device handle
 * @param[out] mode Pointer to chip mode variable
 * @param[out] cmd_status Pointer to command status variable
 * @return chip status
 */
sx1262_status_t sx1262_get_chip_status(sx1262_t *dev, chip_mode_t *mode,
		command_status_t *cmd_status) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[2] = { 0 };
	uint8_t res_buf[2] = { 0 };

	sx1262_status_t ret;

	cmd_buf[0] = SX126X_CMD_GET_STATUS;
	cmd_buf[1] = 0x00; //Dummy byte, status will overwrite this

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	*mode = (chip_mode_t) (res_buf[1] >> 4) & 0x07; //Chip mode is bits [6:4] (3-bits)
	*cmd_status = (res_buf[1] >> 1) & 0x07; //Command status is bits [3:1] (3-bits)

	return SX1262_OK;
}

/**
 * @brief Set RF frequency
 * @param[in] dev Device handle
 * @param[in] frequency_hz Frequency in Hz
 */
sx1262_status_t sx1262_set_frequency(sx1262_t *dev, uint32_t frequency) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	if (frequency < 150000000 || frequency > 960000000) {
		return SX1262_INVALID_PARAM;
	}

	uint8_t cmd_buf[5];
	uint8_t res_buf[5];

	// Calculate frequency register value
	// Freq = RF_FREQ * FREQ_STEP
	// FREQ_STEP = 32MHz / 2^25 = 0.9536743164 Hz

	uint32_t rf_freq = (uint32_t) ((double) frequency
			/ SX126X_FREQUENCY_STEP_SIZE);

	cmd_buf[0] = SX126X_CMD_SET_RF_FREQUENCY;
	cmd_buf[1] = (rf_freq >> 24) & 0xFF;
	cmd_buf[2] = (rf_freq >> 16) & 0xFF;
	cmd_buf[3] = (rf_freq >> 8) & 0xFF;
	cmd_buf[4] = rf_freq & 0xFF;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 5, 100, 0);

	if (ret == SX1262_OK) {
		dev->config.frequency = frequency;
	}
	return ret;
}

/**
 * @brief Set LoRa modulation parameters
 * @param[in] dev Device handle
 * @param[in] spreading_factor SF7 to SF12
 * @param[in] bandwidth Bandwidth index
 * @param[in] coding_rate Coding rate (1=4/5, 2=4/6, 3=4/7, 4=4/8)
 * @param[in] low_data_rate_optimize Enable for SF11 and SF12
 */
sx1262_status_t sx1262_set_lora_modulation_params(sx1262_t *dev,
		sx1262_lora_sf_t spreading_factor, sx1262_lora_bandwidth_t bandwidth,
		sx1262_lora_cr_t coding_rate, sx1262_bool_t low_data_rate_optimize) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[5];
	uint8_t res_buf[5];

	cmd_buf[0] = SX126X_CMD_SET_MODULATION_PARAMS;
	cmd_buf[1] = spreading_factor;
	cmd_buf[2] = bandwidth;
	cmd_buf[3] = coding_rate;
	cmd_buf[4] = low_data_rate_optimize;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 5, 100, 0);

	if (ret == SX1262_OK) {
		dev->config.spreading_factor = spreading_factor;
		dev->config.bandwidth = bandwidth;
		dev->config.coding_rate = coding_rate;
	}

	return ret;
}

/**
 * @brief Set LoRa packet parameters
 * @param[in] dev Device handle
 * @param[in] preamble_length Preamble length in symbols (LoRa) or bits (FSK)
 * @param[in] header_type Header type: explicit (0x00) or implicit (0x01)
 * @param[in] payload_length Payload length (0-255)
 * @param[in] crc_enable Enable CRC (true = on, false = off)
 * @param[in] invert_iq Invert IQ (true = inverted, false = standard)
 * @return SX1262_OK on success
 *
 * @note For LoRa:
 * - preamble_length: Number of LoRa symbols (typically 8-65535)
 * - header_type: 0x00 = explicit, 0x01 = implicit
 * - payload_length: Maximum payload length expected
 * - crc_enable: 0x00 = off, 0x01 = on
 * - invert_iq: 0x00 = standard, 0x01 = inverted (LoRaWAN uses inverted)
 */
sx1262_status_t sx1262_set_lora_packet_params(sx1262_t *dev,
		uint16_t preamble_length, sx1262_lora_header_t implicit_header,
		uint8_t payload_length, sx1262_lora_crc_type_t crc_enable,
		sx1262_bool_t invert_iq) {

	if (dev->initialized == 0) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	if (preamble_length > 0xFFFF) {
		return SX1262_INVALID_PARAM;
	}

	uint8_t cmd_buf[7];
	uint8_t res_buf[7];

	cmd_buf[0] = SX126X_CMD_SET_PACKET_PARAMS;
	cmd_buf[1] = (preamble_length >> 8) & 0xFF;
	cmd_buf[2] = preamble_length & 0xFF;
	cmd_buf[3] = implicit_header;
	cmd_buf[4] = payload_length;
	cmd_buf[5] = crc_enable;
	cmd_buf[6] = invert_iq;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 7, 100, 0);

	if (ret == SX1262_OK) {
		dev->config.preamble_length = preamble_length;
		dev->config.implicit_header = implicit_header;
		dev->config.crc_enabled = crc_enable;
		dev->config.iq_inverted = invert_iq;
	}

	return ret;

}

/**
 * @brief Set buffer base addresses
 * @param[in] dev Device handle
 * @param[in] tx_base_addr TX buffer base address (usually 0x00)
 * @param[in] rx_base_addr RX buffer base address (usually 0x00)
 */
sx1262_status_t sx1262_set_buffer_base_address(sx1262_t *dev,
		uint8_t tx_base_addr, uint8_t rx_base_addr) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[3];
	uint8_t res_buf[3];

	cmd_buf[0] = SX126X_CMD_SET_BUFFER_BASE_ADDRESS;
	cmd_buf[1] = tx_base_addr;
	cmd_buf[2] = rx_base_addr;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 3, 100, 0);

	return ret;

}

/**
 * @brief Configure DIO interrupts
 * @param[in] dev Device handle
 * @param[in] irq_mask Mask of interrupts to enable
 * @param[in] dio1_mask Interrupts to map to DIO1 pin
 */
sx1262_status_t sx1262_set_dio_irq_params(sx1262_t *dev, uint16_t irq_mask,
		uint16_t dio1_mask) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[9];
	uint8_t res_buf[9];

	cmd_buf[0] = SX126X_CMD_SET_DIO_IRQ_PARAMS;
	cmd_buf[1] = (irq_mask >> 8) & 0xFF;        // IRQ mask MSB
	cmd_buf[2] = irq_mask & 0xFF;               // IRQ mask LSB
	cmd_buf[3] = (dio1_mask >> 8) & 0xFF;       // DIO1 mask MSB
	cmd_buf[4] = dio1_mask & 0xFF;				// DIO1 mask LSB
	cmd_buf[5] = 0x00;							// DIO2 mask MSB
	cmd_buf[6] = 0x00;							// DIO2 mask LSB
	cmd_buf[7] = 0x00;							// DIO3 mask MSB
	cmd_buf[8] = 0x00;							// DIO3 mask LSB

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 9, 100, 0);

	return ret;

}

sx1262_status_t sx1262_set_dio2_as_rf_switch_crtl(sx1262_t *dev) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[2];
	uint8_t res_buf[2];

	cmd_buf[0] = SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL;
	cmd_buf[1] = 0x01; //Enable

	return sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);

}

sx1262_status_t sx1262_set_stop_timer_on_sync_word(sx1262_t *dev) {
	uint8_t cmd_buf[2];
	uint8_t res_buf[2];

	cmd_buf[0] = SX126X_CMD_STOP_TIMER_ON_PREAMBLE;
	cmd_buf[1] = 0x00;

	return sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);
}

sx1262_status_t sx1262_set_pa_config(sx1262_t *dev, uint8_t duty_cycle,
		uint8_t hp_max, uint8_t device_select, uint8_t paLut) {
	uint8_t cmd_buf[5];
	uint8_t res_buf[5];

	cmd_buf[0] = SX126X_CMD_SET_PA_CONFIG;  // Opcode for "SetPaConfig"
	cmd_buf[1] = duty_cycle;  // PA duty cycle
	cmd_buf[2] = hp_max;        // Max power (0x00-0x07)
	cmd_buf[3] = device_select; // 0x00=SX1262, 0x01=SX1261
	cmd_buf[4] = paLut;        // Reserved, always 1

	return sx1262_send_command(dev, cmd_buf, res_buf, 5, 100, 0);

}

sx1262_status_t sx1262_calibrate_image(sx1262_t *dev, uint32_t freq_hz) {
	if(dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}
	uint8_t freq1, freq2;

	sx1262_get_image_calib_params(freq_hz, &freq1, &freq2);

	uint8_t cmd_buf[3];
	uint8_t res_buf[3];

	cmd_buf[0] = SX126X_CMD_CALIBRATE_IMAGE; // Opcode for CalibrateImage
	cmd_buf[1] = freq1; // Frequency parameter 1
	cmd_buf[2] = freq2; // Frequency parameter 2

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 3, 100, 0);

	return ret;

}

sx1262_status_t sx1262_set_tx_params(sx1262_t *dev, int8_t power,
		sx1262_ramp_time_t rampTime) {
	uint8_t cmd_buf[3];
	uint8_t res_buf[3];

	cmd_buf[0] = SX126X_CMD_SET_TX_PARAMS;      // Opcode for SetTxParams
	cmd_buf[1] = (uint8_t) power;    // Power (-17 to +22)
	cmd_buf[2] = rampTime;  // Ramp time (0x02=40uS)

	return sx1262_send_command(dev, cmd_buf, res_buf, 3, 100, 10);
}

sx1262_status_t sx1262_set_lora_symbol_timeout(sx1262_t *dev, uint8_t symb_num) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[2];
	uint8_t res_buf[2];

	cmd_buf[0] = SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT;
	cmd_buf[1] = symb_num;

	return sx1262_send_command(dev, cmd_buf, res_buf, 2, 2, 100);
}

sx1262_status_t SX1262_set_fallback_mode(sx1262_t *dev,
		sx1262_rx_tx_fallback_mode_t mode) {
	uint8_t cmd_buf[2];
	uint8_t res_buf[2];

	cmd_buf[0] = SX126X_CMD_SET_RX_TX_FALLBACK_MODE; // Opcode for setFallbackMode
	cmd_buf[1] = mode;

	return sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 10);
}

/**
 * @brief Get current IRQ status from device
 * @param[in] dev Device handle
 * @param[out] irq_status 16-bit IRQ status mask
 * @return SX1262_OK on success
 *
 * @note Response format: [Status][IRQ_Status_MSB][IRQ_Status_LSB]
 * @note IRQ bits are active HIGH
 */
sx1262_status_t sx1262_get_irq_status(sx1262_t *dev, uint16_t *irq_status) {

	if (irq_status == NULL || dev == NULL) {
		return SX1262_INVALID_PARAM;
	}

	if (dev->initialized == 0) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[4];
	uint8_t res_buf[4] = { 0 };

	cmd_buf[0] = SX126X_CMD_GET_IRQ_STATUS;
	cmd_buf[1] = 0xFF;
	cmd_buf[2] = 0xFF;
	cmd_buf[3] = 0xFF;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

//	*irq_status = ((uint16_t)res_buf[1] << 8) | res_buf[2];

	*irq_status = (uint16_t) res_buf[3];

	return ret;
}

sx1262_status_t sx1262_clear_irq_status(sx1262_t *dev) {
	if (dev->initialized == 0) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[3];
	uint8_t res_buf[3];

	cmd_buf[0] = SX126X_CMD_CLEAR_IRQ_STATUS;
	cmd_buf[1] = 0xFF;
	cmd_buf[2] = 0xFF;

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 3, 100, 0);

	return ret;

}

sx1262_status_t sx1262_irq_handler(sx1262_t *dev) {
	if (dev->initialized == 0) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint16_t irq_status;
	sx1262_status_t ret = sx1262_get_irq_status(dev, &irq_status);

	if (ret != SX1262_OK) {
		return ret;
	}

	if (irq_status & SX1262_IRQ_RX_DONE) {
		uint8_t recv_buf[256] = { 0 };
		uint8_t recv_len = 0;
		sx1262_handle_rx_done(dev, recv_buf, &recv_len);
//        sx1262_set_mode_continuous_receive(dev);
	}

	if (irq_status & SX1262_IRQ_TX_DONE) {
		dev->hal.delay_ms(1);
//		sx1262_set_mode_continuous_receive(dev);
//        sx1262_handle_tx_done(dev, SX1262_OK);
	}

	if (irq_status & SX1262_IRQ_CRC_ERR) {
//        sx1262_handle_rx_error(dev, SX1262_ERROR_CRC);
	}

	if (irq_status & SX1262_IRQ_HEADER_ERR) {
//        sx1262_handle_rx_error(dev, SX1262_ERROR_HEADER);
	}

	if (irq_status & SX1262_IRQ_TIMEOUT) {
//        sx1262_handle_timeout(dev);
	}

	ret = sx1262_clear_irq_status(dev);

	return ret;
}

sx1262_status_t sx1262_get_packet_status(sx1262_t *dev, uint8_t *status,
		uint8_t *rssi, uint8_t *snr, uint8_t *signal_rssi) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[4] = { 0 };
	uint8_t res_buf[4] = { 0 };

	cmd_buf[0] = SX126X_CMD_GET_PACKET_STATUS;
	cmd_buf[1] = 0x00; //Dummy byte
	cmd_buf[2] = 0x00; //Dummy byte
	cmd_buf[3] = 0x00; //Dummy byte

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	if (status != NULL)
		*status = res_buf[0];
	if (rssi != NULL)
		*rssi = res_buf[1];
	if (snr != NULL)
		*snr = res_buf[2];
	if (signal_rssi != NULL)
		*signal_rssi = res_buf[3];

	return SX1262_OK;
}

sx1262_status_t sx1262_handle_rx_done(sx1262_t *dev, uint8_t *buf, uint8_t *len) {

	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[4] = { 0 };
	uint8_t res_buf[4] = { 0 };
	sx1262_status_t ret;

	// Get RX buffer status
	cmd_buf[0] = SX126X_CMD_GET_RX_BUFFER_STATUS;
	cmd_buf[1] = 0xFF;
	cmd_buf[2] = 0xFF;
	cmd_buf[3] = 0xFF;

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);
	if (ret != SX1262_OK)
		return ret;

	uint8_t payload_len = res_buf[2];    // Payload length
	uint8_t start_addr = res_buf[3];     // Start address

	if (payload_len == 0) {
		*len = 0;
		return SX1262_OK;
	}

	cmd_buf[0] = SX126X_CMD_READ_BUFFER;
	cmd_buf[1] = start_addr;
	cmd_buf[2] = 0x00; // Dummy

	// We'll use a single buffer for the whole transaction
	uint8_t spi_tx_buf[3 + 256] = { 0 };
	uint8_t spi_rx_buf[3 + 256] = { 0 };

	spi_tx_buf[0] = SX126X_CMD_READ_BUFFER;
	spi_tx_buf[1] = start_addr;
	spi_tx_buf[2] = 0x00;

	for (int i = 3; i < 3 + payload_len; i++) {
		spi_tx_buf[i] = 0x00;
	}

	sx1262_busy_wait(dev, 100);
	sx1262_cs_low(dev);

	dev->hal.spi_transmit_receive(spi_tx_buf, spi_rx_buf, 3 + payload_len, 100);

	sx1262_cs_high(dev);
	sx1262_busy_wait(dev, 100);

	// Parse response: spi_rx_buf[0]=status, [1]=dummy, [2...]=data
	// Skip first 2 bytes (status + dummy)
	if (buf && len) {
		memcpy(buf, &spi_rx_buf[3], payload_len);
		*len = payload_len;

		// Also store in device
		memcpy(dev->rx_buffer, &spi_rx_buf[3], payload_len);
//		dev->rx_buffer_length = payload_len;

		//Call Callback if set
		if (dev->callbacks.rx_done) {
			dev->callbacks.rx_done(buf, payload_len);
		}
	}

	return SX1262_OK;

}

sx1262_status_t sx1262_lora_receive(sx1262_t *dev, uint8_t *buf, uint8_t *len,
		uint32_t timeout) {
	sx1262_status_t ret;

	ret = sx1262_set_mode_continuous_receive(dev);
	if (ret != SX1262_OK) {
		return ret;
	}

	uint32_t start = dev->hal.get_tick();
	while (1) {
		uint16_t irq_status;
		ret = sx1262_get_irq_status(dev, &irq_status);
		if (ret != SX1262_OK) {
			return ret;
		}

		if (irq_status & SX1262_IRQ_RX_DONE) {
			ret = sx1262_handle_rx_done(dev, buf, len);
			if (ret != SX1262_OK) {
				return ret;
			}
			break;
		}

		if (irq_status & SX1262_IRQ_TIMEOUT) {
			return SX1262_TIMEOUT;
		}

		if ((dev->hal.get_tick() - start) >= timeout) {
			return SX1262_TIMEOUT;
		}

		ret = sx1262_clear_irq_status(dev);

		if (ret != SX1262_OK) {
			return ret;
		}

		dev->hal.delay_ms(10);

		ret = SX1262_set_fallback_mode(dev,
				SX1262_RX_TX_FALLBACK_MODE_STDBY_XOSC);

		if (ret != SX1262_OK) {
			return ret;
		}

	}

	return SX1262_OK;
}

/**
 * @brief Set the LoRa sync word
 * @param[in] dev Pointer to device handle
 * @param[in] sync_word Sync word to set
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_lora_sync_word(sx1262_t *dev, uint16_t sync_word) {
	if (dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	uint8_t cmd_buf[5] = { 0 };
	uint8_t res_buf[5] = { 0 };

	cmd_buf[0] = SX126X_CMD_WRITE_REGISTER;
	cmd_buf[1] = (SX126X_REG_LORA_SYNC_WORD_MSB >> 8) & 0xFF; // MSB of address
	cmd_buf[2] = SX126X_REG_LORA_SYNC_WORD_MSB & 0xFF;        // LSB of address
	cmd_buf[3] = (sync_word >> 8) & 0xFF;                     // Sync word MSB
	cmd_buf[4] = sync_word & 0xFF;                            // Sync word LSB

	sx1262_status_t ret = sx1262_send_command(dev, cmd_buf, res_buf, 5, 100, 0);

	return ret;
}

/**
 * @brief Transmit data using LoRa in interrupt mode
 * @param[in] dev Pointer to device handle
 * @param[in] data Pointer to data buffer
 * @param[in] length Length of data to transmit
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_lora_transmit_it(sx1262_t *dev, uint8_t *data,
		uint8_t length) {
	sx1262_status_t ret;
	uint8_t cmd_buf[4] = { 0 };
	uint8_t res_buf[256] = { 0 };

	if (!dev || dev->initialized != SX1262_BOOL_TRUE) {
		return SX1262_ERR_NOT_INITIALIZED;
	}

	if (!data || length == 0 || length > 255) {
		return SX1262_INVALID_PARAM;
	}

	// Set TX base address to 0x00
	cmd_buf[0] = SX126X_CMD_WRITE_BUFFER;
	cmd_buf[1] = 0x00; // Start address

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 2, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	ret = sx1262_send_command(dev, data, res_buf, length + 1, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	dev->hal.delay_ms(10);

	cmd_buf[0] = SX126X_CMD_SET_TX;
	cmd_buf[1] = 0xFF; // Dummy byte
	cmd_buf[2] = 0xFF;
	cmd_buf[3] = 0xFF;

	ret = sx1262_send_command(dev, cmd_buf, res_buf, 4, 100, 0);

	if (ret != SX1262_OK) {
		return ret;
	}

	dev->mode = CHIP_MODE_TX;

	return SX1262_OK;

}

sx1262_status_t sx1262_lora_transmit(sx1262_t *dev, uint8_t *data,
		uint8_t length, uint32_t timeout) {
	sx1262_status_t ret;

	ret = sx1262_lora_transmit_it(dev, data, length);

	if (ret != SX1262_OK) {
		return ret;
	}

	uint32_t start = dev->hal.get_tick();
	while (1) {
		uint16_t irq_status;
		ret = sx1262_get_irq_status(dev, &irq_status);
		if (ret != SX1262_OK) {
			return ret;
		}

		if (irq_status & SX1262_IRQ_TX_DONE) {
			ret = sx1262_clear_irq_status(dev);
			if (ret != SX1262_OK) {
				return ret;
			}
			break;
		}

		if ((dev->hal.get_tick() - start) >= timeout) {
			return SX1262_TIMEOUT;
		}

		dev->hal.delay_ms(10);
	}

	return SX1262_OK;
}

