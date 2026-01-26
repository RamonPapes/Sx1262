/*
 * sx1262.h
 *
 *  Created on: Jan 15, 2026
 *      Author: ramon.papes
 */

#ifndef LIBS_DEV_SX1262_SX1262_H_
#define LIBS_DEV_SX1262_SX1262_H_

#include "sx1262_defines.h"
#include "sx1262_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SX1262_BOOL_FALSE = 0x00, /**< disable function */
	SX1262_BOOL_TRUE = 0x01, /**< enable function */
} sx1262_bool_t;

/**
 * @brief sx1262 configuration structure
 */
typedef struct {
	uint32_t frequency;
	uint8_t spreading_factor;
	uint8_t bandwidth;
	uint8_t coding_rate;
	uint8_t tx_power;
	uint16_t preamble_length;
	bool crc_enabled;
	bool implicit_header;
	bool iq_inverted;
} sx1262_settings_t;

typedef enum {
	SX1262_OK = 0,
	SX1262_ERROR = 1,
	SX1262_BUSY = 2,
	SX1262_TIMEOUT = 3,
	SX1262_INVALID_PARAM = 4,
	SX1262_ERR_NOT_INITIALIZED = 5
} sx1262_status_t;

/**
 * chip status enum
 */
typedef enum {
	CHIP_MODE_SLEEP = 0x00,
	CHIP_MODE_RESERVED = 0x01,
	CHIP_MODE_STDBY_RC = 0x02,
	CHIP_MODE_STDBY_XOSC = 0x03,
	CHIP_MODE_FS = 0x04,
	CHIP_MODE_RX = 0x05,
	CHIP_MODE_TX = 0x06,
	CHIP_MODE_UNKNOWN = 0x07
} chip_mode_t;

/**
 * command status enum
 */
typedef enum {
	CMD_STATUS_RESERVED = 0x00,
	CMD_STATUS_RFU = 0X01,
	CMD_STATUS_DATA_AVAILABLE = 0x02,
	CMD_STATUS_CMD_TIMEOUT = 0x03,
	CMD_STATUS_CMD_PROCCESS_ERROR = 0x04,
	CMD_STATUS_CMD_FAILED = 0x05,
	CMD_STATUS_TX_DONE = 0x06,
} command_status_t;

/**
 * @brief packet type enum
 */
typedef enum {
	PACKET_TYPE_GFSK = 0x00, PACKET_TYPE_LORA = 0x01,
} packet_type_t;

/**
 * @brief sx1262 lora spreading factor enumeration definition
 */
typedef enum {
	SX1262_LORA_SF_5 = 0x05, /**< spreading factor 5 */
	SX1262_LORA_SF_6 = 0x06, /**< spreading factor 6 */
	SX1262_LORA_SF_7 = 0x07, /**< spreading factor 7 */
	SX1262_LORA_SF_8 = 0x08, /**< spreading factor 8 */
	SX1262_LORA_SF_9 = 0x09, /**< spreading factor 9 */
	SX1262_LORA_SF_10 = 0x0A, /**< spreading factor 10 */
	SX1262_LORA_SF_11 = 0x0B, /**< spreading factor 11 */
	SX1262_LORA_SF_12 = 0x0C, /**< spreading factor 12 */
} sx1262_lora_sf_t;

/**
 * @brief sx1262 lora bandwidth enumeration definition
 */
typedef enum {
	SX1262_LORA_BANDWIDTH_7P81_KHZ = 0x00, /**< 7.81 kHz */
	SX1262_LORA_BANDWIDTH_10P42_KHZ = 0x08, /**< 10.42 kHz */
	SX1262_LORA_BANDWIDTH_15P63_KHZ = 0x01, /**< 15.63 kHz */
	SX1262_LORA_BANDWIDTH_20P83_KHZ = 0x09, /**< 20.83 kHz */
	SX1262_LORA_BANDWIDTH_31P25_KHZ = 0x02, /**< 31.25 kHz */
	SX1262_LORA_BANDWIDTH_41P67_KHZ = 0x0A, /**< 41.67 kHz */
	SX1262_LORA_BANDWIDTH_62P50_KHZ = 0x03, /**< 62.50 kHz */
	SX1262_LORA_BANDWIDTH_125_KHZ = 0x04, /**< 125 kHz */
	SX1262_LORA_BANDWIDTH_250_KHZ = 0x05, /**< 250 kHz */
	SX1262_LORA_BANDWIDTH_500_KHZ = 0x06, /**< 500 kHz */
} sx1262_lora_bandwidth_t;

/**
 * @brief sx1262 lora coding rate enumeration definition
 */
typedef enum {
	SX1262_LORA_CR_4_5 = 0x01, /**< cr 4/5 */
	SX1262_LORA_CR_4_6 = 0x02, /**< cr 4/6 */
	SX1262_LORA_CR_4_7 = 0x03, /**< cr 4/7 */
	SX1262_LORA_CR_4_8 = 0x04, /**< cr 4/8 */
} sx1262_lora_cr_t;

typedef enum {
	SX1262_LORA_HEADER_EXPLICIT = 0x00, /**< explicit header */
	SX1262_LORA_HEADER_IMPLICIT = 0x01, /**< implicit header */
} sx1262_lora_header_t;

/**
 * @brief sx1262 lora crc type enumeration definition
 */
typedef enum {
	SX1262_LORA_CRC_TYPE_OFF = 0x00, /**< crc off */
	SX1262_LORA_CRC_TYPE_ON = 0x01, /**< crc on */
} sx1262_lora_crc_type_t;

/**
 * @brief sx1262 ramp time enumeration definition
 */
typedef enum {
	SX1262_RAMP_TIME_10US = 0x00, /**< 10us */
	SX1262_RAMP_TIME_20US = 0x01, /**< 20us */
	SX1262_RAMP_TIME_40US = 0x02, /**< 40us */
	SX1262_RAMP_TIME_80US = 0x03, /**< 80us */
	SX1262_RAMP_TIME_200US = 0x04, /**< 200us */
	SX1262_RAMP_TIME_800US = 0x05, /**< 800us */
	SX1262_RAMP_TIME_1700US = 0x06, /**< 1700us */
	SX1262_RAMP_TIME_3400US = 0x07, /**< 3400us */
} sx1262_ramp_time_t;

/**
 * @brief sx1262 rx tx fallback mode enumeration definition
 */
typedef enum {
	SX1262_RX_TX_FALLBACK_MODE_FS = 0x40, /**< the radio goes into fs mode after tx or rx */
	SX1262_RX_TX_FALLBACK_MODE_STDBY_XOSC = 0x30, /**< the radio goes into standby_xosc mode after tx or rx */
	SX1262_RX_TX_FALLBACK_MODE_STDBY_RC = 0x20, /**< the radio goes into standby_rc mode after tx or rx */
} sx1262_rx_tx_fallback_mode_t;

/**
 * @brief SX1262 interrupt flags enumeration
 * @note Based on SX126X_CMD_SET_DIO_IRQ_PARAMS defines
 * @note 16-bit mask for interrupt configuration
 */
typedef enum {
	/* Individual interrupt flags (bits 0-14) */
	SX1262_IRQ_NONE = SX126X_IRQ_NONE, /**< 0x0000 - No interrupts */
	SX1262_IRQ_TX_DONE = SX126X_IRQ_TX_DONE, /**< 0x0001 - Packet transmission completed (bit 0) */
	SX1262_IRQ_RX_DONE = SX126X_IRQ_RX_DONE, /**< 0x0002 - Packet received (bit 1) */
	SX1262_IRQ_PREAMBLE_DETECTED = SX126X_IRQ_PREAMBLE_DETECTED, /**< 0x0004 - Preamble detected (bit 2) */
	SX1262_IRQ_SYNC_WORD_VALID = SX126X_IRQ_SYNC_WORD_VALID, /**< 0x0008 - Valid sync word detected (bit 3) */
	SX1262_IRQ_HEADER_VALID = SX126X_IRQ_HEADER_VALID, /**< 0x0010 - Valid LoRa header received (bit 4) */
	SX1262_IRQ_HEADER_ERR = SX126X_IRQ_HEADER_ERR, /**< 0x0020 - LoRa header CRC error (bit 5) */
	SX1262_IRQ_CRC_ERR = SX126X_IRQ_CRC_ERR, /**< 0x0040 - Wrong CRC received (bit 6) */
	SX1262_IRQ_CAD_DONE = SX126X_IRQ_CAD_DONE, /**< 0x0080 - Channel activity detection finished (bit 7) */
	SX1262_IRQ_CAD_DETECTED = SX126X_IRQ_CAD_DETECTED,/**< 0x0100 - Channel activity detected (bit 8) */
	SX1262_IRQ_TIMEOUT = SX126X_IRQ_TIMEOUT, /**< 0x0200 - Rx or Tx timeout (bit 9) */
} sx1262_irq_t;

/**
 * @brief SX1262 callback functions structure
 *
 */
typedef struct sx1262_callbacks {
	void (*tx_done)(void);
	void (*rx_done)(uint8_t *data, uint8_t length);
	void (*rx_error)(sx1262_status_t error);
	void (*timeout)(void);
} sx1262_callbacks_t;

/**
 * @brief SX1262 device handle structure
 *
 */
typedef struct sx1262_t{
	sx1262_hal_t hal;
	chip_mode_t mode;
	sx1262_settings_t config;
	sx1262_bool_t initialized;
	uint8_t in_receive_mode;
	sx1262_callbacks_t callbacks;
	uint8_t tx_buffer[SX126X_MAX_PACKET_LENGTH];
	uint8_t rx_buffer[SX126X_MAX_PACKET_LENGTH];
} sx1262_t;

/**
 * @brief SX1262 driver functions
 * @defgroup sx1262_driver_functions SX1262 driver functions
 * @{
 */

/**
 * @brief Set CS pin low
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_cs_low(const sx1262_t *dev);

/**
 * @brief Set CS pin high
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_cs_high(const sx1262_t *dev);

/**
 * @brief Wait until device is not busy or timeout occurs
 * @param[in] dev Pointer to device handle
 * @param[in] timeout Timeout in milliseconds
 * @return SX1262_OK if device is ready | SX1262_TIMEOUT if timeout occurs | error code
 */
sx1262_status_t sx1262_busy_wait(const sx1262_t *dev, uint32_t timeout);

/**
 * @brief Check if device is busy
 * @param[in] dev Pointer to device handle
 * @return 1 if busy, 0 if not busy | error code
 */
uint8_t sx1262_is_busy(const sx1262_t *dev);

/**
 * @brief Send SPI command to device
 * @param[in] dev Pointer to device handle
 * @param[in] tx_data Data to transmit
 * @param[out] rx_data Buffer for received data
 * @param[in] tx_len Length of data to transmit
 * @param[in] rx_len Expected response length
 * @param[in] timeout_ms Timeout in milliseconds
 */
sx1262_status_t sx1262_send_command(sx1262_t *dev, uint8_t *cmd, uint8_t *res,
		uint16_t len, uint32_t timeout, uint16_t delay);

/**
 * @brief sx1262 init function
 * @param[in] dev pointer to an sx1262 handle structure
 * @param[in] hal pointer to an sx1262 hal structure
 * @note    none
 */
sx1262_status_t sx1262_init(sx1262_t *dev, const sx1262_hal_t *hal);

/**
 * @brief Check if device is the correct SX1262
 * @param[in] dev Pointer to device handle
 * @return 1 if correct device, 0 if not
 */
uint8_t sx1262_check_correct(sx1262_t *dev);


/**
 * @brief Set device to sleep mode
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */

//sx1262_status_t sx1262_set_mode_sleep(sx1262_t *dev);

/**
 * @brief Set device to standby mode
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_mode_standby(sx1262_t *dev);

/**
 * @brief Set device to receive mode
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_mode_continuous_receive(sx1262_t *dev);

/**
 * @brief
 */
uint8_t sx1262_get_status(sx1262_t *dev);


/**
 * @brief Check device communication
 * @param[in] dev Pointer to device handle
 * @return SX1262_OK if device responds correctly
 */
sx1262_status_t sx1262_check_communication(sx1262_t *dev);

/**
 * @brief Get chip status
 * @param[in] dev Pointer to device handle
 * @param[out] mode Pointer to chip mode variable
 * @param[out] cmd_status Pointer to command status variable
 * @return chip status
 */
sx1262_status_t sx1262_get_chip_status(sx1262_t *dev, chip_mode_t *mode,
		command_status_t *cmd_status);

/**
 * @brief Set packet type
 * @param[in] dev Pointer to device handle
 * @param[in] packet_type Packet type to set
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_packet_type(sx1262_t *dev, packet_type_t pkt_type);

/**
 * @brief Get packet type
 * @param[in] dev Pointer to device handle
 * @param[out] pkt_type Pointer to packet type variable
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_get_packet_type(sx1262_t *dev, packet_type_t *pkt_type);

/**
 * @brief Set RF frequency
 * @param[in] dev Device handle
 * @param[in] frequency_hz Frequency in Hz
 */
sx1262_status_t sx1262_set_frequency(sx1262_t *dev, uint32_t frequency);

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
		sx1262_lora_cr_t coding_rate, sx1262_bool_t low_data_rate_optimize);

/**
 * @brief Set LoRa packet parameters
 * @param[in] dev Device handle
 * @param[in] preamble_length Preamble length in symbols
 * @param[in] implicit_header Enable implicit header mode
 * @param[in] payload_length Payload length (0-255)
 * @param[in] crc_enable Enable CRC
 * @param[in] invert_iq Invert IQ (for LoRaWAN)
 */
sx1262_status_t sx1262_set_lora_packet_params(sx1262_t *dev,
		uint16_t preamble_length, sx1262_lora_header_t implicit_header,
		uint8_t payload_length, sx1262_lora_crc_type_t crc_enable,
		sx1262_bool_t invert_iq);

/**
 * @brief Set buffer base addresses
 * @param[in] dev Device handle
 * @param[in] tx_base_addr TX buffer base address (usually 0x00)
 * @param[in] rx_base_addr RX buffer base address (usually 0x00)
 */
sx1262_status_t sx1262_set_buffer_base_address(sx1262_t *dev,
		uint8_t tx_base_addr, uint8_t rx_base_addr);

/**
 * @brief Configure DIO interrupts
 * @param[in] dev Device handle
 * @param[in] irq_mask Mask of interrupts to enable
 * @param[in] dio1_mask Interrupts to map to DIO1 pin
 */
sx1262_status_t sx1262_set_dio_irq_params(sx1262_t *dev, uint16_t irq_mask,
		uint16_t dio1_mask);

/**
 * @brief Set DIO2 as RF switch control
 * @param[in] dev Device handle
 * @return SX1262_OK on success | error code
 * @note This function configures DIO2 pin to control the RF switch
 * for TX/RX switching.
 */
sx1262_status_t sx1262_set_dio2_as_rf_switch_crtl(sx1262_t *dev);

/**
 * @brief Disable stop timer on sync word detection
 * @param[in] dev Device handle
 * @return SX1262_OK on success | error code
 * @note This function disables the stop timer on sync word detection.
 */
sx1262_status_t sx1262_set_stop_timer_on_sync_word(sx1262_t *dev);

/**
 * @brief Set PA configuration
 * @param[in] dev Device handle
 * @param[in] duty_cycle PA duty cycle
 * @param[in] hp_max Max power (0x00-0x07)
 * @param[in] device_select 0x00=SX1262, 0x01=SX1261
 * @param[in] paLut Reserved, always 1
 */
sx1262_status_t sx1262_set_pa_config(sx1262_t *dev, uint8_t duty_cycle,
		uint8_t hp_max, uint8_t device_select, uint8_t paLut);

/**
 * @brief Set TX parameters
 * @param[in] dev Device handle
 * @param[in] power TX power (-17 to +22 dBm)
 * @param[in] rampTime Ramp time (sx1262_ramp_time_t)
 */
sx1262_status_t sx1262_set_tx_params(sx1262_t *dev, int8_t power,
		sx1262_ramp_time_t rampTime);

/**
 * @brief Set RX/TX fallback mode
 * @param[in] dev Device handle
 * @param[in] mode Fallback mode
 */
sx1262_status_t SX1262_set_fallback_mode(sx1262_t *dev,
		sx1262_rx_tx_fallback_mode_t mode);

/**
 * @brief Set LoRa symbol timeout
 * @param[in] dev Device handle
 * @param[in] symb_num Number of symbols for timeout
 * @return SX1262_OK on success | error code
 * @note This function sets the number of symbols after which
 * the RX timeout occurs in LoRa mode.
 */
sx1262_status_t sx1262_set_lora_symbol_timeout(sx1262_t *dev, uint8_t symb_num);

/**
 * @brief Get current IRQ status from device
 * @param[in] dev Device handle
 * @param[out] irq_status 16-bit IRQ status mask
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_get_irq_status(sx1262_t *dev, uint16_t *irq_status);

/**
 * @brief Clear IRQ status flags
 * @param[in] dev Device handle
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_clear_irq_status(sx1262_t *dev);

/**
 * @brief SX1262 IRQ handler
 * @param[in] dev Device handle
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_irq_handler(sx1262_t *dev);

/**
 * @brief Get packet status (RSSI, SNR)
 * @param[in] dev Device handle
 * @param[out] status Pointer to packet status byte
 * @param[out] rssi Pointer to RSSI value
 * @param[out] snr Pointer to SNR value
 * @param[out] signal_rssi Pointer to signal RSSI value
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_get_packet_status(sx1262_t *dev, uint8_t *status, uint8_t *rssi, uint8_t *snr, uint8_t *signal_rssi);

/**
 * @brief Handle RX done event
 * @param[in] dev Device handle
 * @param[out] buf Buffer to store received data
 * @param[out] len Length of received data
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_handle_rx_done(sx1262_t *dev, uint8_t *buf, uint8_t *len);

/**
 * @brief Handle TX done event
 * @param[in] dev Device handle
 * @param[in] status Status of the transmission
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_handle_tx_done(sx1262_t *dev, sx1262_status_t status);

/**
 * @brief Transmit data
 * @param[in] dev Device handle
 * @param[in] data Data buffer to transmit
 * @param[in] length Length of data to transmit
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_lora_transmit(sx1262_t *dev, uint8_t *data, uint8_t length, uint32_t timeout_ms);

/**
 * @brief Transmit data in interrupt mode
 * @param[in] dev Device handle
 * @param[in] data Data buffer to transmit
 * @param[in] length Length of data to transmit
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_lora_transmit_it(sx1262_t *dev, uint8_t *data, uint8_t length);

/**
 * @brief Receive single packet
 * @param[in] dev Device handle
 * @param[out] buf Buffer to store received data
 * @param[out] len Length of received data
 * @return SX1262_OK on success
 */
sx1262_status_t sx1262_receive_single_packet(sx1262_t *dev, uint8_t *buf, uint8_t *len, uint32_t timeout);

/**
 * @brief Set LoRa sync word
 * @param[in] dev Device handle
 * @param[in] sync_word 16-bit sync word
 * @return SX1262_OK on success | error code
 */
sx1262_status_t sx1262_set_lora_sync_word(sx1262_t *dev, uint16_t sync_word);

/**
 * @brief Set TX done callback function
 * @param[in] dev Device handle
 * @param[in] callback Pointer to TX done callback function
 */
void sx1262_set_tx_done_callback(sx1262_t *dev, void (*callback)(void));

/**
 * @brief Set RX done callback function
 * @param[in] dev Device handle
 * @param[in] callback Pointer to RX done callback function
 */
void sx1262_set_rx_done_callback(sx1262_t *dev,
		void (*callback)(uint8_t *data, uint8_t length));

/**
 * @brief Set RX error callback function
 * @param[in] dev Device handle
 * @param[in] callback Pointer to RX error callback function
 */
void sx1262_set_rx_error_callback(sx1262_t *dev,
		void (*callback)(sx1262_status_t error));

/**
 * @brief Set timeout callback function
 * @param[in] dev Device handle
 * @param[in] callback Pointer to timeout callback function
 */
void sx1262_set_timeout_callback(sx1262_t *dev, void (*callback)(void));

/**
 * @}
 * end of sx1262_driver_functions
 */
#ifdef __cplusplus
}
#endif

#endif /* LIBS_DEV_SX1262_SX1262_H_ */
