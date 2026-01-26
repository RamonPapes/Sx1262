/*
 * sx126x_hal.h
 *
 *  Created on: Jan 9, 2026
 *      Author: ramon.papes
 */

#ifndef INC_SX126X_HAL_H_
#define INC_SX126X_HAL_H_

#include "main.h"

uint8_t sx1262_reset_gpio_init(void);
uint8_t sx1262_reset_gpio_deinit(void);
uint8_t sx1262_reset_gpio_write(uint8_t value);
uint8_t sx1262_busy_gpio_init(void);
uint8_t sx1262_busy_gpio_deinit(void);
uint8_t sx1262_busy_gpio_read(uint8_t *value);
uint8_t sx1262_spi_init(void);
uint8_t sx1262_spi_deinit(void);
uint8_t sx1262_spi_write_read(uint8_t *in_buf, uint32_t in_len,uint8_t *out_buf, uint32_t out_len);
void sx1262_delay_ms(uint32_t ms);
void sx1262_debug_print(const char *const fmt, ...);
void sx1262_receive_callback(uint16_t type, uint8_t *buf, uint16_t len);
uint8_t sx1262_dio1_gpio_init(void);
uint8_t sx1262_dio1_gpio_deinit(void);



#endif /* INC_SX126X_HAL_H_ */
