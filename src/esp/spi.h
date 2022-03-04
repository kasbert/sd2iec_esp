/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2022 Jarkko Sonninen <kasper@iki.fi>
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   spi.h: Definitions for the low-level SPI routines - ESP version

*/
#ifndef SPI_H
#define SPI_H

#include "config.h"

/* Low speed 400kHz for init, fast speed <=20MHz (MMC limit) */
typedef enum { SPI_SPEED_FAST, SPI_SPEED_SLOW } spi_speed_t;

/* Available SPI devices - special case to select all SD cards during init */
/* Note: SD cards must be 1 and 2 */
/* AVR note: The code assumes that spi_device_t can be used as bit field of
 * selected cards */
typedef enum {
  SPIDEV_NONE = 0,
  SPIDEV_CARD0 = 1,
  SPIDEV_CARD1 = 2,
  SPIDEV_ALLCARDS = 3
} spi_device_t;

/* Initialize SPI interface */
void spi_init(spi_speed_t speed);

void spi_select_device(spi_device_t dev);

/* Transmit a single byte */
void spi_tx_byte(uint8_t data);

/* Receive a data block */
void spi_tx_block(const void *data, unsigned int length);

/* Receive a single byte */
uint8_t spi_rx_byte(void);

/* Receive a data block */
void spi_rx_block(void *data, unsigned int length);

/* Switch speed of SPI interface */
void spi_set_speed(spi_speed_t speed);

uint32_t spi_transaction(uint8_t cmd_bits, uint16_t cmd_data,
                         uint32_t addr_bits, uint32_t addr_data,
                         uint32_t dout_bits, const void *dout_data,
                         uint32_t din_bits, void *din_data,
                         uint32_t dummy_bits);

#endif
