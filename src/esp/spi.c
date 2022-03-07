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


   spi.c: Low-level SPI routines, ESP version

*/

#include "spi.h"
#include "config.h"
#include "timer.h"

#include "eagle_soc.h"
#include "osapi.h"
#include "spi_register.h"

typedef enum {
  SpiSpeed_0_5MHz = 160,
  SpiSpeed_1MHz = 80,
  SpiSpeed_2MHz = 40,
  SpiSpeed_5MHz = 16,
  SpiSpeed_8MHz = 10,
  SpiSpeed_10MHz = 8,
} SpiSpeed;

static int selected_spi_no = 1;

static void esp_spi_set_speed(uint8_t spi_no, SpiSpeed speed);

/* SD SS pin default implementation */
#ifndef SDCARD_SS_SPECIAL
static inline __attribute__((always_inline)) void sdcard_set_ss(uint8_t state) {
}
#endif

/* select device */
void spi_select_device(spi_device_t dev) {
  selected_spi_no = (dev == SPIDEV_CARD0) ? 1 : 2;
  // ETS_DEBUG("spi_select_device %d %d ", dev, spi_no);
  if (selected_spi_no != 1) {
    // NONE selected
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
    gpio_output_set(BIT(15), 0, BIT(15), 0); // Set as high-level output.
  } else {
    // SDCARD selected
    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, FUNC_GPIO15);
    // gpio_output_set(0, BIT(15), BIT(15), 0); // Set as low-level output.
    // Let HW SPI control the SPI CS line
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);

    // Somehow some block reads fail without these
    spi_tx_byte(0xff); // TODO remove
    spi_tx_byte(0xff); // TODO remove
    spi_tx_byte(0xff); // TODO remove
  }
  // TODO remove
  if (dev & 1)
    sdcard_set_ss(0);
  else
    sdcard_set_ss(1);
#ifdef CONFIG_TWINSD
  if (dev & 2)
    sdcard2_set_ss(0);
  else
    sdcard2_set_ss(1);
#endif
}

void spi_set_speed(spi_speed_t speed) {
  if (speed == SPI_SPEED_SLOW) {
    esp_spi_set_speed(selected_spi_no, SpiSpeed_0_5MHz);
  } else {
    esp_spi_set_speed(selected_spi_no, SpiSpeed_5MHz);
  }
}

void spi_tx_byte(uint8_t data) {
  // ETS_DEBUG(" SPI transmit 1 byte: %02x  (at tick %d)\r\n", data, ticks);
  spi_transaction(0, 0, 0, 0, 8, &data, 0, 0, 0);
}

uint8_t spi_rx_byte(void) {
  uint8_t data = 0xff;
  spi_transaction(0, 0, 0, 0, 0, 0, 8, &data, 0);
  // if (data == 0xff)
  //  uart_tx_one_char(0, '.');
  // else
  //  ETS_DEBUG("%d SPI receive 1 byte: %02x \r\n", ticks, data);
  return data;
}

void spi_tx_block(const void *data2, unsigned int length) {
  int i;
  const uint8_t *data = data2;
  // debug_print_buffer("SPI send ", data, length > 16 ? 16 : length);
  for (i = 0; i < length; i++) {
    spi_tx_byte(data[i]);
  }
  return;
  for (i = length, data = data2; i > 0; i -= 64, data += 64) {
    int datalen = (length > 64 ? 64 : length) * 8;
    spi_transaction(0, 0, 0, 0, datalen, data, 0, 0, 0);
  }
}

void spi_rx_block(void *data2, unsigned int length) {
  uint8_t *data = data2;
  int i;
  /*
for (i = 0; i < length; i++) {
 data[i] = spi_rx_byte();
}
  return;
  */
  for (data = data2, i = length; i > 0; i -= 64, data += 64) {
    int datalen = (length > 64 ? 64 : length) * 8;
    spi_transaction(0, 0, 0, 0, 0, 0, datalen, data, 0);
  }
  // debug_print_buffer("SPI receive ", data, length > 16 ? 16 : length);
}

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 David Ogilvy (MetalPhreak)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//#include "driver/spi.h"

// Define SPI hardware modules
#define SPI 0
#define HSPI 1

#define SPI_CLK_USE_DIV 0
#define SPI_CLK_80MHZ_NODIV 1

#define SPI_BYTE_ORDER_HIGH_TO_LOW 1
#define SPI_BYTE_ORDER_LOW_TO_HIGH 0

#ifndef CPU_CLK_FREQ // Should already be defined in eagle_soc.h
#define CPU_CLK_FREQ 80 * 1000000
#endif

// Define some default SPI clock settings
#define SPI_CLK_PREDIV 10
#define SPI_CLK_CNTDIV 2
#define SPI_CLK_FREQ                                                           \
  CPU_CLK_FREQ / (SPI_CLK_PREDIV * SPI_CLK_CNTDIV) // 80 / 20 = 4 MHz

void spi_mode(uint8_t spi_no, uint8_t spi_cpha, uint8_t spi_cpol);
void spi_init_gpio(uint8_t spi_no, uint8_t sysclk_as_spiclk);
void spi_clock(uint8_t spi_no, uint16_t prediv, uint8_t cntdiv);
void spi_tx_byte_order(uint8_t spi_no, uint8_t byte_order);
void spi_rx_byte_order(uint8_t spi_no, uint8_t byte_order);

// Expansion Macros
#define spi_busy(spi_no) READ_PERI_REG(SPI_CMD(spi_no)) & SPI_USR

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_init
//   Description: Wrapper to setup HSPI/SPI GPIO pins and default SPI clock
//    Parameters: spi_no - SPI (0) or HSPI (1)
//
////////////////////////////////////////////////////////////////////////////////

void spi_init(spi_speed_t speed) {
  uint8_t spi_no = 1;
  spi_init_gpio(spi_no, SPI_CLK_USE_DIV);

  // spi_clock(spi_no, SPI_CLK_PREDIV, SPI_CLK_PREDIV);
  spi_set_speed(speed);
  // spi_clock(spi_no, SPI_CLK_PREDIV, SPI_CLK_CNTDIV);

  spi_tx_byte_order(spi_no, SPI_BYTE_ORDER_HIGH_TO_LOW);
  // spi_tx_byte_order(spi_no, SPI_BYTE_ORDER_LOW_TO_HIGH);
  // spi_rx_byte_order(spi_no, SPI_BYTE_ORDER_HIGH_TO_LOW);
  spi_rx_byte_order(spi_no, SPI_BYTE_ORDER_LOW_TO_HIGH);

  SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_CS_SETUP | SPI_CS_HOLD);
  CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_FLASH_MODE);

  spi_mode(spi_no, 0, 0);
  ETS_DEBUG("spi_init");
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_mode
//   Description: Configures SPI mode parameters for clock edge and clock
//   polarity.
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  spi_cpha - (0) Data is valid on clock leading
//edge 				             (1) Data is valid on clock trailing edge 				  spi_cpol - (0) Clock is low when
//inactive 				             (1) Clock is high when inactive
//
////////////////////////////////////////////////////////////////////////////////

void spi_mode(uint8_t spi_no, uint8_t spi_cpha, uint8_t spi_cpol) {
  if (!spi_cpha == !spi_cpol) {
    CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_CK_OUT_EDGE);
  } else {
    SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_CK_OUT_EDGE);
  }

  if (spi_cpol) {
    SET_PERI_REG_MASK(SPI_PIN(spi_no), SPI_IDLE_EDGE);
  } else {
    CLEAR_PERI_REG_MASK(SPI_PIN(spi_no), SPI_IDLE_EDGE);
  }
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_init_gpio
//   Description: Initialises the GPIO pins for use as SPI pins.
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  sysclk_as_spiclk - SPI_CLK_80MHZ_NODIV (1) if using
//80MHz 									 sysclock for SPI clock. 									 SPI_CLK_USE_DIV (0) if using divider to 									 get
//lower SPI clock speed.
//
////////////////////////////////////////////////////////////////////////////////

void spi_init_gpio(uint8_t spi_no, uint8_t sysclk_as_spiclk) {

  //	if(spi_no > 1) return; //Not required. Valid spi_no is checked with
  //if/elif below.

  uint32_t clock_div_flag = 0;
  if (sysclk_as_spiclk) {
    clock_div_flag = 0x0001;
  }

  if (spi_no == SPI) {
    WRITE_PERI_REG(
        PERIPHS_IO_MUX,
        0x005 | (clock_div_flag << 8)); // Set bit 8 if 80MHz sysclock required
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CLK_U, 1);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CMD_U, 1);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U, 1);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, 1);
  } else if (spi_no == HSPI) {
    WRITE_PERI_REG(
        PERIPHS_IO_MUX,
        0x105 | (clock_div_flag << 9)); // Set bit 9 if 80MHz sysclock required
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,
                    2); // GPIO12 is HSPI MISO pin (Master Data In)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,
                    2); // GPIO13 is HSPI MOSI pin (Master Data Out)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2); // GPIO14 is HSPI CLK pin (Clock)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U,
                    2); // GPIO15 is HSPI CS pin (Chip Select / Slave Select)
  }
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_clock
//   Description: sets up the control registers for the SPI clock
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  prediv - predivider value (actual division
//value) 				  cntdiv - postdivider value (actual division value) 				  Set either divider
//to 0 to disable all division (80MHz sysclock)
//
////////////////////////////////////////////////////////////////////////////////

static void esp_spi_set_speed(uint8_t spi_no, SpiSpeed speed) {
  if (1 < speed) {
    uint8_t i, k;
    i = (speed / 40) ? (speed / 40) : 1;
    k = speed / i;
    CLEAR_PERI_REG_MASK(SPI_CLOCK(spi_no), SPI_CLK_EQU_SYSCLK);
    WRITE_PERI_REG(SPI_CLOCK(spi_no),
                   (((i - 1) & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S) |
                       (((k - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) |
                       ((((k + 1) / 2 - 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) |
                       (((k - 1) & SPI_CLKCNT_L)
                        << SPI_CLKCNT_L_S)); // clear bit 31,set SPI clock div
  } else {
    WRITE_PERI_REG(SPI_CLOCK(spi_no), SPI_CLK_EQU_SYSCLK); // 80Mhz speed
  }
}

void spi_clock(uint8_t spi_no, uint16_t prediv, uint8_t cntdiv) {

  if (spi_no > 1)
    return;

  if ((prediv == 0) | (cntdiv == 0)) {

    WRITE_PERI_REG(SPI_CLOCK(spi_no), SPI_CLK_EQU_SYSCLK);

  } else {

    WRITE_PERI_REG(SPI_CLOCK(spi_no),
                   (((prediv - 1) & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S) |
                       (((cntdiv - 1) & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) |
                       (((cntdiv >> 1) & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) |
                       ((0 & SPI_CLKCNT_L) << SPI_CLKCNT_L_S));
  }
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_tx_byte_order
//   Description: Setup the byte order for shifting data out of buffer
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  byte_order - SPI_BYTE_ORDER_HIGH_TO_LOW (1)
//							   Data is sent out starting with
//Bit31 and down to Bit0
//
//							   SPI_BYTE_ORDER_LOW_TO_HIGH
//(0) 							   Data is sent out starting with the lowest BYTE, from 							   MSB to LSB, followed
//by the second lowest BYTE, from 							   MSB to LSB, followed by the second highest
//BYTE, from 							   MSB to LSB, followed by the highest BYTE, from MSB to LSB
//							   0xABCDEFGH would be sent as
//0xGHEFCDAB
//
//
////////////////////////////////////////////////////////////////////////////////

void spi_tx_byte_order(uint8_t spi_no, uint8_t byte_order) {

  if (spi_no > 1)
    return;

  if (byte_order) {
    SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_WR_BYTE_ORDER);
  } else {
    CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_WR_BYTE_ORDER);
  }
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_rx_byte_order
//   Description: Setup the byte order for shifting data into buffer
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  byte_order - SPI_BYTE_ORDER_HIGH_TO_LOW (1)
//							   Data is read in starting with
//Bit31 and down to Bit0
//
//							   SPI_BYTE_ORDER_LOW_TO_HIGH
//(0) 							   Data is read in starting with the lowest BYTE, from 							   MSB to LSB, followed
//by the second lowest BYTE, from 							   MSB to LSB, followed by the second highest
//BYTE, from 							   MSB to LSB, followed by the highest BYTE, from MSB to LSB
//							   0xABCDEFGH would be read as
//0xGHEFCDAB
//
//
////////////////////////////////////////////////////////////////////////////////

void spi_rx_byte_order(uint8_t spi_no, uint8_t byte_order) {

  if (spi_no > 1)
    return;

  if (byte_order) {
    SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_RD_BYTE_ORDER);
  } else {
    CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_RD_BYTE_ORDER);
  }
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
//
// Function Name: spi_transaction
//   Description: SPI transaction function
//    Parameters: spi_no - SPI (0) or HSPI (1)
//				  cmd_bits - actual number of bits to transmit
//				  cmd_data - command data
//				  addr_bits - actual number of bits to transmit
//				  addr_data - address data
//				  dout_bits - actual number of bits to transmit
//				  dout_data - output data
//				  din_bits - actual number of bits to receive
//
//		 Returns: read data - uint32_t containing read in data only if RX
//was set 				  0 - something went wrong (or actual read data was 0) 				  1 - data sent ok
//(or actual read data is 1) 				  Note: all data is assumed to be stored in the lower
//bits of 				  the data variables (for anything <32 bits).
//
////////////////////////////////////////////////////////////////////////////////

uint32_t spi_transaction(uint8_t cmd_bits, uint16_t cmd_data,
                         uint32_t addr_bits, uint32_t addr_data,
                         uint32_t dout_bits, const void *dout_data,
                         uint32_t din_bits, void *din_data,
                         uint32_t dummy_bits) {
  uint8_t spi_no = 1;

  // FIXME send some clocks
  if (selected_spi_no > 1) { // Check for a valid SPI
    // ETS_DEBUG("Invalid spi_transaction\n");
    /*
    dummy_bits = 8;
    cmd_bits = addr_bits = dout_bits = 0;
    dout_data = din_data = 0;
    din_bits = 0;
    */
    // CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_CS_SETUP|SPI_CS_HOLD);
    // SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_CS_SETUP|SPI_CS_HOLD);
    // return 0;
  }

  // code for custom Chip Select as GPIO PIN here

  while (spi_busy(spi_no))
    ; // wait for SPI to be ready

  //########## Enable SPI Functions ##########//
  // disable MOSI, MISO, ADDR, COMMAND, DUMMY in case previously set.
  CLEAR_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MOSI | SPI_USR_MISO |
                                            SPI_USR_COMMAND | SPI_USR_ADDR |
                                            SPI_USR_DUMMY);

  // enable functions based on number of bits. 0 bits = disabled.
  // This is rather inefficient but allows for a very generic function.
  // CMD ADDR and MOSI are set below to save on an extra if statement.
  //	if(cmd_bits) {SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_COMMAND);}
  //	if(addr_bits) {SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_ADDR);}
  if (din_bits) {
    SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_MISO);
  }
  if (dummy_bits) {
    SET_PERI_REG_MASK(SPI_USER(spi_no), SPI_USR_DUMMY);
  }
  //########## END SECTION ##########//

  //########## Setup Bitlengths ##########//
  WRITE_PERI_REG(
      SPI_USER1(spi_no),
      ((addr_bits - 1) & SPI_USR_ADDR_BITLEN)
              << SPI_USR_ADDR_BITLEN_S | // Number of bits in Address
          ((dout_bits - 1) & SPI_USR_MOSI_BITLEN)
              << SPI_USR_MOSI_BITLEN_S | // Number of bits to Send
          ((din_bits - 1) & SPI_USR_MISO_BITLEN)
              << SPI_USR_MISO_BITLEN_S | // Number of bits to receive
          ((dummy_bits - 1) & SPI_USR_DUMMY_CYCLELEN)
              << SPI_USR_DUMMY_CYCLELEN_S); // Number of Dummy bits to insert
  //########## END SECTION ##########//

  //########## Setup Command Data ##########//
  if (cmd_bits) {
    SET_PERI_REG_MASK(SPI_USER(spi_no),
                      SPI_USR_COMMAND); // enable COMMAND function in SPI module
    uint16_t command = cmd_data
                     << (16 - cmd_bits); // align command data to high bits
    command =
        ((command >> 8) & 0xff) | ((command << 8) & 0xff00); // swap byte order
    WRITE_PERI_REG(SPI_USER2(spi_no),
                   ((((cmd_bits - 1) & SPI_USR_COMMAND_BITLEN)
                     << SPI_USR_COMMAND_BITLEN_S) |
                    (command & SPI_USR_COMMAND_VALUE)));
  }
  //########## END SECTION ##########//

  //########## Setup Address Data ##########//
  if (addr_bits) {
    SET_PERI_REG_MASK(SPI_USER(spi_no),
                      SPI_USR_ADDR); // enable ADDRess function in SPI module
    WRITE_PERI_REG(SPI_ADDR(spi_no),
                   addr_data
                       << (32 - addr_bits)); // align address data to high bits
  }

  //########## END SECTION ##########//

  uint32_t data[17];
  if (dout_bits && dout_data) {
    ets_memcpy(data, dout_data, (dout_bits + 7) / 8);
    // Handle padding of the last word
    data[dout_bits / 32] <<= 32 - (dout_bits & 31);
  }

  //########## Setup DOUT data ##########//
  if (dout_bits) {
    SET_PERI_REG_MASK(SPI_USER(spi_no),
                      SPI_USR_MOSI); // enable MOSI function in SPI module
    // copy data to W0
    for (int i = 0; i < (dout_bits + 31) / 32; i++) {
      WRITE_PERI_REG(SPI_W0(spi_no) + i * 4, data[i]);
      // ETS_DEBUG("SPI send [%d] %02x\n", i, data[i]);
    }
  }
  //########## END SECTION ##########//

  //########## Begin SPI Transaction ##########//
  SET_PERI_REG_MASK(SPI_CMD(spi_no), SPI_USR);
  //########## END SECTION ##########//

  //########## Return DIN data ##########//
  if (din_bits) {
    while (spi_busy(spi_no))
      ; // wait for SPI transaction to complete

    for (int i = 0; i < (din_bits + 31) / 32; i++) {
      data[i] = READ_PERI_REG(SPI_W0(spi_no) + i * 4);
      // ETS_DEBUG("SPI rec [%d] %08x\n", i, data[i]);
    }
    if (din_data) {
      ets_memcpy(din_data, data, (din_bits + 7) / 8);
    }
    // FIXME handle padding & byteorder
  }
  //########## END SECTION ##########//

  // Transaction completed
  return 1; // success
}
