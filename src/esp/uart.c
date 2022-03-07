/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2021 Jarkko Sonninen <kasper@iki.fi>
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


   uart.c: UART access routines

*/

#include <stdio.h>
#include "config.h"
#include "uart.h"

#undef bool
#undef true
#undef false
//#include "driver/uart.h"
#include "driver/uart_register.h"
#define Uart_Buf_Init() Uart_Buf_Init(void)
#define uart_init driver_uart_init
#include <driver/uart.h>
#undef uart_init

void uart_div_modify(uint8_t uart_no, uint32_t DivLatchValue);

#define UART0   0
#define UART1   1

static char txbuf[1 << CONFIG_UART_TX_BUF_SHIFT];
static volatile uint16_t read_idx;
static volatile uint16_t write_idx;

void uart_tx(char c);

void uart_puthex(uint8_t num) {
  uint8_t tmp;
  tmp = (num & 0xf0) >> 4;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);

  tmp = num & 0x0f;
  if (tmp < 10)
    uart_putc('0'+tmp);
  else
    uart_putc('a'+tmp-10);
}

void uart_trace(void *ptr, uint16_t start, uint16_t len) {
  uint16_t i;
  uint8_t j;
  uint8_t ch;
  uint8_t *data = ptr;

  data+=start;
  for(i=0;i<len;i+=16) {

    uart_puthex(start>>8);
    uart_puthex(start&0xff);
    uart_putc('|');
    uart_putc(' ');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data + j);
        uart_puthex(ch);
      } else {
        uart_putc(' ');
        uart_putc(' ');
      }
      uart_putc(' ');
    }
    uart_putc('|');
    for(j=0;j<16;j++) {
      if(i+j<len) {
        ch=*(data++);
        if(ch<32 || ch>0x7e)
          ch='.';
        uart_putc(ch);
      } else {
        uart_putc(' ');
      }
    }
    uart_putc('|');
    uart_putcrlf();
    start+=16;
  }
}

void uart_flush(void) {
  while (read_idx != write_idx) {
    uart_tx(txbuf[read_idx]);
    read_idx = (read_idx+1) & (sizeof(txbuf)-1);
  }
}

void uart_puts_P(const char *text) {
  uint8_t ch;

  while ((ch = pgm_read_byte(text++))) {
    uart_putc(ch);
  }
}

void uart_putcrlf(void) {
  uart_putc('\n');
}

uint8_t uart_getc(void) {
  // FIXME not implemented
  uint8_t c = -1;
  return c;
}

static void uart_fill_buffer(void) {
  uint8_t uart = 0;
  while (read_idx != write_idx) {
    char c = txbuf[read_idx];
    uint32_t fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);
    if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) >= 126) {
      break;
    }
    WRITE_PERI_REG(UART_FIFO(uart), c);
    read_idx = (read_idx+1) & (sizeof(txbuf)-1);
  }
}

void uart_putc(char c) {
  int tmp = (write_idx+1) & (sizeof(txbuf)-1);
  if (read_idx == tmp) {
    uart_flush();
  }
  txbuf[write_idx] = c;
  write_idx = tmp;
  uart_fill_buffer();
}

/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

void uart_tx(char c)
{
    uint8_t uart = 0;
    while (1) {
        uint32_t fifo_cnt = READ_PERI_REG(UART_STATUS(uart)) & (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);
        if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
            break;
        }
    }

    WRITE_PERI_REG(UART_FIFO(uart), c);
}

void uart_init(void) {
  uint8_t uart_no = 0;
  UartBautRate         baut_rate = BIT_RATE_115200;
  UartBitsNum4Char     data_bits = EIGHT_BITS;
  UartExistParity      exist_parity = STICK_PARITY_EN;
  UartParityMode       parity = NONE_BITS;
  UartStopBitsNum      stop_bits = ONE_STOP_BIT;
  uint8_t      TrigLvl = 0;

    if (uart_no == UART1) {
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U, FUNC_U1TXD_BK);
    } else {
        /* rcv_buff size if 0x100 */
        //ETS_UART_INTR_ATTACH(uart0_rx_intr_handler,  &(UartDev.rcv_buff));
        PIN_PULLUP_DIS(PERIPHS_IO_MUX_U0TXD_U);
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
    }

    uart_div_modify(uart_no, UART_CLK_FREQ / (baut_rate));//SET BAUDRATE

    WRITE_PERI_REG(UART_CONF0(uart_no), ((exist_parity & UART_PARITY_EN_M)  <<  UART_PARITY_EN_S) //SET BIT AND PARITY MODE
                   | ((parity & UART_PARITY_M)  << UART_PARITY_S)
                   | ((stop_bits & UART_STOP_BIT_NUM) << UART_STOP_BIT_NUM_S)
                   | ((data_bits & UART_BIT_NUM) << UART_BIT_NUM_S));

    //clear rx and tx fifo,not ready
    SET_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);    //RESET FIFO
    CLEAR_PERI_REG_MASK(UART_CONF0(uart_no), UART_RXFIFO_RST | UART_TXFIFO_RST);

    if (uart_no == UART0) {
        //set rx fifo trigger
        WRITE_PERI_REG(UART_CONF1(uart_no),
                       ((100 & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S) |
                       (0x02 & UART_RX_TOUT_THRHD) << UART_RX_TOUT_THRHD_S |
                       UART_RX_TOUT_EN |
                       ((0x10 & UART_TXFIFO_EMPTY_THRHD) << UART_TXFIFO_EMPTY_THRHD_S)); //wjl
        SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_TOUT_INT_ENA | UART_FRM_ERR_INT_ENA);
    } else {
        WRITE_PERI_REG(UART_CONF1(uart_no), ((TrigLvl & UART_RXFIFO_FULL_THRHD) << UART_RXFIFO_FULL_THRHD_S)); //TrigLvl default val == 1
    }

    //clear all interrupt
    WRITE_PERI_REG(UART_INT_CLR(uart_no), 0xffff);
    //enable rx_interrupt
    SET_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_RXFIFO_FULL_INT_ENA | UART_RXFIFO_OVF_INT_ENA);
}


