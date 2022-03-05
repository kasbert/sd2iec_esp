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


   llfl-common.c: Common subroutines for low-level fastloader code

*/

#include <stdint.h>

#include "llfl-common.h"
#include "config.h"
#include "arch-timer.h"
#include "fastloader.h"
#include "iec-bus.h"


static uint32_t llfl_reference_time;
#define CLOCKS_PER_100ns (CONFIG_MCU_FREQ / 10000000)

/* ---------- utility functions ---------- */

/**
 * llfl_set_reference - set current time with given offset of 100ns's
 */
IRAM_ATTR
void llfl_set_reference(uint32_t time) {
  llfl_reference_time = asm_ccount() + time * CLOCKS_PER_100ns;
}

IRAM_ATTR
static inline void wait_to(uint32_t time) {
  uint32_t timeout = llfl_reference_time + time * CLOCKS_PER_100ns;
  while (1) {
    uint32_t delta = timeout - asm_ccount();
    if (delta >= 0x8000000) {
      return;
    }
  }
}

/**
 * llfl_wait_atn - wait until ATN has the specified level and capture time
 * @state: line level to wait for (0 low, 1 high)
 *
 * This function waits until the ATN line has the specified level and captures
 * the time when its level changed.
 */
IRAM_ATTR
void llfl_wait_atn(unsigned int state) {
  while (!IEC_ATN != !state)
    ;
  llfl_reference_time = asm_ccount();
}

/* llfl_wait_clock - see llfl_wait_atn, aborts on ATN low if atnabort is true */
IRAM_ATTR
void llfl_wait_clock(unsigned int state, llfl_atnabort_t atnabort) {
  if (atnabort == ATNABORT) {
    while (IEC_ATN && !IEC_CLOCK != !state)
      ;
  } else {
    while (!IEC_CLOCK != !state)
      ;
  }
  llfl_reference_time = asm_ccount();
}

/* llfl_wait_data - see llfl_wait_atn, aborts on ATN low if atnabort is true  */
IRAM_ATTR
void llfl_wait_data(unsigned int state, llfl_atnabort_t atnabort) {
  if (atnabort == ATNABORT) {
    while (IEC_ATN && !IEC_DATA != !state)
      ;
  } else {
    while (!IEC_DATA != !state)
      ;
  }
  llfl_reference_time = asm_ccount();
}

IRAM_ATTR
void llfl_set_2bit_at(uint32_t time, unsigned int clock_state,
                      unsigned int data_state) {
  wait_to(time);
  set_clock(clock_state);
  set_data(data_state);
  /*
  uint32_t mask1 = 0, mask0 = 0;
  if (COND_INV(clock_state)) {
    mask0 |= IEC_BIT_CLOCK;
  } else {
    mask1 |= IEC_BIT_CLOCK;
  }
  if (COND_INV(data_state)) {
    mask0 |= IEC_BIT_DATA;
  } else {
    mask1 |= IEC_BIT_DATA;
  }
  wait_to(time);
  //gpio_output_set(0, mask1, mask1, mask0);
  GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, mask0); // GPIO input
  GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, mask0);
  GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, mask1); // GPIO output
  GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, mask1);
  */
}

/**
 * llfl_set_clock_at - sets clock line at a specified time offset
 * @time : change time in 100ns after llfl_reference_time
 * @state: new line state (0 low, 1 high)
 *
 * This function sets the clock line to a specified state at a defined time
 * after the llfl_reference_time set by a previous wait_* function.
 */
IRAM_ATTR
void llfl_set_clock_at(uint32_t time, unsigned int state) {
  wait_to(time);
  set_clock(state);
}

/* llfl_set_data_at - see llfl_set_clock_at */
IRAM_ATTR
void llfl_set_data_at(uint32_t time, unsigned int state) {
  wait_to(time);
  set_data(state);
}

/* llfl_set_srq_at - see llfl_set_clock_at (nice for debugging) */
IRAM_ATTR
void llfl_set_srq_at(uint32_t time, unsigned int state) {
  wait_to(time);
  set_srq(state);
}

/**
 * llfl_read_bus_at - reads the IEC bus at a certain time
 * @time: read time in 100ns after llfl_reference_time
 *
 * This function returns the current IEC bus state at a certain time
 * after the llfl_reference_time set by a previous wait_* function.
 */
IRAM_ATTR
uint32_t llfl_read_bus_at(uint32_t time) {
  wait_to(time);
  return iec_bus_read();
}

/**
 * llfl_generic_load_2bit - generic 2-bit fastloader transmit
 * @def : pointer to fastloader definition struct
 * @byte: data byte
 *
 * This function implements generic 2-bit fastloader
 * transmission based on a generic_2bit_t struct.
 */
IRAM_ATTR
void llfl_generic_load_2bit(const generic_2bit_t *def, uint8_t byte) {
  unsigned int i;

  byte ^= def->eorvalue;

  for (i = 0; i < 4; i++) {
    uint8_t clock_state = byte & (1 << def->clockbits[i]);
    uint8_t data_state = byte & (1 << def->databits[i]);
    wait_to(def->pairtimes[i]);
    set_clock(clock_state);
    set_data(data_state);
    /*
    uint32_t mask1 = 0, mask0 = 0;
    if (COND_INV(byte & (1 << def->clockbits[i]))) {
      mask0 |= IEC_BIT_CLOCK;
    } else {
      mask1 |= IEC_BIT_CLOCK;
    }
    if (COND_INV(byte & (1 << def->databits[i]))) {
      mask0 |= IEC_BIT_DATA;
    } else {
      mask1 |= IEC_BIT_DATA;
    }
    wait_to(def->pairtimes[i]);
    // gpio_output_set(0, mask1, mask1, mask0);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, mask0); // GPIO input
    GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, mask0);
    GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, mask1); // GPIO output
    GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, mask1);
    */
  }
}

/**
 * llfl_generic_save_2bit - generic 2-bit fastsaver receive
 * @def: pointer to fastloader definition struct
 *
 * This function implements genereic 2-bit fastsaver reception
 * based on a generic_2bit_t struct.
 */
IRAM_ATTR
uint8_t llfl_generic_save_2bit(const generic_2bit_t *def) {
  unsigned int i;
  uint8_t result = 0;

  for (i = 0; i < 4; i++) {
    uint32_t bus = llfl_read_bus_at(def->pairtimes[i]);

    result |= (!!(bus & IEC_BIT_CLOCK)) << def->clockbits[i];
    result |= (!!(bus & IEC_BIT_DATA)) << def->databits[i];
  }

  return result ^ def->eorvalue;
}
