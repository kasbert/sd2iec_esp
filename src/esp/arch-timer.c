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


   arch-timer.c: Architecture-specific timer functions

*/

#include "config.h"
#include "timer.h"
#include <driver/hw_timer.h>

int    system_get_time(void);
SYSTEM_TICK_HANDLER;

void timer_init(void) {
  hw_timer_init(FRC1_SOURCE, 1);
  hw_timer_set_func(systick_handler);
  hw_timer_arm(10000);
}

static uint32_t timeout;
/**
 * start_timeout - start a timeout
 * @usecs: number of microseconds before timeout
 *
 * This function sets up a timer so it times out after the specified
 * number of microseconds.
 */

IRAM_ATTR
void start_timeout(uint32_t usecs) {
  timeout = system_get_time() + usecs;
}

/**
 * has_timed_out - returns true if timeout was reached
 *
 * This function returns true if the timer started by start_timeout
 * has reached its timeout value.
 */
IRAM_ATTR
unsigned int has_timed_out(void) {
  return timeout - system_get_time() >= 0x80000000;
}
