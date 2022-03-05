/* sd2iec - SD/MMC to Commodore serial bus interface/controller
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


   llfl-ulm3.c: Low level handling of ULoad Model 3 fastloader

*/

#include "config.h"
#include "iec-bus.h"
#include "llfl-common.h"
#include "system.h"
#include "timer.h"
#include "fastloader-ll.h"


static const generic_2bit_t uload3_get_def = {
  .pairtimes = {140, 240, 380, 480},
  .clockbits = {7, 6, 3, 2},
  .databits  = {5, 4, 1, 0},
  .eorvalue  = 0xff
};

static const generic_2bit_t uload3_send_def = {
  .pairtimes = {140, 220, 300, 380},
  .clockbits = {0, 2, 4, 6},
  .databits  = {1, 3, 5, 7},
  .eorvalue  = 0
};

IRAM_ATTR
int16_t uload3_get_byte(void) {
  uint8_t result;

  /* initial handshake */
  set_clock(0);
  while (IEC_DATA && IEC_ATN) ;
  if (!IEC_ATN)
    return -1;

  llfl_setup();
  disable_interrupts();

  /* wait for start signal */
  set_clock(1);
  llfl_wait_data(1, NO_ATNABORT);

  /* receive data */
  result = llfl_generic_save_2bit(&uload3_get_def);

  /* wait until the C64 releases the bus */
  delay_us(20);

  enable_interrupts();
  llfl_teardown();
  return result;
}

IRAM_ATTR
void uload3_send_byte(uint8_t byte) {
  llfl_setup();
  disable_interrupts();

  /* initial handshake */
  set_data(0);
  while (IEC_CLOCK && IEC_ATN) ;
  if (!IEC_ATN)
    goto exit;

  /* wait for start signal */
  set_data(1);
  llfl_wait_clock(1, ATNABORT);
  if (!IEC_ATN)
    goto exit;

  /* transmit data */
  llfl_generic_load_2bit(&uload3_send_def, byte);

  /* exit with clock+data high */
  llfl_set_2bit_at(480, 1, 1);

 exit:
  enable_interrupts();
  llfl_teardown();
}
