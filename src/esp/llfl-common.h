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


   llfl-common.h: Definition for common routines in low-level fastloader impl

*/

#ifndef LLFL_COMMON_H
#define LLFL_COMMON_H

typedef enum { NO_ATNABORT, ATNABORT } llfl_atnabort_t;

typedef struct {
  uint32_t pairtimes[4];
  uint8_t  clockbits[4];
  uint8_t  databits[4];
  uint8_t  eorvalue;
} generic_2bit_t;

#define llfl_setup() do{}while(0)
#define llfl_teardown() do{}while(0)

void llfl_wait_atn(unsigned int state);
void llfl_wait_clock(unsigned int state, llfl_atnabort_t atnabort);
void llfl_wait_data(unsigned int state, llfl_atnabort_t atnabort);

void llfl_set_2bit_at(uint32_t time, unsigned int clock_state, unsigned int data_state);
void llfl_set_clock_at(uint32_t time, unsigned int state);
void llfl_set_data_at(uint32_t time, unsigned int state);
void llfl_set_srq_at(uint32_t time, unsigned int state);
uint32_t llfl_read_bus_at(uint32_t time);
void llfl_set_reference(uint32_t time);
void llfl_generic_load_2bit(const generic_2bit_t *def, uint8_t byte);
uint8_t llfl_generic_save_2bit(const generic_2bit_t *def);

#endif
