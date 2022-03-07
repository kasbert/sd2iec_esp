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


   arch-eeprom.c: EEPROM access functions

*/

#include "config.h"
#include "arch-eeprom.h"

#include <spi_flash.h>

// TODO the flash is also memory mapped, there might be no need for spi_flash_read...

/*
uint8_t eeprom_read_byte(uint32_t ptr) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  uint8_t val;
  spi_flash_read(addr, (uint32_t*)&val, 1);
  //ETS_DEBUG("eeprom_read_byte %x: %x", ptr, val);
  //ETS_DEBUG("eeprom_read_direct %x + %x: %x", FLASH_EEPROM_OFFSET, ptr, ((uint8_t*)(0x40200000 + 0x10000 + ptr))[0]);
  return val;
}

uint16_t eeprom_read_word(uint32_t ptr) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  uint16_t val;
  spi_flash_read(addr, (uint32_t*)&val, 2);
  ETS_DEBUG("eeprom_read_word %x + %x: %x", FLASH_EEPROM_OFFSET, ptr, val);
  //ETS_DEBUG("eeprom_read_direct %x + %x: %x", FLASH_EEPROM_OFFSET, ptr, ((uint16_t*)(0x40200000 + 0x10000 + ptr))[0]);
  return val;
}
*/

void eeprom_read_block(void *destptr, uint32_t ptr, unsigned int length) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  ETS_DEBUG("eeprom_read_block %x[length]: %x", ptr, length, *((uint32_t*)destptr));
  spi_flash_read(addr, destptr, length);
}

/*
// Do not use! There are alignment problems
void eeprom_write_byte(uint32_t ptr, uint8_t value) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  if (ptr >= FLASH_EEPROM_SIZE) {
    return;
  }
  ETS_DEBUG("eeprom_write_byte %x %x", ptr, value);
  spi_flash_write(addr, (uint32_t*)&value, 1);
}

void eeprom_write_word(uint32_t ptr, uint16_t value) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  if (ptr >= FLASH_EEPROM_SIZE) {
    return;
  }
  ETS_DEBUG("eeprom_write_word %x + %x %x", FLASH_EEPROM_OFFSET, ptr, value);
  spi_flash_write(addr, (uint32_t*)&value, 2);
}
*/

void eeprom_write_block(void *srcptr, uint32_t ptr, unsigned int length) {
  uint32_t addr = ptr + FLASH_EEPROM_OFFSET;
  if (ptr >= FLASH_EEPROM_SIZE) {
    return;
  }
  ETS_DEBUG("eeprom_write_block %x[%d]", addr, length);
  spi_flash_erase_sector(addr / 0x1000);
  spi_flash_write(addr, srcptr, length);
}
