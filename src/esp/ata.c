/*
    Copyright (C) 2022 Jarkko Sonninen <kasper@iki.fi>
    Copyright Jim Brain and Brain Innovations, 2005

    This file is part of uIEC.

    uIEC is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    uIEC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with uIEC; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


    The exported functions in this file are weak-aliased to their corresponding
    versions defined in diskio.h so when this file is the only diskio provider
    compiled in they will be automatically used by the linker.

*/
#include <inttypes.h>
#include "config.h"

#include "diskio.h"
#include "ata.h"

#include <spi_flash.h>

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/


static DSTATUS ATA_drv_flags[2];

#define SECTOR_SIZE 512
static const uint32_t capacity = FS_PARTITION_SIZE / SECTOR_SIZE; // sectors
#define FLASH_FS_OFFSET FS_PARTITION_ADDR


/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

void ata_init(void) {
}
void disk_init(void) __attribute__ ((weak, alias("ata_init")));


/*-----------------------------------------------------------------------*/
/* Initialize Disk Drive                                                 */
/*-----------------------------------------------------------------------*/

DSTATUS ata_initialize (BYTE drv) {
  ETS_DEBUG("ata_initialize %d", drv);
  if(drv>1) return STA_NOINIT;

  if (!capacity) {
    return STA_NOINIT | STA_NODISK;
  }

  disk_state = DISK_OK;
  ATA_drv_flags[drv] &= (BYTE)~( STA_NOINIT | STA_NODISK);
  ETS_DEBUG("ata_initialize OK %d sectors * %d bytes at %x", capacity, SECTOR_SIZE, FLASH_FS_OFFSET);
  return 0;
}
DSTATUS disk_initialize (BYTE drv) __attribute__ ((weak, alias("ata_initialize")));


/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS ata_status (BYTE drv) {
  if(drv>1)
     return STA_NOINIT;
  return ATA_drv_flags[drv] & (STA_NOINIT | STA_NODISK);
}
DSTATUS disk_status (BYTE drv) __attribute__ ((weak, alias("ata_status")));


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT ata_read (BYTE drv, BYTE *data, DWORD sector, BYTE count) {
  uint32_t addr = (uint32_t)sector * SECTOR_SIZE + FLASH_FS_OFFSET;
  if (drv > 1 || !count) return RES_PARERR;
  if (ATA_drv_flags[drv] & STA_NOINIT) return RES_NOTRDY;
  if (sector > capacity || sector + count > capacity) return RES_PARERR;

  spi_flash_read(addr, (uint32_t *)data, count * SECTOR_SIZE);

  return RES_OK;
}
DRESULT disk_read (BYTE drv, BYTE *data, DWORD sector, BYTE count) __attribute__ ((weak, alias("ata_read")));


/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _READONLY == 0
DRESULT ata_write (BYTE drv, const BYTE *data, DWORD sector, BYTE count) {
  uint32_t addr = (uint32_t)sector * SECTOR_SIZE + FLASH_FS_OFFSET;
  if (drv > 1 || !count) return RES_PARERR;
  if (ATA_drv_flags[drv] & STA_NOINIT) return RES_NOTRDY;
  if (sector > capacity || sector + count > capacity) return RES_PARERR;

  spi_flash_write(addr, (uint32_t *)data, count * SECTOR_SIZE);

  return RES_OK;
}
DRESULT disk_write (BYTE drv, const BYTE *data, DWORD sector, BYTE count) __attribute__ ((weak, alias("ata_write")));
#endif /* _READONLY == 0 */


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL != 0
DRESULT ata_ioctl (BYTE drv, BYTE ctrl, void *buff) {

  if (drv > 1) return RES_PARERR;
  if (ATA_drv_flags[drv] & STA_NOINIT) return RES_NOTRDY;

  switch (ctrl) {
    case GET_SECTOR_COUNT : /* Get number of sectors on the disk (DWORD) */
      *(WORD*)buff = capacity;
      break;

    case GET_SECTOR_SIZE :  /* Get sectors on the disk (WORD) */
      *(WORD*)buff = SECTOR_SIZE / 256;
      return RES_OK;

    case GET_BLOCK_SIZE :   /* Get erase block size in sectors (DWORD) */
      *(DWORD*)buff = 16; // FIXME
      return RES_OK;

    case CTRL_SYNC :        /* Nothing to do */
      return RES_OK;

    case ATA_GET_REV :      /* Get firmware revision (8 chars) */
      strcpy(buff, "Rev 0.1");
      break;

    case ATA_GET_MODEL :    /* Get model name (40 chars) */
      strcpy(buff, "Model 0.1");
      break;

    case ATA_GET_SN :       /* Get serial number (20 chars) */
      strcpy(buff, "Serial 0.1");
      break;

    default:
      return RES_PARERR;
  }

  return RES_OK;
}
DRESULT disk_ioctl (BYTE drv, BYTE ctrl, void *buff) __attribute__ ((weak, alias("ata_ioctl")));
#endif /*  _USE_IOCTL != 0 */

DRESULT ata_getinfo(BYTE drv, BYTE page, void *buffer) {
  diskinfo0_t *di = buffer;

  if (page != 0)
    return RES_ERROR;

  di->validbytes  = sizeof(diskinfo0_t);
  di->disktype    = DISK_TYPE_ATA; // TODO
  di->sectorsize  = SECTOR_SIZE / 256;
  di->sectorcount = capacity;

  return RES_OK;
}
DRESULT disk_getinfo(BYTE drv, BYTE page, void *buffer) __attribute__ ((weak, alias("ata_getinfo")));
