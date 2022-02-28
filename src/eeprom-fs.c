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


   eeprom-fs.c: Tiny file system in the unused EEPROM space

*/

#include "config.h"
#include <alloca.h>
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "arch-eeprom.h"
#include "buffers.h"
#include "eeprom-fs.h"

/* ------------------------------------------------------------------------- */
/*  data structures, constants, global variables                             */
/* ------------------------------------------------------------------------- */

/* avoid writing unchanged bytes to the EEPROM  */
/* currently implemented only for the directory */
#define EEPROMFS_MINIMIZE_WRITES

/* a buffer with at least 32 bytes */
#define EEPROMFS_BUFFER ops_scratch

/* another buffer with at least 32 bytes */
/* used only once to initialize a uint8_t pointer, */
/* so using an expression or alloca() is ok        */
#define EEPROMFS_CMP_BUFFER alloca(32)

/* marker for free sectors */
#define SECTOR_FREE      0xff

/* file flags */
#define EEFS_FLAG_DELETED   (1<<0)
#define EEFS_FLAG_LISTENTRY (1<<1)
// TODO: hidden, read-only?

typedef struct {
  uint8_t  sectors[12];            // must be in same position in both name and list
  uint8_t  name[EEFS_NAME_LENGTH];
  uint16_t size;
  uint8_t  flags;
  uint8_t  nextentry;
} nameentry_t;

typedef struct {
  uint8_t  sectors[30];
  uint8_t  flags;
  uint8_t  nextentry;
} listentry_t;

_Static_assert(sizeof(nameentry_t) == sizeof(listentry_t),
               "nameentry and listentry sizes must match");
_Static_assert(offsetof(nameentry_t, sectors) == offsetof(listentry_t, sectors),
               "sector list must start at same offset in both structs");

#define DATA_OFFSET  (EEPROMFS_OFFSET + EEPROMFS_ENTRIES * sizeof(nameentry_t))
#define SECTOR_COUNT ((EEPROMFS_SIZE - EEPROMFS_ENTRIES * sizeof(nameentry_t)) / EEPROMFS_SECTORSIZE)

_Static_assert(SECTOR_COUNT < 256,     "eepromfs must have less than 256 sectors");
_Static_assert(EEPROMFS_ENTRIES < 256, "eepromfs must have less than 256 directory entries");

static uint8_t used_sectors[(SECTOR_COUNT + 7) / 8];
static uint8_t used_entries[(EEPROMFS_ENTRIES + 7) / 8];
static uint8_t free_sectors;

static nameentry_t *nameptr = (nameentry_t *)EEPROMFS_BUFFER;
static listentry_t *listptr = (listentry_t *)EEPROMFS_BUFFER;

/* ------------------------------------------------------------------------- */
/*  Utility functions                                                        */
/* ------------------------------------------------------------------------- */

static void set_bit(uint8_t *bitmap, uint8_t bit, bool state) {
  if (state)
    bitmap[bit / 8] |= 1 << (bit % 8);
  else
    bitmap[bit / 8] &= ~(1 << (bit % 8));
}

static bool get_bit(uint8_t *bitmap, uint8_t bit) {
  return bitmap[bit / 8] & (1 << (bit % 8));
}

/**
 * read_entry - read a directory entry
 * @index: index of the directory entry
 *
 * This function reads the directory entry @index
 * from the EEPROM into the buffer.
 */
static void read_entry(uint8_t index) {
  eeprom_read_block(EEPROMFS_BUFFER,
                    (uint8_t *)(EEPROMFS_OFFSET + sizeof(nameentry_t) * index),
                    sizeof(nameentry_t));
}

/**
 * write_entry - write a directory entry
 * @index: index of the directory entry
 *
 * This function writes the directory entry @index
 * from the buffer into the EEPROM.
 */
static void write_entry(uint8_t index) {
#ifdef EEPROMFS_MINIMIZE_WRITES
  /* write just the changed bytes to the EEPROM */

  uint8_t *orig = EEPROMFS_CMP_BUFFER;
  uint8_t i, nonmatch_start;
  bool cur_nonmatching = false;

  /* read current entry to minimize writes */
  eeprom_read_block(orig,
                    (uint8_t *)(EEPROMFS_OFFSET + sizeof(nameentry_t) * index),
                    sizeof(nameentry_t));

  for (i = 0; i < sizeof(nameentry_t); i++) {
    if (cur_nonmatching) {
      if (EEPROMFS_BUFFER[i] == orig[i]) {
        cur_nonmatching = false;

        eeprom_write_block(EEPROMFS_BUFFER + nonmatch_start,
                             (uint8_t *)(EEPROMFS_OFFSET + sizeof(nameentry_t) * index + nonmatch_start),
                             i - nonmatch_start);
      }
    } else {
      if (EEPROMFS_BUFFER[i] != orig[i]) {
        cur_nonmatching = true;
        nonmatch_start  = i;
      }
    }
  }

  if (cur_nonmatching) {
    /* final block */
    eeprom_write_block(EEPROMFS_BUFFER + nonmatch_start,
                       (uint8_t *)(EEPROMFS_OFFSET + sizeof(nameentry_t) * index + nonmatch_start),
                       sizeof(nameentry_t) - nonmatch_start);
  }

#else
  /* write everything */
  eeprom_write_block(EEPROMFS_BUFFER,
                     (uint8_t *)(EEPROMFS_OFFSET + sizeof(nameentry_t) * index),
                     sizeof(nameentry_t));
#endif
}

/**
 * curentry_is_listentry - checks if the current entry is a listentry
 *
 * This function checks if the entry currently in the buffer is
 * a listentry. Returns true if yes, false if not.
 */
static bool curentry_is_listentry(void) {
  if (listptr->flags & EEFS_FLAG_LISTENTRY)
    return true;
  else
    return false;
}

/**
 * curentry_max_sectors - returns the maximum number of sectors storable in the current entry
 *
 * This function returns the maximum number of sector numbers
 * that can be stored in the direntry currently in the buffer.
 */
static uint8_t curentry_max_sectors(void) {
  if (curentry_is_listentry())
    return sizeof(listptr->sectors);
  else
    return sizeof(nameptr->sectors);
}

/**
 * mark_sector - mark a sector as free/used
 * @sector: number of the sector to mark
 * @used  : mark as used (true) or free (false)
 *
 * This function marks the sector @sector as used or
 * free depending on @used and updates the free_sectors
 * counter. Assumes that it will always be used to
 * flip the state of a sector.
 */
static void mark_sector(uint8_t sector, bool used) {
  assert(get_bit(used_sectors, sector) != used);

  set_bit(used_sectors, sector, used);
  if (used)
    free_sectors--;
  else
    free_sectors++;
}

/**
 * next_free_sector - find the next free sector
 * @start: sector to start the search at
 *
 * This function searches for the next free sector, starting
 * at @start (to allow some small amount of wear-levelling).
 * Returns a sector number or SECTOR_FREE if none is available.
 */
static uint8_t next_free_sector(uint8_t start) {
  uint8_t cur = start;

  while (get_bit(used_sectors, cur)) {
    cur++;
    if (cur == SECTOR_COUNT)
      cur = 0;

    if (cur == start)
      return SECTOR_FREE;
  }

  return cur;
}

/**
 * find_entry - find a directory entry by name
 * @name: pointer to file name array
 *
 * This function searches for a directory entry with the file
 * name @name. All characters of the name must be a binary match.
 * Returns the index of the entry or 0xff if not found.
 */
static uint8_t find_entry(uint8_t *name) {
  uint8_t idx = 0;

  while (idx < EEPROMFS_ENTRIES) {
    read_entry(idx++);

    if (nameptr->flags & (EEFS_FLAG_DELETED | EEFS_FLAG_LISTENTRY))
      continue;

    if (!memcmp(nameptr->name, name, EEFS_NAME_LENGTH))
      return idx - 1;
  }

  return 0xff;
}

/**
 * next_free_entry - find an empty directory entry
 * @start: entry to start search at
 *
 * This function searches for an empty directory entry, starting at entry
 * @start to provide a simple wear-levelling mechanism. The entry is NOT
 * marked as used. Returns the index of the entry or 0xff if none available
 * and marks it as used in used_entries.
 */
static uint8_t next_free_entry(uint8_t start) {
  uint8_t cur = start;

  while (get_bit(used_entries, cur)) {
    cur++;
    if (cur == EEPROMFS_ENTRIES)
      cur = 0;

    if (cur == start)
      return 0xff;
  }

  set_bit(used_entries, cur, true);

  return cur;
}

/**
 * min - returns minimum of two numbers
 * @a: first number
 * @b: second number
 *
 * This function returns the numerically smaller of @a and @b.
 */
static uint16_t min(uint16_t a, uint16_t b) {
  if (a < b)
    return a;
  else
    return b;
}

/* ------------------------------------------------------------------------- */
/*  external API                                                             */
/* ------------------------------------------------------------------------- */

void eepromfs_init(void) {
  uint8_t i, j;

  free_sectors = SECTOR_COUNT;
  memset(used_sectors, 0, sizeof(used_sectors));
  memset(used_entries, 0, sizeof(used_entries));

  /* scan all directory entries */
  for (i = 0; i < EEPROMFS_ENTRIES; i++) {
    read_entry(i);

    /* check if entry is in use */
    if (!(nameptr->flags & EEFS_FLAG_DELETED)) {
      set_bit(used_entries, i, true);

      /* mark their sectors as used */
      for (j = 0; j < curentry_max_sectors(); j++) {
        if (listptr->sectors[j] != SECTOR_FREE)
          mark_sector(listptr->sectors[j], true);
      }
    }
  }
}

void eepromfs_format(void) {
  uint32_t *addr = (uint32_t *)EEPROMFS_OFFSET;

  while (addr < (uint32_t *)(EEPROMFS_OFFSET + EEPROMFS_SIZE)) {
    uint32_t val;

    eeprom_read_block(&val, addr, sizeof(val));
    if (val != 0xffffffffUL) {
      /* clear only if neccessary to reduce number of writes */
      val = 0xffffffffUL;
      eeprom_write_block(&val, addr, sizeof(val));
    }
    addr++;
  }

  /* data structures have changed, re-init */
  eepromfs_init();
}

uint8_t eepromfs_free_sectors(void) {
  return free_sectors;
}

void eepromfs_opendir(eefs_dir_t *dh) {
  dh->entry = 0;
}

/**
 * eepromfs_readdir - read a directory entry
 * @dh   : pointer to directory handle
 * @entry: pointer to dirent structore for the result
 *
 * This function reads the next entry from the directory
 * handle @dh to the dirent structure @entry. Returns
 * zero if an entry was returned or non-zero if there
 * was no more entry to read.
 */
uint8_t eepromfs_readdir(eefs_dir_t *dh, eefs_dirent_t *entry) {
  /* loop until a valid new entry is found */
  while (dh->entry < EEPROMFS_ENTRIES) {
    /* read current entry */
    read_entry(dh->entry++);

    if (nameptr->flags & EEFS_FLAG_DELETED)
      continue;

    if (curentry_is_listentry())
      continue;

    /* copy information */
    memcpy(entry->name, nameptr->name, EEFS_NAME_LENGTH);
    entry->flags = nameptr->flags;
    entry->size  = nameptr->size;
    return 0;
  }

  return 1;
}

/**
 * eepromfs_open - open a file
 * @name : pointer to file name
 * @fh   : pointer to file handle
 * @flags: access flags
 *
 * This function sets up the handle give via @fh for access to the
 * file named in @name. Depending on the settings in @flags this
 * can be a read, write or append-access. Returns zero if successful,
 * non-zero if not.
 */
eefs_error_t eepromfs_open(uint8_t *name, eefs_fh_t *fh, uint8_t flags) {
  uint8_t diridx;

  memset(fh, 0, sizeof(eefs_fh_t));

  diridx = find_entry(name);

  if (flags == EEFS_MODE_WRITE) {
    /* write: forbid multiple files with the same name */
    if (diridx != 0xff)
      return EEFS_ERROR_FILEEXISTS;

    /* allocate directory entry */
    fh->entry = next_free_entry(0);
    if (fh->entry == 0xff)
      return EEFS_ERROR_DIRFULL;

    fh->cur_entry  = fh->entry;
    fh->cur_sindex = (uint8_t)-1;
    fh->filemode   = EEFS_MODE_WRITE;

    /* create direntry */
    memset(EEPROMFS_BUFFER, 0xff, sizeof(nameentry_t));
    memcpy(nameptr->name, name, EEFS_NAME_LENGTH);
    nameptr->size  = 0;
    nameptr->flags = 0;
    write_entry(fh->entry);

  } else {
    /* read, append: return error if the file does not exist */
    if (diridx == 0xff)
      return EEFS_ERROR_FILENOTFOUND;

    fh->entry = diridx;
    fh->size  = nameptr->size;

    if (flags == EEFS_MODE_APPEND) {
      /* load last direntry */
      while (listptr->nextentry != 0xff) {
        diridx = listptr->nextentry;
        read_entry(diridx);
      }

      /* search for last allocated sector */
      uint8_t sectoridx = 0;
      for (uint8_t i = 0; i < curentry_max_sectors(); i++) {
        if (listptr->sectors[i] == SECTOR_FREE)
          break;

        sectoridx = i;
      }

      fh->cur_sector  = listptr->sectors[sectoridx];
      fh->cur_entry   = diridx;
      fh->cur_offset  = fh->size;
      fh->cur_soffset = fh->size % EEPROMFS_SECTORSIZE;
      fh->filemode    = EEFS_MODE_WRITE;

    } else {
      /* read mode */
      fh->cur_sector = nameptr->sectors[0];
      fh->cur_entry  = diridx;
      fh->filemode   = EEFS_MODE_READ;
    }
  } // non-write

  return EEFS_ERROR_OK;
}

/**
 * eepromfs_write - write to a file
 * @fh           : pointer to file handle
 * @data         : pointer to data to be written
 * @length       : number of bytes to write
 * @bytes_written: pointer to output var for number of bytes written
 *
 * This function writes @length bytes from @data to the file opened on @fh.
 * The actual number of bytes written to the file is stored in
 * @bytes_written. Returns an eepromfs error code depending on the
 * result.
 */
eefs_error_t eepromfs_write(eefs_fh_t *fh, void *data, uint16_t length, uint16_t *bytes_written) {
  uint8_t *bdata = data;

  if (fh->filemode != EEFS_MODE_WRITE)
    return EEFS_ERROR_INVALID;

  *bytes_written = 0;

  while (length > 0) {
    if (fh->cur_soffset == 0) {
      /* need to allocate another sector */
      uint8_t next_sector = next_free_sector(fh->cur_sector);

      if (next_sector == SECTOR_FREE)
        return EEFS_ERROR_DISKFULL;

      fh->cur_sector = next_sector;

      /* mark it in the direntry */
      read_entry(fh->cur_entry);

      fh->cur_sindex++;
      if (fh->cur_sindex >= curentry_max_sectors()) {
        /* need to allocate another direntry */
        uint8_t next_entry = next_free_entry(fh->cur_entry);

        if (next_entry == 0xff)
          return EEFS_ERROR_DIRFULL;

        /* write link */
        listptr->nextentry = next_entry;
        write_entry(fh->cur_entry);

        /* build new entry in buffer */
        memset(listptr, 0xff, sizeof(listentry_t));
        listptr->flags = EEFS_FLAG_LISTENTRY;

        fh->cur_entry  = next_entry;
        fh->cur_sindex = 0;
      }

      /* mark sector as used in internal bookkeeping */
      mark_sector(next_sector, true);

      /* write new sector number to direntry */
      listptr->sectors[fh->cur_sindex] = next_sector;
      write_entry(fh->cur_entry);
    }

    uint8_t bytes_to_write = min(length, EEPROMFS_SECTORSIZE - fh->cur_soffset);
    eeprom_write_block(bdata,
                       (uint8_t *)(DATA_OFFSET + EEPROMFS_SECTORSIZE * fh->cur_sector + fh->cur_soffset),
                       bytes_to_write);

    /* adjust state */
    bdata           += bytes_to_write;
    fh->size        += bytes_to_write;
    fh->cur_offset  += bytes_to_write;
    fh->cur_soffset  = (fh->cur_soffset + bytes_to_write) % EEPROMFS_SECTORSIZE;
    length          -= bytes_to_write;
    *bytes_written  += bytes_to_write;
  }

  return EEFS_ERROR_OK;
}

/**
 * eepromfs_read - read from a file
 * @fh        : pointer to file handle
 * @data      : pointer to buffer for data to be read
 * @length    : number of bytes to read
 * @bytes_read: pointer to output var for number of bytes read
 *
 * This function writes @length bytes from @data to the file opened on @fh.
 * The actual number of bytes written to the file is stored in
 * @bytes_written. Returns an eepromfs error code depending on the
 * result.
 */
eefs_error_t eepromfs_read(eefs_fh_t *fh, void *data, uint16_t length, uint16_t *bytes_read) {
  uint8_t *bdata = data;

  if (fh->filemode != EEFS_MODE_READ)
    return EEFS_ERROR_INVALID;

  *bytes_read = 0;

  if (length > fh->size - fh->cur_offset)
    length = fh->size - fh->cur_offset;

  while (length > 0) {
    uint8_t bytes_to_read = min(length, EEPROMFS_SECTORSIZE - fh->cur_soffset);

    eeprom_read_block(bdata,
                      (uint8_t *)(DATA_OFFSET + EEPROMFS_SECTORSIZE * fh->cur_sector + fh->cur_soffset),
                      bytes_to_read);

    /* adjust state */
    *bytes_read    += bytes_to_read;
    bdata          += bytes_to_read;
    length         -= bytes_to_read;
    fh->cur_offset += bytes_to_read;
    fh->cur_soffset = (fh->cur_soffset + bytes_to_read) % EEPROMFS_SECTORSIZE;

    if (fh->cur_offset != fh->size && fh->cur_soffset == 0) {
      // FIXME: If rw-mode-support is added, this code generates a state that
      //        differs from the assumptions in the write path
      //        (current sector advances here, write assumes it hasn't)
      /* move to next sector */
      read_entry(fh->cur_entry);
      fh->cur_sindex++;

      if (fh->cur_sindex >= curentry_max_sectors()) {
        /* all sectors within current entry are processed */
        assert(listptr->nextentry != 0xff);

        fh->cur_entry  = listptr->nextentry;
        fh->cur_sindex = 0;
        read_entry(fh->cur_entry);
      }

      assert(listptr->sectors[fh->cur_sindex] != SECTOR_FREE);

      fh->cur_sector = listptr->sectors[fh->cur_sindex];
    }
  }

  return EEFS_ERROR_OK;
}

/**
 * eepromfs_close - close a file
 * @fh: handle of the file to close
 *
 * This function closes the file opened on @fh.
 * No return value.
 */
void eepromfs_close(eefs_fh_t *fh) {
  // FIXME: Read mode check could be removed if write_entry only writes changed bytes (EEPROMFS_MINIMIZE_WRITES)
  if (fh->filemode != EEFS_MODE_READ) {
    /* write new file length */
    read_entry(fh->entry);
    nameptr->size = fh->size;
    write_entry(fh->entry);
  }
  fh->filemode = 0;
}

/**
 * eepromfs_rename - rename a file
 * @oldname: current file name
 * @newname: new file name
 *
 * This function renames the file @oldname to @newname.
 * Returns an eepromfs error code depending on the result.
 */
eefs_error_t eepromfs_rename(uint8_t *oldname, uint8_t *newname) {
  uint8_t diridx;

  /* check if the new file name already exists */
  diridx = find_entry(newname);

  if (diridx != 0xff)
    return EEFS_ERROR_FILEEXISTS;

  /* read the directory entry */
  diridx = find_entry(oldname);

  if (diridx == 0xff)
    return EEFS_ERROR_FILENOTFOUND;

  memcpy(nameptr->name, newname, EEFS_NAME_LENGTH);

  write_entry(diridx);

  return EEFS_ERROR_OK;
}

/**
 * eepromfs_delete - delete a file
 * @name: name of the file to delete
 *
 * This function deletes the file @name.
 * Returns an eepromfs error code depending on the result.
 */
// FIXME: If this operation is interrupted it could
// leave a chain of listentries with no nameentry
// Add a small fsck in init?
eefs_error_t eepromfs_delete(uint8_t *name) {
  uint8_t diridx, old_diridx;

  /* look up the first directory entry */
  diridx = find_entry(name);
  if (diridx == 0xff)
    return EEFS_ERROR_FILENOTFOUND;

  while (diridx != 0xff) {
    /* free all sectors from the current entry */
    for (uint8_t i = 0; i < curentry_max_sectors(); i++) {
      if (listptr->sectors[i] == SECTOR_FREE)
        break;

      mark_sector(listptr->sectors[i], false);
    }

    /* remember number of next entry */
    old_diridx = diridx;
    diridx = listptr->nextentry;

    /* erase the current entry */
    memset(listptr, 0xff, sizeof(*listptr));
    write_entry(old_diridx);

    /* read next entry */
    if (diridx != 0xff)
      read_entry(diridx);
  }

  return EEFS_ERROR_OK;
}
