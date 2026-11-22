/* NODISKEMU - SD/MMC to IEEE-488 interface/controller

   NODISKEMU is a fork of sd2iec by Ingo Korb (et al.), http://sd2iec.de

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA


   multidrive.c: Multi-drive support

   The configuration file DRIVES.CFG in the root directory of the card
   assigns a directory or disk image to each drive, one per line:

     # comment
     8:0 = //
     8:1 = //GAMES/ARCHON.D64
     9:1 = //WORK/
     9:2 = 2//BACKUP/
     0:1 = //GAMES/

   Address 0 is the device address of the SD2IEC itself (EEPROM, U0>,
   XW): a single device with several drives keeps its address. Lines
   with that address written as number belong to the same device then.

   The path uses the syntax of the CD command, as typed on the computer
   (names as the computer shows them, written in ASCII), and starts at
   the root directory of a partition of the card. A partition number in
   front of it selects the partition, the first one is the default.

   A device with drive 0 has CBM DOS 2 semantics: single drive or dual
   drive with drive 0 and 1. A device without drive 0 has CMD semantics:
   its drives are partitions 1..9, partition 0 is the current one and
   CP selects it.

   The drives of all devices come from a pool of MD_MAX_SLOTS slots in
   partition[], each pointing to the FAT volume of its partition.
   Without a configuration file nothing changes and the legacy single
   drive behaviour is kept.

   With CONFIG_MD_CONTAINERS a line with drive "*" makes a device of all
   partitions of a container image (D1M, D2M, D4M, RL) at once:

     10:* = //GAMES/COLLECTION.D4M

   The drive numbers are the partition numbers of its table. Such a
   device takes no other drives, the container is mounted completely
   or not at all.

   The XV commands set up drives without the file (XV+, XV-), read it
   on request (XVL) and switch reading it at power-on and card change
   on or off (XVA+/-, saved by XW). A card change always discards the
   drives set up by command.

*/

#include <ctype.h>
#include <stdint.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "bus.h"
#include "d64ops.h"
#include "dirent.h"
#include "doscmd.h"
#include "errormsg.h"
#include "eefs-ops.h"
#include "fatops.h"
#include "ff.h"
#include "filesystem.h"
#include "flags.h"
#include "parser.h"
#include "ustring.h"
#include "utils.h"
#include "multidrive.h"

uint8_t     md_active;
md_device_t md_dev[MD_MAX_DEVICES];
uint8_t     md_slot_dev[MD_MAX_SLOTS];
uint8_t     md_slot_drv[MD_MAX_SLOTS];
uint32_t    md_slot_root[MD_MAX_SLOTS];
uint16_t    md_slot_imgroot;
#ifdef CONFIG_MD_CONTAINERS
uint8_t     md_slot_owner[MD_MAX_SLOTS] = { [0 ... MD_MAX_SLOTS - 1] = MD_NO_SLOT };
uint16_t    md_slot_base[MD_MAX_SLOTS];
uint16_t    md_slot_size[MD_MAX_SLOTS];
#endif
uint8_t     md_slots;
uint8_t     md_volumes;
uint8_t     md_device_lost;
uint8_t     md_layout_changed;
uint8_t     md_cur;
uint8_t     md_drive_given;
static uint8_t md_merge_own;    /* lines with device_address go to device 0 */
static uint8_t md_implicit;     /* device made from the single drive by XV+ */
static uint8_t load_req;        /* md_load: MD_LOAD_*                       */

static uint8_t first_drive(uint8_t dev);

#if MD_MAX_SLOTS < 2
#  error CONFIG_MAX_PARTITIONS is too small for multi-drive support
#endif

/**
 * md_slot_drive - return the drive number of a partition slot
 * @part: partition index
 *
 * Returns the drive number from the configuration file, 0 for a
 * partition that is not a drive slot.
 */
uint8_t md_slot_drive(uint8_t part) {
  if (part >= md_slots)
    return 0;
  return md_slot_drv[part];
}

/* slot of a drive of a device or MD_NO_SLOT */
static uint8_t find_slot(uint8_t dev, uint8_t drive) {
  uint8_t i;

  for (i = 0; i < md_slots; i++)
    if (md_slot_dev[i] == dev && md_slot_drv[i] == drive)
      return i;
  return MD_NO_SLOT;
}

/**
 * md_drive_slot - return the partition index of a drive number
 * @drive: drive number of the current device
 *
 * With CMD semantics drive 0 is the current partition. Returns
 * MD_NO_SLOT if the current device has no such drive.
 */
uint8_t md_drive_slot(uint8_t drive) {
  if (drive == 0 && !md_dev[md_cur].dual)
    return current_part;
  return find_slot(md_cur, drive);
}

/**
 * md_is_dual - check for a CBM DOS 2 dual drive
 *
 * Returns non-zero if the current device has drive 0 and drive 1.
 */
uint8_t md_is_dual(void) {
  return md_active && md_dev[md_cur].dual &&
         find_slot(md_cur, 1) != MD_NO_SLOT;
}

/**
 * md_find - find the device with a bus address
 * @addr: bus address
 *
 * Returns the index of the device in md_dev or MD_NO_DEVICE.
 * Without a configuration only device_address is used.
 */
uint8_t md_find(uint8_t addr) {
  uint8_t i;

  if (!md_active)
    return addr == device_address ? 0 : MD_NO_DEVICE;

  for (i = 0; i < MD_MAX_DEVICES; i++)
    if (md_dev[i].address && md_dev_address(i) == addr)
      return i;

  return MD_NO_DEVICE;
}

/**
 * md_select - make a device the current one
 * @idx: index in md_dev
 *
 * Saves the current partition and the error channel state of the
 * previous device and restores the ones of the new device, so all code
 * using current_part works on it.
 */
void md_select(uint8_t idx) {
  md_device_t *dev;

  if (idx == md_cur || idx >= MD_MAX_DEVICES)
    return;

  dev = &md_dev[md_cur];
  dev->cur_part   = current_part;
  dev->err_code   = current_error;
  dev->err_track  = current_error_track;
  dev->err_sector = current_error_sector;

  md_cur       = idx;
  dev          = &md_dev[idx];
  current_part = dev->cur_part;
  set_error_ts(dev->err_code, dev->err_track, dev->err_sector);
}

/**
 * md_set_address - change the bus address of the current device
 * @addr: new address
 *
 * The change lasts until the next restart or card change. A device with
 * address 0 in the file changes the address of the SD2IEC itself, so XW
 * stores it in the EEPROM. Returns 0 if successful, 1 if the address is
 * out of range or already used by another device.
 */
uint8_t md_set_address(uint8_t addr) {
  if (addr < 4 || addr > 30)
    return 1;

  if (md_find(addr) != MD_NO_DEVICE && md_find(addr) != md_cur)
    return 1;

  if (!md_dev[md_cur].file_address)
    device_address = addr;
  md_dev[md_cur].address = addr;
  return 0;
}

/* the device of an address in the file (0 = own address) */
static uint8_t file_key(uint8_t address) {
  return (md_merge_own && address == device_address) ? 0 : address;
}

/* index of the device with an address in the file or MD_NO_DEVICE */
static uint8_t find_device(uint8_t file_address) {
  uint8_t i;

  for (i = 0; i < MD_MAX_DEVICES; i++)
    if (md_dev[i].address && md_dev[i].file_address == file_address)
      return i;
  return MD_NO_DEVICE;
}

/* find the device with an address in the file, create it if required */
static uint8_t get_device(uint8_t file_address) {
  uint8_t i = find_device(file_address);

  if (i != MD_NO_DEVICE)
    return i;

  for (i = 0; i < MD_MAX_DEVICES; i++) {
    if (!md_dev[i].address) {
      md_dev[i].address      = file_address ? file_address : device_address;
      md_dev[i].file_address = file_address;
      return i;
    }
  }
  return MD_NO_DEVICE;
}



/* Start of a drive line: "<address> : <drive> = <path>" */
typedef struct {
  uint8_t  address;
  uint8_t  drive;
  uint8_t  addr_len;     /* number of address digits */
  uint16_t addr_start;   /* offset of the address    */
  uint16_t path_start;   /* offset of the path       */
} md_head_t;

/**
 * parse_head - parse the start of a configuration file line
 * @line: line, need not be 0-terminated
 * @len : length of the line
 * @h   : receives the parsed values
 *
 * Returns 1 for a drive line with an address of 4..30 or 0 (the own
 * address) and a drive number of 0..9 (or "*", MD_CONTAINER), 0 for
 * comments, blank lines and everything else.
 */
static uint8_t parse_head(uint8_t *line, uint16_t len, md_head_t *h) {
  uint16_t i = 0;

#define BLANKS() while (i < len && (line[i] == ' ' || line[i] == '\t')) i++

  BLANKS();
  h->addr_start = i;
  h->address    = 0;
  while (i < len && isdigit(line[i]) && i - h->addr_start < 3)
    h->address = h->address * 10 + (line[i++] - '0');
  h->addr_len = i - h->addr_start;
  if (h->addr_len == 0 ||
      (h->address != 0 && (h->address < 4 || h->address > 30)))
    return 0;

  BLANKS();
  if (i >= len || line[i++] != ':')
    return 0;
  BLANKS();
#ifdef CONFIG_MD_CONTAINERS
  if (i < len && line[i] == '*') {
    h->drive = MD_CONTAINER;
    i++;
  } else
#endif
  {
    if (i >= len || !isdigit(line[i]))
      return 0;
    h->drive = line[i++] - '0';
  }

  BLANKS();
  if (i >= len || line[i++] != '=')
    return 0;
  BLANKS();
  h->path_start = i;
  return 1;

#undef BLANKS
}

/* cut trailing blanks of a 0-terminated string */
static void strip_trailing(uint8_t *str) {
  uint8_t *end = str + ustrlen(str);

  while (end > str && (end[-1] == ' ' || end[-1] == '\t'))
    end--;
  *end = 0;
}

/**
 * mount_slot - set up a partition slot for a path
 * @path : path, 0-terminated, at least three bytes after the start of
 *         the buffer it is in, which is modified in place
 * @slot : partition slot
 * @start: partition whose current directory relative paths start in,
 *         MD_NO_SLOT for DRIVES.CFG: the root, and the path is ASCII
 *
 * The path uses the CMD syntax of the CD command: "//" starts at the
 * root, everything else ("/DIR/", "NAME", ":NAME", nothing) is relative.
 * A partition number in front selects a FAT partition of the card,
 * relative paths start at its root then. Without one XV+ uses the
 * partition of @start, DRIVES.CFG the first one.
 *
 * Returns 1 if successful, otherwise 0 with the error channel set.
 * md_active must be 0, so the legacy partition syntax of do_chdir
 * reaches every slot.
 */
static uint8_t mount_slot(uint8_t *path, uint8_t slot, uint8_t start) {
  uint8_t *p, vol = 0;

  strip_trailing(path);

  /* optional partition number in front, like in the CD command */
  for (p = path; isdigit(*p) && p - path < 2; p++)
    vol = vol * 10 + (*p - '0');
  if (p != path && (*p == 0 || *p == ':' || *p == '/')) {
    path = p;
    if (vol) {
      /* a partition of the card: no current directory there */
      vol--;
      start = MD_NO_SLOT;
      if (vol >= md_volumes) {
        set_error(ERROR_PARTITION_ILLEGAL);
        return 0;
      }
    }
  } else {
    vol = 0;                    // digits of a name, not a partition
  }

  /* start in the root or in the current directory of @start */
  partition[slot].fop             = &fatops;
  partition[slot].fatfs           = &fatvol[vol];
  partition[slot].current_dir.fat = 0;
  md_slot_imgroot &= (uint16_t)~(1U << slot);
  if (start != MD_NO_SLOT) {
    partition[slot].fatfs = partition[start].fatfs;
    if (!(path[0] == '/' && path[1] == '/')) {
      if (partition[start].fop != &fatops) {
        /* a relative path inside an image */
        set_error(ERROR_SYNTAX_UNKNOWN);
        return 0;
      }
      partition[slot].current_dir = partition[start].current_dir;
    }
  }

  /* Build a legacy CD string for the slot: "<slot+1>" and the path,  */
  /* one or two digits. At least three characters precede the path     */
  /* ("8:0=" in the file, "XV+" in the command). Nothing means the     */
  /* start directory itself.                                           */
  if (*path) {
    uint32_t before = partition[slot].current_dir.fat;
    uint8_t  last, num = slot + 1;

    if (path[0] != '/' && path[0] != ':') {
      /* "DIR/NAME" or "NAME", the others are "//DIR/", "/DIR/NAME", ":NAME" */
      path--;
      path[0] = ustrchr(path + 1, '/') ? '/' : ':';
    }
    do {
      *--path = '0' + num % 10;
      num /= 10;
    } while (num);
    if (start == MD_NO_SLOT)
      asc2pet(path);            // digits, '/' and ':' stay as they are
    last = path[ustrlen(path) - 1];

    set_error(ERROR_OK);
    do_chdir(path);
    if (current_error != ERROR_OK) {
      if (partition[slot].fop != &fatops)
        image_unmount(slot);
      partition[slot].fop = &fatops;
      return 0;
    }

    /* CD into a file that is neither a directory nor a disk image  */
    /* succeeds without changing anything. A drive must not end up  */
    /* in the start directory instead, e.g. for an unknown format.  */
    if (partition[slot].fop == &fatops &&
        partition[slot].current_dir.fat == before &&
        last != '/' && last != ':' && last != '_') {
      set_error(ERROR_FILE_TYPE_MISMATCH);
      return 0;
    }
  }
  set_error(ERROR_OK);

  /* The slot is confined to this directory or image. For an image */
  /* the FAT directory it is in becomes the root, in case the image */
  /* is unmounted in some other way (e.g. by the disk swap list).   */
  md_slot_root[slot] = partition[slot].current_dir.fat;
  if (partition[slot].fop != &fatops)
    md_slot_imgroot |= 1U << slot;
  return 1;
}

#ifdef CONFIG_MD_CONTAINERS
/* ------------------------------------------------------------------------- */
/*  Container images: D1M, D2M, D4M (CMD FD), RL (CMD RAMLink)               */
/* ------------------------------------------------------------------------- */

/* A container holds the partitions of a CMD drive and a partition table   */
/* of 32 byte entries, 8 in each 256 byte block: partition 0 is the system  */
/* partition (type 255), the type is at offset 2, the name at 5, the start  */
/* at 0x15 (3 bytes) and the size at 0x1e (2 bytes, both high byte first).  */
#define CE_TYPE          2
#define CE_NAME          5
#define CE_START         0x15
#define CE_SIZE          0x1e
#define CT_SYSTEM        255
#define CT_NATIVE        1     /* 2: 1541, 3: 1571, 4: 1581 */
#define CT_MAX_BLOCKS    32    /* table blocks searched at most */

/**
 * container_table - find the partition table of a container file
 * @name : name of the file
 * @fsize: size of the file
 * @unit : receives the unit of the table entries in 256 byte blocks
 *
 * The FD images have their system partition behind 800 KB per density
 * step, the table 2 KB into it, with 512 byte units. A RAMLink image has
 * it 2 KB before its end, with 256 byte units. Returns the offset of the
 * table in 256 byte blocks, 0 if the name has no container extension or
 * the file cannot hold the table.
 */
static uint16_t container_table(uint8_t *name, uint32_t fsize, uint8_t *unit) {
  uint8_t  *ext = ustrrchr(name, '.');
  uint32_t table;

  if (ext == NULL)
    return 0;

  *unit = 2;
  if (toupper(ext[1]) == 'D' && toupper(ext[3]) == 'M' && ext[4] == 0 &&
      (ext[2] == '1' || ext[2] == '2' || ext[2] == '4')) {
    table = (ext[2] - '0') * 0xc8000UL + 0x800;
  } else if (toupper(ext[1]) == 'R' && toupper(ext[2]) == 'L' && ext[3] == 0 &&
             fsize >= 0x4000 && !(fsize & 0xff)) {
    *unit = 1;
    table = fsize - 0x800;
  } else {
    return 0;
  }

  if (table + 256 > fsize)
    return 0;
  return table >> 8;
}

/**
 * scan_table - check or mount the partitions of a container
 * @owner: slot with the open container file in its image handle
 * @dev  : device the partitions are for, MD_NO_DEVICE to only check
 * @table: offset of the partition table in 256 byte blocks
 * @unit : unit of the table entries in 256 byte blocks
 * @data : 256 byte work area
 *
 * Partitions of the types 1541, 1571, 1581 and native become drives,
 * the others are left out. Every one of them must lie in the file and
 * have room for its format. Without @dev nothing is changed and the
 * number of partitions is returned, 0 with the error channel set for
 * an invalid table. With @dev each partition is mounted on a free slot,
 * the first one on @owner, and the number of slots used is returned,
 * 0 if a partition could not be mounted.
 */
static uint8_t scan_table(uint8_t owner, uint8_t dev, uint16_t table,
                          uint8_t unit, uint8_t *data) {
  FIL      *fil   = &partition[owner].imagehandle;
  uint32_t blocks = fil->fsize >> 8;
  uint32_t start, size;
  uint16_t need;
  uint8_t  count = 0, k, i, num, slot = owner;
  uint8_t  *e;
  UINT     n;
  path_t   path;
  uint8_t  dnp[] = ".DNP";      // tells d64_mount a 3 track DNP from a D41

  for (k = 0; k < CT_MAX_BLOCKS && table + k < blocks; k++) {
    if (f_lseek(fil, (uint32_t)(table + k) << 8) != FR_OK ||
        f_read(fil, data, 256, &n) != FR_OK || n != 256)
      goto invalid;

    for (i = 0, e = data; i < 8; i++, e += 32) {
      num = k * 8 + i;
      if (num == 0) {
        if (e[CE_TYPE] != CT_SYSTEM)
          goto invalid;
        continue;
      }
      if (e[CE_TYPE] < CT_NATIVE || e[CE_TYPE] > 4)
        continue;               // empty, or no drive (CP/M, print buffer, ...)
      if (num > MD_MAX_PART)
        goto invalid;

      start = (((uint32_t)e[CE_START] << 16) |
               ((uint16_t)e[CE_START+1] << 8) | e[CE_START+2]) * unit;
      size  = (((uint16_t)e[CE_SIZE] << 8) | e[CE_SIZE+1]) * (uint32_t)unit;

      /* the part of the partition its format uses */
      switch (e[CE_TYPE]) {
      case 2:  need =  683; break;
      case 3:  need = 1366; break;
      case 4:  need = 3200; break;
      default:                  // native: whole tracks of 64 KB
        need = size > 255 * 256UL ? 255 * 256U : size & ~0xffUL;
        if (need == 0)
          goto invalid;
        break;
      }
      if (need > size || start > 0xffff || start + need > blocks)
        goto invalid;

      if (dev == MD_NO_DEVICE) {
        count++;
        continue;
      }

      /* mount it on the next free slot, the first one on the owner */
      if (count)
        for (slot = 0; md_slot_dev[slot] != MD_NO_DEVICE ||
                       md_slot_owner[slot] != MD_NO_SLOT; slot++) ;
      partition[slot].fop             = &fatops;
      partition[slot].fatfs           = partition[owner].fatfs;
      partition[slot].current_dir.fat = md_slot_root[owner];
      md_slot_owner[slot] = owner;
      md_slot_base[slot]  = start;
      md_slot_size[slot]  = need;
      path.part = slot;
      path.dir  = partition[slot].current_dir;
      if (d64_mount(&path, dnp)) {
        md_slot_owner[slot] = MD_NO_SLOT;
        return 0;
      }
      partition[slot].fop         = &d64ops;
      partition[slot].current_dir = path.dir;
      md_slot_root[slot] = md_slot_root[owner];
      md_slot_imgroot   |= 1U << slot;
      md_slot_dev[slot]  = dev;
      md_slot_drv[slot]  = num;
      count++;
    }

    if (data[0] == 0)
      break;                    // no link to a further table block
  }

  if (count)
    return count;

 invalid:
  set_error(ERROR_IMAGE_INVALID);
  return 0;
}

/**
 * mount_container - set up all partitions of a container image
 * @path : path of the container file like for mount_slot, modified
 * @dev  : device the partitions are for
 * @owner: free slot for the first partition, which holds the file
 * @start: like for mount_slot
 *
 * Returns the number of slots used, 0 with the error channel set if
 * the container could not be mounted completely. Then nothing changed.
 * md_active must be 0, like for mount_slot.
 */
static uint8_t mount_container(uint8_t *path, uint8_t dev, uint8_t owner,
                               uint8_t start) {
  uint8_t     *name, *p, saved = 0, unit = 0, count, nfree, i, err;
  uint16_t    table = 0;
  buffer_t    *buf;
  cbmdirent_t *dent;
  path_t      dir;
  FRESULT     res;
  FIL         *fil = &partition[owner].imagehandle;

  /* the directory part goes to mount_slot, the name is the file */
  strip_trailing(path);
  name = path;
  for (p = path; *p; p++)
    if (*p == '/' || *p == ':')
      name = p + 1;
  if (!*name) {
    set_error(ERROR_SYNTAX_NONAME);
    return 0;
  }
  if (name == path || name[-1] == ':') {
    if (name != path)
      name[-1] = 0;             // "2:NAME": partition 2, ":NAME": start dir
    p = (name == path) ? name + ustrlen(name) : path;
  } else {
    saved   = *name;            // "...DIR/NAME": keep the slash
    *name   = 0;
    p       = path;
  }
  i = mount_slot(p, owner, start);
  if (saved)
    *name = saved;
  if (!i)
    return 0;
  if (partition[owner].fop != &fatops) {
    image_unmount(owner);       // a directory part inside an image
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return 0;
  }
  if (start == MD_NO_SLOT)
    asc2pet(name);

  buf = alloc_system_buffer();
  if (!buf)
    return 0;
  dent = (cbmdirent_t *)buf->data;
  count = 0;

  /* find and open the file */
  dir.part = owner;
  dir.dir  = partition[owner].current_dir;
  if (first_match(&dir, name, FLAG_HIDDEN, dent))
    goto done;
  if ((dent->typeflags & TYPE_MASK) == TYPE_DIR) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    goto done;
  }
  part_fatfs(owner)->curr_dir = dir.dir.fat;
  res = f_open(part_fatfs(owner), fil, dent->pvt.fat.realname,
               FA_OPEN_EXISTING | FA_READ | FA_WRITE);
  if (res == FR_DENIED || res == FR_WRITE_PROTECTED)
    res = f_open(part_fatfs(owner), fil, dent->pvt.fat.realname,
                 FA_OPEN_EXISTING | FA_READ);
  if (res != FR_OK) {
    parse_error(res, 1);
    goto done;
  }
  table = container_table(dent->pvt.fat.realname, fil->fsize, &unit);
  if (table == 0) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    goto close;
  }

  /* check everything first, the partitions must all fit */
  count = scan_table(owner, MD_NO_DEVICE, table, unit, buf->data);
  if (count == 0)
    goto close;
  for (i = 0, nfree = 0; i < MD_MAX_SLOTS; i++)
    if (md_slot_dev[i] == MD_NO_DEVICE && md_slot_owner[i] == MD_NO_SLOT)
      nfree++;
  if (count > nfree) {
    count = 0;
    set_error(ERROR_NO_CHANNEL);
    goto close;
  }

  count = scan_table(owner, dev, table, unit, buf->data);
  if (count) {
    md_dev[dev].table = table;
    set_error(ERROR_OK);
    goto done;
  }

  /* undo: a partition could not be mounted */
  err = current_error;
  for (i = 0; i < MD_MAX_SLOTS; i++)
    if (md_slot_owner[i] == owner)
      md_slot_dev[i] = MD_NO_DEVICE;
  if (partition[owner].fop != &fatops) {
    image_unmount(owner);       // closes the file too
    set_error(err);
    goto done;
  }
  md_slot_owner[owner] = MD_NO_SLOT;
  set_error(err);

 close:
  f_close(fil);
 done:
  free_buffer(buf);
  return count;
}

/**
 * md_table_name - read a partition name from the table of a container
 * @part: a partition of the container
 * @num : partition number, 0 for the system partition
 * @name: receives the name, CBM_NAME_LENGTH+1 bytes
 *
 * The name is 0-terminated, without its padding. Returns 0 if
 * successful, otherwise the error channel is set.
 */
uint8_t md_table_name(uint8_t part, uint8_t num, uint8_t *name) {
  FIL     *fil = &partition[md_slot_owner[part]].imagehandle;
  FRESULT res;
  UINT    n;
  uint8_t i;

  res = f_lseek(fil, ((uint32_t)md_dev[md_slot_dev[part]].table << 8) +
                     num * 32 + CE_NAME);
  if (res == FR_OK)
    res = f_read(fil, name, CBM_NAME_LENGTH, &n);
  if (res != FR_OK) {
    parse_error(res, 1);
    return 1;
  }
  name[CBM_NAME_LENGTH] = 0;
  for (i = CBM_NAME_LENGTH; i-- && (name[i] == 0xa0 || name[i] == ' '); )
    name[i] = 0;
  return 0;
}
#endif

/**
 * apply_line - parse one line of the configuration file
 * @line : 0-terminated line, ASCII
 * @slot : next free partition slot
 *
 * Returns the number of slots used (more than one for a container),
 * 0 if none. The line buffer must be the command buffer, it is
 * modified in place.
 */
static uint8_t apply_line(uint8_t *line, uint8_t slot) {
  uint8_t dev;
  md_head_t h;

  if (!parse_head(line, ustrlen(line), &h))
    return 0;

  /* devices were created in the first pass, drive 0 made them dual */
  dev = find_device(file_key(h.address));
#ifdef CONFIG_MD_CONTAINERS
  /* a container device has its partitions and nothing else */
  if (dev != MD_NO_DEVICE && h.drive == MD_CONTAINER) {
    if (md_dev[dev].dual || first_drive(dev) != MD_NO_SLOT)
      return 0;
    return mount_container(line + h.path_start, dev, slot, MD_NO_SLOT);
  }
  if (dev != MD_NO_DEVICE && md_dev[dev].table)
    return 0;
#endif
  if (dev == MD_NO_DEVICE || (md_dev[dev].dual && h.drive > 1) ||
      find_slot(dev, h.drive) != MD_NO_SLOT)
    return 0;

  if (!mount_slot(line + h.path_start, slot, MD_NO_SLOT))
    return 0;

  md_slot_dev[slot] = dev;
  md_slot_drv[slot] = h.drive;
  return 1;
}

/**
 * read_line - read the next line of the configuration file
 * @fh : file
 * @pos: file position, advanced past the line
 *
 * Puts the line 0-terminated into the command buffer. Lines too long
 * for it are skipped. Returns 1 if a line was read, 0 at the end.
 */
static uint8_t read_line(FIL *fh, uint32_t *pos) {
  UINT     n;
  uint8_t  *p;
  uint8_t  skipping = 0;

  for (;;) {
    if (f_lseek(fh, *pos) != FR_OK)
      return 0;
    if (f_read(fh, command_buffer, CONFIG_COMMAND_BUFFER_SIZE - 1, &n) != FR_OK)
      return 0;
    if (n == 0)
      return 0;
    command_buffer[n] = 0;

    p = command_buffer;
    while (*p && *p != '\r' && *p != '\n')
      p++;
    *pos += p - command_buffer;

    if (*p == 0 && n == CONFIG_COMMAND_BUFFER_SIZE - 1) {
      /* line too long for the buffer, ignore it */
      skipping = 1;
      continue;
    }
    if (*p)
      (*pos)++;     // skip one line terminator, empty lines are ignored
    *p = 0;

    if (skipping) {
      skipping = 0;
      continue;
    }
    return 1;
  }
}

/**
 * md_edit_load - read the drive lines of DRIVES.CFG for editing
 * @tab   : table to fill, path pointers set by the caller
 * @max   : number of entries in tab
 * @exists: set to 1 if the file exists
 *
 * Duplicate lines are skipped like md_load does. Returns the number of
 * entries filled. Uses the command buffer.
 */
uint8_t md_edit_load(md_edit_t *tab, uint8_t max, uint8_t *exists) {
  FIL       fh;
  uint32_t  pos = 0;
  uint8_t   count = 0, i;
  uint8_t   name[] = MD_CONFIG_FILE;
  md_head_t h;
  uint8_t   *path;

  *exists = 0;
  fatvol[0].curr_dir = 0;
  if (f_open(&fatvol[0], &fh, name, FA_READ | FA_OPEN_EXISTING) != FR_OK)
    return 0;
  *exists = 1;

  while (count < max && read_line(&fh, &pos)) {
    if (!parse_head(command_buffer, ustrlen(command_buffer), &h))
      continue;
    for (i = 0; i < count; i++)
      if (tab[i].file_address == h.address && tab[i].file_drive == h.drive)
        break;
    if (i < count)
      continue;

    path = command_buffer + h.path_start;
    strip_trailing(path);
    ustrncpy(tab[count].path, path, MD_PATH_MAX - 1);
    tab[count].path[MD_PATH_MAX - 1] = 0;
    tab[count].file_address = h.address;
    tab[count].address      = h.address;
    tab[count].file_drive   = h.drive;
    tab[count].drive        = h.drive;
    tab[count].flags        = 0;
    count++;
  }

  f_close(&fh);
  return count;
}

/* write to a file, a short write means the card is full */
static FRESULT put(FIL *out, const void *data, UINT len) {
  UINT    done;
  FRESULT res = f_write(out, data, len, &done);

  if (res == FR_OK && done != len)
    res = FR_DENIED;
  return res;
}

/* write a line end, CR LF or LF */
static FRESULT put_eol(FIL *out, uint8_t crlf) {
  static const uint8_t eol[] = "\r\n";

  return put(out, eol + !crlf, 1 + crlf);
}

/* write the address as decimal number */
static FRESULT put_address(FIL *out, uint8_t address) {
  uint8_t digits[2], n = 0;

  if (address >= 10)
    digits[n++] = '0' + address / 10;
  digits[n++] = '0' + address % 10;
  return put(out, digits, n);
}

/* write a complete drive line "<address>:<drive> = <path>" */
static FRESULT put_line(FIL *out, md_edit_t *e, uint8_t crlf) {
  uint8_t mid[5] = { ':', '0', ' ', '=', ' ' };
  FRESULT res;

#ifdef CONFIG_MD_CONTAINERS
  if (e->drive == MD_CONTAINER)
    mid[1] = '*';
  else
#endif
    mid[1] += e->drive;
  res = put_address(out, e->address);
  if (res == FR_OK)
    res = put(out, mid, sizeof(mid));
  if (res == FR_OK)
    res = put(out, e->path, ustrlen(e->path));
  if (res == FR_OK)
    res = put_eol(out, crlf);
  return res;
}

static const PROGMEM uint8_t cfg_header[] =
  "# Drive configuration, written by the menu Configure drives\r\n"
  "# Format: device:drive = path   (drive 0..9, path like CD, from the root)\r\n"
  "# Drive 0: single/dual drive (0 and 1), otherwise partitions 1..9\r\n"
  "# Device 0: the own address of the device (EEPROM)\r\n"
  "\r\n";

/**
 * md_edit_save - write DRIVES.CFG with changes, line by line
 * @tab   : changed drive lines
 * @count : number of entries in tab
 * @create: create the file if it does not exist
 *
 * Lines of entries with a file address get the new address, are
 * written anew (MDE_NEWPATH, MDE_NEWDRIVE) or dropped (MDE_REMOVED).
 * MDE_ALLDRIVES entries match the lines of all drives of their file
 * address. Entries without a file address are appended. Comments,
 * blank lines and all other lines stay as they are. The new contents
 * go to DRIVES.TMP first, which replaces DRIVES.CFG at the end, so a
 * card pulled in between leaves the old file intact. Returns 0 if successful, otherwise the
 * error channel is set.
 */
uint8_t md_edit_save(md_edit_t *tab, uint8_t count, uint8_t create) {
  FATFS    *fs = &fatvol[0];
  FIL      in, out;
  FRESULT  res;
  UINT     n;
  uint32_t pos = 0;
  uint16_t len;
  uint8_t  midline = 0, crlf = 1, eol_seen = 0, have_in, i;
  uint8_t  name_cfg[] = MD_CONFIG_FILE;
  uint8_t  name_tmp[] = MD_TEMP_FILE;
  uint32_t saved_dir = fs->curr_dir;
  md_head_t h;
  md_edit_t *e;
  buffer_t *buf;
  uint8_t  *line;

  buf = alloc_system_buffer();
  if (!buf)
    return 1;               // error set by alloc
  line = buf->data;

  fs->curr_dir = 0;
  res = f_open(fs, &in, name_cfg, FA_READ | FA_OPEN_EXISTING);
  have_in = (res == FR_OK);
  if (!have_in && (res != FR_NO_FILE || !create))
    goto fail;

  res = f_open(fs, &out, name_tmp, FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK) {
    if (have_in)
      f_close(&in);
    goto fail;
  }

  if (!have_in) {
    memcpy_P(line, cfg_header, sizeof(cfg_header) - 1);
    res = put(&out, line, sizeof(cfg_header) - 1);
  }

  while (have_in && res == FR_OK) {
    res = f_lseek(&in, pos);
    if (res == FR_OK)
      res = f_read(&in, line, 256, &n);
    if (res != FR_OK || n == 0)
      break;

    /* one line including its terminator, or a 256 byte piece of it */
    for (len = 0; len < n && line[len] != '\n'; len++) ;
    if (len < n)
      len++;
    pos += len;

    /* new lines get the line end of the file */
    if (!eol_seen && line[len - 1] == '\n') {
      eol_seen = 1;
      crlf = (len >= 2 && line[len - 2] == '\r');
    }

    e = NULL;
    if (!midline && parse_head(line, len, &h)) {
      for (i = 0; i < count; i++)
        if (tab[i].file_address == h.address &&
            ((tab[i].flags & MDE_ALLDRIVES) || tab[i].file_drive == h.drive)) {
          e = &tab[i];
          break;
        }
    }

    if (e == NULL) {
      res = put(&out, line, len);
    } else if (e->flags & MDE_REMOVED) {
      /* drop the line */
    } else if (e->flags & (MDE_NEWPATH | MDE_NEWDRIVE)) {
      res = put_line(&out, e, crlf);
    } else {
      /* only the address changes */
      res = put(&out, line, h.addr_start);
      if (res == FR_OK)
        res = put_address(&out, e->address);
      if (res == FR_OK)
        res = put(&out, line + h.addr_start + h.addr_len,
                  len - h.addr_start - h.addr_len);
    }
    midline = (line[len - 1] != '\n');
  }

  /* append new lines */
  for (i = 0; i < count && res == FR_OK; i++) {
    e = &tab[i];
    if (e->file_address || (e->flags & MDE_REMOVED))
      continue;
    if (midline) {
      /* the file ended without a line end */
      res = put_eol(&out, crlf);
      midline = 0;
    }
    if (res == FR_OK)
      res = put_line(&out, e, crlf);
  }

  if (res == FR_OK)
    res = f_close(&out);
  else
    f_close(&out);
  if (have_in)
    f_close(&in);

  if (res == FR_OK && have_in)
    res = f_unlink(fs, name_cfg);
  if (res == FR_OK)
    res = f_rename(fs, name_tmp, name_cfg);
  else
    f_unlink(fs, name_tmp);

 fail:
  fs->curr_dir = saved_dir;
  free_buffer(buf);
  if (res != FR_OK) {
    parse_error(res, 0);
    return 1;
  }
  return 0;
}

/**
 * md_store_address - store an address change of the current device
 *
 * Called by XW. If U0> or M-W 119 changed the address of the addressed
 * device, the drive lines of that device in DRIVES.CFG get the new
 * address. Other devices are not touched. If another device still has
 * the new address in the file, nothing is written (30 SYNTAX ERROR),
 * as both would become one device at the next start. A device with
 * address 0 in the file has the address of the SD2IEC itself, which
 * XW stores in the EEPROM: the file stays as it is.
 */
void md_store_address(void) {
  md_device_t *dev = &md_dev[md_cur];
  md_edit_t   e;
  uint8_t     i;

  /* XVA-: DRIVES.CFG is not in use, devices of XV+ are not in it */
  if (!md_active || !dev->file_address || dev->address == dev->file_address ||
      dev->file_address == MD_FA_RUNTIME || (globalflags & MD_NOAUTO))
    return;

  for (i = 0; i < MD_MAX_DEVICES; i++) {
    if (i != md_cur && md_dev[i].address &&
        md_dev[i].file_address == dev->address) {
      set_error(ERROR_SYNTAX_UNKNOWN);
      return;
    }
  }

  e.file_address = dev->file_address;
  e.file_drive   = 0;
  e.drive        = 0;
  e.address      = dev->address;
  e.flags        = MDE_ALLDRIVES;
  e.path         = NULL;

  if (md_edit_save(&e, 1, 0) == 0)
    dev->file_address = dev->address;
}

/* Reset to the legacy single drive state (the partition table aside) */
static void md_reset(void) {
  md_active = 0;
  md_cur    = 0;
  md_slots  = 0;
  md_merge_own = 0;
  md_implicit  = MD_NO_DEVICE;
  md_slot_imgroot = 0;
  memset(md_dev, 0, sizeof(md_dev));
  memset(md_slot_dev, MD_NO_DEVICE, sizeof(md_slot_dev));
#ifdef CONFIG_MD_CONTAINERS
  memset(md_slot_owner, MD_NO_SLOT, sizeof(md_slot_owner));
#endif
}

/* The slot a device starts with: drive 0 of a CBM DOS 2 drive (which  */
/* needs it), otherwise its lowest drive. MD_NO_SLOT if it has none.   */
static uint8_t first_drive(uint8_t dev) {
  uint8_t d, s;

  for (d = 0; d <= MD_MAX_PART; d++) {
    if (d == 0 && !md_dev[dev].dual)
      continue;
    s = find_slot(dev, d);
    if (s != MD_NO_SLOT || d == 0)
      return s;
  }
  return MD_NO_SLOT;
}

/**
 * md_load - read the configuration file and set up the drive slots
 * @prev_address: bus address that stays the current device if it exists
 *
 * Leaves legacy mode active if there is no card or no configuration
 * file, or if reading it is switched off (XVA-) and not requested
 * (XVL). Clears load_req if the file was found.
 */
static void md_load(uint8_t prev_address) {
  FIL      fh;
  FRESULT  res;
  uint32_t pos;
  uint8_t  i, d, pass;
  uint8_t  saved_error, saved_track, saved_sector;
  uint8_t  name[] = MD_CONFIG_FILE;
  buffer_t *save;
  md_head_t h;

  md_reset();

  /* fatops_init mounted the FAT volumes as partitions 1..max_part */
  md_volumes = max_part;

  if (max_part == 0 || partition[0].fop != &fatops)
    return;

  if (load_req == MD_LOAD_NEVER ||
      (load_req == MD_LOAD_AUTO && (globalflags & MD_NOAUTO)))
    return;

  part_fatfs(0)->curr_dir = 0;
  res = f_open(part_fatfs(0), &fh, name, FA_READ | FA_OPEN_EXISTING);
  if (res != FR_OK) {
    /* md_edit_save was interrupted between deleting the old file */
    /* and renaming the new one, which was complete at that point    */
    uint8_t tmpname[] = MD_TEMP_FILE;
    res = f_open(part_fatfs(0), &fh, tmpname, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK)
      return;
  }
  load_req = MD_LOAD_AUTO;

  saved_error  = current_error;
  saved_track  = current_error_track;
  saved_sector = current_error_sector;

  /* After a card change this runs right before the pending command is */
  /* executed, so the command buffer must survive reading the file.    */
  save = alloc_system_buffer();
  if (save)
    memcpy(save->data, command_buffer, sizeof(command_buffer));

  /* allow the legacy partition syntax in do_chdir to reach every slot */
  max_part = MD_MAX_SLOTS;

  /* first pass: devices and their semantics, second pass: the drives */
  for (pass = 0; pass < 2; pass++) {
    pos = 0;
    while (md_slots < MD_MAX_SLOTS && read_line(&fh, &pos)) {
      if (pass) {
        md_slots += apply_line(command_buffer, md_slots);
      } else if (parse_head(command_buffer, ustrlen(command_buffer), &h)) {
        d = get_device(h.address);
        if (d != MD_NO_DEVICE && h.drive == 0)
          md_dev[d].dual = 1;
      }
    }

    /* the own address as number and as 0: one device */
    if (pass == 0) {
      i = find_device(0);
      d = find_device(device_address);
      if (i != MD_NO_DEVICE && d != MD_NO_DEVICE) {
        md_dev[i].dual   |= md_dev[d].dual;
        md_dev[d].address = 0;
        md_merge_own      = 1;
      }
    }
  }

  f_close(&fh);

  /* a device needs a drive, a dual drive device its drive 0 */
  for (i = 0; i < MD_MAX_DEVICES; i++) {
    if (!md_dev[i].address)
      continue;

    md_dev[i].cur_part = first_drive(i);
    if (md_dev[i].cur_part == MD_NO_SLOT) {
      md_dev[i].address = 0;
      for (d = 0; d < md_slots; d++) {
        if (md_slot_dev[d] == i) {
          md_slot_dev[d] = MD_NO_DEVICE;
          if (partition[d].fop != &fatops)
            image_unmount(d);
        }
      }
    }
  }

  if (save) {
    memcpy(command_buffer, save->data, sizeof(command_buffer));
    free_buffer(save);
  } else {
    memset(command_buffer, 0, sizeof(command_buffer));
  }
  set_error_ts(saved_error, saved_track, saved_sector);

  for (i = 0; i < MD_MAX_DEVICES; i++)
    if (md_dev[i].address)
      break;

  if (i == MD_MAX_DEVICES) {
    /* nothing usable configured, fall back to legacy mode */
    md_slots = 0;
    fatops_init(0);
    return;
  }

  /* every device starts with its power-on message */
  for (i = 0; i < MD_MAX_DEVICES; i++) {
    md_dev[i].err_code   = ERROR_DOSVERSION;
    md_dev[i].err_track  = 0;
    md_dev[i].err_sector = 0;
  }

  /* The slots keep their place, so XV+ can fill the free ones. The */
  /* EEPROM file system gets the partition behind them.             */
  max_part  = MD_MAX_SLOTS;
  md_active = 1;

  /* device_address stays untouched: it is the legacy address */
  /* from the EEPROM, used again when the file is removed.     */
  i = md_find(prev_address);
  if (i == MD_NO_DEVICE)
    for (i = 0; !md_dev[i].address; i++) ;
  md_cur       = i;
  current_part = md_dev[md_cur].cur_part;
}

/**
 * md_init - read the configuration file and set up the drive slots
 *
 * Called by filesystem_init after fatops_init. Sets md_device_lost if
 * the device addressed before (e.g. by a command that is still pending
 * after a card change) does not exist anymore and md_layout_changed if
 * the list of bus addresses differs from the one before.
 */
void md_init(void) {
  uint8_t old_address[MD_MAX_DEVICES];
  uint8_t old_active = md_active;
  uint8_t prev_address = MD_CUR_ADDRESS;
  uint8_t i;

  for (i = 0; i < MD_MAX_DEVICES; i++)
    old_address[i] = md_dev[i].address;

  md_load(prev_address);

  md_device_lost = (md_find(prev_address) == MD_NO_DEVICE);

  if (md_active != old_active)
    md_layout_changed = 1;
  for (i = 0; i < MD_MAX_DEVICES; i++)
    if (md_dev[i].address != old_address[i])
      md_layout_changed = 1;
}

/* ------------------------------------------------------------------------- */
/*  XV: drives without DRIVES.CFG                                            */
/* ------------------------------------------------------------------------- */

/* the EEPROM file system goes to the partition behind the FAT ones */
#ifdef CONFIG_HAVE_EEPROMFS
static void place_eefs(void) {
  if (eefs_partition != 255)
    partition[eefs_partition].fop = &fatops;
  eefsops_init();
}
#else
#  define place_eefs() do {} while (0)
#endif

/* set the current slot of a device */
static void set_cur(uint8_t dev, uint8_t slot) {
  if (dev == md_cur)
    current_part = slot;
  else
    md_dev[dev].cur_part = slot;
}

/**
 * close_slots - close the channels of a device and free slots
 * @dev : device index
 * @slot: slot to free, MD_NO_SLOT for all slots of the device
 *
 * All channels of the device are closed, like when a drive is switched
 * off, and the images of the freed slots are unmounted.
 */
static void close_slots(uint8_t dev, uint8_t slot) {
  uint8_t cur = md_cur, i;

  md_cur = dev;               // for FMB_CUR_DEVICE in both calls
  free_multiple_buffers(FMB_DEV_USER_CLEAN);
  for (i = 0; i < md_slots; i++) {
    if (md_slot_dev[i] == dev && (slot == MD_NO_SLOT || slot == i)) {
      if (partition[i].fop != &fatops)
        image_unmount(i);
      md_slot_dev[i] = MD_NO_DEVICE;
    }
  }
  md_cur = cur;
}

/**
 * md_reinit - close everything and set up the drives again
 * @req: MD_LOAD_FORCE to read DRIVES.CFG, MD_LOAD_NEVER for single drive
 *
 * Like a card change, but the channels are closed properly and the
 * images flushed. Returns non-zero if DRIVES.CFG was requested and
 * not found.
 */
uint8_t md_reinit(uint8_t req) {
  uint8_t i;

  free_multiple_buffers(FMB_USER_CLEAN);
  for (i = 0; i < (md_active ? md_slots : md_volumes); i++)
    if ((!md_active || md_slot_dev[i] != MD_NO_DEVICE) &&
        partition[i].fop != &fatops)
      image_unmount(i);
  free_multiple_buffers(FMB_ALL);

  load_req = req;
  filesystem_init(0);
  req = (load_req == MD_LOAD_FORCE);
  load_req = MD_LOAD_AUTO;
  return req;
}

/**
 * to_multi - turn the single drive into the first device
 *
 * The device keeps its address and gets each FAT partition as drive,
 * with CMD semantics. Current directories, images and channels stay.
 */
static void to_multi(void) {
  uint8_t i, n = md_volumes;

  if (n > MD_MAX_SLOTS)
    n = MD_MAX_SLOTS;

  /* the partitions behind the slots become unreachable */
  for (i = n; i < md_volumes; i++)
    if (partition[i].fop != &fatops)
      image_unmount(i);
  if (current_part >= n)
    current_part = 0;

  md_dev[0].address = device_address;   // file_address 0: follows it
  for (i = 0; i < n; i++) {
    md_slot_dev[i]  = 0;
    md_slot_drv[i]  = i + 1;
    md_slot_root[i] = 0;
  }
  md_slots    = n;
  md_implicit = 0;
  md_active   = 1;
  max_part    = MD_MAX_SLOTS;
  place_eefs();
  md_layout_changed = 1;
}

/**
 * xv_add - XV+: add a drive
 * @line: "<address>:<drive>=<path>" like a line of DRIVES.CFG, PETSCII
 *
 * A new address creates a device. The first drive of a device decides
 * its semantics like in DRIVES.CFG: drive 0 makes it a CBM DOS 2 drive.
 * In single drive mode the device itself stays as it is and its address
 * cannot get drives.
 */
static void xv_add(uint8_t *line) {
  md_head_t h;
  uint8_t   addr, dev, slot, i, ok, was_single = !md_active;
  uint8_t   newdev = MD_NO_DEVICE;

  if (!parse_head(line, ustrlen(line), &h))
    goto syntax;

  addr = h.address ? h.address : device_address;
  dev  = md_find(addr);
#ifdef CONFIG_MD_CONTAINERS
  /* a container makes a new device, which takes no other drives */
  if (dev != MD_NO_DEVICE &&
      (h.drive == MD_CONTAINER || md_dev[dev].table))
    goto syntax;
#endif
  if (dev != MD_NO_DEVICE) {
    if (was_single || dev == md_implicit ||
        find_slot(dev, h.drive) != MD_NO_SLOT)
      goto syntax;
    /* drive 0 makes a CBM DOS 2 drive, which has drive 1 at most */
    if (md_dev[dev].dual && h.drive > 1)
      goto syntax;
    for (i = 0; i < md_slots; i++)
      if (h.drive == 0 && md_slot_dev[i] == dev && md_slot_drv[i] > 1)
        goto syntax;
  }

  if (was_single)
    to_multi();

  if (dev == MD_NO_DEVICE) {
    for (newdev = 0; newdev < MD_MAX_DEVICES && md_dev[newdev].address; newdev++) ;
    if (newdev == MD_MAX_DEVICES)
      goto nochannel;
    memset(&md_dev[newdev], 0, sizeof(md_device_t));   // still unused
  }

  for (slot = 0; slot < MD_MAX_SLOTS && md_slot_dev[slot] != MD_NO_DEVICE; slot++) ;
  if (slot == MD_MAX_SLOTS)
    goto nochannel;

  md_active = 0;
#ifdef CONFIG_MD_CONTAINERS
  if (h.drive == MD_CONTAINER)
    ok = mount_container(line + h.path_start, newdev, slot, current_part);
  else
#endif
    ok = mount_slot(line + h.path_start, slot, current_part);
  md_active = 1;
  if (!ok)
    goto undo;

  if (newdev != MD_NO_DEVICE) {
    dev = newdev;
    md_dev[dev].address      = addr;
    md_dev[dev].file_address = MD_FA_RUNTIME;
    md_dev[dev].err_code     = ERROR_DOSVERSION;
    md_dev[dev].cur_part     = slot;
  }
  if (h.drive == 0) {
    md_dev[dev].dual = 1;
    set_cur(dev, slot);
  }
#ifdef CONFIG_MD_CONTAINERS
  if (h.drive == MD_CONTAINER) {
    /* the partitions have their slots already, the first one is current */
    for (i = 0; i < MD_MAX_SLOTS; i++)
      if (md_slot_dev[i] != MD_NO_DEVICE && i >= md_slots)
        md_slots = i + 1;
    set_error(ERROR_OK);
    return;
  }
#endif
  md_slot_dev[slot] = dev;
  md_slot_drv[slot] = h.drive;
  if (slot >= md_slots)
    md_slots = slot + 1;
  set_error(ERROR_OK);
  return;

 nochannel:
  set_error(ERROR_NO_CHANNEL);
 undo:
  if (was_single) {
    /* back to the single drive, nothing else was changed */
    md_reset();
    max_part = md_volumes;
    place_eefs();
  }
  return;

 syntax:
  set_error(ERROR_SYNTAX_UNKNOWN);
}

/**
 * xv_delete - XV-: delete a drive or a device
 * @addr : bus address, 0 for the own one
 * @drive: drive number, MD_NO_SLOT for the whole device
 *
 * Without devices left the SD2IEC returns to single drive mode like
 * after a card change, with 00,OK in the error channel. A device that
 * deletes itself is gone, like a drive that is switched off.
 */
static void xv_delete(uint8_t addr, uint8_t drive) {
  uint8_t dev = md_find(addr ? addr : device_address);
  uint8_t slot = MD_NO_SLOT, cur, i;

  if (dev == MD_NO_DEVICE)
    goto syntax;

  if (drive != MD_NO_SLOT) {
    /* the device made from the single drive has fixed drives, */
    /* a container device has all partitions of the container  */
    if (!md_active || dev == md_implicit)
      goto syntax;
#ifdef CONFIG_MD_CONTAINERS
    if (md_dev[dev].table)
      goto syntax;
#endif
    slot = find_slot(dev, drive);
    if (slot == MD_NO_SLOT)
      goto syntax;
  }

  if (md_active) {
    close_slots(dev, slot);
    if (drive == 0)
      md_dev[dev].dual = 0;   // drive 1 alone has CMD semantics

    cur = (dev == md_cur) ? current_part : md_dev[dev].cur_part;
    if (slot == MD_NO_SLOT || cur == slot)
      cur = first_drive(dev);
    if (cur != MD_NO_SLOT) {
      set_cur(dev, cur);
      set_error(ERROR_OK);
      return;
    }

    /* no drive left: the device is gone */
    md_dev[dev].address = 0;
    if (dev == md_implicit)
      md_implicit = MD_NO_DEVICE;
    for (i = 0; i < MD_MAX_DEVICES; i++) {
      if (md_dev[i].address) {
        if (dev == md_cur)
          md_select(i);
        else
          set_error(ERROR_OK);
        return;
      }
    }
  }

  md_reinit(MD_LOAD_NEVER);
  set_error(ERROR_OK);
  return;

 syntax:
  set_error(ERROR_SYNTAX_UNKNOWN);
}

/**
 * md_xcommand - XV commands, command_buffer+2 follows the XV
 *
 *   XV+a:d=path  add drive d of device a, path like in DRIVES.CFG
 *   XV-a:d       delete drive d of device a
 *   XV-a         delete device a
 *   XV-          delete all devices: single drive
 *   XVL          read DRIVES.CFG now
 *   XVA+/XVA-    read DRIVES.CFG at power-on and card change (XW)
 *
 * Address 0 is the own address of the SD2IEC.
 */
void md_xcommand(void) {
  uint8_t  *str = command_buffer + 3;
  uint16_t addr;
  uint8_t  drive = MD_NO_SLOT;

  switch (command_buffer[2]) {
  case '+':
    xv_add(str);
    return;

  case '-':
    if (!*str) {
      md_reinit(MD_LOAD_NEVER);
      set_error(ERROR_OK);
      return;
    }
    if (!isdigit(*str))
      break;
    addr = parse_number(&str);
    if (*str == ':') {
      str++;
      if (!isdigit(*str))
        break;
      drive = *str++ - '0';
    }
    if (*str || (addr && (addr < 4 || addr > 30)))
      break;
    xv_delete(addr, drive);
    return;

  case 'L':
    if (*str)
      break;
    if (md_reinit(MD_LOAD_FORCE))
      set_error(ERROR_FILE_NOT_FOUND);
    else
      set_error(md_active ? ERROR_OK : ERROR_SYNTAX_UNKNOWN);
    return;

  case 'A':
    if (*str == '+')
      globalflags &= (uint8_t)~MD_NOAUTO;
    else if (*str == '-')
      globalflags |= MD_NOAUTO;
    else
      break;
    set_error_ts(ERROR_STATUS, MD_CUR_ADDRESS, 0);
    return;
  }

  set_error(ERROR_SYNTAX_UNKNOWN);
}
