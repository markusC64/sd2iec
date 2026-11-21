/* NODISKEMU - SD/MMC to IEEE-488 interface/controller

   Multi-drive support: up to MD_MAX_DEVICES device addresses sharing a
   pool of MD_MAX_SLOTS drives, configured by a text file on the card.

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


   multidrive.h: Definitions for multi-drive support

*/

#ifndef MULTIDRIVE_H
#define MULTIDRIVE_H

#include <stdint.h>
#include "config.h"
#include "bus.h"

#define MD_MAX_DEVICES   4
#define MD_MAX_DRIVE     9     /* highest drive number in the file     */

/* The drives of all devices together get the partition table, except  */
/* for the entry of the EEPROM file system. The slot number must fit    */
/* into a single digit for do_chdir, so there are at most 9.            */
#ifdef CONFIG_HAVE_EEPROMFS
#  define MD_RESERVED_PARTS 1
#else
#  define MD_RESERVED_PARTS 0
#endif
#if CONFIG_MAX_PARTITIONS - MD_RESERVED_PARTS > 9
#  define MD_MAX_SLOTS   9
#else
#  define MD_MAX_SLOTS   (CONFIG_MAX_PARTITIONS - MD_RESERVED_PARTS)
#endif
#define MD_NO_SLOT       0xff
#define MD_NO_DEVICE     0xff
#define MD_FA_RUNTIME    0xff  /* file_address of a device made by XV+ */

/* Name of the configuration file in the root directory of the card */
#define MD_CONFIG_FILE   "DRIVES.CFG"
#define MD_TEMP_FILE     "DRIVES.TMP"  /* used while rewriting it */

/* A device has CBM DOS 2 semantics (single or dual drive) if drive 0 is  */
/* configured: drive numbers 0 and 1 are fixed drives. Otherwise it has   */
/* CMD semantics: drive numbers 1..9 are partitions, 0 is the current one.*/
/* Address 0 in DRIVES.CFG stands for the device address of the SD2IEC */
/* itself (EEPROM, U0>, XW), so a single device with several drives    */
/* keeps its address.                                                  */
typedef struct {
  uint8_t address;            /* bus address, 0 = entry unused       */
  uint8_t file_address;       /* address as written in DRIVES.CFG,   */
                              /*   0 = follows device_address,       */
                              /*   MD_FA_RUNTIME = made by XV+        */
  uint8_t dual;               /* drive 0 configured: CBM DOS 2 style */
  uint8_t cur_part;           /* current partition while not addressed */
  uint8_t err_code;           /* saved error channel state while the */
  uint8_t err_track;          /*   device is not addressed            */
  uint8_t err_sector;
} md_device_t;

#ifdef CONFIG_MULTIDRIVE

extern uint8_t     md_active;       /* non-zero if a configuration was loaded */
extern md_device_t md_dev[MD_MAX_DEVICES];
extern uint8_t     md_slot_dev[MD_MAX_SLOTS]; /* device of each slot or NO_DEVICE */
extern uint8_t     md_slot_drv[MD_MAX_SLOTS]; /* drive number of each slot   */
extern uint32_t    md_slot_root[MD_MAX_SLOTS]; /* root directory cluster of each slot */
extern uint16_t    md_slot_imgroot; /* bit mask: the root of the slot is an image */
extern uint8_t     md_slots;        /* number of slots in use (partition 0..) */
extern uint8_t     md_volumes;      /* number of FAT volumes on the card     */
extern uint8_t     md_cur;          /* index of the currently addressed device */
extern uint8_t     md_drive_given;  /* last parsed name had an explicit drive */
extern uint8_t     md_device_lost;  /* md_init: addressed device is gone   */
extern uint8_t     md_layout_changed; /* md_init: address list changed, cleared by the user */

/* Read the configuration file and set up the drive slots. */
/* Must be called after fatops_init and before eefsops_init. */
void md_init(void);

/* Return the index of the device with bus address addr or MD_NO_DEVICE */
uint8_t md_find(uint8_t addr);

/* Make device idx the current one, switching partition and error channel */
void md_select(uint8_t idx);

/* Change the bus address of the current device until the next restart */
/* Returns 0 if successful, 1 if the address is invalid or in use       */
uint8_t md_set_address(uint8_t addr);

/* XW: store an address change of the current device in DRIVES.CFG */
void md_store_address(void);

/* XV commands: drives without DRIVES.CFG, reading it on/off */
void md_xcommand(void);

/* Close everything and set up the drives again, see md_reinit */
#define MD_LOAD_AUTO     0     /* read DRIVES.CFG unless XVA- is set  */
#define MD_LOAD_FORCE    1     /* read it anyway (XVL, drives menu)   */
#define MD_LOAD_NEVER    2     /* single drive (XV-)                  */
uint8_t md_reinit(uint8_t req);

/* ---- editing DRIVES.CFG (drives menu, XW) ---- */

#define MD_PATH_MAX      128   /* path storage per drive line incl. 0 */
#define MD_PATH_LIMIT    112   /* longest path that fits a line:     */
                               /* 119 characters minus "30:1 = "      */
#define MD_EDIT_MAX      12    /* drive lines while editing           */

/* flags of md_edit_t */
#define MDE_REMOVED      1     /* drop the line                       */
#define MDE_NEWPATH      2     /* write the line anew with path       */
#define MDE_ALLDRIVES    4     /* matches the lines of all drives     */
#define MDE_NEWDRIVE     8     /* write the line anew with new drive  */

/* One drive line while editing the configuration */
typedef struct {
  uint8_t file_address;  /* address in the file, 0 = new line         */
  uint8_t file_drive;    /* drive number in the file                  */
  uint8_t drive;         /* drive number to write                     */
  uint8_t address;       /* address to write                          */
  uint8_t flags;         /* MDE_*                                     */
  uint8_t *path;         /* ASCII path, needed for new and MDE_NEW*   */
} md_edit_t;

/* Read the drive lines of DRIVES.CFG into tab, whose path pointers must */
/* point to MD_PATH_MAX bytes each. Returns the number of lines read,    */
/* *exists tells if the file exists. Uses the command buffer.           */
uint8_t md_edit_load(md_edit_t *tab, uint8_t max, uint8_t *exists);

/* Write DRIVES.CFG with the changes in tab via DRIVES.TMP.        */
/* Returns 0 if successful, otherwise the error channel is set.    */
uint8_t md_edit_save(md_edit_t *tab, uint8_t count, uint8_t create);

#ifdef CONFIG_ONBOARD_DISPLAY
/* The "Configure drives" menu */
void menu_drives(void);

/* "Browse files": let the user choose a drive and make it current. */
/* Returns 0 if the user went back.                                  */
uint8_t md_browse_select(void);
#endif

/* Bus address of a device: address 0 in the file follows device_address */
static inline uint8_t md_dev_address(uint8_t idx) {
  return md_dev[idx].file_address ? md_dev[idx].address : device_address;
}

/* Bus address of the currently addressed device */
#define MD_CUR_ADDRESS (md_active ? md_dev_address(md_cur) : device_address)

/* Non-zero if a bus address belongs to one of the devices */
#define md_is_address(a) (md_find(a) != MD_NO_DEVICE)

/* Non-zero if a TALK (base 0x40) or LISTEN (0x20) command is for us */
#define md_bus_cmd(cmd, base) \
  (((cmd) & 0xe0) == (base) && md_is_address((cmd) & 0x1f))

/* Return the partition index of a drive number of the current device:  */
/* with CMD semantics drive 0 is the current partition. MD_NO_SLOT if   */
/* the drive does not exist.                                            */
uint8_t md_drive_slot(uint8_t drive);

/* Return the drive number a partition index belongs to */
uint8_t md_slot_drive(uint8_t part);

/* Non-zero if the current device is a CBM DOS 2 dual drive */
uint8_t md_is_dual(void);

/* Visible partition number of a partition index (header, CP, G-P, ...) */
static inline uint8_t md_part_number(uint8_t part) {
  return md_active ? md_slot_drive(part) : part + 1;
}

/* Partition index of a visible partition number >= 1, >= max_part if  */
/* it does not exist                                                    */
static inline uint8_t md_number_part(uint8_t number) {
  return md_active ? md_drive_slot(number) : (uint8_t)(number - 1);
}

/* Root directory cluster of a partition: a slot is confined to the    */
/* directory of its configuration line ("//" and left arrow stop there) */
static inline uint32_t md_root_cluster(uint8_t part) {
  return (md_active && part < md_slots) ? md_slot_root[part] : 0;
}

/* Non-zero if the image of a partition is the root of its slot and */
/* must not be left with left arrow                                  */
static inline uint8_t md_root_locked(uint8_t part) {
  return md_active && part < md_slots && (md_slot_imgroot & (1U << part));
}

/* Non-zero if a partition index is visible for the current device.  */
/* The EEPROM file system behind the slots has no drive number, it is */
/* reached with "!" only.                                              */
static inline uint8_t md_part_visible(uint8_t part) {
  return !md_active || (part < md_slots && md_slot_dev[part] == md_cur);
}

#else

#  define md_active      0
#  define md_cur         0
#  define md_drive_given 0
#  define md_device_lost 0
#  define md_layout_changed 0
#  define md_find(a)     ((a) == device_address ? 0 : MD_NO_DEVICE)
#  define md_is_address(a) ((a) == device_address)
#  define md_bus_cmd(cmd, base) ((cmd) == (base) + device_address)
#  define md_select(i)   do {} while (0)
#  define MD_CUR_ADDRESS device_address
#  define md_init()      do {} while (0)
#  define md_drive_slot(d) MD_NO_SLOT
#  define md_slot_drive(p) 0
#  define md_is_dual()   0
#  define md_part_number(p) ((uint8_t)((p) + 1))
#  define md_number_part(n) ((uint8_t)((n) - 1))
#  define md_part_visible(p) 1
#  define md_root_cluster(p) 0
#  define md_root_locked(p)  0

#endif

#endif
