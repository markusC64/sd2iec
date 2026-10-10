/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>

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


   fileops.h: Definitions for file operations

*/

#ifndef FILEOPS_H
#define FILEOPS_H

#include "buffers.h"
#include "dirent.h"
#include "progmem.h"

enum open_modes { OPEN_READ, OPEN_WRITE, OPEN_APPEND, OPEN_MODIFY };

/* Flags for open_read, all other bits are reserved and must be zero */
/* The file system delivers the file as a GEOS CVT stream: Signature */
/* block, [info block, record block,] data. If the file isn't a CVT  */
/* already, the signature block is added (see cvt.c).                */
#define OPENFLAG_CVT (1<<0)

/* With bit 7 the open flags select a VLIR record: bits 0-6 are its number, */
/* the file is opened as with OPENFLAG_CVT and then positioned to the       */
/* record as with P on the channel. Bit 0 is part of the number then.       */
#define OPENFLAG_RECORD 0x80
#define OPENFLAG_RECNUM 0x7f

#ifdef CONFIG_CVT
/* A record selected with P on channel 15 for the next open (or copy), */
/* RECSEL_NONE if there is none                                        */
#define RECSEL_NONE 0xff
extern uint8_t record_select;

/* An open for writing drops the selected record too. Without this, */
/* only opens for reading (and commands) take it away.               */
#define RECSEL_CLEAR_ON_WRITE
#endif

/* The first block of such a stream is the signature block (doc/CVT.TXT): the  */
/* name of the file is in it, 16 characters from this offset, padded with $a0. */
/* Every file system delivers it that way, so it may be changed on the way.    */
#define CVT_NAME_OFFSET 3
extern const PROGMEM uint8_t filetypes[];

/* saved dirent of the last opened file */
extern cbmdirent_t previous_file_dirent;

/* Refill-callback for large buffers, only used for comparision */
uint8_t directbuffer_refill(buffer_t *buf);

/* reopen the last opened file on secondary 0 */
void file_open_previous(void);

/* Parses a filename in command_buffer and opens that file */
void file_open(uint8_t secondary);

#endif
