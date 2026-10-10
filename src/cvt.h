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


   cvt.h: GEOS CVT container layer

   This layer only knows the CVT format (see doc/CVT.TXT). It has no
   callbacks, no state and does not touch any buffer callback: What is
   delivered and in which order is decided by the backend in its open_read
   function, which also sets the refill pointer. The backend only asks
   this layer for the format-dependent parts.

   The layer is independent of any file system. It uses ops_scratch as
   work area, so it must not be called while that is in use.

*/

#ifndef CVT_H
#define CVT_H

#include <stdint.h>
#include "buffers.h"
#include "dirent.h"
#include "fileops.h"

#ifdef CONFIG_CVT

/**
 * cvt_peek_t - raw read access to a stored file, provided by the backend
 * @ctx   : opaque pointer handed through from the cvt_ function
 * @offset: byte offset in the stored file
 * @dst   : destination buffer
 * @len   : number of bytes to read
 *
 * Must read exactly len bytes from the stored file, without any
 * interpretation, and return 0 on success or non-zero on any error.
 */
typedef uint8_t (*cvt_peek_t)(void *ctx, uint16_t offset, uint8_t *dst, uint8_t len);

/**
 * cvt_poke_t - raw write access to a stored file, provided by the backend
 *
 * Writes len bytes at offset into the stored file, without any
 * interpretation. Returns 0 on success or non-zero on any error.
 */
typedef uint8_t (*cvt_poke_t)(void *ctx, uint16_t offset, const uint8_t *src, uint8_t len);

/**
 * cvt_probe - check if a stored file is a CVT (for readdir)
 *
 * REL files are CVTs as well. If it is, type and size of dent are replaced by those of the original
 * file and 0 is returned, as is the name unless CVT_FSNAME is set in
 * globalflags. Otherwise returns non-zero and dent is not modified.
 */
uint8_t cvt_probe(cvt_peek_t peek, void *ctx, cbmdirent_t *dent);

/**
 * cvt_set_name - write a new name into the entry of a stored CVT
 *
 * name is the new name (CBM name, 0 terminated, up to 16 characters). Only
 * the name is changed, the rest of the file is left alone. Returns 0 on
 * success, non-zero if poke failed.
 */
uint8_t cvt_set_name(cvt_poke_t poke, void *ctx, const uint8_t *name);

/**
 * cvt_set_type - change the type in the entry of a stored CVT
 *
 * Only the type bits of the first byte of the signature block are changed,
 * closed and lock are kept. A REL CVT keeps its type, it has the record
 * length where the others have their structure: Error 64 is set and 2 is
 * returned. Returns 0 on success, 1 if peek or poke failed.
 */
uint8_t cvt_set_type(cvt_peek_t peek, cvt_poke_t poke, void *ctx, uint8_t type);

/**
 * cvt_data_offset - find the data of a stored sequential CVT
 *
 * Returns the offset at which the data starts, i.e. the size of the
 * signature block plus INFO block (if there is one, the CVT can also be
 * one without, see addendum 2 of doc/CVT.TXT). This is what a
 * normal OPEN of the file delivers, like a real drive would. Returns 0
 * and sets the error channel if the file is not a valid CVT or is a VLIR
 * or REL file (not readable without OPENFLAG_CVT). A truncated file is not an
 * error, it just has no data.
 */
uint16_t cvt_data_offset(cvt_peek_t peek, void *ctx);

/**
 * cvt_record - find a record of a stored VLIR CVT
 * @peek  : raw read access to the stored file
 * @ctx   : handed through to peek
 * @record: record number, 0..126
 * @offset: returns the offset of the record in the stored file
 * @length: returns the length of the record in bytes
 *
 * The position follows from the RECORD block (and the V2 high block): every
 * record before it takes its sector count times 254 bytes. Returns 0 on
 * success. Otherwise the error channel is set and non-zero returned: 64 if
 * the file is no VLIR CVT, 50 if the record does not exist (00/FF, behind
 * the end of the list or a number above 126).
 */
uint8_t cvt_record(cvt_peek_t peek, void *ctx, uint8_t record,
                   uint32_t *offset, uint32_t *length);

/**
 * cvt_make_header - make a file that is not a CVT look like one
 *
 * Builds the signature block for dent in the data area of buf (data[0]
 * and data[1] are not touched) and sets position/lastused/sendeoi, so
 * buf can be delivered as the first block of the stream. The file
 * data follows as the next block. The block describes a non-GEOS file,
 * so no INFO block follows, the track and sector references are 0 (see the
 * addenda in doc/CVT.TXT). For a REL file (type of dent) recordlen is the
 * record length, which goes into the entry, the records are the data. It is
 * 0 for all other files. This is what makes it possible to look at any file.
 */
void cvt_make_header(const cbmdirent_t *dent, buffer_t *buf, uint8_t recordlen);

#endif

#endif
