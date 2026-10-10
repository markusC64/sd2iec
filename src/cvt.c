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


   cvt.c: GEOS CVT container layer

   A CVT is the sequential form of a GEOS file (see doc/CVT.TXT):

     signature block (254 bytes) = directory entry (30 bytes, without the
                                   two unused link bytes) + signature
     info block      (254 bytes)   only if the file has a GEOS info block
     record block    (254 bytes)   only for VLIR files (two in GeoConvert98 V2)
     data

   Any file can formally be seen as a CVT: Put a signature block in front
   of it. Files without GEOS info block have no info block in the CVT.

   A REL file is a CVT too (addendum 2): The entry has the type REL and the
   record length where a GEOS file has its structure ($15), the side sector
   references are 0 because the side sectors hold nothing but sector numbers
   and are rebuilt. The records follow the signature block.

   Because the signature block is the directory entry, opening any file with
   OPENFLAG_CVT also shows what an unknown file is.

   Both signatures are accepted: The standard one and the one of GeoConvert98
   (V2). V2 only changes the RECORD block of a VLIR, which is never taken
   apart here, so it makes no difference for sequential files. A CVT without
   INFO block (addendum 2) is recognized by the INFO block track in the entry.

   Because all blocks are 254 bytes, the header is a sequence of whole
   buffer blocks. The layer only provides the format-dependent parts, the
   backend decides what is delivered (see cvt.h).

*/

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "dirent.h"
#include "errormsg.h"
#include "fileops.h"
#include "flags.h"
#include "progmem.h"
#include "cvt.h"

#ifdef CONFIG_CVT

#define CVT_BLOCK_SIZE    254
#define CVT_ENTRY_SIZE    0x1e   /* $00-$1d: directory entry              */
#define CVT_SIG_OFFSET    0x1e   /* $1e-$39: "PRG formatted GEOS file..." */
#define CVT_SIG_SIZE      28

/* offsets inside the signature block */
#define CVT_OFS_TYPE      0x00   /* CBM file type                         */
#define CVT_OFS_NAME      CVT_NAME_OFFSET  /* 16 characters, $a0-padded   */
#define CVT_OFS_STRUCT    0x15   /* GEOS structure: 0 = SEQ, 1 = VLIR     */
                                 /* (REL: the record length, check the type first) */
#define CVT_OFS_INFOTRK   0x13   /* INFO block track, 0 = no INFO block   */
#define CVT_OFS_DATE      0x17   /* year (mod 100), month, day, hour, min */
#define CVT_OFS_BLOCKS    0x1c   /* size in blocks of the original, LE    */

/* "PRG" can also be "SEQ" and the last digit may differ, the rest of the */
/* text is what identifies a CVT: Standard (written) or GeoConvert98 V2.   */
static const PROGMEM char sig_text[] = "PRG formatted GEOS file V1.0";
static const PROGMEM char sig_tail2[] = " GeoConvert98-format V2.";
#define SIG_TAIL_OFFSET   3
#define SIG_TAIL_LENGTH   24

/* A real INFO block always starts like this */
static const PROGMEM uint8_t info_start[] = { 0x03, 0x15, 0xbf };

/* The name field: 16 characters, padded with $a0 */
static void put_name(uint8_t *field, const uint8_t *name) {
  uint8_t i;

  memset(field, 0xa0, CBM_NAME_LENGTH);
  for (i = 0; i < CBM_NAME_LENGTH && name[i]; i++)
    field[i] = name[i];
}

/* Read the directory entry of a CVT into ops_scratch, NULL if it isn't one */
static uint8_t *get_entry(cvt_peek_t peek, void *ctx) {
  uint8_t *e = ops_scratch;

  /* the signature goes first, the entry overwrites it */
  if (peek(ctx, CVT_SIG_OFFSET, e, CVT_SIG_SIZE) ||
      (memcmp_P(e + SIG_TAIL_OFFSET, sig_text + SIG_TAIL_OFFSET, SIG_TAIL_LENGTH) &&
       memcmp_P(e + SIG_TAIL_OFFSET, sig_tail2, SIG_TAIL_LENGTH)) ||
      peek(ctx, 0, e, CVT_ENTRY_SIZE))
    return NULL;

  return e;
}

uint8_t cvt_probe(cvt_peek_t peek, void *ctx, cbmdirent_t *dent) {
  uint8_t *e = get_entry(peek, ctx);
  uint8_t type, len;
  uint16_t blocks;

  if (e == NULL)
    return 1;

  /* only SEQ, PRG, USR and REL */
  type = e[CVT_OFS_TYPE] & TYPE_MASK;
  if ((uint8_t)(type - TYPE_SEQ) > TYPE_REL - TYPE_SEQ)
    return 1;

  dent->typeflags = (dent->typeflags & ~TYPE_MASK) | type;

  /* The size of the original file as the entry says, for the listing only.  */
  /* By the specification the only block counts that count are those in the */
  /* VLIR table, they set the record boundaries in the data. All others, and */
  /* this one in particular, are as good or bad as on any disk. 0: unknown.  */
  blocks = e[CVT_OFS_BLOCKS] | (e[CVT_OFS_BLOCKS + 1] << 8);
  if (blocks)
    dent->blocksize = blocks;
  dent->remainder = 0xff;

  /* name without the $a0 padding, unless the name of the file is wanted */
  if (!(globalflags & CVT_FSNAME)) {
    e += CVT_OFS_NAME;
    for (len = CBM_NAME_LENGTH; len && e[len-1] == 0xa0; len--) ;
    memcpy(dent->name, e, len);
    /* Everything after the name must be 0, the rest of the old FAT name is */
    /* still in there and createentry copies all 16 characters.            */
    memset(dent->name + len, 0, sizeof(dent->name) - len);
  }

  return 0;
}

uint8_t cvt_set_name(cvt_poke_t poke, void *ctx, const uint8_t *name) {
  uint8_t field[CBM_NAME_LENGTH];

  put_name(field, name);
  return poke(ctx, CVT_OFS_NAME, field, CBM_NAME_LENGTH);
}

uint8_t cvt_set_type(cvt_peek_t peek, cvt_poke_t poke, void *ctx, uint8_t type) {
  uint8_t b;

  if (peek(ctx, CVT_OFS_TYPE, &b, 1))
    return 1;

  if ((b & TYPE_MASK) == TYPE_REL) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return 2;
  }

  b = (b & ~TYPE_MASK) | type;
  return poke(ctx, CVT_OFS_TYPE, &b, 1);
}

/* Size of signature block and INFO block (if there is one), e is the entry */
static uint16_t header_size(cvt_peek_t peek, void *ctx, uint8_t *e) {
  uint8_t info;

  /* The entry says if there is an INFO block. Older writers left the track */
  /* 0 although there was one, it is recognized by how it starts.           */
  info = e[CVT_OFS_INFOTRK];
  if (!info)
    info = !peek(ctx, CVT_BLOCK_SIZE, e, sizeof(info_start)) &&
           !memcmp_P(e, info_start, sizeof(info_start));

  /* signature block, plus INFO block if there is one */
  return info ? 2 * CVT_BLOCK_SIZE : CVT_BLOCK_SIZE;
}

uint16_t cvt_data_offset(cvt_peek_t peek, void *ctx) {
  uint8_t *e = get_entry(peek, ctx);

  /* VLIR and REL are as unreadable as a file that is not a CVT */
  if (e == NULL || e[CVT_OFS_STRUCT]) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return 0;
  }

  return header_size(peek, ctx, e);
}

uint8_t cvt_record(cvt_peek_t peek, void *ctx, uint8_t record,
                   uint32_t *offset, uint32_t *length) {
  uint8_t *e = get_entry(peek, ctx);
  uint8_t  v2, i, idx, entry[2];
  uint16_t rec, count;
  uint32_t ofs;

  /* only a GEOS VLIR file: SEQ..USR (REL has the record length there) */
  if (e == NULL || e[CVT_OFS_STRUCT] != 1 ||
      (uint8_t)((e[CVT_OFS_TYPE] & TYPE_MASK) - TYPE_SEQ) > TYPE_USR - TYPE_SEQ) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return 1;
  }

  /* get_entry accepted one of both signatures, one byte tells them apart: */
  /* "PRG formatted ..." or "PRG GeoConvert98-format V2.0"                 */
  if (peek(ctx, CVT_SIG_OFFSET + SIG_TAIL_OFFSET + 1, entry, 1))
    goto missing;
  v2 = (entry[0] == 'G');

  /* RECORD block, V2 high block, data */
  rec = header_size(peek, ctx, e);
  ofs = rec + (v2 ? 2 * CVT_BLOCK_SIZE : CVT_BLOCK_SIZE);

  if (record >= CVT_BLOCK_SIZE / 2)
    goto missing;

  for (i = 0; ; i++) {
    if (peek(ctx, rec + 2 * i, entry, 2))
      goto missing;
    count = entry[0];
    idx   = entry[1];
    if (v2) {
      if (peek(ctx, rec + CVT_BLOCK_SIZE + 2 * i, entry, 1))
        goto missing;
      count |= entry[0] << 8;
    }

    if (!count && !idx)
      goto missing;                       /* 00/00: no more records */
    if (i == record)
      break;
    ofs += (uint32_t)count * CVT_BLOCK_SIZE;
  }

  /* 00/FF: the record does not exist, index < 2: no valid record */
  if (!count || idx < 2)
    goto missing;

  *offset = ofs;
  *length = (uint32_t)(count - 1) * CVT_BLOCK_SIZE + idx - 1;
  return 0;

 missing:
  set_error(ERROR_RECORD_MISSING);
  return 1;
}

void cvt_make_header(const cbmdirent_t *dent, buffer_t *buf, uint8_t recordlen) {
  uint8_t *p = buf->data + 2;
  uint8_t i;

  memset(p, 0, CVT_BLOCK_SIZE);

  p[CVT_OFS_TYPE] = 0x80 | (dent->typeflags & (TYPE_MASK | FLAG_RO));

  put_name(p + CVT_OFS_NAME, dent->name);

  /* GEOS file type and INFO block stay 0: not a GEOS file. The structure   */
  /* byte is 0 (SEQ) or, for a REL file, the record length.                 */
  p[CVT_OFS_STRUCT] = recordlen;

  i = dent->date.year;               /* since 1900, the CVT has two digits */
  p[CVT_OFS_DATE + 0] = i >= 100 ? i - 100 : i;
  p[CVT_OFS_DATE + 1] = dent->date.month;
  p[CVT_OFS_DATE + 2] = dent->date.day;
  p[CVT_OFS_DATE + 3] = dent->date.hour;
  p[CVT_OFS_DATE + 4] = dent->date.minute;

  p[CVT_OFS_BLOCKS]     = dent->blocksize & 0xff;
  p[CVT_OFS_BLOCKS + 1] = dent->blocksize >> 8;

  memcpy_P(p + CVT_SIG_OFFSET, sig_text, CVT_SIG_SIZE);

  buf->position = 2;
  buf->lastused = 255;
  buf->sendeoi  = 0;
}

#endif /* CONFIG_CVT */
