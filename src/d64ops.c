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


   d64ops.c: D64 operations

*/

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "dirent.h"
#include "errormsg.h"
#include "fatops.h"
#include "ff.h"
#include "parser.h"
#include "progmem.h"
#include "rtc.h"
#include "ustring.h"
#include "wrapops.h"
#include "d64ops.h"
#include "flags.h"
#include "cvt.h"

#define D41_SIZE_MIN      D41_SIZE
/* Max 7 additional tracks with 17 256 byte sectors each + 802 error bytes */
#define D41_SIZE_MAX      (D41_SIZE_MIN + 7*17*256 + 802)
#define D41_ERROR_OFFSET 174848
#define D71_ERROR_OFFSET 349696
#define D81_ERROR_OFFSET 819200
#define D80_ERROR_OFFSET 533248
#define D82_ERROR_OFFSET 1066496

#define D41_BAM_TRACK           18
#define D41_BAM_SECTOR          0
#define D41_BAM_BYTES_PER_TRACK 4
#define D41_BAM_BITFIELD_BYTES  3

#define D81_BAM_TRACK           40
#define D81_BAM_SECTOR1         1
#define D81_BAM_SECTOR2         2
#define D81_BAM_OFFSET          10
#define D81_BAM_BYTES_PER_TRACK 6
#define D81_BAM_BITFIELD_BYTES  5

#define D71_BAM2_TRACK  53
#define D71_BAM2_SECTOR 0
#define D71_BAM2_BYTES_PER_TRACK 3
#define D71_BAM_COUNTER2OFFSET   0xdd

#define DNP_BAM_TRACK                     1
#define DNP_BAM_SECTOR                    2
#define DNP_BAM_LAST_TRACK_OFS            8
#define DNP_ROOTDIR_SECTOR               34 // only for formatting!
#define DNP_BAM_BYTES_PER_TRACK          32
#define DNP_DIRHEADER_DIR_TRACK           0
#define DNP_DIRHEADER_DIR_SECTOR          1
#define DNP_DIRHEADER_ROOTHDR_TRACK      32
#define DNP_DIRHEADER_ROOTHDR_SECTOR     33
#define DNP_DIRHEADER_PARENTHDR_TRACK    34
#define DNP_DIRHEADER_PARENTHDR_SECTOR   35
#define DNP_DIRHEADER_PARENTENTRY_TRACK  36
#define DNP_DIRHEADER_PARENTENTRY_SECTOR 37
#define DNP_DIRHEADER_PARENTENTRY_OFFSET 38
#define DNP_LABEL_OFFSET                  4
#define DNP_LABEL_AREA_SIZE             (28-4+1)
#define DNP_ID_OFFSET                    22

#define D80_BAM_TRACK           38
#define D80_DIR_TRACK           39
#define D80_BAM_SECTOR          0
#define D80_BAM_INTERLEAVE      3
#define D80_BAM_BYTES_PER_TRACK 5
#define D80_BAM_BITFIELD_BYTES  4

/* used for error info only */
#define MAX_SECTORS_PER_TRACK 40

typedef enum { BAM_BITFIELD, BAM_FREECOUNT } bamdata_t;

struct {
  uint8_t part;
  uint8_t track;
  uint8_t mustflush;
  uint8_t errors[MAX_SECTORS_PER_TRACK];
} errorcache = { 255, 0, 0, {0} };

d64_lastread_t d64_lastread;  // last read track/sector

static buffer_t *bam_buffer;  // recently-used buffer
static buffer_t *bam_buffer2; // secondary buffer
static uint8_t   bam_refcount;

/* ------------------------------------------------------------------------- */
/*  Forward declarations                                                     */
/* ------------------------------------------------------------------------- */

static uint8_t errorcache_read(uint8_t part, uint8_t track);
static uint8_t errorcache_commit(void);

static uint8_t d64_opendir(dh_t *dh, path_t *path);

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_d71_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_d81_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
#ifdef CONFIG_D80D82
static void format_d80_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
static void format_d82_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);
#endif
static void format_dnp_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);

/* ------------------------------------------------------------------------- */
/*  Utility functions                                                        */
/* ------------------------------------------------------------------------- */

static const PROGMEM struct param_s d41param = {
  18, 1, 35, 35, 0x90, 0xa2, 10, 3, D41_SIZE_MIN, format_d41_image
};

static const PROGMEM struct param_s d71param = {
  18, 1, 70, 70, 0x90, 0xa2, 6, 3,  D71_SIZE,     format_d71_image
};

static const PROGMEM struct param_s d81param = {
  40, 3, 80, 80, 0x04, 0x16, 1, 1,  D81_SIZE,     format_d81_image
};

static const PROGMEM struct param_s dnpparam = {
  1, 1, 0, 0, DNP_LABEL_OFFSET, DNP_ID_OFFSET, 1, 1, 0, format_dnp_image
};

#ifdef CONFIG_D80D82
static const PROGMEM struct param_s d80param = {
  39, 1, 77, 77, 6, 0x18, 5, 3, 533248, format_d80_image
};

static const PROGMEM struct param_s d82param = {
  39, 1, 154, 154, 6, 0x18, 5, 3,  1066496, format_d82_image
};
#endif


/**
 * get_param - get a format-dependent parameter
 * @part : partition number
 * @param: parameter to retrieve
 *
 * Returns a parameter that is format-dependent for the image type mounted
 * on the specified partition.
 */
static uint8_t get_param(uint8_t part, param_t param) {
  return *(((uint8_t *)&partition[part].d64data)+param);
}

/**
 * sector_lba - Transform a track/sector pair into a LBA sector number
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an LBA-style sector number for a given track/sector pair.
 */
/* This version used the least code of all tested variants. */
static uint16_t sector_lba(uint8_t part, uint8_t track, const uint8_t sector) {
  uint16_t offset = 0;

  track--; /* Track numbers are 1-based */

  switch (partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D71:
  default:
    if (track >= 35) {
      offset = 683;
      track -= 35;
    }
    /* falls through */
  case D64_TYPE_D41:
    if (track < 17)
      return track*21 + sector + offset;
    if (track < 24)
      return 17*21 + (track-17)*19 + sector + offset;
    if (track < 30)
      return 17*21 + 7*19 + (track-24)*18 + sector + offset;
    return 17*21 + 7*19 + 6*18 + (track-30)*17 + sector + offset;

  case D64_TYPE_D81:
    return track*40 + sector;

  case D64_TYPE_DNP:
    return track*256 + sector;
#ifdef CONFIG_D80D82
  case D64_TYPE_D80:
  case D64_TYPE_D82:
    if (track >= 78) {
      offset = 2083;
      track -= 77;
    }
    if (track < 39)
      return track*29 + sector + offset;
    if (track < 53)
      return 39*29 + (track-39)*27 + sector + offset;
    if (track < 64)
      return 39*29 + 14*27 + (track-53)*25 + sector + offset;
    return 39*29 + 14*27 + 11*25 + (track-64)*23 + sector + offset;
#endif
  }
}

/**
 * sector_offset - Transform a track/sector pair into a D64 offset
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an offset into a D64 file from a track and sector number.
 */
static uint32_t sector_offset(uint8_t part, uint8_t track, const uint8_t sector) {
  return 256L * sector_lba(part,track,sector);
}

/**
 * d64_sectors_per_track - number of sectors on given track
 * @part : partition number
 * @track: Track number
 *
 * This function returns the number of sectors on the given track
 * of a 1541/71/81 disk. Invalid track numbers will return invalid results.
 */
uint16_t d64_sectors_per_track(uint8_t part, uint8_t track) {
  switch (partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D71:
  default:
    if (track > 35)
      track -= 35;
    /* falls through */
  case D64_TYPE_D41:
    if (track < 18)
      return 21;
    if (track < 25)
      return 19;
    if (track < 31)
      return 18;
    return 17;

  case D64_TYPE_D81:
    return 40;

  case D64_TYPE_DNP:
    return 256;

#ifdef CONFIG_D80D82
  case D64_TYPE_D80:
  case D64_TYPE_D82:
    if (track > 77)
       track -= 77;
    if (track < 40)
       return 29;
    if (track < 54)
       return 27;
    if (track < 65)
       return 25;
    return 23;
#endif
  }
}

/**
 * checked_read - read a specified sector after range-checking
 * @part  : partition number
 * @track : track number to be read
 * @sector: sector number to be read
 * @buf   : pointer to where the data should be read to
 * @len   : number of bytes to be read
 * @error : error number to be flagged if the range check fails
 *
 * This function checks if the track and sector are within the
 * limits for the image format and calls image_read to read
 * the data if they are. Returns the result of image_read or
 * 2 if the range check failed.
 */
static uint8_t checked_read(uint8_t part, uint8_t track, uint8_t sector, uint8_t *buf, uint16_t len, uint8_t error) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= d64_sectors_per_track(part, track)) {
    set_error_ts(error,track,sector);
    return 2;
  }

  /* Check if the sector is marked as bad */
  if (!errorcache_read(part, track)) {
    /* Calculate error message from the code */
    if (errorcache.errors[sector] >= 2 && errorcache.errors[sector] <= 11) {
      /* Most codes can be mapped directly */
      set_error_ts(errorcache.errors[sector]-2+20,track,sector);
      return 2;
    }
    if (errorcache.errors[sector] == 15) {
      /* Drive not ready */
      set_error(74);
      return 2;
    }
    /* 1 is OK, unknown values are accepted too */
  }

  return image_read(part, sector_offset(part,track,sector), buf, len);
}

/**
 * update_timestamp - update timestamp of a directory entry
 * @buffer: pointer to the directory entry
 *
 * This function updates the time stamp of a directory entry
 * at the address pointed to by @buffer with the current
 * date/time if available or the default timestamp if not.
 */
static void update_timestamp(uint8_t *buffer) {
#ifdef HAVE_RTC
  struct tm time;

  read_rtc(&time);
  buffer[DIR_OFS_YEAR]   = time.tm_year % 100;
  buffer[DIR_OFS_MONTH]  = time.tm_mon + 1;
  buffer[DIR_OFS_DAY]    = time.tm_mday;
  buffer[DIR_OFS_HOUR]   = time.tm_hour;
  buffer[DIR_OFS_MINUTE] = time.tm_min;
#else
  buffer[DIR_OFS_YEAR]   = 82;
  buffer[DIR_OFS_MONTH]  = 8;
  buffer[DIR_OFS_DAY]    = 31;
  buffer[DIR_OFS_HOUR]   = 0;
  buffer[DIR_OFS_MINUTE] = 0;
#endif
}

/**
 * write_entry - write a single directory entry
 * @part : partition number
 * @dh   : pointer to d64dh pointing to the entry
 * @buf  : pointer to the buffer where the data should be read from
 * @flush: if true, data is flushed to disk immediately
 *
 * This function writes a single directory entry specified by @dh
 * from the buffer @buf.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_write
 */
static uint8_t write_entry(uint8_t part, struct d64dh *dh, uint8_t *buf, uint8_t flush) {
  return image_write(part, sector_offset(part, dh->track, dh->sector) +
                           dh->entry * 32, buf, 32, flush);
}

/**
 * read_entry - read a single directory entry
 * @part: partition number
 * @dh  : pointer to d64dh pointing to the entry
 * @buf : pointer to the buffer where the data should be stored
 *
 * This function reads a single directory entry specified by dh
 * into the buffer buf which needs to be at least 32 bytes big.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_read (0 success, 1 partial read, 2 failed)
 */
static uint8_t read_entry(uint8_t part, struct d64dh *dh, uint8_t *buf) {
  return image_read(part, sector_offset(part, dh->track, dh->sector) +
                          dh->entry * 32, buf, 32);
}

/**
 * strnsubst - substitute one character with another in a buffer
 * @buffer : pointer to the buffer
 * @len    : length of buffer
 * @oldchar: character to be replaced
 * @newchar: character to be used as a replacement
 *
 * This functions changes all occurences of oldchar in the first len
 * byte of buffer with newchar. Although it is named str*, it doesn't
 * care about zero bytes in any way.
 */
static void strnsubst(uint8_t *buffer, uint8_t len, uint8_t oldchar, uint8_t newchar) {
  uint8_t i=len-1;

  do {
    if (buffer[i] == oldchar)
      buffer[i] = newchar;
  } while (i--);
}

/* fill a sector with 0 0xff 0* to generate a new empty directory sector */
/* needs a 256 byte work area in *data */
static uint8_t clear_dir_sector(uint8_t part, uint8_t t, uint8_t s, uint8_t *data) {
  memset(data, 0, 256);

  data[1] = 0xff;
  return image_write(part, sector_offset(part, t, s), data, 256, 0);
}


/* ------------------------------------------------------------------------- */
/*  BAM buffer handling                                                      */
/* ------------------------------------------------------------------------- */

/**
 * bam_buffer_flush - write BAM buffer to disk
 * @buf: pointer to the BAM buffer
 *
 * This function writes the contents of the BAM buffer to the disk image.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t bam_buffer_flush(buffer_t *buf) {
  uint8_t res;

  if (buf->mustflush && buf->pvt.bam.part < max_part) {
    res = image_write(buf->pvt.bam.part,
                      sector_offset(buf->pvt.bam.part,
                                    buf->pvt.bam.track,
                                    buf->pvt.bam.sector),
                      buf->data, 256, 1);
    buf->mustflush = 0;

    return res;
  } else
    return 0;
}

/**
 * bam_buffer_alloc - allocates a buffer for the BAM
 * @buf: pointer to the BAM buffer pointer
 *
 * This function tries to allocate a buffer for holding the BAM,
 * using the pointer pointed to by @buf. Returns 0 if successful,
 * != 0 otherwise.
 */
static uint8_t bam_buffer_alloc(buffer_t **buf) {
  *buf = alloc_system_buffer();
  if (!*buf)
    return 1;

  (*buf)->secondary    = BUFFER_SYS_BAM;
  (*buf)->pvt.bam.part = 255;
  (*buf)->cleanup      = bam_buffer_flush;
  stick_buffer(*buf);

  return 0;
}

/**
 * bam_buffer_swap - swap the two BAM buffers
 *
 * Thus function swaps the pointers to the two BAM buffers
 */
static void bam_buffer_swap(void) {
  buffer_t *tmp = bam_buffer;
  bam_buffer    = bam_buffer2;
  bam_buffer2   = tmp;
}

/**
 * bam_buffer_match - checks if a BAM buffer contains a specific sector
 * @buf   : BAM buffer to check
 * @part  : partition
 * @track : track
 * @sector: sector
 *
 * This function checks if the BAM buffer @buf currently holds the data
 * for the given @part, @track and @sector. Returns 1 if it does,
 * 0 otherwise.
 */
static uint8_t bam_buffer_match(buffer_t *buf, uint8_t part,
                                uint8_t track, uint8_t sector) {
  if (buf->pvt.bam.part   == part  &&
      buf->pvt.bam.track  == track &&
      buf->pvt.bam.sector == sector)
    return 1;
  else
    return 0;
}

/**
 * move_bam_window - read correct BAM sector into window.
 * @part  : partition
 * @track : track number
 * @type  : type of pointer requested
 * @ptr   : pointer to track information in BAM sector
 *
 * This function reads the correct BAM sector into memory for the requested
 * track, flushing an existing BAM sector to disk if needed.  It also
 * calculates the correct pointer into the BAM sector for the appropriate
 * track.  Since the BAM contains both sector counts and sector allocation
 * bitmaps, type is used to signal which reference is desired.
 * This function may swap the BAM buffer pointers, after it returns
 * bam_buffer is always the buffer with the requested sector.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t move_bam_window(uint8_t part, uint8_t track, bamdata_t type, uint8_t **ptr) {
  uint8_t res;
  uint8_t t,s, pos;

  switch(partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D41:
  default:
    t   = D41_BAM_TRACK;
    s   = D41_BAM_SECTOR;
    pos = D41_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ? 1:0);
    break;

  case D64_TYPE_D71:
    if (track > 35 && type == BAM_BITFIELD) {
      t   = D71_BAM2_TRACK;
      s   = D71_BAM2_SECTOR;
      pos = (track - 36) * D71_BAM2_BYTES_PER_TRACK;
    } else {
      t = D41_BAM_TRACK;
      s = D41_BAM_SECTOR;
      if (track > 35) {
        pos = (track - 36) + D71_BAM_COUNTER2OFFSET;
      } else {
        pos = D41_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ? 1:0);
      }
    }
    break;

  case D64_TYPE_D81:
    t   = D81_BAM_TRACK;
    s   = (track < 41 ? D81_BAM_SECTOR1 : D81_BAM_SECTOR2);
    if (track > 40)
      track -= 40;
    pos = D81_BAM_OFFSET + track * D81_BAM_BYTES_PER_TRACK + (type == BAM_BITFIELD ? 1:0);
    break;

  case D64_TYPE_DNP:
    t   = DNP_BAM_TRACK;
    s   = DNP_BAM_SECTOR + (track >> 3);
    pos = (track & 0x07) * 32;
    break;

#ifdef CONFIG_D80D82
  case D64_TYPE_D80:
  case D64_TYPE_D82:
    t    = D80_BAM_TRACK;
    s    = D80_BAM_SECTOR;
    while (track > 50) {
      s += D80_BAM_INTERLEAVE;
      track -= 50;
    }
    pos = 1 + D80_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ?  1:0);
    break;
#endif
  }

  if (!bam_buffer_match(bam_buffer, part, t, s)) {
    /* check if the second BAM buffer exists */
    if (bam_buffer2) {
      /* unconditionally swap to the other BAM buffer        */
      /* either it has the data or it was less-recently used */

      /* Note for future optimization:
       * If buf1 is invalid and buf2 valid but not matching this will swap and
       * use buf2 for the data instead of buf1 - using buf1
       * while keeping buf2 without swapping would be better.
       */
      bam_buffer_swap();

      /* check again, now in the other buffer */
      if (bam_buffer_match(bam_buffer, part, t, s))
        goto found;
    } else {
      /* allocate and swap to the second BAM buffer if the first one is valid */
      if (bam_buffer->pvt.bam.part != 255) {
        if (bam_buffer_alloc(&bam_buffer2)) {
          /* allocation failed, reset error and continue with just one buffer */
          set_error(ERROR_OK);
        } else {
          /* switch to the new buffer */
          bam_buffer_swap();
        }
      }
    }

    /* Need to read the BAM sector - flush only the target buffer */
    if (bam_buffer->cleanup(bam_buffer))
      return 1;

    res = image_read(part, sector_offset(part, t, s), bam_buffer->data, 256);
    if(res)
      return res;

    bam_buffer->pvt.bam.part   = part;
    bam_buffer->pvt.bam.track  = t;
    bam_buffer->pvt.bam.sector = s;
  }

 found:
  *ptr = bam_buffer->data + pos;
  return 0;
}

/**
 * is_free - checks if the given sector is marked as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function checks if the given sector is marked as free in
 * the BAM of drive "part". Returns 0 if allocated, >0 if free, <0 on error.
 */
static int8_t is_free(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t res;
  uint8_t *ptr = NULL;

  res = move_bam_window(part,track,BAM_BITFIELD,&ptr);
  if(res)
    return -1;

  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
    return (ptr[sector>>3] & (0x80>>(sector&7))) != 0;
  else
    return (ptr[sector>>3] & (1<<(sector&7))) != 0;
}

/**
 * sectors_free - returns the number of free sectors on a given track
 * @part  : partition
 * @track : track number
 *
 * This function returns the number of free sectors on the given track
 * of partition part.
 */
static uint16_t sectors_free(uint8_t part, uint8_t track) {
  uint8_t *trackmap = NULL;

  if (track < 1 || track > get_param(part, LAST_BAM_TRACK))
    return 0;

  switch (partition[part].imagetype & D64_TYPE_MASK) {

  case D64_TYPE_DNP:
    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 0;

    uint16_t blocks = 0;
    for (uint8_t i=0;i < DNP_BAM_BYTES_PER_TRACK;i++) {
      // From http://everything2.com/title/counting%25201%2520bits
      uint8_t b = (trackmap[i] & 0x55) + (trackmap[i]>>1 & 0x55);
      b = (b & 0x33) + (b >> 2 & 0x33);
      b = (b & 0x0f) + (b >> 4 & 0x0f);
      blocks += b;
    }
    return blocks;

  case D64_TYPE_D71:
  case D64_TYPE_D81:
  case D64_TYPE_D41:
#ifdef CONFIG_D80D82
  case D64_TYPE_D80:
  case D64_TYPE_D82:
#endif
  default:
    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 0;
    return *trackmap;
  }
}

/**
 * allocate_sector - mark a sector as used
 * @part  : partitoin
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as used in the BAM of drive part.
 * If the sector was already marked as used nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t allocate_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res != 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
      /* For some reason DNP has its bitfield reversed */
      trackmap[sector>>3] &= (uint8_t)~(0x80>>(sector&7));

      /* DNP has no counter in its BAM */
      return 0;
    }

    trackmap[sector>>3] &= (uint8_t)~(1<<(sector&7));

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if (trackmap[0] > 0) {
      trackmap[0]--;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * free_sector - mark a sector as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as free in the BAM of partition
 * part. If the sector was already marked as free nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t free_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res == 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
      /* For some reason DNP has its bitfield reversed */
      trackmap[sector>>3] |= 0x80>>(sector&7);

      /* DNP has no counter in its BAM */
      return 0;
    }

    trackmap[sector>>3] |= 1<<(sector&7);

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if(trackmap[0] < d64_sectors_per_track(part, track)) {
      trackmap[0]++;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * get_first_sector - calculate the first sector for a new file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the first sector to be allocated for a new
 * file. The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 *
 * This code will not skip track 53 of a D71 if there are any free
 * sectors on it - this behaviour is consistent with that of a 1571
 * with a revision 3.0 ROM.
 */
static uint8_t get_first_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  int8_t distance = 1;

  /* DNP uses a simple "first free" allocation scheme, starting at track 2.  */
  /* It is not known if this is the same algorithm as used in the original   */
  /* CMD drives, but track 1 seems to be semi-reserved for directory sectors.*/
  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
    *track = 2;
    while (sectors_free(part, *track) == 0) {
      (*track)++;

      if (*track == get_param(part, LAST_BAM_TRACK) ||
          *track == 0) {
        /* Wrap to track 1 */
        *track = 1;
      }

      if (*track == 2)
        /* If we're at track 2 again there are no free sectors anywhere */
        return 1;
    }
  } else {
    /* Look for a track with free sectors close to the directory */
    while (distance < get_param(part, LAST_BAM_TRACK)) {
      if (sectors_free(part, get_param(part, DIR_TRACK)-distance))
        break;

      /* Invert sign */
      distance = -distance;

      /* Increase distance every second try */
      if (distance > 0)
        distance++;
    }

    if (distance == get_param(part, LAST_BAM_TRACK)) {
      if (current_error == ERROR_OK)
        set_error(ERROR_DISK_FULL);
      return 1;
    }

    *track = get_param(part, DIR_TRACK)-distance;
  }

  /* Search for the first free sector on this track */
  for (*sector = 0;*sector < d64_sectors_per_track(part, *track); *sector += 1)
    if (is_free(part, *track, *sector) > 0)
      return 0;

  /* If we're here the BAM is invalid or couldn't be read */
  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * get_next_sector - calculate the next sector for a file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the next sector to be allocated for a file
 * based on the current sector in the variables pointed to by track/sector.
 * The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 */
static uint8_t get_next_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
    uint8_t newtrack = *track;

    /* Find a track with free sectors */
    while (sectors_free(part, newtrack) == 0) {
      newtrack++;

      if (newtrack == get_param(part, LAST_BAM_TRACK) ||
          newtrack == 0) {
        /* Wrap to track 1 */
        newtrack = 1;
      }

      if (newtrack == *track)
        /* Returned to the start track -> no free sectors anywhere */
        return 1;
    }

    uint8_t newsector;

    if (newtrack == *track) {
      /* Same track: start at the previous sector */
      newsector = *sector;
    } else {
      /* New track: start at sector 0 */
      newsector = 0;
    }

    while (!is_free(part, newtrack, newsector))
      newsector++; // will automatically wrap from 255->0

    *track = newtrack;
    *sector = newsector;

    return 0;
  }

  uint8_t interleave,tries;

  if (*track == get_param(part, DIR_TRACK)) {
    if (sectors_free(part, get_param(part, DIR_TRACK)) == 0) {
      if (current_error == ERROR_OK)
        set_error(ERROR_DISK_FULL);
      return 1;
    }
    interleave = get_param(part, DIR_INTERLEAVE);
  } else
    interleave = get_param(part, FILE_INTERLEAVE);

  /* Look for a track with free sectors */
  tries = 0;
  while (tries < 3 && !sectors_free(part, *track)) {
    /* No more space on current track, try another */
    if (*track < get_param(part, DIR_TRACK))
      *track -= 1;
    else {
      *track += 1;
      /* Skip track 53 on D71 images */
      if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_D71 &&
          *track == D71_BAM2_TRACK)
        *track += 1;
    }

    if (*track < 1) {
      *track = get_param(part, DIR_TRACK) + 1;
      *sector = 0;
      tries++;
    }
    if (*track > get_param(part, LAST_BAM_TRACK)) {
      *track = get_param(part, DIR_TRACK) - 1;
      *sector = 0;
      tries++;
    }
  }

  if (tries == 3) {
    if (current_error == ERROR_OK)
      set_error(ERROR_DISK_FULL);
    return 1;
  }

  /* Look for a sector at interleave distance */
  *sector += interleave;
  if (*sector >= d64_sectors_per_track(part, *track)) {
    *sector -= d64_sectors_per_track(part, *track);
    if (*sector != 0)
      *sector -= 1;
  }

  /* Increase distance until an empty sector is found */
  tries = 99;
  while (is_free(part,*track,*sector) <= 0 && tries--) {
    *sector += 1;
    if (*sector >= d64_sectors_per_track(part, *track))
      *sector = 0;
  }

  if (tries)
    return 0;

  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * nextdirentry - read the next dir entry to ops_scratch
 * @dh: directory handle
 *
 * This function reads the next directory entry from the disk
 * into ops_scratch. Returns 1 if an error occured, -1 if there
 * are no more directory entries and 0 if successful. This
 * function will return every entry, even deleted ones.
 */
static int8_t nextdirentry(dh_t *dh) {
  /* End of directory entries in this sector? */
  if (dh->dir.d64.entry == 8) {
    /* Read link pointer */
    if (checked_read(dh->part, dh->dir.d64.track, dh->dir.d64.sector, ops_scratch, 2, ERROR_ILLEGAL_TS_LINK))
      return 1;

    /* Final directory sector? */
    if (ops_scratch[0] == 0 || ops_scratch[1] == 0)
      return -1;

    dh->dir.d64.track  = ops_scratch[0];
    dh->dir.d64.sector = ops_scratch[1];
    dh->dir.d64.entry  = 0;
  }

  if (dh->dir.d64.track < 1 || dh->dir.d64.track > get_param(dh->part, LAST_TRACK) ||
      dh->dir.d64.sector >= d64_sectors_per_track(dh->part, dh->dir.d64.track)) {
    set_error_ts(ERROR_ILLEGAL_TS_LINK,dh->dir.d64.track,dh->dir.d64.sector);
    return 1;
  }

  if (read_entry(dh->part, &dh->dir.d64, ops_scratch))
    return 1;

  dh->dir.d64.entry++;

  return 0;
}

/**
 * find_empty_entry - find an empty directory entry
 * @path: path of the directory
 * @dh: directory handle
 *
 * This funtion finds an empty entry in the directory, creating
 * a new directory sector if required.
 * Returns 0 if successful or 1 if not.
 */
static uint8_t find_empty_entry(path_t *path, dh_t *dh) {
  int8_t res;

  d64_opendir(dh, path);
  do {
    res = nextdirentry(dh);
    if (res > 0)
      return 1;
  } while (res == 0 && ops_scratch[DIR_OFS_FILE_TYPE] != 0);

  /* Allocate a new directory sector if no empty entries were found */
  if (res < 0) {
    uint8_t t,s;

    t = dh->dir.d64.track;
    s = dh->dir.d64.sector;

    if (get_next_sector(path->part, &dh->dir.d64.track, &dh->dir.d64.sector))
      return 1;

    /* Link the old sector to the new */
    ops_scratch[0] = dh->dir.d64.track;
    ops_scratch[1] = dh->dir.d64.sector;
    if (image_write(path->part, sector_offset(path->part,t,s), ops_scratch, 2, 0))
      return 1;

    if (allocate_sector(path->part, dh->dir.d64.track, dh->dir.d64.sector))
      return 1;

    /* DNP only: Increment the block count in the parent directory */
    if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
      if (image_read(path->part,
                     DNP_DIRHEADER_PARENTENTRY_TRACK + sector_offset(path->part,
                                                                     path->dir.dxx.track,
                                                                     path->dir.dxx.sector),
                     ops_scratch, 3))
        return 1;

      if (ops_scratch[0] != 0) {
        /* Read block count of entry in parent directory */
        if (image_read(path->part,
                       sector_offset(path->part, ops_scratch[0], ops_scratch[1]) + ops_scratch[2] + DIR_OFS_SIZE_LOW - 2,
                       ops_scratch + 3, 2))
          return 1;

        uint16_t *blocks = (uint16_t *)(ops_scratch + 3);
        (*blocks)++;

        /* Write new block count */
        if (image_write(path->part,
                        sector_offset(path->part, ops_scratch[0], ops_scratch[1]) + ops_scratch[2] + DIR_OFS_SIZE_LOW - 2,
                        ops_scratch + 3, 2, 1))
          return 1;
      }
    }

    /* Clear the new directory sector */
    memset(ops_scratch, 0, 32);
    ops_scratch[1] = 0xff;
    for (uint8_t i=0;i<256/32;i++) {
      dh->dir.d64.entry = i;
      if (write_entry(path->part, &dh->dir.d64, ops_scratch, 0))
        return 1;

      ops_scratch[1] = 0;
    }

    /* Mark full sector as used */
    ops_scratch[1] = 0xff;
    dh->dir.d64.entry = 0;

  } else {
    /* nextdirentry has already incremented this variable, undo it */
    dh->dir.d64.entry--;
  }

  return 0;
}

/**
 * d64_read - refill-callback used for reading
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for reading.
 */
static uint8_t d64_read(buffer_t *buf) {
  /* Store the current sector, used for append */
  buf->pvt.d64.track  = buf->data[0];
  buf->pvt.d64.sector = buf->data[1];

  if (checked_read(buf->pvt.d64.part, buf->data[0], buf->data[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK)) {
    free_buffer(buf);
    return 1;
  }

  buf->position = 2;

  if (buf->data[0] == 0) {
    /* Final sector of the file */
    buf->lastused = buf->data[1];
    buf->sendeoi  = 1;
  } else {
    buf->lastused = 255;
    buf->sendeoi  = 0;
  }

  return 0;
}

/**
 * d64_seek - seek-callback
 * @buf     : target buffer
 * @position: offset to seek to
 * @index   : offset within the record to seek to
 *
 * This is the function used as the seek callback. Since seeking
 * isn't supported for D64 files it just sets an error message
 * and returns 1.
 */
static uint8_t d64_seek(buffer_t *buf, uint32_t position, uint8_t index) {
  (void)buf;
  (void)position;
  (void)index;

  set_error(ERROR_SYNTAX_UNABLE);
  return 1;
}

/**
 * d64_write - refill-callback used for writing
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for writing.
 */
static uint8_t d64_write(buffer_t *buf) {
  uint8_t t, s, savederror;

  savederror = 0;
  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;

  buf->pvt.d64.blocks++;

  /* Mark as last sector in case something below fails */
  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  /* Find another free sector */
  if (get_next_sector(buf->pvt.d64.part, &t, &s)) {
    t = 0;
    savederror = current_error;
    goto storedata;
  }

  buf->data[0] = t;
  buf->data[1] = s;

  /* Allocate it */
  if (allocate_sector(buf->pvt.d64.part, t, s)) {
    free_buffer(buf);
    return 1;
  }

 storedata:
  /* Store data in the already-reserved sector */
  if (image_write(buf->pvt.d64.part,
                  sector_offset(buf->pvt.d64.part,
                                buf->pvt.d64.track,
                                buf->pvt.d64.sector),
                  buf->data, 256, 1)) {
    free_buffer(buf);
    return 1;
  }

  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
  buf->position  = 2;
  buf->lastused  = 1;
  buf->mustflush = 0;
  mark_buffer_clean(buf);

  if (savederror) {
    set_error(savederror);
    free_buffer(buf);
    return 1;
  } else
    return 0;
}

/**
 * d64_write_finish - write the last sector and close the directory entry
 * @buf: buffer of a file opened for writing
 *
 * The part of the cleanup that finishes the file, the buffer is not freed.
 * Returns 0 on success, 1 on error.
 */
static uint8_t d64_write_finish(buffer_t *buf) {
  uint8_t t,s;

  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;
  buf->pvt.d64.blocks++;

  /* Track=0 means that there was an error somewhere earlier */
  if (t == 0)
    return 1;

  /* Store data */
  if (image_write(buf->pvt.d64.part, sector_offset(buf->pvt.d64.part,t,s), buf->data, 256, 1))
    return 1;

  /* Update directory entry */
  if (read_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch))
    return 1;

  ops_scratch[DIR_OFS_FILE_TYPE] |= FLAG_SPLAT;
  ops_scratch[DIR_OFS_SIZE_LOW]   = buf->pvt.d64.blocks & 0xff;
  ops_scratch[DIR_OFS_SIZE_HI]    = buf->pvt.d64.blocks >> 8;
  update_timestamp(ops_scratch);

  if (write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1))
    return 1;

  return 0;
}

static uint8_t d64_write_cleanup(buffer_t *buf) {
  if (d64_write_finish(buf))
    return 1;

  buf->cleanup = callback_dummy;
  free_buffer(buf);

  return 0;
}


/* ------------------------------------------------------------------------- */
/*  error cache handling                                                     */
/* ------------------------------------------------------------------------- */

/**
 * errorcache_read - read error info for specified track
 * @part : partition number
 * @track: track number for which error info should be fetched
 *
 * This function updates the error cache with the error info
 * for the given track. It will first flush the current error
 * cache back to disk if it is valid and was modified.
 *
 * Returns 0 on success, 1 in case of an error.
 */

static uint8_t errorcache_read(uint8_t part, uint8_t track) {
  uint32_t pos;

  if ((partition[part].imagetype & D64_HAS_ERRORINFO) == 0)
    return 1;

  if (errorcache.part == part && errorcache.track == track)
    return 0;

  /* write back the current error cache if it was modified */
  if (errorcache_commit())
    return 1;

  /* Read the error info for this track */
  memset(errorcache.errors, 1, sizeof(errorcache.errors));
  errorcache.part = 255;

  pos = partition[part].d64data.error_offset + sector_lba(part,track,0);

  switch (partition[part].imagetype & D64_TYPE_MASK) {
  case D64_TYPE_D41:
  case D64_TYPE_D71:
  case D64_TYPE_D81:
#ifdef CONFIG_D80D82
  case D64_TYPE_D80:
  case D64_TYPE_D82:
#endif
    if (image_read(part, pos, errorcache.errors, d64_sectors_per_track(part, track)) >= 2)
      return 1;
    break;

  default:
    /* Should not happen unless someone enables error info
        for additional image formats */
    return 1;

  }

  errorcache.part  = part;
  errorcache.track = track;

  return 0;
}

/**
 * errorcache_commit - write error cache to disk, if modified
 *
 * This function writes the error cache back to the image file
 * if it is valid and was modified.
 *
 * Returns 0 on success, 1 if there was an error updating
 * the image file.
 */
static uint8_t errorcache_commit(void) {
  uint8_t  part;
  uint8_t  track;
  uint32_t pos;

  if (errorcache.mustflush == 0 || errorcache.part == 255)
    return 0;

  part = errorcache.part;
  track = errorcache.track;
  pos = partition[part].d64data.error_offset + sector_lba(part,track,0);

  if (image_write(part, pos, errorcache.errors, d64_sectors_per_track(part, track), 1))
    return 1;

  errorcache.mustflush = 0;

  return 0;
}


/* ------------------------------------------------------------------------- */
/*  fileops-API                                                              */
/* ------------------------------------------------------------------------- */

uint8_t d64_mount(path_t *path, uint8_t *name) {
  uint8_t imagetype;
  uint8_t part = path->part;
  uint32_t fsize = partition[part].imagehandle.fsize;
  uint8_t trk;
  uint8_t sec;

  switch (fsize) {
  case D71_SIZE:
    imagetype = D64_TYPE_D71;
    memcpy_P(&partition[part].d64data, &d71param, sizeof(struct param_s));
    trk = 18; sec = 0;
    break;

  case D71_SIZE + D71_SIZE/256:
    imagetype = D64_TYPE_D71 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d71param, sizeof(struct param_s));
    trk = 18; sec = 0;
    break;

  case D81_SIZE:
    imagetype = D64_TYPE_D81;
    memcpy_P(&partition[part].d64data, &d81param, sizeof(struct param_s));
    trk = 40; sec = 0;
    break;

  case D81_SIZE + D81_SIZE/256:
    imagetype = D64_TYPE_D81 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d81param, sizeof(struct param_s));
    trk = 18; sec = 0;
    break;

#ifdef CONFIG_D80D82
  case 533248:
    imagetype = D64_TYPE_D80;
    memcpy_P(&partition[part].d64data, &d80param, sizeof(struct param_s));
    trk = 38; sec = 0;
    break;

  case 535331:
    imagetype = D64_TYPE_D80 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d80param, sizeof(struct param_s));
    trk = 38; sec = 0;
    break;

  case 1066496:
    imagetype = D64_TYPE_D82;
    memcpy_P(&partition[part].d64data, &d82param, sizeof(struct param_s));
    trk = 38; sec = 0;
    break;

  case 1070662:
    imagetype = D64_TYPE_D82 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d82param, sizeof(struct param_s));
    trk = 38; sec = 0;
    break;
#endif

  default:
    /* check for D41 image (with possibly more than 35 tracks) */
    if (fsize >= D41_SIZE_MIN && fsize <= D41_SIZE_MAX) {
      /* Make sure we don't confuse a 3*256*256 DNP image for a 40 track D41 */
      uint8_t *ptr = fsize != 196608 ? NULL : ustrrchr(name, '.');

      if (ptr == NULL || ptr[2] != 'N') {
        uint16_t extra = fsize - D41_SIZE_MIN;

        /*
         * check if the total image size could be valid for either
         * - an image without error info (256 byte per sector), or
         * - an image with 1 additional byte error info per sector (256+1 bytes
         *   per sector).
         *
         * In both cases the additional data must also be a multiple of
         * 17 sectors (the expected sector count for tracks > 35).
         */
        if (fsize % 256 == 0 && (extra/256) % 17 == 0) {
          imagetype = D64_TYPE_D41;
          extra /= 256; /* extra bytes => extra sectors */
        } else if (fsize % (256+1) == 0 && ((extra-683)/(256+1)) % 17 == 0) {
          imagetype = D64_TYPE_D41 | D64_HAS_ERRORINFO;
          extra = (extra - D41_SIZE_MIN/256) / 257; /* bytes => sectors */
        } else {
          imagetype = 0;
        }

        if (imagetype != 0) { /* found a valid D41 image */
          memcpy_P(&partition[part].d64data, &d41param, sizeof(struct param_s));
          partition[part].d64data.last_track += extra / 17;
          partition[part].d64data.error_offset += 256 * extra;
          trk = 18; sec = 0;
          break;
        }
      }
    }

    /* Not recognized as neither D41, D71 nor D81 image; last try is DNP */
    if ((fsize == 0 || fsize % (256*256L)) != 0 || fsize / (256*256L) > 255) {
      set_error(ERROR_IMAGE_INVALID);
      return 1;
    }

    imagetype = D64_TYPE_DNP;
    memcpy_P(&partition[part].d64data, &dnpparam, sizeof(struct param_s));
    partition[part].d64data.last_track = fsize / (256*256L);
    partition[part].d64data.last_bam_track = partition[part].d64data.last_track;
    partition[part].d64data.error_offset = fsize;
    trk = 1; sec = 1;
  }

  /* allocate the first BAM buffer if required */
  if (bam_buffer == NULL) {
    if (bam_buffer_alloc(&bam_buffer))
      return 1;
  }

  partition[part].imagetype = imagetype;
  path->dir.dxx.track  = get_param(part, DIR_TRACK);
  path->dir.dxx.sector = get_param(part, DIR_START_SECTOR);

  bam_refcount++;

  if (imagetype & D64_HAS_ERRORINFO)
    /* Invalidate error cache */
    errorcache.part = 255;
    
  uint8_t tmpByte;
  if (image_read(path->part, sector_offset(path->part, trk, sec)+2, &tmpByte, 1))
  {
      return 0;
  }
  if (tmpByte && tmpByte < 0x40)
      partition[part].imagetype |= D64_IS_READLNLY;

  return 0;
}

static uint8_t d64_opendir(dh_t *dh, path_t *path) {
  dh->part = path->part;
  dh->dir.d64.track  = path->dir.dxx.track;
  dh->dir.d64.sector = path->dir.dxx.sector;
  dh->dir.d64.entry  = 0;
  dh->dir.d64.hidden = 0;

  if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
    /* Read the real first directory sector from the header sector */
    uint8_t tmp[2];
    if (image_read(path->part,
                   sector_offset(path->part, dh->dir.d64.track, dh->dir.d64.sector),
                   tmp, 2))
      return 1;

    dh->dir.d64.track  = tmp[0];
    dh->dir.d64.sector = tmp[1];
  }
  return 0;
}

static int8_t d64_readdir(dh_t *dh, cbmdirent_t *dent) {
  int8_t res;

  do {
    res = nextdirentry(dh);
    if (res)
      return res;

    if (ops_scratch[DIR_OFS_FILE_TYPE] != 0)
      break;

    /* Treat file type 0 as hidden file if requested by flag and track != 0 */
    if (ops_scratch[DIR_OFS_TRACK] != 0 && dh->dir.d64.hidden)
      break;
  } while (1);

  memset(dent, 0, sizeof(cbmdirent_t));

  dent->opstype = OPSTYPE_DXX;
  dent->typeflags = (ops_scratch[DIR_OFS_FILE_TYPE] ^ FLAG_SPLAT);
  if (!(globalflags & D64_WITH_HIDDEN))
    dent->typeflags &= ~FLAG_HIDDEN;

  if ((dent->typeflags & EXT_TYPE_MASK) > TYPE_DIR)
    /* Mark invalid file types (will be displayed as "???") */
    /* FIXME: Should exclude DIR on non-DNP */
    dent->typeflags = (dent->typeflags & (uint8_t)~EXT_TYPE_MASK) | TYPE_INV;

  dent->pvt.dxx.dh = dh->dir.d64;
  dent->pvt.dxx.dh.entry -= 1; /* undo increment in nextdirentry */
  dent->blocksize = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI];
  dent->remainder = 0xff;
  memcpy(dent->name, ops_scratch + DIR_OFS_FILE_NAME, CBM_NAME_LENGTH);
  strnsubst(dent->name, 16, 0xa0, 0);

  /* sanitize values in case they were not stored */
  dent->date.minute=  ops_scratch[DIR_OFS_MINUTE] % 60;
  dent->date.hour  =  ops_scratch[DIR_OFS_HOUR] % 24;
  dent->date.day   = (ops_scratch[DIR_OFS_DAY]   - 1) % 31 + 1;
  dent->date.month = (ops_scratch[DIR_OFS_MONTH] - 1) % 12 + 1;
  dent->date.year  =  ops_scratch[DIR_OFS_YEAR] % 100;
  /* adjust year into 1980..2079 range */
  if (dent->date.year < 80)
    dent->date.year += 100;

  return 0;
}

/* Reads and converts a string from the dir header sector (BAM for D41/D71) to the buffer */
/* Used by d64_get(disk|dir)label and d64_getid */
static uint8_t read_string_from_dirheader(path_t *path, uint8_t *buffer, param_t what, uint8_t size) {
  uint8_t sector;

  if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
    sector = path->dir.dxx.sector;
  else
    sector = 0;

  if (image_read(path->part,
                 sector_offset(path->part, path->dir.dxx.track, sector)
                 + get_param(path->part, what),
                 buffer, size))
    return 1;

  strnsubst(buffer, size, 0xa0, 0x20);
  return 0;
}

static uint8_t d64_getdirlabel(path_t *path, uint8_t *label) {
  return read_string_from_dirheader(path, label, LABEL_OFFSET, 16);
}

static uint8_t d64_getdisklabel(uint8_t part, uint8_t *label) {
  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP) {
    /* Read directly from root dir header */
    if (image_read(part, 256 + DNP_LABEL_OFFSET, label, 16))
      return 1;
  } else {
    /* Use getdirlabel instead */
    path_t curpath;

    curpath.part = part;
    curpath.dir = partition[part].current_dir;
    if (d64_getdirlabel(&curpath, label))
      return 1;
  }

  /* Zero-terminate label */
  uint8_t *ptr = label+16;
  *ptr-- = 0;
  while (ptr != label && *ptr == ' ')
    *ptr-- = 0;

  return 0;
}

static uint8_t d64_getid(path_t *path, uint8_t *id) {
  return read_string_from_dirheader(path, id, ID_OFFSET, 5);
}

static uint16_t d64_freeblocks(uint8_t part) {
  uint16_t blocks = 0;
  uint8_t i;

  for (i = 1; i != 0 && i <= get_param(part, LAST_BAM_TRACK); i++) {
    /* Skip directory track */
    switch (partition[part].imagetype & D64_TYPE_MASK) {
    case D64_TYPE_D81:
      if (i == D81_BAM_TRACK)
        continue;
      break;

    case D64_TYPE_DNP:
      /* DNP reserves a partial track, handled below */
      break;

    case D64_TYPE_D41:
    case D64_TYPE_D71:
    default:
      if (i == D41_BAM_TRACK || i == D71_BAM2_TRACK)
        continue; // continue the for loop
      break;      // break out of the switch

#ifdef CONFIG_D80D82
    case D64_TYPE_D80:
    case D64_TYPE_D82:
      if (i == D80_DIR_TRACK)
        continue; // continue the for loop
      break;      // break out of the switch
#endif
    }

    if ((partition[part].imagetype & D64_TYPE_MASK)
        == D64_TYPE_DNP && i == 1) {
      /* DNP: ignore sectors 0-63 on track 1 */
      for (uint16_t j = 64; j < 256; j++) {
        if (is_free(part, 1, j) > 0)
          blocks++;
      }

    } else {
      blocks += sectors_free(part,i);
    }
  }

  return blocks;
}

#ifdef CONFIG_CVT
/* ------------------------------------------------------------------------- */
/*  CVT stream of a file in an image                                         */
/* ------------------------------------------------------------------------- */

/*
 * With OPENFLAG_CVT a file is delivered as if it were stored as a CVT
 * (see doc/CVT.TXT). Nothing is written, the stream is put together block
 * by block from the image while it is read:
 *
 *   signature block  directory entry + signature, built by open
 *   INFO block       the INFO sector, if the GEOS file has one
 *   RECORD block     VLIR only: the RECORD sector, each T/S replaced by the
 *                    sector count and last-sector index of its chain
 *   RECORD high      VLIR only, GeoConvert98 V2: high bytes of the counts
 *   data             SEQ: the chain; VLIR: all record chains in order, each
 *                    one but the last padded with 0 to full 254 bytes
 *
 * All blocks are 254 bytes, so each one is exactly one buffer, and the data
 * is read sector by sector like a normal file: data[0]/data[1] hold the next
 * sector as usual. At the end of a record the next record is looked up in
 * the RECORD sector and its first sector is put into data[0]/data[1], so the
 * stream goes on as if it were one long chain.
 *
 * The RECORD block needs the length of every record before the data, and
 * the signature needs to know if V2 is required (a record with more than
 * 255 sectors). There is no RAM to keep the counts, so the chains are walked
 * at open (V2 check, this also finds broken chains before anything is sent)
 * and again for the RECORD block, a third time for the V2 high block. Only
 * the link bytes are read for that.
 *
 * Files that are no GEOS files (incl. REL) get the same signature block as
 * on FAT (cvt_make_header), followed by their data chain. The side sectors
 * of a REL file are not part of the stream, the record length is in the
 * entry.
 */

/* GEOS part of a D64 directory entry */
#define DIR_OFS_GEOS_INFO_TRACK  0x15
#define DIR_OFS_GEOS_INFO_SECTOR 0x16
#define DIR_OFS_GEOS_STRUCT      0x17   /* REL: the record length */
#define DIR_OFS_GEOS_TYPE        0x18

/* pvt.d64cvt.state: which block comes next, plus flags */
#define CVT_NEXT_INFO     0
#define CVT_NEXT_RECLOW   1
#define CVT_NEXT_RECHIGH  2
#define CVT_NEXT_DATA     3
#define CVT_NEXT_MASK     3
#define CVT_FLAG_VLIR     (1<<6)
#define CVT_FLAG_V2       (1<<7)

/* in the signature block: the entry and the signature text */
#define CVT_ENTRY_LENGTH  30            /* entry without the two link bytes */
#define CVT_OFS_DATA_TS   0x01          /* data / RECORD block T/S          */
#define CVT_OFS_INFO_TS   (DIR_OFS_GEOS_INFO_TRACK - 2)
#define CVT_OFS_V2SIG     0x22          /* "PRG " + this + ".0"             */
#define CVT_OFS_STRUCT    (DIR_OFS_GEOS_STRUCT - 2)

static const PROGMEM char cvt_v2sig[] = "GeoConvert98-format V2";

/**
 * cvt_chain_length - count the sectors of a chain
 * @part: partition
 * @ts  : first sector of the chain, returns the link bytes of the last one
 *
 * Returns the number of sectors in the chain, ts[1] is then the
 * last-sector index of the CVT (bytes used + 1, link byte of the last
 * sector). Returns 0 if a sector can't be read (error channel is set).
 */
static uint16_t cvt_chain_length(uint8_t part, uint8_t *ts) {
  uint16_t count = 0;

  do {
    if (checked_read(part, ts[0], ts[1], ts, 2, ERROR_ILLEGAL_TS_LINK))
      return 0;
    if (++count == 0) {
      /* a chain longer than any image: it runs in a circle */
      set_error(ERROR_ILLEGAL_TS_LINK);
      return 0;
    }
  } while (ts[0]);

  return count;
}

/**
 * cvt_record_block - build the RECORD block of the CVT in buf
 * @buf : buffer, pvt.d64cvt is set up
 * @high: 0 for the normal (low) block, 1 for the V2 high block
 *
 * Reads the RECORD sector of the VLIR file and replaces every T/S in it by
 * the sector count and last-sector index of the record (low block) or by
 * the high byte of the count and 0 (high block). 00/FF (no record) is kept
 * in the low block, everything from 00/00 (end) on becomes 0. The low block
 * sets CVT_FLAG_V2 if a record has more than 255 sectors.
 * Returns 0 on success, non-zero on error.
 */
static uint8_t cvt_record_block(buffer_t *buf, uint8_t high) {
  uint8_t part = buf->pvt.d64cvt.part;
  uint8_t *p, ts[2];
  uint16_t count;

  if (checked_read(part, buf->pvt.d64cvt.track, buf->pvt.d64cvt.sector,
                   buf->data, 256, ERROR_ILLEGAL_TS_LINK))
    return 1;

  for (p = buf->data + 2; p < buf->data + 256; p += 2) {
    if (!p[0]) {
      if (!p[1]) {
        /* 00/00: no more records */
        memset(p, 0, buf->data + 256 - p);
        break;
      }
      /* 00/FF: the record does not exist, the count is 0 */
      if (high)
        p[1] = 0;
      continue;
    }

    ts[0] = p[0];
    ts[1] = p[1];
    count = cvt_chain_length(part, ts);
    if (!count)
      return 1;

    if (high) {
      p[0] = count >> 8;
      p[1] = 0;
    } else {
      p[0] = count;
      p[1] = ts[1];
      if (count > 255)
        buf->pvt.d64cvt.state |= CVT_FLAG_V2;
    }
  }

  buf->position = 2;
  buf->lastused = 255;
  buf->sendeoi  = 0;
  return 0;
}

/**
 * cvt_next_record - find the next record of a VLIR file
 * @buf: buffer, pvt.d64cvt is set up
 *
 * Looks for the next existing record in the RECORD sector, starting at
 * entry pvt.d64cvt.record, and puts its first sector into data[0]/data[1].
 * If there is none, data[0] is 0.
 */
static void cvt_next_record(buffer_t *buf) {
  uint8_t part = buf->pvt.d64cvt.part;
  uint8_t *ts = buf->data;

  while (buf->pvt.d64cvt.record < 127) {
    /* the RECORD sector was checked when it was read for the RECORD block */
    if (image_read(part, sector_offset(part, buf->pvt.d64cvt.track, buf->pvt.d64cvt.sector) +
                         2 + 2 * buf->pvt.d64cvt.record++, ts, 2))
      break;
    if (ts[0])
      return;         /* found */
    if (!ts[1])
      break;          /* 00/00: no more records, 00/FF: skip */
  }

  ts[0] = 0;
}

/**
 * d64_cvt_read - refill-callback for a file read as CVT stream
 * @buf: target buffer
 */
static uint8_t d64_cvt_read(buffer_t *buf) {
  uint8_t state = buf->pvt.d64cvt.state;
  uint8_t next  = state & CVT_NEXT_MASK;

  if (next == CVT_NEXT_RECLOW || next == CVT_NEXT_RECHIGH) {
    if (cvt_record_block(buf, next == CVT_NEXT_RECHIGH))
      goto fail;

    if (next == CVT_NEXT_RECLOW && (state & CVT_FLAG_V2)) {
      state = state - CVT_NEXT_RECLOW + CVT_NEXT_RECHIGH;
    } else {
      /* the data follows: the first record, if there is one */
      state = (state & ~CVT_NEXT_MASK) | CVT_NEXT_DATA;
      cvt_next_record(buf);
      buf->sendeoi = !buf->data[0];
    }

  } else {
    /* INFO block or data: one sector, data[0]/data[1] say which */
    if (checked_read(buf->pvt.d64cvt.part, buf->data[0], buf->data[1],
                     buf->data, 256, ERROR_ILLEGAL_TS_LINK))
      goto fail;

    buf->position = 2;
    buf->lastused = 255;
    buf->sendeoi  = 0;

    if (next == CVT_NEXT_INFO) {
      if (state & CVT_FLAG_VLIR) {
        state = (state & ~CVT_NEXT_MASK) | CVT_NEXT_RECLOW;
      } else {
        /* sequential: the data chain follows (checked by open) */
        state = (state & ~CVT_NEXT_MASK) | CVT_NEXT_DATA;
        buf->data[0] = buf->pvt.d64cvt.track;
        buf->data[1] = buf->pvt.d64cvt.sector;
      }

    } else if (buf->data[0] == 0) {
      /* last sector of a chain */
      uint8_t used = buf->data[1];

      buf->lastused = used;
      if (state & CVT_FLAG_VLIR) {
        /* All records but the last are padded to full blocks. Whether */
        /* this is the last one shows when looking for the next.       */
        memset(buf->data + used + 1, 0, 255 - used);
        cvt_next_record(buf);
        if (buf->data[0])
          buf->lastused = 255;
      }
      buf->sendeoi = !buf->data[0];
    }
  }

  buf->pvt.d64cvt.state = state;
  return 0;

 fail:
  free_buffer(buf);
  return 1;
}

/**
 * d64_open_cvt - set up buf to read a file as CVT stream
 * @part: partition
 * @dent: directory entry of the file
 * @buf : buffer to be used
 *
 * The directory entry of the file must be in ops_scratch. Builds the
 * signature block as first block of the stream, the rest is delivered by
 * d64_cvt_read. All sectors needed for the header are checked here, so an
 * error shows up at open, not in the middle of the stream.
 */
/* GEOS file: DEL..USR (REL and the others use the bytes differently), */
/* structure SEQ or VLIR, and structure and GEOS type not both 0.      */
static uint8_t cvt_is_geos(const uint8_t *e) {
  uint8_t struc = e[DIR_OFS_GEOS_STRUCT];

  return (e[DIR_OFS_FILE_TYPE] & TYPE_MASK) <= TYPE_USR && struc <= 1 &&
         (struc | e[DIR_OFS_GEOS_TYPE]);
}

/**
 * cvt_geos_signature - build the signature block of a GEOS file
 * @dent: directory entry of the file
 * @buf : buffer, the block is built in its data area
 * @e   : raw directory entry of the file (32 bytes)
 * @v2  : non-zero if the GeoConvert98 V2 signature is needed
 *
 * The signature text comes from the CVT layer, the entry is the raw one of
 * the image. The T/S references mean nothing in a CVT, they only say
 * whether there is an INFO block (addendum 2).
 */
static void cvt_geos_signature(cbmdirent_t *dent, buffer_t *buf, const uint8_t *e, uint8_t v2) {
  uint8_t *p = buf->data + 2;
  uint8_t info = e[DIR_OFS_GEOS_INFO_TRACK];

  cvt_make_header(dent, buf, 0);
  memcpy(p, e + 2, CVT_ENTRY_LENGTH);
  p[CVT_OFS_DATA_TS]     = 0;
  p[CVT_OFS_DATA_TS + 1] = 0;
  p[CVT_OFS_INFO_TS]     = info ? 0x01 : 0;
  p[CVT_OFS_INFO_TS + 1] = info ? 0xff : 0;
  if (v2)
    memcpy_P(p + CVT_OFS_V2SIG, cvt_v2sig, sizeof(cvt_v2sig) - 1);
}

static void d64_open_cvt(uint8_t part, cbmdirent_t *dent, buffer_t *buf) {
  uint8_t *e = ops_scratch;
  uint8_t type   = e[DIR_OFS_FILE_TYPE] & TYPE_MASK;
  uint8_t struc  = e[DIR_OFS_GEOS_STRUCT];
  uint8_t info_t = e[DIR_OFS_GEOS_INFO_TRACK];
  uint8_t info_s = e[DIR_OFS_GEOS_INFO_SECTOR];
  uint8_t state;
  uint8_t ts[2];

  buf->pvt.d64cvt.part   = part;
  buf->pvt.d64cvt.record = 0;
  buf->pvt.d64cvt.track  = e[DIR_OFS_TRACK];
  buf->pvt.d64cvt.sector = e[DIR_OFS_SECTOR];

  /* The first data sector (SEQ) or RECORD block (VLIR) must be readable, */
  /* as the first sector of any file must be when it is opened.          */
  if (checked_read(part, e[DIR_OFS_TRACK], e[DIR_OFS_SECTOR], ts, 2, ERROR_ILLEGAL_TS_LINK))
    return;

  if (!cvt_is_geos(e)) {
    /* Not a GEOS file: same signature block as on FAT, the data follow. */
    /* The record length of a REL file goes into the entry.              */
    cvt_make_header(dent, buf, type == TYPE_REL ? struc : 0);
    state = CVT_NEXT_DATA;

  } else {
    state = CVT_NEXT_DATA;
    if (struc) {
      /* VLIR: walk all records once, for V2 and to find broken chains */
      state = CVT_FLAG_VLIR | CVT_NEXT_RECLOW;
      buf->pvt.d64cvt.state = state;
      if (cvt_record_block(buf, 0))
        return;
      state = buf->pvt.d64cvt.state;
    }

    if (info_t) {
      if (checked_read(part, info_t, info_s, ts, 2, ERROR_ILLEGAL_TS_LINK))
        return;
      state = (state & ~CVT_NEXT_MASK) | CVT_NEXT_INFO;
    }

    cvt_geos_signature(dent, buf, e, state & CVT_FLAG_V2);
  }

  /* next sector to read: the INFO block or the data */
  if ((state & CVT_NEXT_MASK) == CVT_NEXT_INFO) {
    buf->data[0] = info_t;
    buf->data[1] = info_s;
  } else {
    buf->data[0] = buf->pvt.d64cvt.track;
    buf->data[1] = buf->pvt.d64cvt.sector;
  }

  buf->pvt.d64cvt.state = state;
  buf->read   = 1;
  buf->refill = d64_cvt_read;
  buf->seek   = d64_seek;
  stick_buffer(buf);
}
#endif /* CONFIG_CVT */

/**
 * d64_open_read - opens a file for reading
 * @path : path of the file
 * @dent : directory entry of the file
 * @buf  : buffer to be used
 * @flags: OPENFLAG_* mask
 *
 * With OPENFLAG_CVT the file is delivered as a CVT stream, see above.
 */
static void d64_open_read(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t flags) {
  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

#ifdef CONFIG_CVT
  if (flags & OPENFLAG_CVT) {
    d64_open_cvt(path->part, dent, buf);
    return;
  }
#else
  (void)flags;
#endif

  buf->data[0] = ops_scratch[DIR_OFS_TRACK];
  buf->data[1] = ops_scratch[DIR_OFS_SECTOR];

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

#ifdef CONFIG_CVT
static uint8_t d64_cvt_write_cleanup(buffer_t *buf);
#endif

static void d64_open_write(path_t *path, cbmdirent_t *dent, uint8_t type, buffer_t *buf, uint8_t append) {
  dh_t dh;
  uint8_t *ptr;

#ifdef CONFIG_CVT
  uint8_t cvt = 0;

  /* A CVT is written as a normal PRG file, the close turns it into the */
  /* GEOS file (see d64_cvt_write_cleanup). Without a type, a file is a */
  /* PRG, so that is the natural one if it can't be converted.          */
  if (type == TYPE_CVT) {
    type = TYPE_PRG;
    cvt  = 1;
  }
#endif

  /* Check for read-only image file */
  if (!(partition[path->part].imagehandle.flag & FA_WRITE)) {
    set_error(ERROR_WRITE_PROTECT);
    return;
  }

  if (append) {
    /* Append case: Open the file and read the last sector */
    d64_open_read(path, dent, buf, 0);
    while (!current_error && buf->data[0])
      buf->refill(buf);

    if (current_error)
      return;

    /* Modify the buffer for writing */
    buf->pvt.d64.dh     = dent->pvt.dxx.dh;
    buf->pvt.d64.blocks = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI]-1;
    buf->read       = 0;
    buf->position   = buf->lastused+1;
    if (buf->position == 0)
      buf->mustflush = 1;
    else
      buf->mustflush = 0;
    buf->refill     = d64_write;
    buf->cleanup    = d64_write_cleanup;
    buf->seek       = d64_seek;
    mark_write_buffer(buf);

    update_timestamp(ops_scratch);
    write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1);

    return;
  }

  /* Non-append case */

  /* Search for an empty directory entry */
  if (find_empty_entry(path, &dh))
    return;

  /* Create directory entry in ops_scratch */
  uint8_t *name = dent->name;
  memset(ops_scratch + 2, 0, sizeof(ops_scratch) - 2);  /* Don't overwrite the link pointer! */
  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*name) *ptr++ = *name++;
  ops_scratch[DIR_OFS_FILE_TYPE] = type;

  /* Find a free sector and allocate it */
  uint8_t t,s;

  if (get_first_sector(path->part,&t,&s))
    return;

  ops_scratch[DIR_OFS_TRACK]  = t;
  ops_scratch[DIR_OFS_SECTOR] = s;

  if (allocate_sector(path->part,t,s))
    return;

  /* Write the directory entry */
  update_timestamp(ops_scratch);
  if (write_entry(path->part, &dh.dir.d64, ops_scratch, 1))
    return;

  /* Prepare the data buffer */
  mark_write_buffer(buf);
  buf->position       = 2;
  buf->lastused       = 2;
  buf->cleanup        = d64_write_cleanup;
  buf->refill         = d64_write;
  buf->seek           = d64_seek;
  buf->data[2]        = 13; /* Verified on VICE */
  buf->pvt.d64.dh     = dh.dir.d64;
  buf->pvt.d64.part   = path->part;
  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
#ifdef CONFIG_CVT
  if (cvt)
    buf->cleanup      = d64_cvt_write_cleanup;
#endif
}

static void d64_open_rel(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t length, uint8_t mode) {
  (void)path;
  (void)dent;
  (void)buf;
  (void)length;
  (void)mode;

  set_error(ERROR_SYNTAX_UNABLE);
}

static uint8_t d64_delete(path_t *path, cbmdirent_t *dent) {
  uint8_t linkbuf[2];

  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return 255;
    
  if (ops_scratch[DIR_OFS_FILE_TYPE] & FLAG_RO)
    return 0;

  /* Free the sector chain in the BAM */
  linkbuf[0] = ops_scratch[DIR_OFS_TRACK];
  linkbuf[1] = ops_scratch[DIR_OFS_SECTOR];

  do {
    free_sector(path->part, linkbuf[0], linkbuf[1]);

    if (checked_read(path->part, linkbuf[0], linkbuf[1], linkbuf, 2, ERROR_ILLEGAL_TS_LINK))
      return 255;
  } while (linkbuf[0]);

  /* Clear directory entry */
  ops_scratch[DIR_OFS_FILE_TYPE] = 0;
  if (write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1))
    return 255;

  return 1;
}

static void d64_read_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  d64_lastread.track = track;
  d64_lastread.sector = sector;
  checked_read(part, track, sector, buf->data, 256, ERROR_ILLEGAL_TS_COMMAND);
}

static void d64_write_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= d64_sectors_per_track(part, track)) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND,track,sector);
  } else
    image_write(part, sector_offset(part,track,sector), buf->data, 256, 1);
}

static void d64_rename(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  uint8_t *ptr;

  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*newname) *ptr++ = *newname++;

  write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1);
}


/**
 * d64_raw_directory - open directory track as file
 * @buf: target buffer
 *
 * This function opens the directory track as a file on the
 * buffer passed in buf. Used when $ is opened on a secondary
 * address > 0.
 */
void d64_raw_directory(path_t *path, buffer_t *buf) {
  /* Copy&Waste from d64_open_read */
  buf->data[0] = path->dir.dxx.track;
  if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
    buf->data[1] = path->dir.dxx.sector;
  else
    buf->data[1] = 0;

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

/**
 * d64_chdir - chdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be changed into, NULL to move one directory
 *           up (left arrow) - unmounts the image at the root
 *
 * Changes the directory in the path object for DNP files,
 * returns an error for everything else.
 * Returns 0 if successful, 1 otherwise.
 */
static uint8_t d64_chdir(path_t *path, cbmdirent_t *dirname) {
  if ((partition[path->part].imagetype & D64_TYPE_MASK) != D64_TYPE_DNP)
    return image_chdir(path,dirname);

  if (dirname == NULL) {
    /* Move up a directory, unmount if at the root */
    uint8_t parent[2];

    if (image_read(path->part,
                   sector_offset(path->part, path->dir.dxx.track, path->dir.dxx.sector) + DNP_DIRHEADER_PARENTHDR_TRACK,
                   parent, 2))
      return 1;

    if (parent[0] == 0)
      /* Already at the root directory */
      return image_unmount(path->part);

    path->dir.dxx.track  = parent[0];
    path->dir.dxx.sector = parent[1];
    return 0;
  }

  if (dirname->name[0] == 0) {
    /* Empty string: root directory */
    path->dir.dxx.track  = 1;
    path->dir.dxx.sector = 1;
    return 0;
  }

  if (read_entry(path->part, &dirname->pvt.dxx.dh, ops_scratch))
    return 1;

  path->dir.dxx.track  = ops_scratch[DIR_OFS_TRACK];
  path->dir.dxx.sector = ops_scratch[DIR_OFS_SECTOR];

  return 0;
}

/**
 * d64_mkdir - mkdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be created
 *
 * Creates a subdirectory for DNP files, returns an error
 * for everything else.
 */
static void d64_mkdir(path_t *path, uint8_t *dirname) {
  dh_t dh;
  buffer_t *buf;
  uint8_t h_track,h_sector,d_track,d_sector;
  uint8_t *ptr, *name;

  if ((partition[path->part].imagetype & D64_TYPE_MASK) != D64_TYPE_DNP) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  buf = alloc_buffer();
  if (buf == NULL)
    return;

  /* Find an empty directory entry */
  if (find_empty_entry(path, &dh))
    return;

  /* Find a free track/sector for the dir header */
  h_track  = path->dir.dxx.track;
  h_sector = path->dir.dxx.sector;

  if (get_next_sector(path->part, &h_track, &h_sector)) {
    set_error(ERROR_DISK_FULL);
    return;
  }

  /* Allocate it now so the next call to get_next doesn't return it again */
  if (allocate_sector(path->part, h_track, h_sector))
    return;

  /* Find a free track/sector for the first directory sector */
  d_track  = h_track;
  d_sector = h_sector;

  if (get_next_sector(path->part, &d_track, &d_sector)) {
    set_error(ERROR_DISK_FULL);
    free_sector(path->part, h_track, h_sector);
    return;
  }

  if (allocate_sector(path->part, d_track, d_sector))
    return;

  /* Build a directory header */
  memset(buf->data, 0, 256);
  memset(buf->data + DNP_LABEL_OFFSET, 0xa0, DNP_LABEL_AREA_SIZE);

  /* Unfortunately this generates much smaller code on AVR */
  ptr = buf->data;
  *ptr++ = d_track;
  *ptr++ = d_sector;
  *ptr++ = 'H';

  /* Note: The CMD FD manual says these bytes point to the root header, */
  /*       but all observed DNP files actually store the t/s of the     */
  /*       sector itself here.                                          */
  ptr = buf->data + DNP_DIRHEADER_ROOTHDR_TRACK;
  *ptr++ = h_track;
  *ptr++ = h_sector;

  *ptr++ = path->dir.dxx.track;
  *ptr++ = path->dir.dxx.sector;

  *ptr++ = dh.dir.d64.track;
  *ptr++ = dh.dir.d64.sector;
  *ptr++ = dh.dir.d64.entry * 32 + 2;

  /* Fill name and ID */
  ptr = buf->data + DNP_LABEL_OFFSET;
  name = dirname;
  while (*name) *ptr++ = *name++;

  ptr = buf->data + DNP_ID_OFFSET;
  *ptr++ = dirname[0];
  *ptr = dirname[1];
  if (!*ptr) *ptr=0xA0;
   ptr+=2;
  *ptr++ = '1';
  *ptr++ = 'H';

  /* Write directory header sector */
  if (image_write(path->part,
                  sector_offset(path->part, h_track, h_sector),
                  buf->data, 256, 0))
    return;

  /* Create empty directory sector */
  memset(buf->data, 0, 256);
  buf->data[1] = 0xff;
  if (image_write(path->part,
                  sector_offset(path->part, d_track, d_sector),
                  buf->data, 256, 0))
    return;

  /* Create the directory entry */
  /* FIXME: Isn't similiar code duplicated in open_write and rename? */
  memset(ops_scratch + 2, 0, sizeof(ops_scratch) - 2);
  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);

  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*dirname) *ptr++ = *dirname++;

  ops_scratch[DIR_OFS_FILE_TYPE] = TYPE_DIR | FLAG_SPLAT;
  ops_scratch[DIR_OFS_TRACK]     = h_track;
  ops_scratch[DIR_OFS_SECTOR]    = h_sector;
  ops_scratch[DIR_OFS_SIZE_LOW]  = 2;
  update_timestamp(ops_scratch);

  image_write(path->part, sector_offset(path->part, dh.dir.d64.track, dh.dir.d64.sector)
                          + dh.dir.d64.entry * 32 + 2, ops_scratch + 2, 30, 1);
}

/**
 * d64_commit - write BAM buffers and errorcache to disk
 *
 * This function is the exported interface to force the BAM buffer and the
 * errorcache contents to disk. Returns 0 if successful, != 0 otherwise.
 */
uint8_t d64_commit(void) {
  uint8_t res = 0;

  if (bam_buffer)
    res |= bam_buffer->cleanup(bam_buffer);

  if (bam_buffer2)
    res |= bam_buffer2->cleanup(bam_buffer2);

  res |= errorcache_commit();

  return res;
}

/**
 * d64_invalidate - invalidate internal state
 *
 * This function invalidates all cached state when
 * a card change is detected.
 */
void d64_invalidate(void) {
  free_buffer(bam_buffer);
  bam_buffer   = NULL;
  free_buffer(bam_buffer2);
  bam_buffer2  = NULL;
  bam_refcount = 0;

  errorcache.part = 255;
  errorcache.mustflush = 0;
}

/**
 * d64_unmount - unmount disk image
 * @part: partition number
 *
 * This function in called on image unmount and handles
 * refcounting for the BAM buffers.
 */
void d64_unmount(uint8_t part) {
  /* invalidate BAM buffers that point to the current partition */
  if (bam_buffer) {
    bam_buffer->cleanup(bam_buffer);
    if (bam_buffer->pvt.bam.part == part)
      bam_buffer->pvt.bam.part = 255;
  }

  if (bam_buffer2) {
    bam_buffer2->cleanup(bam_buffer2);
    if (bam_buffer2->pvt.bam.part == part)
      bam_buffer2->pvt.bam.part = 255;
  }

  /* decrease BAM buffer refcounter - it can never be zero while a Dxx is mounted*/
  if (--bam_refcount == 0) {
    free_buffer(bam_buffer);
    free_buffer(bam_buffer2);
    bam_buffer  = NULL;
    bam_buffer2 = NULL;
  }

  if (errorcache.part == part) {
    errorcache_commit();
    errorcache.part = 255;
  }
}

static uint8_t add_errorinfo(uint8_t part, buffer_t *buf) {
  int16_t  sectors;
  uint32_t pos;

  memset(buf->data, 1, 256);

  pos = partition[part].d64data.error_offset;
  sectors = pos / 256;

  while (sectors > 0) {
    if (image_write(part, pos, buf->data, sectors > 256 ? 256 : sectors, 0))
      return 1;

    pos += 256;
    sectors -= 256;
  }

  partition[part].imagetype |= D64_HAS_ERRORINFO;

  return 0;
}

/**
 * d64_set_error - set error code for sector
 * @part  : partition number
 * @track : track number
 * @sector: sector number
 * @error : error code
 *
 * Sets the error code for the specified sector in the error info block
 * of the image. Adds the error info block if necessary.
 *
 * Returns 0 on success and != 0 on error.
 *
 * | error code |         CBM DOS equivalent          |
 * |:----------:|:------------------------------------|
 * |      0     | invalid (!)                         |
 * |      1     | 00 (OK)                             |
 * |      2     | 20 (block header not found)         |
 * |      3     | 21 (no SYNC)                        |
 * |      4     | 22 (data block not found)           |
 * |      5     | 23 (data block checksum error)      |
 * |      6     | 24 (invalid GCR)                    |
 * |    7, 8    | 25, 26 (write errors)               |
 * |      9     | 27 (header checksum error)          |
 * |     10     | 28 (write error)                    |
 * |     11     | 29 (disc id mismatch)               |
 * |   12..14   | invalid                             |
 * |     15     | 74 (drive not ready)                |
 */
uint8_t d64_set_error(uint8_t part, uint8_t track, uint8_t sector, uint8_t error) {
  buffer_t *buf;

  // Refuse if no D64 image is mounted
  if (partition[part].fop != &d64ops)
    return 1;

  // DNP doesn't support error info
  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
    return 0;

  // Better be safe and check that track and sector are valid
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= d64_sectors_per_track(part, track)) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND, track, sector);
    return 1;
  }

  // Add error info if not already present
  if ((partition[part].imagetype & D64_HAS_ERRORINFO) == 0) {
    if (error == 1) // this is the default; nothing to do
      return 0;

    buf = alloc_buffer();
    if (!buf)
      return 1;

    if (add_errorinfo(part, buf))
      return 1;
  }

  if (errorcache_read(part, track))
    return 1;

  if (errorcache.errors[sector] != error) {
    errorcache.errors[sector] = error;
    errorcache.mustflush = 1;
  }

  return 0;
}

/**
 * d64_extend_image - extend a D41 image up to a maximum of 42 tracks
 * @part  : partition number
 * @tracks: number of tracks to extend to
 *
 * The image can't be extended if it already contains an error info block.
 * (The error info would need to be relocated, which is not supported).
 *
 * When extending, an error info block is created, though, and all newly
 * added sectors are marked with an NO SYNC error. When a track is
 * formatted, this error is cleared for all its sectors.
 */
uint8_t d64_extend_image(uint8_t part, uint8_t tracks) {
  buffer_t *buf;
  uint8_t  last = get_param(part, LAST_TRACK);

  if (last >= tracks) // nothing to do
    return 0;

  // Only continue if a D41 image without error info is mounted.
  if (partition[part].fop != &d64ops || partition[part].imagetype != D64_TYPE_D41) {
    set_error(ERROR_IMAGE_INVALID);
    return 1;
  }

  // No more than 42 tracks total
  if (tracks > 42) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND, tracks, 0);
    return 1;
  }

  buf = alloc_buffer();
  if (!buf)
    return 1;

  // add error info. This automatically extends the image to the requested size.
  partition[part].d64data.error_offset = sector_offset(part, tracks+1, 0);
  if (add_errorinfo(part, buf))
    return 1;

  // mark all added sectors as NOSYNC. This error will be cleared by formatting.
  memset(buf->data, ERROR_READ_NOSYNC - 20 + 2, 17*(tracks - last));
  if (image_write(part,
                  partition[part].d64data.error_offset + sector_lba(part,last+1,0),
                  buf->data, 17*(tracks - last), 1))
    return 1;

  partition[part].d64data.last_track = tracks;

  return 0;
}


/* ------------------------------------------------------------------------- */
/*  Formatting disk images                                                   */
/* ------------------------------------------------------------------------- */

/* Initializes sector contents and resets error info, if applicable. */
static uint8_t d64_format_track(uint8_t part, buffer_t *buf, uint8_t track) {
  uint16_t sector;

  for (sector = 0; sector < d64_sectors_per_track(part, track); sector++) {
    if (image_write(part, sector_offset(part, track, sector), buf->data, 256, 0))
      return 1;
    if (partition[part].imagetype & D64_HAS_ERRORINFO)
      d64_set_error(part, track, sector, 1); // no error
  }

  return 0;
}

/* create a 1581/DNP BAM signature */
static void format_add_bam_signature(uint8_t doschar, uint8_t *idbuf) {
  uint8_t *ptr = bam_buffer->data + 2;

  *ptr++ = doschar;
  *ptr++ = doschar ^ 0xff;
  *ptr++ = idbuf[0];
  *ptr++ = idbuf[1];
  *ptr++ = 0xc0;
}

/* copy name and id into some buffer */
static void format_copy_label(uint8_t part, uint8_t *data, uint8_t *name, uint8_t *idbuf) {
  uint8_t *ptr;
  uint8_t count;

  ptr = data + get_param(part, LABEL_OFFSET);
  memset(ptr, 0xa0, 25); /* 25=name+id+dos+spaces+padding */

  count = 16;
  while (*name && count--)
    *ptr++ = *name++;

  memcpy(data + get_param(part, ID_OFFSET), idbuf, 5);
}

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* allocate BAM and first directory sector */
  for (uint8_t s=0; s<2; s++)
    allocate_sector(part, D41_BAM_TRACK, s);

  /* 18/0 is now available via bam_buffer */
  uint8_t *ptr = bam_buffer->data;
  *ptr++ = 18;
  *ptr++ = 1;
  *ptr++ = 0x41;

  /* copy disk label and ID */
  idbuf[3] = '2';
  idbuf[4] = 'A';
  format_copy_label(part, bam_buffer->data, name, idbuf);
  /* two additional 0xa0 characters on 5.25" disks */
  bam_buffer->data[0xa9] = 0xa0;
  bam_buffer->data[0xaa] = 0xa0;

  clear_dir_sector(part, D41_BAM_TRACK, 1, buf->data);
}

static void format_d71_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* almost everything is the same as D41 */
  format_d41_image(part, buf, name, idbuf);

  /* add double-sided marker in 18/0 (still in bam_buffer) */
  bam_buffer->data[3] = 0x80;

  /* allocate all of track 53 */
  for (uint8_t s=0; s<19; s++)
    allocate_sector(part, D71_BAM2_TRACK, s);
}

static void format_d81_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* allocate 40/0 to 40/3 */
  for (uint8_t s=0; s<4; s++)
    allocate_sector(part, D81_BAM_TRACK, s);

  /* bam_buffer buffer now holds 40/1 */
  bam_buffer->data[0] = 40;
  bam_buffer->data[1] = 2;
  format_add_bam_signature('D', idbuf);
  // mustflush was already set by allocate_sector

  /* switch bam_buffer to 40/2 */
  (void)sectors_free(part, 41);
  bam_buffer->data[0] = 0;
  bam_buffer->data[1] = 0xff;
  format_add_bam_signature('D', idbuf);
  bam_buffer->mustflush = 1;

  /* build contents of 40/0 */
  uint8_t *ptr = buf->data;
  *ptr++ = 40;
  *ptr++ = 3;
  *ptr++ = 'D';

  /* copy disk label and ID to buffer */
  idbuf[3] = '3';
  idbuf[4] = 'D';
  format_copy_label(part, buf->data, name, idbuf);

  /* write 40/0 */
  if (image_write(part, /*sector_offset(part, 40, 0)*/ (40-1)*40*256L,
                  buf->data, 256, 0))
    return;

  clear_dir_sector(part, D81_BAM_TRACK, 3, buf->data);
}

static void format_dnp_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* Note: Formatting is only accepted while in the root dir, so the
           d64param struct doesn't need to be changed */
  for (uint8_t s=0; s<35; s++)
    allocate_sector(part, DNP_BAM_TRACK, s);

  /* add BAM signature - first BAM sector is in bam_buffer because of allocate_sector */
  format_add_bam_signature('H', idbuf);
  bam_buffer->data[DNP_BAM_LAST_TRACK_OFS] = get_param(part, LAST_TRACK);

  /* build root dirheader */
  uint8_t *ptr = buf->data;
  memset(ptr, 0, 256);
  *ptr++ = 1;
  *ptr++ = 34;
  *ptr++ = 'H';
  idbuf[3] = '1';
  idbuf[4] = 'H';
  format_copy_label(part, buf->data, name, idbuf);
  buf->data[DNP_DIRHEADER_ROOTHDR_TRACK]  = 1;
  buf->data[DNP_DIRHEADER_ROOTHDR_SECTOR] = 1;

  /* write 1/1 */
  if (image_write(part, /*sector_offset(part, 1, 1)*/ 256,
                  buf->data, 256, 0))
    return;

  clear_dir_sector(part, 1, DNP_ROOTDIR_SECTOR, buf->data);
}

#ifdef CONFIG_D80D82
#ifdef CONFIG_NOFORMAT_D80D82
static void format_d80_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
   (void)part;
   (void)buf;
   (void)name;
   (void)idbuf;
   set_error(ERROR_SYNTAX_UNABLE);
}
static void format_d82_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
   (void)part;
   (void)buf;
   (void)name;
   (void)idbuf;
   set_error(ERROR_SYNTAX_UNABLE);
}
#else
static void format_d80_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  allocate_sector(part, 39, 0);
  allocate_sector(part, 39, 1);
  allocate_sector(part, 38, 0);
  allocate_sector(part, 38, 3);
  
  uint8_t *ptr = bam_buffer->data;
  *ptr++ = 38;
  *ptr++ = 3;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 1;
  *ptr++ = 0x33;

  (void)sectors_free(part, 52);
  ptr = bam_buffer->data;
  *ptr++ = 39;
  *ptr++ = 1;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 0x33;
  *ptr++ = 0x4e;
  bam_buffer->mustflush = 1;
  
  ptr = buf->data;
  memset(ptr, 0, 256);
  *ptr++ = 38;
  *ptr++ = 0;
  *ptr++ = 0x43;

  /* copy disk label and ID to buffer */
  idbuf[3] = '2';
  idbuf[4] = 'C';
  format_copy_label(part, buf->data, name, idbuf);

  if (image_write(part, 0x44e00L, buf->data, 256, 0))
    return;

  clear_dir_sector(part, 39, 1, buf->data);
}

static void format_d82_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  allocate_sector(part, 39, 0);
  allocate_sector(part, 39, 1);
  allocate_sector(part, 38, 0);
  allocate_sector(part, 38, 3);
  allocate_sector(part, 38, 6);
  allocate_sector(part, 38, 9);
  
  uint8_t *ptr = bam_buffer->data;
  *ptr++ = 38;
  *ptr++ = 3;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 1;
  *ptr++ = 0x33;
  bam_buffer->mustflush = 1;

  (void)sectors_free(part, 52);
  ptr = bam_buffer->data;
  *ptr++ = 38;
  *ptr++ = 6;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 0x33;
  *ptr++ = 0x65;
  bam_buffer->mustflush = 1;

  (void)sectors_free(part, 102);
  ptr = bam_buffer->data;
  *ptr++ = 38;
  *ptr++ = 9;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 0x65;
  *ptr++ = 0x97;
  bam_buffer->mustflush = 1;

  (void)sectors_free(part, 152);
  ptr = bam_buffer->data;
  *ptr++ = 39;
  *ptr++ = 1;
  *ptr++ = 0x43;
  *ptr++ = 0;
  *ptr++ = 0x97;
  *ptr++ = 0x9B;
  bam_buffer->mustflush = 1;
  
  ptr = buf->data;
  memset(ptr, 0, 256);
  *ptr++ = 38;
  *ptr++ = 0;
  *ptr++ = 0x43;

  /* copy disk label and ID to buffer */
  idbuf[3] = '2';
  idbuf[4] = 'C';
  format_copy_label(part, buf->data, name, idbuf);

  if (image_write(part, 0x44e00L, buf->data, 256, 0))
    return;

  clear_dir_sector(part, 39, 1, buf->data);
}
#endif  
#endif

static void d64_format(path_t *path, uint8_t *name, uint8_t *id) {
  buffer_t *buf;
  uint8_t  part = path->part;
  uint8_t  idbuf[5];
  uint16_t t;
  uint16_t  s;

  /* allow format on DNP only when in the root directory */
  if ((partition[part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP &&
      (partition[part].current_dir.dxx.track  != 1 ||
       partition[part].current_dir.dxx.sector != 1)) {
    /* just ignore, CMD HD doesn't return an error either */
    return;
  }

  /* grab a buffer as work area */
  buf = alloc_buffer();
  if (buf == NULL)
    return;

  mark_write_buffer(buf);
  unstick_buffer(buf);
  mark_buffer_dirty(buf);
  memset(buf->data, 0, 256);

  /* Flush BAM buffers and mark their contents as invalid */
  d64_commit();
  bam_buffer->pvt.bam.part = 0xff;
  if (bam_buffer2)
    bam_buffer2->pvt.bam.part = 0xff;

  if (id != NULL) {
    /* Clear the data area of the disk image */
    int fastformat = globalflags & FASTFORMAT;
    if (id[2] == '-') fastformat = 0;
    if (id[2] == '+') fastformat = 1;
    if (!fastformat)
       for (t=1; t<=get_param(part, LAST_TRACK); t++) {
         if (d64_format_track(part, buf, t))
           return;
       }
    else
    {
      if ((partition[part].imagetype  & D64_TYPE_MASK) == D64_TYPE_D71)
          if (d64_format_track(part, buf, 53))
              return;
#ifdef CONFIG_D80D82
      if (((partition[part].imagetype  & D64_TYPE_MASK) == D64_TYPE_D80) || ((partition[part].imagetype  & D64_TYPE_MASK) == D64_TYPE_D82) )
          if (d64_format_track(part, buf, 38))
              return;
#endif

       /* clear the entire directory track */
       /* This is not accurate, but I do not care. */
       if (d64_format_track(part, buf, get_param(part, DIR_TRACK)))
         return;
    }

    /* Copy the new ID into the buffer */
    idbuf[0] = id[0];
    idbuf[1] = id[1];
  } else {
    /* Read the old ID into the buffer */
    if (d64_getid(path, idbuf))
      return;
   if ((partition[part].imagetype  & D64_TYPE_MASK) == D64_TYPE_D71)
    if (d64_format_track(part, buf, 53))
      return;

    /* clear the entire directory track */
    /* This is not accurate, but I do not care. */
    if (d64_format_track(part, buf, get_param(part, DIR_TRACK)))
      return;
  }
  idbuf[2] = 0xa0;

  /* Mark all sectors as free */
  for (t=1; t<=get_param(part, LAST_BAM_TRACK); t++) {
    for (s=0; s<d64_sectors_per_track(part, t); s++)
      free_sector(part,t,s);
  }

  /* call imagetype-specific format function */
  partition[part].d64data.format_function(part, buf, name, idbuf);
}

static void d64_set_attrib(path_t *path, cbmdirent_t *dent, uint8_t attr)
{
  if (dent != 0)
  {
     uint8_t *ptr;
     /* Read the directory entry of the file */
     if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
       return;

     ptr = ops_scratch + DIR_OFS_FILE_TYPE;
     *ptr = (*ptr & 0x8F) | attr;

     write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1);
  }
  else
  {
     buffer_t *buffer = alloc_buffer();
     uint8_t track, sector;
   
     /* allow format on DNP only when in the root directory */
     if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP &&
        (partition[path->part].current_dir.dxx.track  != 1 ||
         partition[path->part].current_dir.dxx.sector != 1))
       
       return;

     if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
        sector = path->dir.dxx.sector;
     else
        sector = 0;
        
     track = path->dir.dxx.track;
#ifdef CONFIG_D80D82
     switch (partition[path->part].imagetype & D64_TYPE_MASK) {
       case D64_TYPE_D80:
       case D64_TYPE_D82:
          track = 38;
          break;
     }
#endif

     if (image_read(path->part, sector_offset(path->part, track, sector), buffer->data, 256))
     {
       cleanup_and_free_buffer(buffer);
       return;
     }
  
     switch (partition[path->part].imagetype & D64_TYPE_MASK) {
       case D64_TYPE_D41:
       case D64_TYPE_D71:
          buffer->data[2] = attr ? 0x3C : 0x41;
          break;

       case D64_TYPE_D81:
          buffer->data[2] = attr ? 0x3D : 0x44;
          break;

       case D64_TYPE_DNP:
          buffer->data[2] = attr ? 0x3E : 0x48;
          break;

#ifdef CONFIG_D80D82
       case D64_TYPE_D80:
       case D64_TYPE_D82:
          buffer->data[2] = attr ? 0x3F : 0x43;
          break;
#endif
     }

     partition[path->part].imagetype &= ~D64_IS_READLNLY;
     image_write(path->part, sector_offset(path->part, track, sector), buffer->data, 256, 1);
     cleanup_and_free_buffer(buffer);
     if (attr) partition[path->part].imagetype |= D64_IS_READLNLY;
  }
}

static void d64_set_headername(path_t *path, uint8_t *newname, uint8_t *newid)
{
   buffer_t *buffer = alloc_buffer();
   uint8_t track, sector;
   
   if ((partition[path->part].imagetype & D64_TYPE_MASK) == D64_TYPE_DNP)
     sector = path->dir.dxx.sector;
   else
     sector = 0;
     
  track = path->dir.dxx.track;

  if (image_read(path->part, sector_offset(path->part, track, sector), buffer->data, 256))
  {
      cleanup_and_free_buffer(buffer);
      return;
   }
  
   int offset = get_param(path->part, LABEL_OFFSET);
   
   memset(buffer->data + offset, 0xa0, 16);
   memcpy(buffer->data + offset, newname, ustrlen(newname));
   
   if (newid) {
      memset(buffer->data + offset + 18, 0x20, 2);
      memcpy(buffer->data + offset + 18, newid, ustrlen(newid));
   }

  image_write(path->part, sector_offset(path->part, track, sector), buffer->data, 256, 1);
   cleanup_and_free_buffer(buffer);
}

#ifdef CONFIG_CVT
/* ------------------------------------------------------------------------- */
/*  Converting a file between CVT and GEOS form in place                     */
/* ------------------------------------------------------------------------- */

/*
 * Every block of a CVT is 254 bytes, i.e. exactly one sector. A CVT stored
 * as a normal file has block n in the n-th sector of its chain, so the GEOS
 * file is already there, only linked differently:
 *
 *   signature  -> becomes the directory entry, the sector is freed
 *   INFO       -> INFO sector, link 00/FF
 *   RECORD     -> RECORD sector: link 00/FF, count/index of each record
 *                 replaced by the T/S of its first sector
 *   RECORD high-> (V2) freed
 *   data       -> the chain is cut at the record boundaries: the last
 *                 sector of each record gets the link 00/index, its old
 *                 link is the first sector of the next record
 *
 * GEOS -> CVT (EC:name,C) is the same backwards: one or two sectors are
 * allocated, the records are linked into one chain (padding set to 0 as
 * the specification says) and the RECORD sector gets counts and indexes.
 * The stored CVT is a PRG file, the GEOS part of its entry is cleared.
 *
 * Both directions check everything first and change nothing if the input
 * is not valid: Error 64, the file stays as it is. This is also what
 * happens when a CVT written with ,V,W can't be converted at the close.
 */

#define CVT_BLOCK 254

/* a real INFO block always starts like this (see cvt.c) */
static const PROGMEM uint8_t cvt_info_start[] = { 0x03, 0x15, 0xbf };

/* Write the link bytes of a sector */
static uint8_t cvt_write_link(uint8_t part, uint8_t t, uint8_t s, const uint8_t *link) {
  return image_write(part, sector_offset(part, t, s), (void *)link, 2, 0);
}

/* Peek callback for the CVT layer: the signature block, in a buffer */
static uint8_t cvt_buffer_peek(void *ctx, uint16_t offset, uint8_t *dst, uint8_t len) {
  if (offset + len > CVT_BLOCK)
    return 1;
  memcpy(dst, (uint8_t *)ctx + 2 + offset, len);
  return 0;
}

/**
 * cvt_split - take the data of a stored VLIR CVT apart into its records
 * @part : partition
 * @buf  : buffer with the RECORD block (low bytes) of the CVT
 * @high : T/S of the V2 high block, track 0 for V1
 * @ts   : first data sector, track 0 if there is none
 * @write: 0 to check only, 1 to convert
 *
 * Walks the data chain record by record as the RECORD block says. With
 * write, the last sector of each record but the last gets the link
 * 00/index and the entries in buf become the T/S of the records. Sectors
 * left after the last record make the CVT defective.
 * Returns 0 on success, otherwise the error channel is set (64 if the CVT
 * is defective).
 */
static uint8_t cvt_split(uint8_t part, buffer_t *buf, const uint8_t *high,
                         const uint8_t *ts, uint8_t write) {
  uint8_t *p, link[2];
  uint8_t t = ts[0], s = ts[1];
  uint8_t first_t, first_s, last_t, last_s;
  uint16_t count;

  for (p = buf->data + 2; p < buf->data + 256; p += 2) {
    count = p[0];
    if (high[0]) {
      /* V2: the high byte of the count is in the high block */
      if (image_read(part, sector_offset(part, high[0], high[1]) + (p - buf->data), link, 2))
        return 1;
      count += link[0] << 8;
    }

    if (!count) {
      if (!p[1])
        break;                   /* 00/00: no more records          */
      if (p[1] == 0xff)
        continue;                /* 00/FF: the record does not exist */
      goto defective;
    }

    /* a record: its sectors must be there, the index valid */
    if (p[1] < 2 || !t)
      goto defective;

    first_t = t;
    first_s = s;
    do {
      last_t = t;
      last_s = s;
      if (checked_read(part, t, s, link, 2, ERROR_ILLEGAL_TS_LINK))
        return 1;
      t = link[0];
      s = link[1];
    } while (--count && t);

    if (count)
      goto defective;            /* the chain ends inside the record */

    if (!t) {
      /* end of the chain: this must be the last record, the bytes used */
      /* in its last sector are those the RECORD block says             */
      if (s != p[1])
        goto defective;
    } else if (write) {
      link[0] = 0;
      link[1] = p[1];
      if (cvt_write_link(part, last_t, last_s, link))
        return 1;
    }

    if (write) {
      p[0] = first_t;
      p[1] = first_s;
    }
  }

  if (write)
    memset(p, 0, buf->data + 256 - p);

  /* sectors left after the last record */
  if (t)
    goto defective;

  return 0;

 defective:
  set_error(ERROR_FILE_TYPE_MISMATCH);
  return 1;
}

/**
 * cvt_to_geos - convert a CVT stored as file into the GEOS file
 * @part: partition
 * @dh  : directory entry of the file
 * @buf : work buffer
 *
 * The CVT is checked completely before anything is changed. The directory
 * entry is restored from the signature block except the name, which is set
 * with d64_rename at the end (that is the place for a check that the name
 * does not exist yet). Returns 0 on success, otherwise the error channel is
 * set (64: not a valid CVT, nothing changed).
 */
static uint8_t cvt_to_geos(uint8_t part, struct d64dh *dh, buffer_t *buf) {
  uint8_t *p = buf->data + 2;
  uint8_t sig[2], info[2], rec[2], high[2], data[2], hdr[5];
  uint8_t struc, v2, freed;
  uint16_t blocks;
  cbmdirent_t dent;
  path_t path;

  /* the file itself: a GEOS file is not a CVT */
  if (read_entry(part, dh, ops_scratch))
    return 1;
  if (cvt_is_geos(ops_scratch))
    goto mismatch;

  sig[0] = ops_scratch[DIR_OFS_TRACK];
  sig[1] = ops_scratch[DIR_OFS_SECTOR];
  blocks = ops_scratch[DIR_OFS_SIZE_LOW] | (ops_scratch[DIR_OFS_SIZE_HI] << 8);

  /* the signature block; images can't hold REL files yet */
  if (checked_read(part, sig[0], sig[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK))
    return 1;
  if (cvt_probe(cvt_buffer_peek, buf->data, &dent) ||
      (p[0] & TYPE_MASK) == TYPE_REL)
    goto mismatch;

  /* buf->data is laid out like a directory entry: the signature block */
  /* has the entry without its first two bytes                         */
  struc = cvt_is_geos(buf->data) ? p[CVT_OFS_STRUCT] : 0;
  v2 = !memcmp_P(p + CVT_OFS_V2SIG, cvt_v2sig, sizeof(cvt_v2sig) - 1);

  /* the sector after the signature block */
  data[0] = buf->data[0];
  data[1] = buf->data[1];
  info[0] = info[1] = 0;
  high[0] = high[1] = 0;
  if (!data[0])
    goto mismatch;               /* nothing but a signature block */

  if (cvt_is_geos(buf->data)) {
    /* The entry says if there is an INFO block. Older writers left the */
    /* track 0 although there was one, it is recognized by its start.   */
    uint8_t has_info = p[CVT_OFS_INFO_TS];

    if (checked_read(part, data[0], data[1], hdr, sizeof(hdr), ERROR_ILLEGAL_TS_LINK))
      return 1;
    if (has_info || !memcmp_P(hdr + 2, cvt_info_start, sizeof(cvt_info_start))) {
      info[0] = data[0];
      info[1] = data[1];
      data[0] = hdr[0];
      data[1] = hdr[1];
      if (!data[0])
        goto mismatch;
    }
  }

  if (struc) {
    /* VLIR: RECORD block, V2 high block, then the data */
    rec[0] = data[0];
    rec[1] = data[1];
    if (checked_read(part, rec[0], rec[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK))
      return 1;
    data[0] = buf->data[0];
    data[1] = buf->data[1];

    if (v2) {
      if (!data[0])
        goto mismatch;
      high[0] = data[0];
      high[1] = data[1];
      if (checked_read(part, high[0], high[1], data, 2, ERROR_ILLEGAL_TS_LINK))
        return 1;
    }

    /* check everything, then convert */
    if (cvt_split(part, buf, high, data, 0) ||
        cvt_split(part, buf, high, data, 1))
      return 1;

    /* RECORD sector of the GEOS file */
    buf->data[0] = 0;
    buf->data[1] = 0xff;
    if (image_write(part, sector_offset(part, rec[0], rec[1]), buf->data, 256, 0))
      return 1;

    data[0] = rec[0];
    data[1] = rec[1];
  }

  if (info[0]) {
    hdr[0] = 0;
    hdr[1] = 0xff;
    if (cvt_write_link(part, info[0], info[1], hdr))
      return 1;
  }

  /* The entry from the signature block: type with flags, INFO T/S,  */
  /* structure, GEOS type and date. Not the name and not the size.   */
  if (checked_read(part, sig[0], sig[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK) ||
      read_entry(part, dh, ops_scratch))
    return 1;

  freed = high[0] ? 2 : 1;
  blocks -= freed;

  ops_scratch[DIR_OFS_FILE_TYPE] = p[0];
  ops_scratch[DIR_OFS_TRACK]     = data[0];
  ops_scratch[DIR_OFS_SECTOR]    = data[1];
  memcpy(ops_scratch + DIR_OFS_GEOS_INFO_TRACK, p + CVT_OFS_INFO_TS,
         DIR_OFS_SIZE_LOW - DIR_OFS_GEOS_INFO_TRACK);
  ops_scratch[DIR_OFS_GEOS_INFO_TRACK]  = info[0];
  ops_scratch[DIR_OFS_GEOS_INFO_SECTOR] = info[1];
  ops_scratch[DIR_OFS_SIZE_LOW] = blocks & 0xff;
  ops_scratch[DIR_OFS_SIZE_HI]  = blocks >> 8;
  if (write_entry(part, dh, ops_scratch, 1))
    return 1;

  free_sector(part, sig[0], sig[1]);
  if (high[0])
    free_sector(part, high[0], high[1]);

  /* The name from the signature block, 16 bytes with the $a0 padding.   */
  /* TODO (beta): the rename must check that the name does not exist yet */
  memmove(buf->data, p + CVT_NAME_OFFSET, CBM_NAME_LENGTH);
  buf->data[CBM_NAME_LENGTH] = 0;
  path.part = part;
  dent.pvt.dxx.dh = *dh;
  d64_rename(&path, &dent, buf->data);

  return 0;

 mismatch:
  set_error(ERROR_FILE_TYPE_MISMATCH);
  return 1;
}

/* Set the bytes after the last used one of a sector to 0 */
static uint8_t cvt_zero_padding(uint8_t part, uint8_t t, uint8_t s, uint8_t used) {
  uint16_t pos = used + 1;
  uint8_t len;

  memset(ops_scratch, 0, sizeof(ops_scratch));
  while (pos < 256) {
    len = 256 - pos > sizeof(ops_scratch) ? sizeof(ops_scratch) : 256 - pos;
    if (image_write(part, sector_offset(part, t, s) + pos, ops_scratch, len, 0))
      return 1;
    pos += len;
  }
  return 0;
}

/**
 * cvt_from_geos - convert a file into a CVT stored as PRG file
 * @part: partition
 * @dh  : directory entry of the file
 * @dent: directory entry of the file (for the signature block)
 * @buf : work buffer
 *
 * Any file can be converted, a file that is no GEOS file gets the same
 * signature block as when it is read with ,V. Images can't hold REL files
 * yet, so REL (and the other types) are refused with error 64.
 * Returns 0 on success, otherwise the error channel is set.
 */
static uint8_t cvt_from_geos(uint8_t part, struct d64dh *dh, cbmdirent_t *dent, buffer_t *buf) {
  uint8_t *e = ops_scratch;
  uint8_t start[2], info[2], sig[2], high[2], first[2], cur[2], link[2];
  uint8_t struc = 0, v2 = 0;
  uint16_t blocks;

  if (read_entry(part, dh, e))
    return 1;
  if ((e[DIR_OFS_FILE_TYPE] & TYPE_MASK) > TYPE_USR) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return 1;
  }

  start[0] = e[DIR_OFS_TRACK];
  start[1] = e[DIR_OFS_SECTOR];
  /* all of them, older gcc can't see that byte 1 is only used with byte 0 */
  info[0]  = info[1]  = 0;
  first[0] = first[1] = 0;
  high[0]  = high[1]  = 0;

  /* check everything first: the first sector must be there ... */
  if (checked_read(part, start[0], start[1], link, 2, ERROR_ILLEGAL_TS_LINK))
    return 1;

  if (cvt_is_geos(e)) {
    struc   = e[DIR_OFS_GEOS_STRUCT];
    info[0] = e[DIR_OFS_GEOS_INFO_TRACK];
    info[1] = e[DIR_OFS_GEOS_INFO_SECTOR];

    /* ... the INFO block ... */
    if (info[0] && checked_read(part, info[0], info[1], link, 2, ERROR_ILLEGAL_TS_LINK))
      return 1;
  }

  buf->pvt.d64cvt.part   = part;
  buf->pvt.d64cvt.track  = start[0];
  buf->pvt.d64cvt.sector = start[1];
  buf->pvt.d64cvt.state  = 0;

  if (struc) {
    /* ... and all records (this also finds out if V2 is needed) */
    if (cvt_record_block(buf, 0))
      return 1;
    v2 = buf->pvt.d64cvt.state & CVT_FLAG_V2;

    buf->pvt.d64cvt.record = 0;
    cvt_next_record(buf);
    first[0] = buf->data[0];
    first[1] = buf->data[1];
  }

  /* the new sectors: signature block and V2 high block */
  if (get_first_sector(part, &sig[0], &sig[1]) ||
      allocate_sector(part, sig[0], sig[1]))
    return 1;

  if (v2) {
    high[0] = sig[0];
    high[1] = sig[1];
    if (get_next_sector(part, &high[0], &high[1]) ||
        allocate_sector(part, high[0], high[1])) {
      free_sector(part, sig[0], sig[1]);
      return 1;
    }
  }

  if (struc) {
    /* The counts must be taken before the records are linked, after that */
    /* a chain runs into the next record. First the V2 high block ...     */
    if (v2) {
      if (cvt_record_block(buf, 1))
        return 1;
      buf->data[0] = first[0];
      buf->data[1] = first[1];
      if (image_write(part, sector_offset(part, high[0], high[1]), buf->data, 256, 0))
        return 1;
    }

    /* ... then the RECORD block, it stays in buf while the records are */
    /* linked: that only uses data[0]/data[1] (cvt_next_record).        */
    if (cvt_record_block(buf, 0))
      return 1;

    buf->pvt.d64cvt.record = 0;
    cvt_next_record(buf);
    while (buf->data[0]) {
      /* the last sector of this record */
      cur[0] = buf->data[0];
      cur[1] = buf->data[1];
      do {
        if (checked_read(part, cur[0], cur[1], link, 2, ERROR_ILLEGAL_TS_LINK))
          return 1;
        if (!link[0])
          break;
        cur[0] = link[0];
        cur[1] = link[1];
      } while (1);

      /* link it to the next record, padded with 0 */
      cvt_next_record(buf);
      if (buf->data[0]) {
        if (cvt_zero_padding(part, cur[0], cur[1], link[1]) ||
            cvt_write_link(part, cur[0], cur[1], buf->data))
          return 1;
      }
    }

    /* RECORD block: on to the high block or the data, if there is any */
    if (high[0]) {
      buf->data[0] = high[0];
      buf->data[1] = high[1];
    } else if (first[0]) {
      buf->data[0] = first[0];
      buf->data[1] = first[1];
    } else {
      buf->data[1] = 0xff;      /* data[0] is 0: last block, full */
    }
    if (image_write(part, sector_offset(part, start[0], start[1]), buf->data, 256, 0))
      return 1;
  }

  /* INFO block: on to the RECORD block or the data */
  if (info[0] && cvt_write_link(part, info[0], info[1], start))
    return 1;

  /* signature block: on to the INFO block or what follows it */
  if (read_entry(part, dh, e))
    return 1;
  if (cvt_is_geos(e))
    cvt_geos_signature(dent, buf, e, v2);
  else
    cvt_make_header(dent, buf, 0);
  if (info[0]) {
    buf->data[0] = info[0];
    buf->data[1] = info[1];
  } else {
    buf->data[0] = start[0];
    buf->data[1] = start[1];
  }
  if (image_write(part, sector_offset(part, sig[0], sig[1]), buf->data, 256, 0))
    return 1;

  /* the entry: a PRG file, no GEOS file any more */
  blocks = e[DIR_OFS_SIZE_LOW] + 256 * e[DIR_OFS_SIZE_HI] + (high[0] ? 2 : 1);
  e[DIR_OFS_FILE_TYPE] = (e[DIR_OFS_FILE_TYPE] & 0xe0) | TYPE_PRG;
  e[DIR_OFS_TRACK]     = sig[0];
  e[DIR_OFS_SECTOR]    = sig[1];
  memset(e + DIR_OFS_GEOS_INFO_TRACK, 0, DIR_OFS_GEOS_TYPE - DIR_OFS_GEOS_INFO_TRACK + 1);
  e[DIR_OFS_SIZE_LOW]  = blocks & 0xff;
  e[DIR_OFS_SIZE_HI]   = blocks >> 8;

  return write_entry(part, dh, e, 1);
}

/**
 * d64_cvt_write_cleanup - cleanup-callback of a file written with ,V
 * @buf: buffer of the file
 *
 * The CVT has been written as a normal PRG file. When it is complete, it
 * is converted into the GEOS file. If that is not possible, the PRG file
 * stays as it is and the error channel says why.
 */
static uint8_t d64_cvt_write_cleanup(buffer_t *buf) {
  struct d64dh dh = buf->pvt.d64.dh;
  uint8_t part = buf->pvt.d64.part;

  if (d64_write_finish(buf))
    return 1;

  cvt_to_geos(part, &dh, buf);

  buf->cleanup = callback_dummy;
  free_buffer(buf);

  return 0;
}
#endif /* CONFIG_CVT */

static void d64_change_type(path_t *path, cbmdirent_t *dent, uint8_t newtype) {
     uint8_t *ptr;

#ifdef CONFIG_CVT
     /* EC:name,C/G: not a new type, the file is converted */
     if (newtype & CHTYPE_CONVERT) {
       buffer_t *buf;

       if (!(partition[path->part].imagehandle.flag & FA_WRITE)) {
         set_error(ERROR_WRITE_PROTECT);
         return;
       }

       buf = alloc_system_buffer();
       if (buf == NULL)
         return;

       if ((newtype & (uint8_t)~CHTYPE_CONVERT) == TYPE_CVT)
         cvt_from_geos(path->part, &dent->pvt.dxx.dh, dent, buf);
       else
         cvt_to_geos(path->part, &dent->pvt.dxx.dh, buf);

       free_buffer(buf);
       return;
     }
#endif

#ifdef CONFIG_CVT
     /* There is no type inside a file here: the flag means the one type */
     newtype &= (uint8_t)~CHTYPE_INNER;

     /* TYPE_CVT (ET:name,V) is a FAT thing, the file name is what makes a CVT */
     if (newtype == TYPE_CVT) {
       set_error(ERROR_SYNTAX_UNABLE);
       return;
     }
#endif

     /* Read the directory entry of the file */
     if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
       return;

     ptr = ops_scratch + DIR_OFS_FILE_TYPE;
     *ptr = (*ptr & 0xE0) | newtype;

     write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1);
}

static void d64_convert(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  (void)path; (void)dent; (void)newname;
  set_error(ERROR_SYNTAX_UNABLE);
}



/* ------------------------------------------------------------------------- */
/*  ops struct                                                               */
/* ------------------------------------------------------------------------- */

const PROGMEM fileops_t d64ops = {
  d64_open_read,
  d64_open_write,
  d64_open_rel,
  d64_delete,
  d64_getdisklabel,
  d64_getdirlabel,
  d64_getid,
  d64_freeblocks,
  d64_read_sector,
  d64_write_sector,
  d64_format,
  d64_opendir,
  d64_readdir,
  d64_mkdir,
  d64_chdir,
  d64_rename,
  d64_set_attrib,
  d64_set_headername,
  d64_change_type,
  d64_convert
};
