/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2022  Ingo Korb <ingo@akana.de>
   ASCII/PET conversion Copyright (C) 2008 Jim Brain <brain@jbrain.com>

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


   fatops.c: FAT operations

*/

#include <ctype.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "buffers.h"
#include "d64ops.h"
#include "diskchange.h"
#include "diskio.h"
#include "display.h"
#include "doscmd.h"
#include "errormsg.h"
#include "ff.h"
#include "cvt.h"
#include "fileops.h"
#include "flags.h"
#include "led.h"
#include "m2iops.h"
#include "p00cache.h"
#include "parser.h"
#include "progmem.h"
#include "uart.h"
#include "utils.h"
#include "ustring.h"
#include "wrapops.h"
#include "fatops.h"

#ifdef CONFIG_LCD_DISPLAY
#include "display_lcd.h"
#endif

#define P00_HEADER_SIZE       26
#define P00_CBMNAME_OFFSET    8
#define P00_RECORDLEN_OFFSET  25

#define BOOTSECTOR_FILE       "bootsect.128"

static const PROGMEM char p00marker[] = "C64File";
#define P00MARKER_LENGTH 7

typedef enum { EXT_UNKNOWN, EXT_IS_X00, EXT_IS_TYPE
#ifdef CONFIG_CVT
             , EXT_IS_CVT
#endif
} exttype_t;

uint8_t file_extension_mode;

/* ------------------------------------------------------------------------- */
/*  Utility functions                                                        */
/* ------------------------------------------------------------------------- */

static const char *reserved_names[] = {
    "CON", "PRN", "AUX", "NUL",
    "COM1", "COM2", "COM3", "COM4", "COM5", "COM6", "COM7", "COM8", "COM9",
    "LPT1", "LPT2", "LPT3", "LPT4", "LPT5", "LPT6", "LPT7", "LPT8", "LPT9",
    NULL
  };


static bool hex2bin(char c, uint8_t* out)
{
    if ((c >= '0') && (c <= '9')) {
        *out = (uint8_t)(c - '0');
        return true;
    }
    if ((c >= 'A') && (c <= 'F')) {
        *out = (uint8_t)(c - 'A') + 10;
        return true;
    }
    if ((c >= 'a') && (c <= 'f')) {
        *out = (uint8_t)(c - 'a') + 10;
        return true;
    }
    return false;
}

void petscii_to_fat(const char *pet, char *fat, int maxlen)
{
    int match=0, i;
     for (i = 0; reserved_names[i] != NULL; i++) {
        match=1;
        unsigned char *a = (unsigned char *) pet;
        unsigned char *b = (unsigned char *) reserved_names[i];

        while (match)
        {
           unsigned char aa = *a;
           unsigned char bb = *b;
           if (!bb && aa == '.')
              aa=0;  
           if(aa!= bb)
              match=0;
           if (!aa) break;
           if (!bb) break;
           a++; b++;
        }
      
        if (match) break;
     }
     if (match) {
        fat[0] = '{';
        fat[1] = '}';
     }

    // clear output string
    const char *hex = "0123456789ABCDEF";
    bool first = true;
    bool escape = false;
    i = match ? 2 : 0;
    while(*pet) {
        unsigned char p = *(unsigned char *)(pet++);
        if (p == 160) {
           const unsigned char* q = (unsigned char*) pet;
           while (*q == 160) q++;
           if (!*q) 
              break;
        }
        if ((p < 32) || (p >= 96) || (p == ':') || (p == '/') ||
                (p == '\\') || (p == '*') || (p == '\x22') ||
                (p == '<') || (p == '>') || (p == '?') || (first && p == '.')) {  // '|' > 96 ;)

            if ((i + 4) > maxlen) {
                break;
            }
            if (!escape) {
                fat[i++] = '{';
                escape = true;
            }
            fat[i++] = hex[((uint8_t)p) >> 4];
            fat[i++] = hex[p & 15];
        } else {
            if ((i + 2) > maxlen) {
                break;
            }
            if (escape) {
                fat[i++] = '}';
                escape = false;
            }
            fat[i++] = p;
        }
        first = false;
    }
    if (escape) {
        fat[i++] = '}';
        escape = false;
    }
    fat[i] = 0;
    
    if (strlen(fat) > 3)
    {
       char* ext = fat + strlen(fat) - 4; 
       if (!strcasecmp(ext, ".prg") ||!strcasecmp(ext, ".seq") || !strcasecmp(ext, ".usr") || !strcasecmp(ext, ".rel"))
          strcat(fat, "{}");
    }
}

void fat_to_petscii(const char *fat, bool cutExt, char *pet, int len, bool term)
{
    int i = 0, k = 0;
    bool escape = false;
    int max = strlen(fat);

    if (cutExt) {
        for(int j=max-1;j>=max-4;j--) {
            if(fat[j] == '.') {
                max = j;
                break;
            }
        }
    }

    while(fat[k] && (k < max) && (i < len)) {
        char f = fat[k];
        if (f == '{') {
            escape = true;
            k++;
        }
        if (escape) {
            char h = fat[k+0];
            char l = fat[k+1];
            if (!h || !l) {
                break;
            }
            uint8_t hb, lb;
            if (!hex2bin(h, &hb) || !hex2bin(l, &lb)) {
                escape = false;
                // do not advance
            } else {
                pet[i++] = (hb << 4) | lb;
                k += 2;
            }
        } else {
            if (f == '}') {
                escape = false;
                k++;
            } else {
                pet[i++] = toupper(f);
                k++;
            }
        }
    }
    if (term) {
        pet[i] = 0;
    }
}










/**
 * parse_error - translates a ff FRESULT into a commodore error message
 * @res     : FRESULT to be translated
 * @readflag: Flags if it was a read operation
 *
 * This function sets the error channel according to the problem given in
 * res. readflag specifies if a READ ERROR or WRITE ERROR should be used
 * if the error is FR_RW_ERROR.
 */
void parse_error(FRESULT res, uint8_t readflag) {
  switch (res) {
  case FR_OK:
    set_error(ERROR_OK);
    break;

  case FR_NO_FILE:
    set_error_ts(ERROR_FILE_NOT_FOUND,res,0);
    break;

  case FR_NO_PATH:
  case FR_NOT_DIRECTORY:
    set_error_ts(ERROR_FILE_NOT_FOUND_39,res,0);
    break;

  case FR_INVALID_NAME:
    set_error_ts(ERROR_SYNTAX_JOKER,res,0);
    break;

  case FR_NOT_READY:
  case FR_INVALID_DRIVE:
  case FR_NOT_ENABLED:
  case FR_NO_FILESYSTEM:
    set_error_ts(ERROR_DRIVE_NOT_READY,res,0);
    break;

  case FR_RW_ERROR:
    /* Just a random READ ERROR */
    if (readflag)
      set_error_ts(ERROR_READ_NOHEADER,res,0);
    else
      set_error_ts(ERROR_WRITE_VERIFY,res,0);
    break;

  case FR_WRITE_PROTECTED:
    set_error_ts(ERROR_WRITE_PROTECT,res,0);
    break;

  case FR_EXIST:
    set_error_ts(ERROR_FILE_EXISTS,res,0);
    break;

  case FR_DIR_NOT_EMPTY:
    // FIXME: What do the CMD drives return when removing a non-empty directory?
    set_error_ts(ERROR_FILE_EXISTS,res,0);
    break;

  case FR_DENIED:
    /* Just a random READ ERROR */
    if (readflag)
      set_error_ts(ERROR_READ_NOHEADER,res,0);
    else
      set_error_ts(ERROR_WRITE_PROTECT,res,0);
    break;

  case FR_IS_READONLY:
  case FR_IS_DIRECTORY:
    set_error_ts(ERROR_FILE_EXISTS,res,0);
    break;

  case FR_INVALID_OBJECT:
    set_error_ts(ERROR_DRIVE_NOT_READY,res,0);
    break;

  default:
    set_error_ts(ERROR_SYNTAX_UNABLE,res,99);
    break;
  }
}

/**
 * check_extension - check for known file-type-based name extensions
 * @name: pointer to the file name
 * @ext : pointer to pointer to the file extension
 *
 * This function checks if the given file name has an extension that
 * indicates a specific file type like PRG/SEQ/P00/S00/... The ext
 * pointer will be set to the first character of the extension if
 * any is present or NULL if not. Returns EXT_IS_X00 for x00,
 * EXT_IS_TYPE for PRG/SEQ/... or EXT_UNKNOWN for an unknown file extension.
 */
static exttype_t check_extension(uint8_t *name, uint8_t **ext) {
  uint8_t f,s,t;

  /* Search for the file extension */
  if ((*ext = ustrrchr(name, '.')) != NULL) {
    f = *(++(*ext));
    s = *(*ext+1);
    t = *(*ext+2);
    if ((f == 'P' || f == 'S' ||
         f == 'U' || f == 'R') &&
        isdigit(s) && isdigit(t))
      return EXT_IS_X00;
    else if ((f=='P' && s == 'R' && t == 'G') ||
             (f=='S' && s == 'E' && t == 'Q') ||
             (f=='R' && s == 'E' && t == 'L') ||
             (f=='U' && s == 'S' && t == 'R'))
      return EXT_IS_TYPE;
#ifdef CONFIG_CVT
    else if (f == 'C' && s == 'V' && t == 'T' && *(*ext+3) == 0)
      return EXT_IS_CVT;
#endif
  }
  return EXT_UNKNOWN;
}

/**
 * check_imageext - check for a known image file extension
 * @name: pointer to the file name
 *
 * This function checks if the given file name has an extension that
 * indicates a known image file type. Returns IMG_IS_M2I for M2I files,
 * IMG_IS_[DNP|D41|D71|D81] for D64/D41/D71/D81 files or IMG_UNKNOWN
 * for an unknown file extension.
 */
imgtype_t check_imageext(uint8_t *name) {
  uint8_t f,s,t;
  uint8_t *ext = ustrrchr(name, '.');

  if (ext == NULL)
    return IMG_UNKNOWN;

  if (ustrlen(ext) != 4)
    return IMG_UNKNOWN;

  f = toupper(*++ext);
  s = toupper(*++ext);
  t = toupper(*++ext);

#ifdef CONFIG_M2I
  if (f == 'M' && s == '2' && t == 'I')
    return IMG_IS_M2I;
#endif

  if (f == 'D') {
    if (s == 'N' && t == 'P')
      return IMG_IS_DNP;
    if ((s == '6' && t == '4') || (s == '4' && t == '1'))
      return IMG_IS_D41;
    if (s == '7' && t == '1')
      return IMG_IS_D71;
    if (s == '8') {
      if (t == '1') return IMG_IS_D81;
#ifdef CONFIG_D80D82 
      else if (t == '0') return IMG_IS_D80;
      else if (t == '2') return IMG_IS_D82;
#endif
    }
  }

  return IMG_UNKNOWN;
}

/**
 * should_save_raw - check if the file should be saved header-free
 * @name: pointer to the file name
 *
 * This function checks if the file of the given name should be saved
 * without any header for improved PC compatiblity.
 */
static bool should_save_raw(uint8_t* name) {
  if (check_imageext(name) != IMG_UNKNOWN)
    return true;

  uint8_t* ext = ustrrchr(name, '.');
  if (ext == NULL)
    return false;

  ext++;

  if (*ext == 't' || *ext == 'T')
    ext++;

  if (ustrlen(ext) != 3)
    return false;

  uint8_t ucext[3];

  for (int i = 0; i < 3; i++) {
    ucext[i] = toupper(*ext++);
  }

  if (ucext[0] == 'C' && ucext[1] == 'R' && ucext[2] == 'T')
    return true;

  return false;
}

/**
 * pet2asc - convert string from PETSCII to ASCII
 * @buf: pointer to the string to be converted
 *
 * This function converts the string in the given buffer from PETSCII to
 * ASCII in-place.
 */
static void pet2asc(uint8_t *buf) {
  uint8_t ch;
  while (*buf) {
    ch = *buf;
    if (ch > (128+64) && ch < (128+91))
      ch -= 128;
    else if (ch > (96-32) && ch < (123-32))
      ch += 32;
    else if (ch > (192-128) && ch < (219-128))
      ch += 128;
    else if (ch == 255)
      ch = '~';
    *buf = ch;
    buf++;
  }
}

/**
 * is_valid_fat_char - checks if a character is valid on FAT
 * @c: character to check
 *
 * This function checks if @c is a valid character for a FAT
 * file name. Returns true if it is, false if not.
 */
static bool is_valid_fat_char(const uint8_t c) {
  if (isalnum(c) || c == '!' || c == ' ' ||
      (c >= '#' && c <= ')') ||
      c == '-' || c == '.')
    return true;
  else
    return false;
}

/**
 * is_valid_fat_name - checks if a file name is valid on FAT
 * @name: name to check
 *
 * This function checks if @name is a valid name for a FAT
 * file. Returns true if it is, false if not.
 */

static bool is_valid_fat_name(const uint8_t *name) {
  const uint8_t *ptr = name;
  int i;

  /* check all characters for validity */
  while (*ptr) {
    if (!is_valid_fat_char(*ptr++))
      return false;
  }

  /* check the last character */
  ptr--;

  if (*ptr == ' ')
    return false;

  if (*ptr == '.')
    return false;

   for (i = 0; reserved_names[i] != NULL; i++) {
      int match=1;
      unsigned char *a = (unsigned char *) name;
      unsigned char *b = (unsigned char *) reserved_names[i];

      while (match)
      {
         unsigned char aa = *a;
         unsigned char bb = *b;
         if (bb >= 'a' && bb <= 'z')          /* Convert to upper case */
            bb -= 0x20;
         if (aa >= 'a' && aa <= 'z')          /* Convert to upper case */
            aa -= 0x20;
         if (!bb && aa == '.')
            aa=0;  
         if(aa!= bb)
            match=0;
         if (!aa) break;
         if (!bb) break;
         a++; b++;
      }
      
      if (match) return false;
  }


  return true;
}

/**
 * build_name - convert PETSCII file name to valid FAT name
 * @name: pointer to a PETSCII file name to be converted
 * @type: file type
 *
 * This function converts a PETSCII file name to a suitable
 * FAT file name in-place. Returns a pointer to the last
 * character of the PC64 file extension if it was
 * created or NULL if not.
 */
static char fatfs_yAbuffer[45];

static uint8_t* build_name(uint8_t *name, uint8_t type, uint8_t isRename) {
  if (file_extension_mode == 5 && isRename != 2)
  {
     petscii_to_fat((const char*)name, fatfs_yAbuffer, 40);
     
     memcpy(name, fatfs_yAbuffer, 45);
     if (type == 0) return NULL; 
     while (*name) name++;
     *name++ = '.';
     memcpy_P(name, filetypes + TYPE_LENGTH * (type & EXT_TYPE_MASK), TYPE_LENGTH);
     *(name+3) = 0;
     return NULL;
  }
#ifdef CONFIG_CVT
  /* The name of the file does not matter for a CVT: If FAT can not hold  */
  /* the name, it gets the form it has in xe5, {xx} for what does not fit. */
  /* Convert it now, pet2asc below can not be undone.                      */
  if (type == TYPE_CVT)
    petscii_to_fat((const char*)name, fatfs_yAbuffer, 40);
#endif

  /* convert to PETSCII */
  pet2asc(name);

#ifdef CONFIG_M2I
  /* do not add a header for raw files, even if the name may be invalid */
  if (type == TYPE_RAW)
    return NULL;
#endif

  /* known disk-image extensions are always without header or suffix */
  if (type == TYPE_PRG && should_save_raw(name))
    return NULL;

  /* PC64 mode or invalid FAT name? */
  if ( isRename != 1 &&
#ifdef CONFIG_CVT
       type != TYPE_CVT &&   /* a CVT is never an x00 file */
#endif
       ((file_extension_mode == 1 && type != TYPE_PRG) ||
      file_extension_mode == 2 || isRename == 2 ||
      !is_valid_fat_name(name))) {

      uint8_t *x00ext = NULL;
      uint8_t *nameBak = name;
      int i;

      /* Append .[PSUR]00 suffix to the file name */
      //int len=0;
      while (*name) {
        if (is_valid_fat_char(*name) && *name != '.') {
          name++;
        } else {
          *name++ = '_';
        }
        //len++;
        //if (len == 8)
        //   break;
      }
      *name++ = '.';
      *name++ = pgm_read_byte(filetypes+3*type);
      *name++ = '0';
      x00ext = name;
      *name++ = '0';
      *name   = 0;

   for (i = 0; reserved_names[i] != NULL; i++) {
      int match=1;
      unsigned char *a = (unsigned char *) nameBak;
      unsigned char *b = (unsigned char *) reserved_names[i];

      while (match)
      {
         unsigned char aa = *a;
         unsigned char bb = *b;
         if (bb >= 'a' && bb <= 'z')          /* Convert to upper case */
            bb -= 0x20;
         if (aa >= 'a' && aa <= 'z')          /* Convert to upper case */
            aa -= 0x20;
         if (!bb && aa == '.')
            aa=0;  
         if(aa!= bb)
            match=0;
         if (!aa) break;
         if (!bb) break;
         a++; b++;
      }
      
      if (match) nameBak[0]='_';
  }

      return x00ext;
  }
  
  if (!is_valid_fat_name(name))
  {
#ifdef CONFIG_CVT
     if (type == TYPE_CVT) {
        /* the xe5 name, it gets the .CVT below */
        memcpy(name, fatfs_yAbuffer, 45);
     } else
#endif
     {
        *name = 0;
        return NULL;
     }
  }

  if (type == 0)
     return NULL;

  /* type-suffix mode? */
  if ((file_extension_mode == 3 && type != TYPE_PRG) ||
#ifdef CONFIG_CVT
      type == TYPE_CVT ||   /* a CVT is recognized by it, ET:name,V */
#endif
      (file_extension_mode == 4)) {
    /* Append type suffix to the file name */
    while (*name) name++;
    *name++ = '.';
    memcpy_P(name, filetypes + TYPE_LENGTH * (type & EXT_TYPE_MASK), TYPE_LENGTH);
    *(name+3) = 0;

    return NULL;
  }

  /* extension mode 0 and no special case */
  return NULL;
}

/* ------------------------------------------------------------------------- */
/*  Callbacks                                                                */
/* ------------------------------------------------------------------------- */

/**
 * fat_file_read - read the next data block into the buffer
 * @buf: buffer to be worked on
 *
 * This function reads the next block of data from the associated file into
 * the given buffer. Used as a refill-callback when reading files
 */
static uint8_t fat_file_read(buffer_t *buf) {
  FRESULT res;
  UINT bytesread;

  uart_putc('#');

  buf->fptr = buf->pvt.fat.fh.fptr - buf->pvt.fat.headersize;

  res = f_read(&buf->pvt.fat.fh, buf->data+2, (buf->recordlen ? buf->recordlen : 254), &bytesread);
  if (res != FR_OK) {
    parse_error(res,1);
    free_buffer(buf);
    return 1;
  }

  /* The bus protocol can't handle 0-byte-files */
  if (bytesread == 0) {
    bytesread = 1;
    /* Experimental data suggests that this may be correct */
    buf->data[2] = (buf->recordlen ? 255 : 13);
  }

  buf->position = 2;
  buf->lastused = bytesread+1;
  if(buf->recordlen) // strip nulls from end of REL record.
    while(!buf->data[buf->lastused] && --(buf->lastused) > 1);
  if (bytesread < 254
      || (buf->pvt.fat.fh.fsize - buf->pvt.fat.fh.fptr) == 0
      || buf->recordlen
     )
    buf->sendeoi = 1;
  else
    buf->sendeoi = 0;

  return 0;
}

/**
 * write_data - write the current buffer data
 * @buf: buffer to be worked on
 *
 * This function writes the current contents of the given buffer into its
 * associated file.
 */
static uint8_t write_data(buffer_t *buf) {
  FRESULT res;
  UINT byteswritten;

  uart_putc('/');

  if(!buf->mustflush)
    buf->lastused = buf->position - 1;

  if(buf->recordlen > buf->lastused - 1)
    memset(buf->data + buf->lastused + 1,0,buf->recordlen - (buf->lastused - 1));

  if(buf->recordlen)
    buf->lastused = buf->recordlen + 1;

  res = f_write(&buf->pvt.fat.fh, buf->data+2, buf->lastused-1, &byteswritten);
  if (res != FR_OK) {
    uart_putc('r');
    parse_error(res,0);
    f_close(&buf->pvt.fat.fh);
    free_buffer(buf);
    return 1;
  }

  if (byteswritten != buf->lastused-1U) {
    uart_putc('l');
    set_error(ERROR_DISK_FULL);
    f_close(&buf->pvt.fat.fh);
    free_buffer(buf);
    return 1;
  }

  mark_buffer_clean(buf);
  buf->mustflush = 0;
  buf->position  = 2;
  buf->lastused  = 2;
  buf->fptr      = buf->pvt.fat.fh.fptr - buf->pvt.fat.headersize;

  return 0;
}

/**
 * fat_file_write - refill-callback for files opened for writing
 * @buf: target buffer
 *
 * This function writes the contents of buf to the associated file.
 */
static uint8_t fat_file_write(buffer_t *buf) {
  FRESULT res = FR_OK;
  uint32_t fptr;
  uint32_t i = 0;

  fptr = buf->pvt.fat.fh.fsize - buf->pvt.fat.headersize;

  // on a REL file, the fptr will be be at the end of the record we just read.  Reposition.
  if (buf->fptr != fptr) {
    res = f_lseek(&buf->pvt.fat.fh, buf->pvt.fat.headersize + buf->fptr);
    if (res != FR_OK) {
      parse_error(res,1);
      f_close(&buf->pvt.fat.fh);
      free_buffer(buf);
      return 1;
    }
  }

  if(buf->fptr > fptr)
    i = buf->fptr - fptr;

  if (res == FR_OK) {
    if (write_data(buf))
      return 1;
  }

  if(i) {
    // we need to fill bytes.
    // position to old end of file.
    res = f_lseek(&buf->pvt.fat.fh, buf->pvt.fat.headersize + fptr);
    buf->mustflush = 0;
    buf->fptr = fptr;
    buf->data[2] = (buf->recordlen?255:0);
    memset(buf->data + 3,0,253);
    while(res == FR_OK && i) {
      if (buf->recordlen)
        buf->lastused = buf->recordlen;
      else
        buf->lastused = (i>254 ? 254 : (uint8_t) i);

      i -= buf->lastused;
      buf->position = buf->lastused + 2;

      if(write_data(buf))
        return 1;
    }
    res = f_lseek(&buf->pvt.fat.fh, buf->pvt.fat.fh.fsize);
    if (res != FR_OK) {
      uart_putc('r');
      parse_error(res,1);
      f_close(&buf->pvt.fat.fh);
      free_buffer(buf);
      return 1;
    }
    buf->fptr = buf->pvt.fat.fh.fptr - buf->pvt.fat.headersize;
  }

  return 0;
}

/**
 * fat_file_seek - callback for seek
 * @buf     : buffer to be worked on
 * @position: offset to seek to
 * @index   : offset within the record to seek to
 *
 * This function seeks to the offset position in the file associated
 * with the given buffer and sets the read pointer to the byte given
 * in index, effectively seeking to (position+index) for normal files.
 * Returns 1 if an error occured, 0 otherwise.
 */
uint8_t fat_file_seek(buffer_t *buf, uint32_t position, uint8_t index) {
  uint32_t pos = position + buf->pvt.fat.headersize;

  if (buf->dirty)
    if (fat_file_write(buf))
      return 1;

  if (buf->pvt.fat.fh.fsize >= pos) {
    FRESULT res = f_lseek(&buf->pvt.fat.fh, pos);
    if (res != FR_OK) {
      parse_error(res,1);
      f_close(&buf->pvt.fat.fh);
      free_buffer(buf);
      return 1;
    }

    if (fat_file_read(buf))
      return 1;
  } else {
    buf->data[2]  = (buf->recordlen ? 255:13);
    buf->lastused = 2;
    buf->fptr     = position;
    set_error(ERROR_RECORD_MISSING);
  }

  buf->position = index + 2;
  if(index + 2 > buf->lastused)
    buf->position = buf->lastused;

  return 0;
}

/**
 * fat_file_sync - synchronize the current REL file.
 * @buf: buffer to be worked on
 *
 */
static uint8_t fat_file_sync(buffer_t *buf) {
  return fat_file_seek(buf,buf->fptr + buf->recordlen,0);
}

/**
 * fat_file_close - close the file associated with a buffer
 * @buf: buffer to be worked on
 *
 * This function closes the file associated with the given buffer. If the buffer
 * was opened for writing the data contents will be stored if required.
 * Additionally the buffer will be marked as free.
 * Used as a cleanup-callback for reading and writing.
 */
static uint8_t fat_file_close(buffer_t *buf) {
  FRESULT res;

  if (!buf->allocated) return 0;

  if (buf->write) {
    /* Write the remaining data using the callback */
    if (buf->refill(buf))
      return 1;
  }

  res = f_close(&buf->pvt.fat.fh);
  parse_error(res,1);
  buf->cleanup = callback_dummy;

  if (res != FR_OK)
    return 1;
  else
    return 0;
}

/* ------------------------------------------------------------------------- */
/*  Internal handlers for the various operations                             */
/* ------------------------------------------------------------------------- */

#ifdef CONFIG_CVT
/* Raw read access for the CVT layer, no interpretation of the content */
static uint8_t fat_peek(void *ctx, uint16_t offset, uint8_t *dst, uint8_t len) {
  UINT bytesread;

  if (f_lseek((FIL *)ctx, offset) != FR_OK)
    return 1;
  if (f_read((FIL *)ctx, dst, len, &bytesread) != FR_OK || bytesread != len)
    return 1;

  return 0;
}

/* Raw write access for the CVT layer, no interpretation of the content */
static uint8_t fat_poke(void *ctx, uint16_t offset, const uint8_t *src, uint8_t len) {
  UINT byteswritten;

  if (f_lseek((FIL *)ctx, offset) != FR_OK)
    return 1;
  if (f_write((FIL *)ctx, src, len, &byteswritten) != FR_OK || byteswritten != len)
    return 1;

  return 0;
}
#endif

#ifdef CONFIG_CVT
/**
 * fat_cvt_record_read - refill-callback for a VLIR record of a stored CVT
 * @buf: target buffer
 *
 * Like fat_file_read, but stops at the end of the record. In this mode
 * buf->fptr holds the number of bytes of the record not read yet (it is
 * not needed for anything else here, and pvt.fat has no room for it
 * without making every buffer bigger).
 */
static uint8_t fat_cvt_record_read(buffer_t *buf) {
  UINT len, bytesread;

  len = buf->fptr > 254 ? 254 : buf->fptr;
  if (f_read(&buf->pvt.fat.fh, buf->data+2, len, &bytesread) != FR_OK) {
    parse_error(FR_RW_ERROR, 1);
    free_buffer(buf);
    return 1;
  }

  /* the bus protocol can't handle 0 bytes (a truncated file) */
  if (bytesread == 0) {
    bytesread = 1;
    buf->data[2] = 13;
  }

  buf->fptr    -= bytesread > buf->fptr ? buf->fptr : bytesread;
  buf->position = 2;
  buf->lastused = bytesread + 1;
  buf->sendeoi  = (bytesread < len || buf->fptr == 0);
  return 0;
}

/**
 * fat_cvt_seek - seek-callback of a ,V channel on a stored CVT
 * @buf  : buffer of the channel
 * @pos  : offset from the P command
 * @index: offset within the record
 *
 * A VLIR file has at most 127 records, so P with an offset below 128
 * selects a record: From now on the channel delivers only its data, the
 * last byte with EOI. From 128 on it is a position in the stored file as
 * before (fat_file_seek), which also ends the record mode. Positions in the
 * first 128 bytes are no loss: they are in the signature block.
 * 64: the file is no VLIR CVT, 50: the record does not exist. After an
 * error the channel delivers a single CR.
 */
static uint8_t fat_cvt_seek(buffer_t *buf, uint32_t pos, uint8_t index) {
  uint32_t offset;

  if (pos >= 128) {
    buf->refill = fat_file_read;
    return fat_file_seek(buf, pos, index);
  }

  if (cvt_record(fat_peek, &buf->pvt.fat.fh, pos, &offset, &buf->fptr)) {
    buf->data[2]  = 13;
    buf->position = 2;
    buf->lastused = 2;
    buf->sendeoi  = 1;
    buf->read     = 1;
    return 1;
  }

  if (f_lseek(&buf->pvt.fat.fh, offset) != FR_OK) {
    parse_error(FR_RW_ERROR, 1);
    return 1;
  }

  buf->refill = fat_cvt_record_read;
  buf->read   = 1;
  return fat_cvt_record_read(buf);
}
#endif

/**
 * fat_open_read - opens a file for reading
 * @path: path of the file
 * @dent: pointer to cbmdirent with name of the file
 * @buf : buffer to be used
 *
 * @flags: OPENFLAG_* mask
 *
 * This functions opens a file in the FAT filesystem for reading and sets up
 * buf to access it.
 *
 * With OPENFLAG_CVT the data delivered is a CVT stream: A stored CVT is
 * delivered as it is, for any other file the CVT layer builds a signature
 * block that becomes the first block of the stream. Without the flag, a
 * stored CVT delivers only its data, like a real drive would.
 */
void fat_open_read(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t flags) {
  FRESULT res;
  uint8_t *name;

#ifndef CONFIG_CVT
  (void)flags;
#else
  /* A record selected at the open: opened as with ,V, then positioned */
  uint8_t record = RECSEL_NONE;

  if (flags & OPENFLAG_RECORD) {
    record = flags & OPENFLAG_RECNUM;
    flags  = OPENFLAG_CVT;
  }
#endif

  if (dent->pvt.fat.realname[0]) {
    name = dent->pvt.fat.realname;
  } else {
    /* only convert if the name is used, a CVT header needs the PETSCII name */
    pet2asc(dent->name);
    name = dent->name;
  }

  partition[path->part].fatfs.curr_dir = path->dir.fat;
  res = f_open(&partition[path->part].fatfs,&buf->pvt.fat.fh, name, FA_READ | FA_OPEN_EXISTING);
  if (res != FR_OK) {
    parse_error(res,1);
    return;
  }

  if (dent->opstype == OPSTYPE_FAT_X00) {
    /* It's a [PSUR]00 file, skip the header */
    /* If anything goes wrong here, refill will notice too */
    f_lseek(&buf->pvt.fat.fh, P00_HEADER_SIZE);
    buf->pvt.fat.headersize = P00_HEADER_SIZE;
  }
#ifdef CONFIG_CVT
  else if (dent->opstype == OPSTYPE_FAT_CVT && !(flags & OPENFLAG_CVT)) {
    /* A stored CVT without the CVT flag: skip to the data */
    uint16_t offset = cvt_data_offset(fat_peek, &buf->pvt.fat.fh);

    if (offset == 0) {
      f_close(&buf->pvt.fat.fh);
      return;
    }
    f_lseek(&buf->pvt.fat.fh, offset);
    buf->pvt.fat.headersize = offset;
  }
#endif

  buf->read      = 1;
  buf->cleanup   = fat_file_close;
  buf->refill    = fat_file_read;
  buf->seek      = fat_file_seek;
#ifdef CONFIG_CVT
  /* a stored CVT on a ,V channel: P can select a VLIR record */
  if ((flags & OPENFLAG_CVT) && dent->opstype == OPSTYPE_FAT_CVT)
    buf->seek    = fat_cvt_seek;
#endif

  stick_buffer(buf);

#ifdef CONFIG_CVT
  if ((flags & OPENFLAG_CVT) && dent->opstype != OPSTYPE_FAT_CVT) {
    uint8_t recordlen = 0;

    /* only a stored CVT can have VLIR records */
    if (record != RECSEL_NONE) {
      set_error(ERROR_FILE_TYPE_MISMATCH);
      f_close(&buf->pvt.fat.fh);
      buf->read = 0;             /* not open: the caller frees it */
      unstick_buffer(buf);
      return;
    }

    /* A REL file: its header ends with the record length, the records */
    /* follow. That is the same for x00 files and for the others.      */
    if ((dent->typeflags & TYPE_MASK) == TYPE_REL) {
      UINT bytesread;

      if (dent->opstype != OPSTYPE_FAT_X00)
        buf->pvt.fat.headersize = 1;
      f_lseek(&buf->pvt.fat.fh, buf->pvt.fat.headersize - 1);
      f_read(&buf->pvt.fat.fh, &recordlen, 1, &bytesread);
    }

    /* Not a CVT: The first block of the stream is a signature block, the */
    /* file follows with the normal refill. Offsets of the file no longer */
    /* match the stream, so the seek callback is not usable.              */
    cvt_make_header(dent, buf, recordlen);
    buf->seek = NULL;

    /* nothing follows the signature block of an empty file */
    if (buf->pvt.fat.fh.fsize == buf->pvt.fat.headersize)
      buf->sendeoi = 1;
    return;
  }
#endif

#ifdef CONFIG_CVT
  /* the selected record instead of the first block; if that fails, */
  /* so does the open (error 50/64 is set)                            */
  if (record != RECSEL_NONE) {
    if (fat_cvt_seek(buf, record, 0)) {
      f_close(&buf->pvt.fat.fh);
      buf->read = 0;             /* not open: the caller frees it */
      unstick_buffer(buf);
    }
    return;
  }
#endif

  /* Call the refill once for the first block of data */
  buf->refill(buf);
}

/**
 * create_file - creates a file
 * @path     : path of the file
 * @dent     : name of the file
 * @type     : type of the file
 * @buf      : buffer to be used
 * @recordlen: length of record, if REL file.
 *
 * This function opens a file in the FAT filesystem for writing and sets up
 * buf to access it. type is ignored here because FAT has no equivalent of
 * file types.
 */
FRESULT create_file(path_t *path, cbmdirent_t *dent, uint8_t type, buffer_t *buf, uint8_t recordlen) {
  FRESULT res;
  uint8_t *name, *x00ext;

  x00ext = NULL;

  /* check if the FAT name is already defined (used only for M2I) */
#ifdef CONFIG_M2I
  if (dent->pvt.fat.realname[0])
    name = dent->pvt.fat.realname;
  else
#endif
  {
    ustrcpy(ops_scratch, dent->name);
    x00ext = build_name(ops_scratch, type, 0);
    name = ops_scratch;
  }

  partition[path->part].fatfs.curr_dir = path->dir.fat;

  res = f_open(&partition[path->part].fatfs, &buf->pvt.fat.fh,
               name, FA_WRITE | FA_CREATE_NEW | (recordlen ? FA_READ : 0));
  while (x00ext != NULL && res == FR_EXIST) {
    /* File exists, increment extension */
    *x00ext += 1;
    if (*x00ext == '9'+1) {
      *x00ext = '0';
      *(x00ext-1) += 1;
      if (*(x00ext-1) == '9'+1)
        break;
    }

    res = f_open(&partition[path->part].fatfs, &buf->pvt.fat.fh,
                 name, FA_WRITE | FA_CREATE_NEW | (recordlen ? FA_READ : 0));
  }

  if (res != FR_OK)
    return res;

  if (x00ext != NULL || recordlen) {
    UINT byteswritten;

    if(x00ext != NULL) {
      /* Write a [PSUR]00 header */

      memset(ops_scratch, 0, P00_HEADER_SIZE);
      ustrcpy_P(ops_scratch, p00marker);
      memcpy(ops_scratch+P00_CBMNAME_OFFSET, dent->name, CBM_NAME_LENGTH);
      if(recordlen)
        ops_scratch[P00_RECORDLEN_OFFSET] = recordlen;
      buf->pvt.fat.headersize = P00_HEADER_SIZE;
    } else if(recordlen) {
      ops_scratch[0] = recordlen;
      buf->pvt.fat.headersize = 1;
    }
    res = f_write(&buf->pvt.fat.fh, ops_scratch, buf->pvt.fat.headersize, &byteswritten);
    if (res != FR_OK || byteswritten != buf->pvt.fat.headersize) {
      return res;
    }
  }

  return FR_OK;
}

/**
 * fat_open_write - opens a file for writing
 * @path  : path of the file
 * @dent  : name of the file
 * @type  : type of the file
 * @buf   : buffer to be used
 * @append: Flags if the new data should be appended to the end of file
 *
 * This function opens a file in the FAT filesystem for writing and sets up
 * buf to access it. type is ignored here because FAT has no equivalent of
 * file types, except for TYPE_CVT: That file gets the extension .CVT. What
 * is written is not checked, it is assumed to be a CVT.
 */
void fat_open_write(path_t *path, cbmdirent_t *dent, uint8_t type, buffer_t *buf, uint8_t append) {
  FRESULT res;

  if (append) {
    partition[path->part].fatfs.curr_dir = path->dir.fat;
    res = f_open(&partition[path->part].fatfs, &buf->pvt.fat.fh, dent->pvt.fat.realname, FA_WRITE | FA_OPEN_EXISTING);
    if (dent->opstype == OPSTYPE_FAT_X00)
      /* It's a [PSUR]00 file */
      buf->pvt.fat.headersize = P00_HEADER_SIZE;
    if (res == FR_OK)
      res = f_lseek(&buf->pvt.fat.fh, buf->pvt.fat.fh.fsize);
    buf->fptr = buf->pvt.fat.fh.fsize - buf->pvt.fat.headersize;
  } else
    res = create_file(path, dent, type, buf, 0);

  if (res != FR_OK) {
    parse_error(res,0);
    return;
  }

  mark_write_buffer(buf);
  buf->position  = 2;
  buf->lastused  = 2;
  buf->cleanup   = fat_file_close;
  buf->refill    = fat_file_write;
  buf->seek      = fat_file_seek;

#ifdef CONFIG_CVT
  /* A CVT is written in one go, there is no position in it */
  if (type == TYPE_CVT)
    buf->seek = NULL;
#endif

  /* If no data is written the file should end up with a single 0x0d byte */
  buf->data[2] = 13;
}

/**
 * fat_open_rel - creates a rel file.
 * @path  : path of the file
 * @dent  : name of the file
 * @buf   : buffer to be used
 * @length: record length
 * @mode  : select between new or existing file
 *
 * This function opens a rel file and prepares it for access.
 * If the mode parameter is 0, create a new file. If it is != 0,
 * open an existing file.
 */
void fat_open_rel(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t length, uint8_t mode) {
  FRESULT res;
  UINT bytesread;

#ifdef CONFIG_CVT
  /* A REL CVT is not a REL file, it can only be read with OPENFLAG_CVT */
  if (dent->opstype == OPSTYPE_FAT_CVT) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return;
  }
#endif

  if(!mode) {
    res = create_file(path, dent, TYPE_REL, buf, length);
    bytesread = 1;
    ops_scratch[0] = length;
  } else {
    partition[path->part].fatfs.curr_dir = path->dir.fat;
    res = f_open(&partition[path->part].fatfs, &buf->pvt.fat.fh, dent->pvt.fat.realname, FA_WRITE | FA_READ | FA_OPEN_EXISTING);
    if (res == FR_OK) {
      if (dent->opstype == OPSTYPE_FAT_X00) {
        res = f_lseek(&buf->pvt.fat.fh, P00_RECORDLEN_OFFSET);
      }
      if(res == FR_OK)
        /* read record length */
        res = f_read(&buf->pvt.fat.fh, ops_scratch, 1, &bytesread);
      if(!length)
        length = ops_scratch[0];
    }
  }

  if (res != FR_OK || bytesread != 1) {
    parse_error(res,0);
    return;
  }

  buf->pvt.fat.headersize = (uint8_t)buf->pvt.fat.fh.fptr;
  buf->recordlen  = length;
  mark_write_buffer(buf);
  buf->read      = 1;
  buf->cleanup   = fat_file_close;
  buf->refill    = fat_file_sync;
  buf->seek      = fat_file_seek;

  /* read the first record */
  if (!fat_file_read(buf) && length != ops_scratch[0])
    set_error(ERROR_RECORD_MISSING);
}

/* ------------------------------------------------------------------------- */
/*  External interface for the various operations                            */
/* ------------------------------------------------------------------------- */

uint8_t fat_opendir(dh_t *dh, path_t *path) {
  FRESULT res;

  res = l_opendir(&partition[path->part].fatfs, path->dir.fat, &dh->dir.fat);
  dh->part = path->part;
  if (res != FR_OK) {
    parse_error(res,1);
    return 1;
  }
  return 0;
}

/**
 * fat_readdir - readdir wrapper for FAT
 * @dh  : directory handle as set up by opendir
 * @dent: CBM directory entry for returning data
 *
 * This function reads the next directory entry into dent.
 * Returns 1 if an error occured, -1 if there are no more
 * directory entries and 0 if successful.
 */
int8_t fat_readdir(dh_t *dh, cbmdirent_t *dent) {
  FRESULT res;
  FILINFO finfo;
  uint8_t *ptr,*nameptr;
  uint8_t typechar;
#ifdef CONFIG_CVT
  uint8_t cvt_candidate = 0;
#endif

  finfo.lfn = ops_scratch;

  do {
    res = f_readdir(&dh->dir.fat, &finfo);
    if (res != FR_OK) {
      if (res == FR_INVALID_OBJECT)
        set_error(ERROR_DIR_ERROR);
      else
        parse_error(res,1);
      return 1;
    }
  } while ((finfo.fname[0] && (finfo.fattrib & AM_VOL)) ||
           (finfo.fname[0] == '.' && finfo.fname[1] == 0) ||
           (finfo.fname[0] == '.' && finfo.fname[1] == '.' && finfo.fname[2] == 0));

  memset(dent, 0, sizeof(cbmdirent_t));

  if (!finfo.fname[0])
    return -1;

  dent->opstype = OPSTYPE_FAT;

  /* Copy name */
  ustrcpy(dent->pvt.fat.realname, finfo.fname);

  unsigned int maxLen = file_extension_mode == 5 ? 44 : CBM_NAME_LENGTH+4;  

  if (!finfo.lfn[0] || ustrlen(finfo.lfn) > maxLen) {
    nameptr = finfo.fname;
    if(file_extension_mode == 5) {
       fat_to_petscii((char*) nameptr, false, (char*) nameptr, 20, true);
    }
  } else {
    /* Convert only LFNs to PETSCII, 8.3 are always upper-case */
    nameptr = finfo.lfn;
    if(file_extension_mode == 5) {
       fat_to_petscii((char*) nameptr, false, (char*) nameptr, 20, true);
    }
    else
       asc2pet(nameptr);
  }

  /* File type */
  if (finfo.fattrib & AM_DIR) {
    dent->typeflags = TYPE_DIR;
    // /* Hide directories starting with . */
    //if (*nameptr == '.')
    //  dent->typeflags |= FLAG_HIDDEN;

  } else {
    /* Search for the file extension */
    exttype_t ext = check_extension(finfo.fname, &ptr);
#ifdef CONFIG_CVT
    cvt_candidate = (ext == EXT_IS_CVT);
#endif
    if (ext == EXT_IS_X00) {
      /* [PSRU]00 file - try to read the internal name */
      uint8_t *name = p00cache_lookup(dh->part, finfo.clust);
      typechar = *ptr;

      if (name != NULL) {
        /* lookup successful */
        memcpy(dent->name, name, CBM_NAME_LENGTH);
      } else {
        /* read name from file */
        UINT bytesread;

        res = l_opencluster(&partition[dh->part].fatfs, &partition[dh->part].imagehandle, finfo.clust);
        if (res != FR_OK)
          goto notp00;

        res = f_read(&partition[dh->part].imagehandle, ops_scratch, P00_HEADER_SIZE, &bytesread);
        if (res != FR_OK)
          goto notp00;

        if (memcmp_P(ops_scratch, p00marker, P00MARKER_LENGTH))
          goto notp00;

        /* Copy the internal name - dent->name is still zeroed */
        ustrcpy(dent->name, ops_scratch + P00_CBMNAME_OFFSET);

        /* Some programs pad the name with 0xa0 instead of 0 */
        ptr = dent->name;
        for (uint8_t i=0;i<16;i++,ptr++)
          if (*ptr == 0xa0)
            *ptr = 0;

        /* add name to cache */
        p00cache_add(dh->part, finfo.clust, dent->name);
      }
      finfo.fsize -= P00_HEADER_SIZE;
      dent->opstype = OPSTYPE_FAT_X00;

    } else if (ext == EXT_IS_TYPE && (globalflags & EXTENSION_HIDING)) {
      /* Type extension */
      typechar = *ptr;
      uint8_t i = ustrlen(nameptr)-4;
      nameptr[i] = 0;

#ifdef CONFIG_CVT
    } else if (ext == EXT_IS_CVT && (globalflags & EXTENSION_HIDING)) {
      /* Like the type extensions, a PRG until the content says otherwise */
      typechar = 'P';
      nameptr[ustrlen(nameptr)-4] = 0;
#endif
    } else { /* ext == EXT_UNKNOWN or EXT_IS_TYPE but hiding disabled */
      /* Unknown extension: PRG */
      typechar = 'P';
    }

  notp00:
    /* Set the file type */
    switch (typechar) {
    case 'P':
      dent->typeflags = TYPE_PRG;
      break;

    case 'S':
      dent->typeflags = TYPE_SEQ;
      break;

    case 'U':
      dent->typeflags = TYPE_USR;
      break;

    case 'R':
      dent->typeflags = TYPE_REL;
      break;
    }
  }

  /* Copy file name into dirent if it fits */
  if (dent->opstype != OPSTYPE_FAT_X00) {
    if (ustrlen(nameptr) > CBM_NAME_LENGTH) {
      ustrcpy(dent->name, finfo.fname);
    } else {
      ustrcpy(dent->name, nameptr);
      ptr = dent->name;
      while (*ptr) {
        if (*ptr == 0xa0)
          *ptr = 0;
        ptr++;
      }

    }

    ptr = dent->name;
    while (*ptr) {
      if (*ptr == '~') *ptr = 0xff;
      ptr++;
    }
  }

  if (finfo.fsize > 16255746)
    /* File too large -> size 63999 blocks */
    dent->blocksize = 63999;
  else
    dent->blocksize = (finfo.fsize+253) / 254;

  dent->remainder = finfo.fsize % 254;

  /* Read-Only and hidden flags */
  if (finfo.fattrib & AM_RDO)
    dent->typeflags |= FLAG_RO;

  if (finfo.fattrib & (AM_HID|AM_SYS))
    dent->typeflags |= FLAG_HIDDEN;

  /* Cluster number */
  dent->pvt.fat.cluster = finfo.clust;

  /* Date/Time */
  dent->date.year  = (finfo.fdate >> 9) + 80;
  dent->date.month = (finfo.fdate >> 5) & 0x0f;
  dent->date.day   = finfo.fdate & 0x1f;

  dent->date.hour   = finfo.ftime >> 11;
  dent->date.minute = (finfo.ftime >> 5) & 0x3f;
  dent->date.second = (finfo.ftime & 0x1f) << 1;

#ifdef CONFIG_CVT
  /* The extension only makes it a candidate, the CVT layer looks at the */
  /* content. If it isn't a CVT the entry stays a normal PRG file.       */
  if (cvt_candidate) {
    partition_t *part = &partition[dh->part];

    if (l_opencluster(&part->fatfs, &part->imagehandle, finfo.clust) == FR_OK &&
        !cvt_probe(fat_peek, &part->imagehandle, dent))
      dent->opstype = OPSTYPE_FAT_CVT;
  }
#endif

  return 0;
}

/**
 * fat_delete - Delete a file/directory on FAT
 * @path: path to the file/directory
 * @dent: pointer to cbmdirent with name of the file/directory to be deleted
 *
 * This function deletes the file filename in path and returns
 * 0 if not found, 1 if deleted or 255 if an error occured.
 */
uint8_t fat_delete(path_t *path, cbmdirent_t *dent) {
  FRESULT res;
  uint8_t *name;

  set_dirty_led(1);
  if (dent->pvt.fat.realname[0]) {
    name = dent->pvt.fat.realname;
    p00cache_invalidate();
  } else {
    name = dent->name;
    pet2asc(name);
  }
  partition[path->part].fatfs.curr_dir = path->dir.fat;
  res = f_unlink(&partition[path->part].fatfs, name);

  update_leds();

  parse_error(res,0);
  if (res == FR_OK)
    return 1;
  else if (res == FR_NO_FILE)
    return 0;
  else
    return 255;
}

/**
 * fat_chdir - change directory in FAT and/or mount image
 * @path: path object for the location of dirname
 * @dent: Name of the directory/image to be changed into,
 *        NULL to move one directory up (left arrow)
 *
 * This function changes the directory of the path object to dirname.
 * If dirname specifies a file with a known extension (e.g. M2I or D64), the
 * current(!) directory will be changed to the directory of the file and
 * it will be mounted as an image file. Returns 0 if successful,
 * 1 otherwise.
 */
uint8_t fat_chdir(path_t *path, cbmdirent_t *dent) {
  FRESULT res;

  partition[path->part].fatfs.curr_dir = path->dir.fat;

  /* A NULL dirent (left arrow) moves one directory up */
  if (dent == NULL) {
    FILINFO finfo;

    /* going up in the root dir is a no-op */
    if (path->dir.fat == 0)
      return 0;

    ops_scratch[0] = '.';
    ops_scratch[1] = '.';
    ops_scratch[2] = 0;

    res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
    if (res != FR_OK) {
      parse_error(res,1);
      return 1;
    }

    path->dir.fat = finfo.clust;
    return 0;
  }

  if (dent->name[0] == 0) {
    /* Empty string moves to the root dir */
    path->dir.fat = 0;
    return 0;
  }

  if ((dent->typeflags & TYPE_MASK) == TYPE_DIR) {
    /* It's a directory, change to its cluster */
    path->dir.fat = dent->pvt.fat.cluster;
  } else {
    /* Changing into a file, could be a mount request */
    if (check_imageext(dent->pvt.fat.realname) != IMG_UNKNOWN) {
      /* D64/M2I mount request */
      free_multiple_buffers(FMB_USER_CLEAN);
      /* Open image file */
      res = f_open(&partition[path->part].fatfs,
                   &partition[path->part].imagehandle,
                   dent->pvt.fat.realname, FA_OPEN_EXISTING|FA_READ|FA_WRITE);

      /* Try to open read-only if medium or file is read-only */
      if (res == FR_DENIED || res == FR_WRITE_PROTECTED)
        res = f_open(&partition[path->part].fatfs,
                     &partition[path->part].imagehandle,
                     dent->pvt.fat.realname, FA_OPEN_EXISTING|FA_READ);

      if (res != FR_OK) {
        parse_error(res,1);
        return 1;
      }

#ifdef CONFIG_M2I
      if (check_imageext(dent->pvt.fat.realname) == IMG_IS_M2I)
        partition[path->part].fop = &m2iops;
      else
#endif
        {
          if (d64_mount(path, dent->pvt.fat.realname))
            return 1;
          partition[path->part].fop = &d64ops;
        }

#ifdef CONFIG_LCD_DISPLAY
      DS_CD((char *)dent->pvt.fat.realname);
#endif

      return 0;
    }
  }
  return 0;
}

/* Create a new directory */
void fat_mkdir(path_t *path, uint8_t *dirname) {
  FRESULT res;

  partition[path->part].fatfs.curr_dir = path->dir.fat;
  if (file_extension_mode == 5)
  {
     petscii_to_fat((const char*)dirname, fatfs_yAbuffer, 40);
     memcpy(dirname, fatfs_yAbuffer, 45); 
  }
  else
     pet2asc(dirname);
  res = f_mkdir(&partition[path->part].fatfs, dirname);
  parse_error(res,0);
}

/**
 * fat_getvolumename - Get the volume label
 * @part : partition to request
 * @label: pointer to the buffer for the label (16 characters+zero-termination)
 *
 * This function reads the FAT volume label and stores it zero-terminated
 * in label. Returns 0 if successfull, != 0 if an error occured.
 */
static uint8_t fat_getvolumename(uint8_t part, uint8_t *label) {
  DIR dh;
  FILINFO finfo;
  FRESULT res;
  uint8_t i,j;

  finfo.lfn = NULL;
  memset(label, 0, CBM_NAME_LENGTH+1);

  res = l_opendir(&partition[part].fatfs, 0, &dh);

  if (res != FR_OK) {
    parse_error(res,1);
    return 1;
  }

  while ((res = f_readdir(&dh, &finfo)) == FR_OK) {
    if (!finfo.fname[0]) break;
    if ((finfo.fattrib & (AM_VOL|AM_SYS|AM_HID)) == AM_VOL) {
      i=0;
      j=0;
      while (finfo.fname[i]) {
        /* Skip dots */
        if (finfo.fname[i] == '.') {
          i++;
          continue;
        }
        label[j++] = finfo.fname[i++];
      }
      return 0;
    }
  }
  return 0;
}

/**
 * fat_getdirlabel - Get the directory label
 * @path : path object of the directory
 * @label: pointer to the buffer for the label (16 characters)
 *
 * This function reads the FAT volume label (if in root directory) or FAT
 * directory name (if not) and stored it space-padded
 * in the first 16 bytes of label.
 * Returns 0 if successfull, != 0 if an error occured.
 */
uint8_t fat_getdirlabel(path_t *path, uint8_t *label) {
  DIR dh;
  FILINFO finfo;
  FRESULT res;
  uint8_t *name = ops_scratch;

  finfo.lfn = ops_scratch;
  memset(label, ' ', CBM_NAME_LENGTH);

  res = l_opendir(&partition[path->part].fatfs, path->dir.fat, &dh);
  if (res != FR_OK)
    goto gl_error;

  while ((res = f_readdir(&dh, &finfo)) == FR_OK) {
    if(finfo.fname[0] == '\0' || finfo.fname[0] != '.') {
      res = fat_getvolumename(path->part, name);
      break;
    }
    if(finfo.fname[0] == '.' && finfo.fname[1] == '.' && finfo.fname[2] == 0) {
      if((res = l_opendir(&partition[path->part].fatfs,finfo.clust,&dh)) != FR_OK) // open .. dir.
        break;
      while ((res = f_readdir(&dh, &finfo)) == FR_OK) {
        if(finfo.fname[0] == '\0')
          break;
        if (finfo.clust == path->dir.fat) {
          if(!*name)
            name = finfo.fname;
          else
            asc2pet(name);
          break;
        }
      }
      break;
    }
  }

  if (*name)
    memcpy(label, name, ustrlen(name));

  if (res == FR_OK)
    return 0;

gl_error:
  parse_error(res,1);
  return 1;
}

/**
 * fat_getid - "Read" a disk id
 * @path: path object
 * @id  : pointer to the buffer for the id (5 characters)
 *
 * This function creates a disk ID from the FAT type (12/16/32)
 * and the usual " 2A" of a 1541 in the first 5 bytes of id.
 * Always returns 0 for success.
 */
uint8_t fat_getid(path_t *path, uint8_t *id) {
  switch (partition[path->part].fatfs.fs_type) {
  case FS_FAT12:
    *id++ = '1';
    *id++ = '2';
    break;

  case FS_FAT16:
    *id++ = '1';
    *id++ = '6';
    break;

  case FS_FAT32:
    *id++ = '3';
    *id++ = '2';
    break;
  }

  *id++ = ' ';
  *id++ = '2';
  *id++ = 'A';
  return 0;
}

/* Returns the number of free blocks */
uint16_t fat_freeblocks(uint8_t part) {
  FATFS *fs = &partition[part].fatfs;
  DWORD clusters;

  if (l_getfree(fs, NULLSTRING, &clusters, 65535) == FR_OK) {
    if (clusters > 65535)
      return 65535;
    else
      return clusters;
  } else
    return 0;
}


/**
 * fat_readwrite_sector - simulate direct sector access
 * @buf   : target buffer
 * @part  : partition number
 * @track : track to read
 * @sector: sector to read
 * @rwflag: read/write flag
 *
 * This function allows access to a file called bootsect.128
 * as track 1 sector 0 to enable the auto-boot function of
 * the C128 on FAT directories. If rwflag is false (0),
 * the sector will be written; otherwise it will be read.
 */
static void fat_readwrite_sector(buffer_t *buf, uint8_t part,
                                 uint8_t track, uint8_t sector, uint8_t rwflag) {
  FRESULT res;
  UINT bytes;
  uint8_t mode;

  if (track != 1 || sector != 0) {
    set_error_ts(ERROR_READ_NOHEADER, track, sector);
    return;
  }

  if (rwflag)
    mode = FA_OPEN_EXISTING | FA_READ;
  else
    mode = FA_OPEN_ALWAYS   | FA_WRITE;

  /* since this is fatops, the imagehandle is currently unused       */
  /* (saves a noticable chunk of stack compared to a local variable) */
  res = f_open(&partition[part].fatfs,
               &partition[part].imagehandle,
               (const UCHAR *)BOOTSECTOR_FILE,
               mode);
  if (res != FR_OK) {
    parse_error(res, rwflag);
    return;
  }

  if (rwflag)
    res = f_read(&partition[part].imagehandle, buf->data, 256, &bytes);
  else
    res = f_write(&partition[part].imagehandle, buf->data, 256, &bytes);

  if (res != FR_OK || bytes != 256)
    parse_error(res, rwflag);

  res = f_close(&partition[part].imagehandle);
  if (res != FR_OK)
    parse_error(res, rwflag);

  return;
}

/**
 * fat_read_sector - simulate direct sector reads
 * @buf   : target buffer
 * @part  : partition number
 * @track : track to read
 * @sector: sector to read
 *
 * Wrapper for fat_readwrite_sector in read mode
 */
void fat_read_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  fat_readwrite_sector(buf, part, track, sector, 1);
}

/**
 * fat_write_sector - simulate direct sector writes
 * @buf   : source buffer
 * @part  : partition number
 * @track : track to write
 * @sector: sector to write
 *
 * Wrapper for fat_readwrite_sector in write mode
 */
void fat_write_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  fat_readwrite_sector(buf, part, track, sector, 0);
}


/**
 * fat_rename - rename a file
 * @path   : path object
 * @dent   : pointer to cbmdirent with old file name
 * @newname: new file name
 *
 * This function renames the file in dent in the directory referenced by
 * path to newname. For an x00 file that is the name inside the file, for a
 * CVT it depends on what the directory shows: The name inside the CVT (the
 * file is then renamed after it, as far as possible), or just the file with
 * XEC+.
 */
#ifndef CONFIG_DISABLE_EC_AND_ET
static void fat_convert(path_t *path, cbmdirent_t *dent, uint8_t *newname);
#endif

void fat_rename(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  uint8_t *ext;
  FRESULT res;
  UINT byteswritten;

  partition[path->part].fatfs.curr_dir = path->dir.fat;

#ifdef CONFIG_CVT
  if (dent->opstype == OPSTYPE_FAT_CVT) {
    partition_t *part = &partition[path->part];
    uint8_t fsnames = globalflags & CVT_FSNAME;
    FILINFO finfo;

    /* With the names inside (XEC-) the rename is done when the name inside   */
    /* is changed. The file is only named after it so it can be told in the   */
    /* other mode or on a PC. That comes second and may fail without making   */
    /* the rename fail.                                                       */
    if (!fsnames) {
      res = f_open(&part->fatfs, &part->imagehandle, dent->pvt.fat.realname,
                   FA_WRITE|FA_OPEN_EXISTING);
      if (res == FR_OK) {
        if (cvt_set_name(fat_poke, &part->imagehandle, newname))
          res = FR_RW_ERROR;
        if (f_close(&part->imagehandle) != FR_OK)
          res = FR_RW_ERROR;
      }
      if (res != FR_OK) {
        parse_error(res, 0);
        return;
      }
    }

    /* The file is named after the new name, with the extension that makes  */
    /* it a CVT. If the FAT can't hold the name, that is an error only if   */
    /* the name of the file is the name shown.                              */
    ustrcpy(ops_scratch, newname);
    build_name(ops_scratch, TYPE_CVT, 1);
    if (!*ops_scratch) {
      if (fsnames)
        set_error_ts(ERROR_SYNTAX_JOKER,0,0);
      return;
    }

    /* It may have that name already (it differs in case only, or the name  */
    /* inside was not the name of the file), then there is nothing to do.   */
    res = f_stat(&part->fatfs, ops_scratch, &finfo);
    if (res != FR_OK || finfo.clust != dent->pvt.fat.cluster) {
      res = f_rename(&part->fatfs, dent->pvt.fat.realname, ops_scratch);
      if (res != FR_OK && fsnames)
        parse_error(res, 0);
    }
    return;
  }
#endif

  if (dent->opstype == OPSTYPE_FAT_X00) {
    uint8_t *x00ext = NULL;
    FILINFO finfo;
    /* [PSUR]00 rename, just change the internal file name */
    p00cache_invalidate();

    res = f_open(&partition[path->part].fatfs, &partition[path->part].imagehandle,
                 dent->pvt.fat.realname, FA_WRITE|FA_OPEN_EXISTING);
    if (res != FR_OK) {
      parse_error(res,0);
      return;
    }

    res = f_lseek(&partition[path->part].imagehandle, P00_CBMNAME_OFFSET);
    if (res != FR_OK) {
      parse_error(res,0);
      return;
    }

    /* Copy the new name into dent->name so we can overwrite all 16 bytes */
    memset(dent->name, 0, CBM_NAME_LENGTH);
    ustrcpy(dent->name, newname);

    res = f_write(&partition[path->part].imagehandle, dent->name, CBM_NAME_LENGTH, &byteswritten);
    if (res != FR_OK || byteswritten != CBM_NAME_LENGTH) {
      parse_error(res,0);
      return;
    }

    res = f_close(&partition[path->part].imagehandle);
    if (res != FR_OK) {
      parse_error(res,0);
      return;
    }

    /* 2. Rename physical file */
    ustrcpy(ops_scratch, newname);
    x00ext = build_name(ops_scratch, dent->typeflags & TYPE_MASK, 2);

    res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
    while (x00ext != NULL && res == FR_OK) {
      /* File exists, increment extension */
      *x00ext += 1;
      if (*x00ext == '9'+1) {
        *x00ext = '0';
        *(x00ext-1) += 1;
        if (*(x00ext-1) == '9'+1)
          break;
      }

      res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
    }
    
    if (x00ext != NULL)
       res = f_rename(&partition[path->part].fatfs, dent->pvt.fat.realname, ops_scratch);
  } else {
    switch (check_extension(dent->pvt.fat.realname, &ext)) {
    case EXT_IS_TYPE:
      /* Keep type extension */
      ustrcpy(ops_scratch, newname);
      build_name(ops_scratch, dent->typeflags & TYPE_MASK, 1);
      if (!*ops_scratch)
      {
         /* Not a FAT name: only an x00 file can have it, convert makes one */
#ifdef CONFIG_DISABLE_EC_AND_ET
         set_error_ts(ERROR_SYNTAX_JOKER,0,0);
#else
         fat_convert(path, dent, newname);
#endif
         return;
      }
      
      res = f_rename(&partition[path->part].fatfs, dent->pvt.fat.realname, ops_scratch);
      if (res != FR_OK)
        parse_error(res, 0);
      break;

    default:
      ustrcpy(ops_scratch, newname);
      build_name(ops_scratch, 0, 1);
      if (!*ops_scratch)
      {
         /* Not a FAT name: only an x00 file can have it, convert makes one */
#ifdef CONFIG_DISABLE_EC_AND_ET
         set_error_ts(ERROR_SYNTAX_JOKER,0,0);
#else
         fat_convert(path, dent, newname);
#endif
         return;
      }
      
      res = f_rename(&partition[path->part].fatfs, dent->pvt.fat.realname, ops_scratch);
      if (res != FR_OK)
        parse_error(res, 0);
      break;
    }
  }
}

/**
 * fatops_init - Initialize fatops module
 * @preserve_path: Preserve the current directory if non-zero
 *
 * This function will initialize the fatops module and force
 * mounting of the card. It can safely be called again if re-mounting
 * is required.
 */
void fatops_init(uint8_t preserve_path) {
  FRESULT res;
  uint8_t realdrive,drive,part;

  max_part = 0;
  drive = 0;
  part = 0;
  while (max_part < CONFIG_MAX_PARTITIONS && drive < MAX_DRIVES) {
    partition[max_part].fop = &fatops;

    /* Map drive numbers in just one place */
    realdrive = map_drive(drive);
    res=f_mount((realdrive * 16) + part, &partition[max_part].fatfs);

    if (!preserve_path)
      partition[max_part].current_dir.fat = 0;

    if (res == FR_OK)
      max_part++;

    if (res != FR_NOT_READY && res != FR_INVALID_OBJECT && part < 15 &&
        /* Don't try to mount partitions on an unpartitioned medium */
        !(res == FR_OK && part == 0))
      /* Try all partitions */
      part++;
    else {
      /* End of extended partition chain, try next drive */
      part = 0;
      drive++;
    }
  }

  if (!preserve_path) {
    current_part = 0;
    display_current_part(0);
    set_changelist(NULL, NULLSTRING);
    previous_file_dirent.name[0] = 0; // clear '*' file
  }

  /* Invalidate some caches */
  d64_invalidate();
  p00cache_invalidate();

#ifndef HAVE_HOTPLUG
  if (!max_part) {
    set_error_ts(ERROR_DRIVE_NOT_READY,0,0);
    return;
  }
#endif
}

/**
 * image_unmount - generic unmounting function for images
 * @part: partition number
 *
 * This function will clear all buffers, close the image file and
 * restore file operations to fatops. It can be used for unmounting
 * any image file types that don't require special cleanups.
 * Returns 0 if successful, 1 otherwise.
 */
uint8_t image_unmount(uint8_t part) {
  FRESULT res;

  free_multiple_buffers(FMB_USER_CLEAN);

  /* call D64 unmount function to handle BAM refcounting etc. */
  // FIXME: ops entry?
  if (partition[part].fop == &d64ops)
    d64_unmount(part);

#ifdef CONFIG_LCD_DISPLAY
  path_t path;
  path.part    = part;
  path.dir.fat = partition[part].current_dir.fat;
  fat_getdirlabel(&path, ops_scratch);
  fs_mode = 0;
  DS_CD((char*)ops_scratch);
  ///DS_TITLE;
#endif

  if (display_found) {
    /* Send current path to display */
    path_t path;

    path.part    = part;
    path.dir.fat = partition[part].current_dir.fat;
    fat_getdirlabel(&path, ops_scratch);
    display_current_directory(part, ops_scratch);
  }

  partition[part].fop = &fatops;
  res = f_close(&partition[part].imagehandle);
  if (res != FR_OK) {
    parse_error(res,0);
    return 1;
  }
  return 0;
}

/**
 * image_chdir - generic chdir for image files
 * @path: path object of the location of dirname
 * @dent: directory to be changed into, NULL to leave the image
 *
 * This function will ignore any dirent except NULL (left arrow)
 * and unmount the image in that case. It can be used as
 * chdir for all image types that don't support subdirectories
 * themselves. Returns 0 if successful, 1 otherwise.
 */
uint8_t image_chdir(path_t *path, cbmdirent_t *dent) {
  if (dent == NULL) {
    /* Unmount request */
    return image_unmount(path->part);
  }
  return 1;
}

/**
 * image_mkdir - generic mkdir for image files
 * @path   : path of the directory
 * @dirname: name of the directory to be created
 *
 * This function only sets an error message.
 */
void image_mkdir(path_t *path, uint8_t *dirname) {
  (void)path;
  (void)dirname;

  set_error(ERROR_SYNTAX_UNABLE);
  return;
}

/**
 * image_read - Seek to a specified image offset and read data
 * @part  : partition number
 * @offset: offset to be seeked to
 * @buffer: pointer to where the data should be read to
 * @bytes : number of bytes to read from the image file
 *
 * This function seeks to offset in the image file and reads bytes
 * byte into buffer. It returns 0 on success, 1 if less than
 * bytes byte could be read and 2 on failure.
 */
uint8_t image_read(uint8_t part, LONG offset, void *buffer, uint16_t bytes) {
  FRESULT res;
  UINT bytesread;

  if (offset != (LONG)-1) {
    res = f_lseek(&partition[part].imagehandle, offset);
    if (res != FR_OK) {
      parse_error(res,1);
      return 2;
    }
  }

  res = f_read(&partition[part].imagehandle, buffer, bytes, &bytesread);
  if (res != FR_OK) {
    parse_error(res,1);
    return 2;
  }

  if (bytesread != bytes)
    return 1;

  return 0;
}

/**
 * image_write - Seek to a specified image offset and write data
 * @part  : partition number
 * @offset: offset to be seeked to
 * @buffer: pointer to the data to be written
 * @bytes : number of bytes to read from the image file
 * @flush : Flags if written data should be flushed to disk immediately
 *
 * This function seeks to offset in the image file and writes bytes
 * byte into buffer. It returns 0 on success, 1 if less than
 * bytes byte could be written and 2 on failure.
 */
uint8_t image_write(uint8_t part, LONG offset, void *buffer, uint16_t bytes, uint8_t flush) {
  FRESULT res;
  UINT byteswritten;

  if (partition[part].imagetype & D64_IS_READLNLY) {
     set_error_ts(ERROR_WRITE_PROTECT,0,0);
     return 2;
  }

  if (offset != (LONG)-1) {
    res = f_lseek(&partition[part].imagehandle, offset);
    if (res != FR_OK) {
      parse_error(res,1);
      return 2;
    }
  }

  res = f_write(&partition[part].imagehandle, buffer, bytes, &byteswritten);
  if (res != FR_OK) {
    parse_error(res,0);
    return 2;
  }

  if (byteswritten != bytes)
    return 1;

  if (flush)
    f_sync(&partition[part].imagehandle);

  return 0;
}

/**
 * check_free - Check if a specified number of bytes is free on the partition
 * @part: partition number
 * @need: minimum number of free bytes needed
 *
 * This function checks if the minimum number of clusters required to hold the
 * specified number of bytes is free in the filesystem.
 *
 * Returns 0 if enough free storage is available, != 0 otherwise.
 */
static uint8_t check_free(uint8_t part, uint32_t need) {
  FRESULT res;
  DWORD have;

  need = need / (512 * partition[part].fatfs.csize) + 1;

  res = l_getfree(&partition[part].fatfs, NULLSTRING, &have, need);
  if (res != FR_OK) {
    parse_error(res,0);
    return 2;
  }

  return have < need;
}

/* Create D41, D71 or D81 image */
void fat_format_image(path_t *path, uint8_t *name, uint8_t *id) {
  (void)path;
  (void)name;
  (void)id;
  FRESULT  res;
  uint8_t  imagetype;
  uint8_t  *ext;
  uint32_t fsize;

  /* derive filename from name given to NEW: and store it in ops_scratch */
  ustrcpy(ops_scratch, name);

  ext = ustrrchr(name, '.');
  imagetype = check_imageext(name);

  if (ext != NULL && imagetype != IMG_UNKNOWN) {
    /* remove extension from name (which will be used as the disk label) */
    *ext = '\0';
  } else {
    /* no extension provided; default to .d64 */
    ext = ops_scratch + ustrlen(ops_scratch);
    if (ext > ops_scratch + sizeof(ops_scratch) - 5)
      ext = ops_scratch + sizeof(ops_scratch) - 5;
    ustrcpy(ext, ".D64");
    imagetype = IMG_IS_D41;
    ext = NULL;
  }

  if (id != NULL && ustrlen(id) != 2 && ustrlen(id) != 3 && imagetype != IMG_IS_DNP) {
    set_error(ERROR_SYNTAX_UNKNOWN);
    return;
  }

  if (id != NULL && imagetype == IMG_IS_DNP && ustrlen(id) != 3) {
    set_error(ERROR_SYNTAX_UNKNOWN);
    return;
  }

  partition[path->part].fatfs.curr_dir = path->dir.fat;
  /* ops_scratch contains the filename for the image */
  if (file_extension_mode == 5)
  {
     petscii_to_fat((const char*)ops_scratch, fatfs_yAbuffer, 40);
     memcpy(ops_scratch, fatfs_yAbuffer, 45); 
  }
  else
     pet2asc(ops_scratch);

  res = f_open(&partition[path->part].fatfs,
               &partition[path->part].imagehandle,
               ops_scratch, FA_OPEN_EXISTING|FA_READ|FA_WRITE);
  if (res == FR_NO_FILE) { // image file doesn't exists, create it
    /* Creating a new image requires a valid id */
    if (id == NULL || id[0] == '\0') {
      set_error(ERROR_SYNTAX_UNKNOWN);
      return;
    }

    switch (imagetype) {
      case IMG_IS_D41:
        fsize = D41_SIZE;
        break;
      case IMG_IS_D71:
        fsize = D71_SIZE;
        break;
      case IMG_IS_D81:
        fsize = D81_SIZE;
        break;
#ifdef CONFIG_D80D82
      case IMG_IS_D80:
        fsize = 533248;
        break;
      case IMG_IS_D82:
        fsize = 1066496;
        break;
#endif
      case IMG_IS_DNP:
        if (id != 0) fsize = 65536 * (100*(id[0]-'0') + 10*(id[1]-'0') + (id[2]-'0'));
        else fsize = 0;
        break;
      default:
        fsize = 0;
        break;
    }

    if (fsize == 0) {
      set_error(ERROR_FILE_NOT_FOUND);
      return;
    }

    if (check_free(path->part, fsize)) {
      set_error(ERROR_DISK_FULL);
      return;
    }

    res = f_open(&partition[path->part].fatfs,
                 &partition[path->part].imagehandle,
                 ops_scratch, FA_CREATE_NEW|FA_READ|FA_WRITE);

    if (res == FR_OK) {
      res = f_lseek(&partition[path->part].imagehandle, fsize);
      partition[path->part].imagehandle.fsize = fsize;
    }
  } else if (res == FR_OK) { // image file exists
    /* Don't overwrite the image if the extension was auto-appended */
    if (ext == NULL || imagetype == IMG_IS_DNP) {
      f_close(&partition[path->part].imagehandle);
      set_error(ERROR_FILE_EXISTS);
      return;
    }
  }

  if (res != FR_OK) {
    parse_error(res,1);
    return;
  }

  if (imagetype == IMG_IS_DNP) {
    f_close(&partition[path->part].imagehandle);
    return;
  }

  if (d64_mount(path, name)) {
    f_close(&partition[path->part].imagehandle);
    return;
  }

  partition[path->part].fop = &d64ops;
  partition[path->part].imagetype &= ~D64_IS_READLNLY;
  format(path, name, id);
  image_unmount(path->part);
}

static void fat_set_attrib(path_t *path, cbmdirent_t *dent, uint8_t attr)
{
  FRESULT res;
  uint8_t *name;
  uint8_t attrib = 0;
  if (attr & FLAG_HIDDEN) attrib |= AM_HID;
  if (attr & FLAG_RO) attrib |= AM_RDO;
  if (attr & 0x10) attrib |= AM_ARC;

  set_dirty_led(1);
  if (dent->pvt.fat.realname[0]) {
    name = dent->pvt.fat.realname;
    p00cache_invalidate();
  } else {
    name = dent->name;
    pet2asc(name);
  }
  partition[path->part].fatfs.curr_dir = path->dir.fat;
  res = f_chmod(&partition[path->part].fatfs, name, attrib, 3);

  update_leds();

  parse_error(res,0);
}

static void fat_set_headername(path_t *path, uint8_t *newname, uint8_t *newid)
{
   (void)path;
   (void)newname;
   (void)newid;
   set_error(ERROR_SYNTAX_UNABLE);
}

#ifndef CONFIG_DISABLE_EC_AND_ET
static void fat_change_type(path_t *path, cbmdirent_t *dent, uint8_t newtype) {
  uint8_t *x00ext;
  FRESULT  res;
  FILINFO  finfo;

#ifdef CONFIG_CVT
  /* EC:name,C/G: a stored CVT is already the form a GEOS file has on FAT */
  if (newtype & CHTYPE_CONVERT) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  /* The type inside the file: only a CVT has one, it is a byte of the     */
  /* signature block. For all other files the flag means nothing.          */
  if (newtype & CHTYPE_INNER) {
    newtype &= (uint8_t)~CHTYPE_INNER;

    if (dent->opstype == OPSTYPE_FAT_CVT && newtype != TYPE_CVT) {
      partition_t *part = &partition[path->part];

      part->fatfs.curr_dir = path->dir.fat;
      res = f_open(&part->fatfs, &part->imagehandle, dent->pvt.fat.realname,
                   FA_READ|FA_WRITE|FA_OPEN_EXISTING);
      if (res == FR_OK) {
        uint8_t err = cvt_set_type(fat_peek, fat_poke, &part->imagehandle, newtype);

        if (err == 1)
          res = FR_RW_ERROR;
        if (f_close(&part->imagehandle) != FR_OK && err == 0)
          res = FR_RW_ERROR;
      }
      if (res != FR_OK)
        parse_error(res, 0);
      return;
    }
  }

  /* x00 files can not be CVTs, the header is in the way. build_name would */
  /* make a .C00 of it.                                                    */
  if (newtype == TYPE_CVT && dent->opstype == OPSTYPE_FAT_X00) {
    set_error(ERROR_FILE_TYPE_MISMATCH);
    return;
  }
#endif

  partition[path->part].fatfs.curr_dir = path->dir.fat;
  p00cache_invalidate();

  ustrcpy(ops_scratch, dent->name);

  if (dent->opstype == OPSTYPE_FAT_X00)
  {
    x00ext = build_name(ops_scratch, newtype, 2) - 1;

    /* Find a free sequence number */
    res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
    while (res == FR_OK) {
      *x00ext += 1;
      if (*x00ext == '9'+1) {
        *x00ext = '0';
        *(x00ext-1) += 1;
        if (*(x00ext-1) == '9'+1) {
          set_error(ERROR_FILE_EXISTS);
          return;
        }
      }
      res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
    }
  }
  else
     x00ext = build_name(ops_scratch, newtype, 1);

  set_dirty_led(1);
  res = f_rename(&partition[path->part].fatfs, dent->pvt.fat.realname, ops_scratch);
  update_leds();

  if (res != FR_OK)
    parse_error(res, 0);
}

static uint8_t fat_find_tempname(path_t *path, uint8_t *name) {
  cbmdirent_t dummy;
  uint16_t    num;

  for (num = 1; num < 10000U; num++) {
    ustrcpy_P(name, PSTR("TMP"));
    /* append decimal number, max 6 digits fits the 80,<name>,0,0 message */
    {
      uint8_t  digits[6];
      uint8_t  i = 0;
      uint32_t v = num;
      uint8_t *p = name + 3;

      do {
        digits[i++] = '0' + (v % 10);
        v /= 10;
      } while (v);
      while (i)
        *p++ = digits[--i];
      *p = 0;
    }

    if (first_match(path, name, FLAG_HIDDEN, &dummy) != 0) {
      /* not found -> name is free */
      set_error(ERROR_OK);
      return 0;
    }
  }

  return 1;
}

static void fat_convert(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  uint8_t tempname[CBM_NAME_LENGTH+1];
  uint8_t needs_copy;
  uint8_t *target;
  FRESULT res;
  FILINFO finfo;
  uint8_t *x00ext = NULL;

  if (newname == NULL) {
    if (file_extension_mode == 1 || file_extension_mode == 2) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }

    /* ---- ec:foo -> TMP<n> ---- */
    if (fat_find_tempname(path, tempname)) {
      set_error(ERROR_FILE_EXISTS);
      return;
    }
    target     = tempname;
    /* x00 files must lose their header, so copy&delete */
    needs_copy = (dent->opstype == OPSTYPE_FAT_X00);
  } else {
    if (dent->opstype == OPSTYPE_FAT_X00) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }    
    
    /* ---- ec:foo=TMP<n> ---- */
    target = newname;

    /* Does the target representation require an x00 header?
       build_name() tells us by returning a non-NULL x00 extension. */
    ustrcpy(ops_scratch, newname);
#ifdef CONFIG_CVT
    /* A CVT is the same in every mode and never an x00 file, whatever type */
    /* its entry has: the entry says if it is one.                          */
    if (dent->opstype == OPSTYPE_FAT_CVT)
      x00ext = build_name(ops_scratch, TYPE_CVT, 0);
    else
#endif
    x00ext = build_name(ops_scratch, dent->typeflags & TYPE_MASK, 0);
    needs_copy = x00ext != NULL;
  }

  if (!needs_copy) {
    /* Plain rename is sufficient */
    fat_rename(path, dent, target);
  } else {
    FIL destination;
    UINT read_count, write_count;
    uint8_t copy_buffer[128];

    partition[path->part].fatfs.curr_dir = path->dir.fat;
    if (x00ext) {
      res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
      while (res == FR_OK) {
        *x00ext += 1;
        if (*x00ext == '9'+1) {
          *x00ext = '0';
          *(x00ext-1) += 1;
          if (*(x00ext-1) == '9'+1) {
            set_error(ERROR_FILE_EXISTS);
            return;
          }
        }
        res = f_stat(&partition[path->part].fatfs, ops_scratch, &finfo);
      }
      if (res != FR_NO_FILE) {
        parse_error(res, 0);
        return;
      }
    } else {
      /* Converting an x00 file to a plain FAT file. */
      ustrcpy(ops_scratch, target);
      build_name(ops_scratch, dent->typeflags & TYPE_MASK, 1);
      if (!*ops_scratch) {
        set_error_ts(ERROR_SYNTAX_JOKER, 0, 0);
        return;
      }
    }

    p00cache_invalidate();
    set_dirty_led(1);
    res = f_open(&partition[path->part].fatfs, &partition[path->part].imagehandle,
                 dent->pvt.fat.realname, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK) goto copy_error;

    res = f_open(&partition[path->part].fatfs, &destination,
                 ops_scratch, FA_WRITE | FA_CREATE_NEW);
    if (res != FR_OK) {
      f_close(&partition[path->part].imagehandle);
      goto copy_error;
    }

    if (x00ext) {
      /* The x00 header contains the original Commodore name. */
      memset(copy_buffer, 0, P00_HEADER_SIZE);
      ustrcpy_P(copy_buffer, p00marker);
      ustrcpy(copy_buffer + P00_CBMNAME_OFFSET, newname);
      res = f_write(&destination, copy_buffer, P00_HEADER_SIZE, &write_count);
      if (res == FR_OK && write_count != P00_HEADER_SIZE) res = FR_RW_ERROR;
    } else {
      if (partition[path->part].imagehandle.fsize < P00_HEADER_SIZE)
        res = FR_RW_ERROR;
      else
        res = f_lseek(&partition[path->part].imagehandle, P00_HEADER_SIZE);
    }

    while (res == FR_OK) {
      res = f_read(&partition[path->part].imagehandle, copy_buffer,
                   sizeof(copy_buffer), &read_count);
      if (res != FR_OK || read_count == 0) break;
      res = f_write(&destination, copy_buffer, read_count, &write_count);
      if (res == FR_OK && write_count != read_count) res = FR_RW_ERROR;
    }

    {
      FRESULT close_res = f_close(&destination);
      if (res == FR_OK) res = close_res;
    }
    {
      FRESULT close_res = f_close(&partition[path->part].imagehandle);
      if (res == FR_OK) res = close_res;
    }
    if (res != FR_OK) {
      f_unlink(&partition[path->part].fatfs, ops_scratch);
      goto copy_error;
    }

    res = f_unlink(&partition[path->part].fatfs, dent->pvt.fat.realname);
    if (res != FR_OK) goto copy_error;
    update_leds();
    set_error(ERROR_OK);
    goto copy_done;

copy_error:
    update_leds();
    parse_error(res, 0);
    return;
  }

copy_done:
  if (current_error != 0 && current_error != ERROR_OK)
    return;

  if (newname == NULL)
    /* Report the generated name: 80,<name>,0,0 */
    set_error_text(80, tempname);
}
#endif

#ifdef CONFIG_EINFO
/* ------------------------------------------------------------------------- */
/*  E?: what a file is on FAT                                                */
/* ------------------------------------------------------------------------- */

#ifndef CONFIG_EINFO_WITHOUT_NAMES
/* A physical (ASCII) FAT name as PETSCII, after a ':'. ~ becomes * and */
/* anything that is no ASCII ? (none of them can be in a FAT name).     */
static uint8_t *put_fatname(uint8_t *out, const uint8_t *name) {
  uint8_t c;

  *out++ = ':';
  while ((c = *name++)) {
    if (c >= 0x80)
      c = '?';
    else if (c == '~')
      c = '*';
    else if (c >= 'a' && c <= 'z')
      c -= 0x20;
    else if (c >= 'A' && c <= 'Z')
      c += 0x80;
    *out++ = c;
  }
  return out;
}

/* a equals the first len characters of b (and has len characters) */
static uint8_t fatname_eq(const uint8_t *a, const uint8_t *b, uint8_t len, uint8_t nocase) {
  uint8_t x, y;

  while (len--) {
    x = *a++;
    y = *b++;
    if (nocase) {
      x = toupper(x);
      y = toupper(y);
    }
    if (x != y || !x)
      return 0;
  }
  return *a == 0;
}

/* The name has a type extension (.PRG/.SEQ/.USR/.REL/.CVT, any case) */
static uint8_t has_type_ext(const uint8_t *name) {
  uint8_t e[5], *ext, i;

  ext = ustrrchr(name, '.');
  if (ext == NULL || ustrlen(ext) != 4)
    return 0;
  for (i = 0; i < 5; i++)
    e[i] = toupper(ext[i]);
  i = check_extension(e, &ext);
  return i == EXT_IS_TYPE
#ifdef CONFIG_CVT
      || i == EXT_IS_CVT
#endif
    ;
}

#endif /* !CONFIG_EINFO_WITHOUT_NAMES */

#ifndef CONFIG_EINFO_WITHOUT_NAMES
/* The names part of E? on FAT, appended at p: physical names, NK, N/A */
static uint8_t describe_names(path_t *path, cbmdirent_t *dent, uint8_t *typed,
                              uint8_t *p, exttype_t et, uint8_t isdir) {
  partition_t *part = &partition[path->part];
  uint8_t *sfn = dent->pvt.fat.realname;
  uint8_t *tmp = p + 120;        /* build_name needs 45 bytes */
  uint8_t *phys;
  uint8_t xe5 = (file_extension_mode == 5);
  uint8_t haslfn, canon, len;
  FILINFO finfo;
  DIR dj;
  FRESULT res;

  /* The LFN, if there is one: dent only has the 8.3 name. f_stat does */
  /* not return a LFN, only f_readdir does: look for the 8.3 name.      */
  res = l_opendir(&part->fatfs, path->dir.fat, &dj);
  finfo.lfn = ops_scratch;
  while (res == FR_OK) {
    res = f_readdir(&dj, &finfo);
    if (res == FR_OK && !finfo.fname[0])
      res = FR_NO_FILE;
    if (res == FR_OK && !ustrcmp(finfo.fname, sfn))
      break;
  }
  if (res != FR_OK) {
    parse_error(res, 1);
    return 1;
  }
  haslfn = (ops_scratch[0] != 0);
  phys   = haslfn ? ops_scratch : sfn;

  p = put_fatname(p, phys);
  if (haslfn && !fatname_eq(ops_scratch, sfn, ustrlen(sfn), 1))
    p = put_fatname(p, sfn);

  /* Is it the name sd2iec would make? */
  if (et == EXT_IS_X00
#ifdef CONFIG_CVT
      /* XEC-: the name shown is the one inside the CVT, not the file's */
      || (dent->opstype == OPSTYPE_FAT_CVT && !(globalflags & CVT_FSNAME))
#endif
     ) {
    canon = 2;
  } else if (!haslfn && !xe5) {
    /* The name matched modulo case. Upper case would need a LFN. */
    canon = 1;
    while (*typed) {
      uint8_t c = *typed++;
      if ((c >= 0xc1 && c <= 0xda) || (c >= 0x61 && c <= 0x7a))
        canon = 0;
    }
  } else if (xe5 && !isdir && !has_type_ext(phys)) {
    canon = 0;                   /* xe5 always writes a type extension */
  } else {
    ustrcpy(tmp, dent->name);
    build_name(tmp, 0, 1);       /* no type, no x00: no extension */
    len = ustrlen(phys);
    canon = *tmp &&
      (fatname_eq(tmp, phys, len, xe5) ||
       (has_type_ext(phys) && fatname_eq(tmp, phys, len - 4, xe5)));
  }

  if (canon == 0)
    ustrcpy_P(p, PSTR(":NK"));
  else if (canon == 2)
    ustrcpy_P(p, PSTR(":N/A"));
  else
    *p = 0;

  return 0;
}
#endif

/**
 * fat_describe - what a file is on FAT, for E?
 * @path : path of the file
 * @dent : directory entry of the file
 * @typed: the name as it was given in the command
 * @out  : the text goes here (at least 256 bytes)
 *
 * The text is <kind>:<LFN>[:<8.3>][:NK|:N/A]. Kind is the type extension
 * (PRG/SEQ/USR/REL), NOTYPE (none: a PRG), X00, CVT with SEQ/VLIR/REL,
 * V1/V2 and I (INFO block), CVT? (.CVT that is no CVT) or DIR. The names
 * are the physical ones (8.3 only if there is no LFN or it differs).
 * NK: the name is not the one sd2iec would make (build_name without type
 * and x00, modulo a type extension; .prg may be implicit, with xe5 a file
 * needs one). Without LFN only the case can be wrong: 8.3 can only hold
 * PETSCII lower case. x00 is N/A, every system has its own idea there, as
 * is a CVT with XEC- (the name shown is the one inside).
 * With CONFIG_EINFO_WITHOUT_NAMES only the kind is given.
 * Returns 0 on success, non-zero if the file can't be examined.
 */
uint8_t fat_describe(path_t *path, cbmdirent_t *dent, uint8_t *typed, uint8_t *out) {
  partition_t *part = &partition[path->part];
  uint8_t *sfn = dent->pvt.fat.realname;
  uint8_t *ext;
  uint8_t isdir = (dent->typeflags & EXT_TYPE_MASK) == TYPE_DIR;
  exttype_t et;
#ifdef CONFIG_CVT
  FRESULT res;
#endif

  part->fatfs.curr_dir = path->dir.fat;

  /* The kind first: the CVT layer uses ops_scratch, the LFN goes there */
  et = check_extension(sfn, &ext);
  if (isdir) {
    ustrcpy_P(out, PSTR("DIR"));
  } else if (et == EXT_IS_X00) {
    ustrcpy_P(out, PSTR("X00"));
#ifdef CONFIG_CVT
  } else if (dent->opstype == OPSTYPE_FAT_CVT) {
    uint8_t f = CVTINFO_NONE;
    uint8_t *p;

    res = f_open(&part->fatfs, &part->imagehandle, sfn, FA_READ | FA_OPEN_EXISTING);
    if (res != FR_OK) {
      parse_error(res, 1);
      return 1;
    }
    f = cvt_info(fat_peek, &part->imagehandle);
    f_close(&part->imagehandle);

    p = out;
    ustrcpy_P(p, PSTR("CVT "));
    p += 4;
    if (f & CVTINFO_REL)
      ustrcpy_P(p, PSTR("REL"));
    else if (f & CVTINFO_VLIR)
      ustrcpy_P(p, PSTR("VLIR"));
    else
      ustrcpy_P(p, PSTR("SEQ"));
    p += ustrlen(p);
    ustrcpy_P(p, (f & CVTINFO_V2) ? PSTR(" V2") : PSTR(" V1"));
    if (f & CVTINFO_INFO)
      ustrcpy_P(p + 3, PSTR(" I"));
  } else if (et == EXT_IS_CVT) {
    ustrcpy_P(out, PSTR("CVT?"));
#endif
  } else if (et == EXT_IS_TYPE) {
    out[0] = ext[0];
    out[1] = ext[1];
    out[2] = ext[2];
    out[3] = 0;
  } else {
    ustrcpy_P(out, PSTR("NOTYPE"));
  }

#ifndef CONFIG_EINFO_WITHOUT_NAMES
  return describe_names(path, dent, typed, out + ustrlen(out), et, isdir);
#else
  (void)typed;
  return 0;
#endif
}
#endif

const PROGMEM fileops_t fatops = {  // These should be at bottom, to be consistent with d64ops and m2iops
  &fat_open_read,
  &fat_open_write,
  &fat_open_rel,
  &fat_delete,
  &fat_getvolumename,
  &fat_getdirlabel,
  &fat_getid,
  &fat_freeblocks,
  &fat_read_sector,
  &fat_write_sector,
  &fat_format_image,
  &fat_opendir,
  &fat_readdir,
  &fat_mkdir,
  &fat_chdir,
  &fat_rename,
  &fat_set_attrib,
#ifdef CONFIG_DISABLE_EC_AND_ET
  &fat_set_headername
#else
  &fat_set_headername,
  &fat_change_type,
  &fat_convert
#endif
};
