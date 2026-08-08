/*-
 * Copyright (c) 2015, 2017 Nils Eilers. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// LCD menu system

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

#include "config.h"
#include "lcd.h"
#include "led.h"
#include "menu.h"
#include "timer.h"
#include "errormsg.h"
#include "bus.h"
#include "eeprom-conf.h"
#include "rtc.h"
#include "ustring.h"
#include "doscmd.h"
#include "uart.h"
#include "dirent.h"
#include "parser.h"     // current_part
#include "wrapops.h"
#include "d64ops.h"     // d64ops (image-context detection)
#include "fatops.h"     // pet2ascn()

void asc2pet(uint8_t *buf);   // from utils.h (not included: its min/max
                              // macros collide here)
void pet2asc(uint8_t *buf);   // from fatops.c (fatops.h declares only pet2ascn)
#include "doscmd.h"


uint8_t menu_system_enabled = true;
uint8_t jump_out_mainmenu = 0;

#ifndef CONFIG_DIR_BUFFERS
#define CONFIG_DIR_BUFFERS 10  // ~70 entries/dir (7 per buffer). Interim
                               // until the windowed browser; safe with the
                               // overflow guard. Uses ~21 of 32 buffers.
#endif

// Windowed directory browser: see menu_browse_files() + fatops.c browse_fat_*.
// Nav rows sit above the file list at virtual coords -nav_rows..-1
// (Exit = -nav_rows, Parent = -1 when nav_rows == 2); files at 0..win_count-1.
#define CAT_DIR   BROWSE_CAT_DIR
#define CAT_IMAGE BROWSE_CAT_IMAGE
#define CAT_FILE  BROWSE_CAT_FILE
#define WIN_OVERHANG 1
#define WIN_SIZE (LCD_LINES + 2*WIN_OVERHANG)   // cached file entries
#define NAV_EXIT   0
#define NAV_PARENT 1
#define SEL_FILE   2


static tick_t lcd_timeout;
static bool lcd_timer;
static uint16_t lcd_current_screen;
// windowed-browser state: small scalars stay static; the bulky arrays live
// in browse_state_t on menu_browse_files() stack (see bstate below).
static uint8_t  win_count;
static bool     win_at_start, win_at_end;
static uint8_t  sel_kind;                 // NAV_EXIT | NAV_PARENT | SEL_FILE
static uint8_t  curs_line;                // LCD row of the highlight
static int8_t   top_v;                    // virtual coord shown on LCD row 0
static uint8_t  nav_rows;                  // nav rows above the list: 1 (root: Exit) or 2 (Exit+Parent)
static path_t   browse_path;              // directory being listed
static bool     browse_is_fat;            // FAT (sorted) vs image (native)
static uint8_t  scroll_off;
static tick_t   scroll_next;
// The highlighted entry's full long name is fetched on demand into the shared
// ops_scratch for scrolling (window entries store only the first BROWSE_NAME_MAX
// chars). scroll_full_ready marks ops_scratch valid for the current selection.
static bool     scroll_full_ready;

// Position restore: remember which child we descended from so we can re-highlight
// it after returning up (out of a subdir OR unmounting an image). One record per
// nesting level; identified by cluster in a FAT parent, by native offset in an image.
#define RET_MAX 8
typedef struct {
  bool     is_fat;    // parent was FAT (match by cluster) vs image (match by offset)
  uint32_t cluster;   // FAT parent: descended child's cluster
  uint16_t offset;    // image parent: descended child's native offset
} retpos_t;

// All bulky browser state (~440 B). menu_browse_files() puts ONE of these on
// its stack and points bstate at it, so this RAM is only used while the menu is
// open - during normal bus operation it costs nothing (just the 2-byte pointer).
typedef struct {
  browse_entry_t win[WIN_SIZE];   // window of cached entries (was browse_win[])
  browse_entry_t anchor;          // highlighted entry (was sel_anchor)
  retpos_t       rstack[RET_MAX]; // descend history (was ret_stack[])
  retpos_t       rpending;        // position to restore next load (was ret_pending)
} browse_state_t;
static browse_state_t *bstate;    // valid ONLY during menu_browse_files()
static uint8_t  ret_depth;                // true nesting depth (may exceed RET_MAX)
static bool     ret_pending_valid;


static inline uint8_t min(uint8_t a, uint8_t b) {
  if (a < b) return a;
  return b;
}


void lcd_update_device_addr(void) {
  if (lcd_current_screen == SCRN_STATUS) {
    lcd_home();
    lcd_printf("Device ID: #%d ", device_address);
  }
}


void lcd_update_disk_status(void) {
  bool visible = true;

  if (lcd_current_screen == SCRN_STATUS) {
    lcd_locate(0, 1);
    lcd_puts_P(PSTR("Status: "));
    for (uint8_t i = 0;
         i < min(CONFIG_ERROR_BUFFER_SIZE, LCD_COLS * (LCD_LINES > 3 ? 3:
             1) - 8);
         i++)
    {
      if (error_buffer[i] == 13) visible = false;
      lcd_putc(visible ? error_buffer[i] : ' ');
    }
  }
}


void lcd_draw_screen(uint16_t screen) {
  extern const char PROGMEM versionstr[];

  lcd_current_screen = screen;
  lcd_clear();

  switch (screen) {
  case SCRN_SPLASH:
    lcd_puts_P(versionstr);
    lcd_locate(0,3); lcd_puts_P(PSTR(HWNAME));
    // TODO: if available, print serial number here
    break;

  case SCRN_STATUS:
    lcd_locate(16, 0);
    if (active_bus == IEC)     lcd_puts_P(PSTR(" IEC"));
    if (active_bus == IEEE488) lcd_puts_P(PSTR("IEEE"));
    lcd_update_device_addr();
    lcd_update_disk_status();
    break;

  default:
    break;
  }
}


void lcd_refresh(void) {
  lcd_draw_screen(lcd_current_screen);
}


void lcd_splashscreen(void) {
  lcd_timeout = getticks() + MS_TO_TICKS(1000 * 5);
  lcd_timer = true;
}


static void menu_select_status(void) {
  lcd_draw_screen(SCRN_STATUS);
  lcd_timer = false;
}


void handle_lcd(void) {
  tick_t ticks;

  if (lcd_timer) {
    ticks = getticks();
    if (time_before(lcd_timeout, ticks)) menu_select_status();
  }
}


bool handle_buttons(void) {
  if (!menu_system_enabled) return false;
  uint8_t buttons = get_key_press(KEY_ANY);
  if (!buttons) return false;
  if (buttons & KEY_PREV) {
    // If there's an error, PREV clears the disk status
    // If not, enter menu system just as any other key
    if (current_error != ERROR_OK) {
      set_error(ERROR_OK);
      return false;
    }
  }
  return menu();
}


int8_t menu_vertical(uint8_t min, uint8_t max) {
  uint8_t pos = min;

  lcd_cursor(true);
  for (;;) {
    lcd_locate(0, pos);
    if (get_key_autorepeat(KEY_PREV)) {
      if (pos > min) --pos;
      else pos = max;
    }
    if (get_key_autorepeat(KEY_NEXT)) {
      if (pos < max) ++pos;
      else pos = min;
    }
    if (get_key_press(KEY_SEL)) break;
  }
  lcd_cursor(false);
  return pos;
}


uint8_t menu_edit_value(uint8_t v, uint8_t min, uint8_t max) {
  uint8_t x = lcd_x;
  uint8_t y = lcd_y;

  lcd_locate(x, y);
  lcd_cursor(true);
  set_busy_led(true);
  for (;;) {
    lcd_printf("%02d", v);
    lcd_locate(x, y);
    for (;;) {
      if (get_key_autorepeat(KEY_PREV)) {
        if (v <= min) v = max;
        else --v;
        break;
      }
      if (get_key_autorepeat(KEY_NEXT)) {
        if (v >= max) v = min;
        else ++v;
        break;
      }
      if (get_key_press(KEY_SEL)) {
        lcd_cursor(false);
        set_busy_led(false);
        return v;
      }
    }
  }
}


void menu_ask_store_settings(void) {
  lcd_clear();
  lcd_puts_P(PSTR("Save settings?\n\nyes\nno"));
  if (menu_vertical(2,3) == 2) {
    write_configuration();
  }
}

#ifdef HAVE_DUAL_INTERFACE
void menu_select_bus(void) {
  lcd_clear();
  lcd_puts_P(PSTR("IEC\nIEEE-488"));
  active_bus = menu_vertical(0, 1);
  menu_ask_store_settings();
}
#else
static inline void menu_select_bus(void) {}
#endif


void menu_device_number(void) {
  lcd_printf("Change device number\nfrom %02d to:", device_address);
  lcd_locate(12, 1);
  device_address = menu_edit_value(device_address, 8, 30);
  menu_ask_store_settings();
}


#ifdef HAVE_RTC
static const PROGMEM uint8_t menu_setclk_pos[] = {0, 3, 9, 12, 15, 18, 0, 8};
#endif
static const PROGMEM uint8_t monthnames[] = "JANFEBMARAPRMAYJUNJULAUGSEPOCTNOVDEC";
static const PROGMEM uint8_t number_of_days[] = {
  31, // January
  28, // February
  31, // March
  30, // April
  31, // May
  30, // June
  31, // July
  31, // August
  30, // September
  31, // October
  30, // November
  31  // December
};

enum set_clock_fields { SETCLK_MDAY, SETCLK_MON, SETCLK_YEAR,
  SETCLK_HOUR, SETCLK_MIN, SETCLK_SEC, SETCLK_SET, SETCLK_ABORT };


void menu_print_month(uint8_t m) {
  const char* p = (const char*) monthnames;

  p += m*3;
  lcd_putc(pgm_read_byte(p++));
  lcd_putc(pgm_read_byte(p++));
  lcd_putc(pgm_read_byte(p));
}


uint8_t menu_edit_month(uint8_t m) {
  for (;;) {
    lcd_locate(3, 0);
    menu_print_month(m);
    lcd_locate(3, 0);
    set_busy_led(true);
    lcd_cursor(true);
    for (;;) {
      if (get_key_autorepeat(KEY_PREV)) {
        if (m == 0) m = 11;
        else --m;
        break;
      }
      if (get_key_autorepeat(KEY_NEXT)) {
        if (m == 11) m = 0;
        else ++m;
        break;
      }
      if (get_key_press(KEY_SEL)) {
        set_busy_led(false);
        lcd_cursor(false);
        return m;
      }
    }
  }
}


uint8_t calc_number_of_days(uint8_t month, uint8_t year) {
  uint8_t days = pgm_read_byte(&number_of_days[month]);
  if ((month == 1) && (year % 4 == 0)) ++days;
  return days;
}


void menu_set_clock(void) {
#ifdef HAVE_RTC
  struct tm t;
  uint8_t p;
  uint8_t days;


  switch (rtc_state) {
    case RTC_INVALID:
      memset(&t, 0, sizeof(t));
      t.tm_mday = 1;
      t.tm_year = 115;
      break;

    case RTC_OK:
      read_rtc(&t);
      break;

    case RTC_NOT_FOUND:
    default:
      return;
  }

  lcd_printf("%02d-MMM-20%2d %02d:%02d:%02d",
      t.tm_mday, t.tm_year - 100, t.tm_hour, t.tm_min, t.tm_sec);
  lcd_locate(3, 0);
  menu_print_month(t.tm_mon);

  p = SETCLK_MDAY;
  for (;;) {
    if (p < SETCLK_SET) {
      lcd_locate(pgm_read_byte(&(menu_setclk_pos[p])), 0);
      switch (p++) {
        case SETCLK_MDAY:
          t.tm_mday = menu_edit_value(t.tm_mday, 1, 31);
          continue;
        case SETCLK_MON:
          t.tm_mon = menu_edit_month(t.tm_mon);
          days = calc_number_of_days(t.tm_mon, t.tm_year);
          if (t.tm_mon == 1) ++days; // Could be a leap year
          if (t.tm_mday > days) p = SETCLK_MDAY;
          continue;
        case SETCLK_YEAR:
          t.tm_year = menu_edit_value(t.tm_year - 100, 15, 99) + 100;
          days = calc_number_of_days(t.tm_mon, t.tm_year);
          if (t.tm_mday > days) p = SETCLK_MDAY;
          continue;
        case SETCLK_HOUR:
          t.tm_hour = menu_edit_value(t.tm_hour, 0, 23);
          continue;
        case SETCLK_MIN:
          t.tm_min = menu_edit_value(t.tm_min, 0, 59);
          continue;
        case SETCLK_SEC:
          t.tm_sec = menu_edit_value(t.tm_sec, 0, 59);
          continue;
      }
    } else {
      lcd_locate(0,1);
      lcd_puts_P(PSTR("Write to RTC\nEdit again\nAbort"));
      uint8_t sel = menu_vertical(1,3);
      lcd_clrlines(1,3);
      switch (sel) {
        case 1:         // Set time
          t.tm_wday = day_of_week(t.tm_year, t.tm_mon + 1, t.tm_mday);
          set_rtc(&t);

        // fall through

        case 3:         // Abort
          return;

        default:        // Edit date & time
          p = SETCLK_MDAY;
      }
    }
  }
#else
  return;               // no RTC
#endif
}

// lcd_print_dir_entry() now lives in the windowed browser below.

void clear_command_buffer(void) {
  memset(command_buffer, 0, sizeof(command_buffer));
  command_length = 0;
}

// compare()/qsort removed: the windowed browser fetches entries in sorted
// order on demand (fatops.c browse_fat_*), so nothing is sorted in RAM.


static void rom_menu_browse(uint8_t y) {
  switch (y) {
    case 0: lcd_puts_P(PSTR("Exit menu\n")); break;
    case 1: lcd_puts_P(PSTR("Change to parent dir\n")); break;
    default: printf("Internal error: rom_menu_browse(%d)\r\n", y);
  }
}


static void rom_menu_main(uint8_t y) {
  switch (y) {
    case 0: lcd_puts_P(PSTR("Exit menu")); break;
    case 1: lcd_puts_P(PSTR("Browse files")); break;
    case 2: lcd_puts_P(PSTR("Change device number")); break;
    case 3:
      if (rtc_state == RTC_NOT_FOUND)
        lcd_printf("Clock not found");
      else
        lcd_printf("Set clock");
      break;
    case 4: lcd_puts_P(PSTR("Select IEC/IEEE-488")); break;
    case 5: lcd_puts_P(PSTR("Adjust LCD contrast")); break;
    case 6: lcd_puts_P(PSTR("Adjust brightness")); break;
    default: break;
  }
}


// ===== windowed directory browser (replaces menu_browse_files) =====
// Anchor-authoritative sliding window over the current directory.
//   FAT  : sorted (keyset fetch, fatops.c browse_fat_*)
//   Image: native order (sequential fetch, image_fill_from below)
// First cut: no subdir position-restore (returns to top), no wraparound.

// virtual coordinates on the LCD:
//   -2 = "Exit menu"   (only if win_at_start)
//   -1 = "Change to parent dir" (only if win_at_start)
//    k = bstate->win[k]  (0..win_count-1)
//  win_count = "-- End of dir --" (only if win_at_end)
// LCD row L shows coord top_v+L; highlight at coord top_v+curs_line.

static uint8_t image_fill_from(uint16_t start_off, browse_entry_t out[],
                               uint8_t cap, bool *hit_end) {
  dh_t        dh;
  cbmdirent_t dent;
  uint16_t    s;
  uint8_t     n = 0;

  *hit_end = true;
  if (opendir(&dh, &browse_path)) return 0;
  for (s = 0; s < start_off; s++)
    if (next_match(&dh, NULL, NULL, NULL, 0, &dent) != 0) return 0;
  while (n < cap) {
    if (next_match(&dh, NULL, NULL, NULL, 0, &dent) != 0) break;
    out[n].cat     = ((dent.typeflags & EXT_TYPE_MASK) == TYPE_DIR)
                       ? CAT_DIR : CAT_FILE;
    out[n].blocks  = dent.blocksize;
    out[n].cluster = 0;
    out[n].offset  = start_off + n;
    out[n].realname[0] = 0;                     // marks an image entry
    ustrncpy(out[n].name, dent.name, BROWSE_NAME_MAX);
    out[n].name[BROWSE_NAME_MAX] = 0;
    n++;
  }
  if (n == cap)
    *hit_end = (next_match(&dh, NULL, NULL, NULL, 0, &dent) != 0);
  return n;
}

// Fill out[] with the LAST `cap` entries of the current image dir, ascending
// (image counterpart of browse_fat_last, for wrap-to-bottom). *at_start = the
// whole dir fits. Two passes (count, then fill) - only on a wrap keypress.
static uint8_t image_fill_last(browse_entry_t out[], uint8_t cap, bool *at_start) {
  dh_t        dh;
  cbmdirent_t dent;
  uint16_t    total = 0;
  uint16_t    start;
  bool        he;

  *at_start = true;
  if (opendir(&dh, &browse_path)) return 0;
  while (next_match(&dh, NULL, NULL, NULL, 0, &dent) == 0) total++;
  if (total == 0) return 0;
  start = (total > cap) ? (uint16_t)(total - cap) : 0;
  *at_start = (total <= cap);
  return image_fill_from(start, out, cap, &he);   // he ignored: this is the dir end
}

// ---- fetch dispatch (FAT keyset vs image native) ----
static uint8_t win_fill_from_start(bool *at_end) {
  if (browse_is_fat)
    return browse_fat_fill(&browse_path, NULL, bstate->win, WIN_SIZE, at_end);
  return image_fill_from(0, bstate->win, WIN_SIZE, at_end);
}
static bool win_fetch_after(const browse_entry_t *edge, browse_entry_t *cand, bool *beyond) {
  if (browse_is_fat)
    return browse_fat_fill(&browse_path, edge, cand, 1, beyond) == 1;
  return image_fill_from(edge->offset + 1, cand, 1, beyond) == 1;
}
static bool win_fetch_before(const browse_entry_t *edge, browse_entry_t *cand) {
  bool d;
  if (browse_is_fat)
    return browse_fat_prev(&browse_path, edge, cand);
  if (edge->offset == 0) return false;
  return image_fill_from(edge->offset - 1, cand, 1, &d) == 1;
}

// ---- window edge operations ----
static bool win_extend_forward(void) {
  browse_entry_t cand;
  bool beyond;
  if (win_count == 0) { win_at_end = true; return false; }
  if (!win_fetch_after(&bstate->win[win_count - 1], &cand, &beyond)) {
    win_at_end = true;
    return false;
  }
  win_at_end = beyond;                           // fetch's hit_end: cand is the last entry
  if (win_count < WIN_SIZE) {
    bstate->win[win_count++] = cand;
  } else {
    memmove(bstate->win, bstate->win + 1, (WIN_SIZE - 1) * sizeof(browse_entry_t));
    bstate->win[WIN_SIZE - 1] = cand;
    win_at_start = false;
    top_v--;                                     // keep visible rows stable after the slide
  }
  return true;
}
static bool win_prepend_backward(void) {
  browse_entry_t cand;
  if (win_count == 0) { win_at_start = true; return false; }
  if (!win_fetch_before(&bstate->win[0], &cand)) {
    win_at_start = true;
    return false;
  }
  if (win_count < WIN_SIZE) {
    memmove(bstate->win + 1, bstate->win, win_count * sizeof(browse_entry_t));
    bstate->win[0] = cand;
    win_count++;                                 // nothing drops off the tail -> win_at_end unchanged
  } else {
    memmove(bstate->win + 1, bstate->win, (WIN_SIZE - 1) * sizeof(browse_entry_t));
    bstate->win[0] = cand;
    win_at_end = false;                          // bstate->win[WIN_SIZE-1] fell off -> no longer at end
  }
  top_v++;                                       // keep visible rows stable after the slide
  return true;
}

// keep the window covering the visible rows (+overhang)
static void win_cover(void) {
  while (!win_at_end   && top_v + LCD_LINES - 1 + WIN_OVERHANG >= (int8_t)win_count)
    if (!win_extend_forward()) break;
  while (!win_at_start && top_v < WIN_OVERHANG)   // keep a backward overhang too
    if (!win_prepend_backward()) break;           // -> cheap direction reversals
  if (!win_at_start && top_v < 0) top_v = 0;     // I1: never show nav rows falsely
}

// build the initial view of the current directory (top of the list)
static void build_view_top(void) {
  scroll_off = 0;
  scroll_next = getticks() + MS_TO_TICKS(1000);
  win_count = win_fill_from_start(&win_at_end);
  win_at_start = true;
  top_v = -nav_rows;                             // nav rows fill LCD rows 0..nav_rows-1
  if (win_count > 0) {
    sel_kind = SEL_FILE; bstate->anchor = bstate->win[0];
    curs_line = nav_rows;                        // highlight window[0], just below the nav rows
  } else {
    sel_kind = (nav_rows == 2) ? NAV_PARENT : NAV_EXIT;  // empty dir: sit on the last nav row
    curs_line = nav_rows - 1;
  }
}

static void lcd_print_dir_entry(const browse_entry_t *e) {
  char f[17];
  if      (e->cat == CAT_DIR)   lcd_puts_P(PSTR("DIR "));
  else if (e->cat == CAT_IMAGE) lcd_puts_P(PSTR("IMG "));
  else                          lcd_printf("%3u ", e->blocks);
  memset(f, 0, sizeof(f));
  ustrncpy(f, e->name, (LCD_COLS - 4) > 16 ? 16 : LCD_COLS - 4);
  pet2asc((uint8_t *) f);
  lcd_puts(f);
}

static void render_view(void) {
  uint8_t L;
  lcd_clear();
  for (L = 0; L < LCD_LINES; L++) {
    int8_t c = top_v + L;
    lcd_locate(0, L);
    if      (c < 0)   rom_menu_browse((uint8_t)(c + nav_rows));  // nav idx 0=Exit, 1=Parent
    else if (c >= 0 && c < (int8_t)win_count) lcd_print_dir_entry(&bstate->win[c]);
    else if (c == (int8_t)win_count && win_at_end) { lcd_puts_P(PSTR("-- End of dir --")); break; }
    else break;
  }
}

// set the highlight to virtual coordinate c
static void sel_set_coord(int8_t c) {
  if (c < 0) sel_kind = (uint8_t)(c + nav_rows);  // 0=NAV_EXIT, 1=NAV_PARENT
  else     { sel_kind = SEL_FILE; bstate->anchor = bstate->win[c]; }
}

static void build_view_bottom(void);             // wrap targets (defined below)
static void wrap_to_top(void);

static void move_down(void) {
  int8_t cur = top_v + curs_line;
  int8_t nxt = cur + 1;
  if (nxt == (int8_t)win_count) {                // at the last cached file
    if (win_at_end) {
      // First press on the last entry: scroll so the "-- End of dir --" line
      // below it becomes visible, keeping the highlight on the last entry.
      if (curs_line > 0 && top_v + (LCD_LINES - 1) < (int8_t)win_count) {
        top_v++;
        curs_line--;
        return;
      }
      wrap_to_top();                             // already at the very bottom -> wrap to Exit
      return;
    }
    if (!win_extend_forward()) return;
    cur = top_v + curs_line;                     // extend may have slid the window
    nxt = cur + 1;
  }
  sel_set_coord(nxt);
  if (curs_line < LCD_LINES - 1) curs_line++;
  else                           top_v++;        // screen scrolls, highlight stays at bottom
  win_cover();
}

static void move_up(void) {
  int8_t cur = top_v + curs_line;
  if (cur == -(int8_t)nav_rows) { build_view_bottom(); return; }  // on Exit -> wrap to the bottom
  int8_t prv = cur - 1;
  if (prv == -1 && !win_at_start) {              // step above window[0], earlier files exist
    if (win_prepend_backward()) { cur = top_v + curs_line; prv = cur - 1; }
    else                        win_at_start = true;
  }
  sel_set_coord(prv);
  if (curs_line > 0) curs_line--;
  else               top_v--;                    // screen scrolls up
  if (!win_at_start && top_v < 0) top_v = 0;
  win_cover();
}

// horizontal scroll of the highlighted long name. full_buf is a transient buffer
// (>= _MAX_LFN_LENGTH+1) supplied by input_loop, used only when the stored name is
// truncated - the full name is re-read into it once per selection.
static void do_scroll(uint8_t *full_buf) {
  const uint8_t *nm = bstate->anchor.name;
  uint8_t len = ustrlen(nm);
  char    w[LCD_COLS - 4 + 1];
  uint8_t k;
  // A window entry stores only the first BROWSE_NAME_MAX chars; a FAT entry whose
  // store is full may have a longer real name that we fetch on demand (below).
  bool maybe_long = (browse_is_fat && bstate->anchor.realname[0] && len >= BROWSE_NAME_MAX);
  if (sel_kind != SEL_FILE) return;
  if (len <= (LCD_COLS - 4) && !maybe_long) {   // fits the display, can't be longer
    scroll_next = getticks() + MS_TO_TICKS(2000);
    return;
  }
  if (!time_after(getticks(), scroll_next)) return;   // only work once the entry sits idle
  if (maybe_long) {                              // now scrolling -> get the full name once
    if (!scroll_full_ready) {
      if (fat_get_longname(&browse_path, bstate->anchor.realname, full_buf)) {
        if (file_extension_mode == 5)                     // XE5: decode canonically (as build does)
          fat_to_petscii((char *)full_buf, false, (char *)full_buf, _MAX_LFN_LENGTH, true);
        else
          asc2pet(full_buf);                              // raw LFN -> PETSCII
        { uint8_t *sp = full_buf; while (*sp && *sp != 0xa0) sp++; *sp = 0; }  // shift-space ends the name
      } else
        ustrcpy(full_buf, bstate->anchor.name);               // no LFN -> stored (already PETSCII/truncated)
      scroll_full_ready = true;
    }
    nm = full_buf;
    len = ustrlen(nm);
    if (len <= (LCD_COLS - 4)) { scroll_next = getticks() + MS_TO_TICKS(2000); return; }
  }
  for (k = 0; k < (LCD_COLS - 4); k++)
    w[k] = (scroll_off + k < len) ? nm[scroll_off + k] : ' ';
  w[LCD_COLS - 4] = 0;
  pet2asc((uint8_t *) w);
  lcd_locate(4, curs_line); lcd_puts(w); lcd_locate(0, curs_line);
  if (scroll_off + (LCD_COLS - 4) >= len) { scroll_off = 0; scroll_next = getticks() + MS_TO_TICKS(1500); }
  else                                    { scroll_off++;   scroll_next = getticks() + MS_TO_TICKS(400); }
}

// idle input loop. Returns which key ended it (BR_ACT_*). It does NOT move - the
// caller does, AFTER this returns, so the 256-byte scroll buffer below is off the
// stack during move-triggered directory scans and the deep CD_ path. noinline keeps
// that buffer out of menu_browse_files' permanent frame.
#define BR_ACT_PREV 1
#define BR_ACT_NEXT 2
#define BR_ACT_SEL  3
static uint8_t __attribute__((noinline)) input_loop(void) {
  uint8_t full_buf[_MAX_LFN_LENGTH + 1];     // transient: full long name, only while scrolling
  scroll_off = 0;
  scroll_full_ready = false;                 // selection may have changed -> re-fetch full name lazily
  scroll_next = getticks() + MS_TO_TICKS(1000);
  for (;;) {
    lcd_locate(0, curs_line);
    if (get_key_autorepeat(KEY_PREV)) return BR_ACT_PREV;
    if (get_key_autorepeat(KEY_NEXT)) return BR_ACT_NEXT;
    if (get_key_press(KEY_SEL))       return BR_ACT_SEL;
    do_scroll(full_buf);
  }
}

// ---- position restore (Stage 3) ----
// Seed bstate->win[] with the target child at index 0 (window[0]); set win_count,
// win_at_start, win_at_end. Returns false if the child no longer exists.
static bool win_seed_fat_cluster(uint32_t cluster) {
  browse_entry_t target, prev;
  if (!browse_fat_find_cluster(&browse_path, cluster, &target)) return false;
  if (browse_fat_prev(&browse_path, &target, &prev)) {         // earlier entries exist
    win_count = browse_fat_fill(&browse_path, &prev, bstate->win, WIN_SIZE, &win_at_end);
    win_at_start = false;
  } else {                                                     // target is the first entry
    win_count = browse_fat_fill(&browse_path, NULL, bstate->win, WIN_SIZE, &win_at_end);
    win_at_start = true;
  }
  return (win_count > 0 && bstate->win[0].cluster == cluster);
}

static bool win_seed_image_offset(uint16_t offset) {
  win_count = image_fill_from(offset, bstate->win, WIN_SIZE, &win_at_end);
  win_at_start = (offset == 0);
  return (win_count > 0 && bstate->win[0].offset == offset);
}

// Index of the highlighted entry within bstate->win[] (matches bstate->anchor by the
// unique key: realname for FAT, native offset for images). -1 if not present.
static int16_t sel_index(void) {
  uint8_t k;
  if (sel_kind != SEL_FILE) return -1;
  for (k = 0; k < win_count; k++) {
    if (browse_is_fat) {
      if (ustrcmp(bstate->win[k].realname, bstate->anchor.realname) == 0) return k;
    } else {
      if (bstate->win[k].offset == bstate->anchor.offset) return k;
    }
  }
  return -1;
}

// Rebuild the view with the remembered child highlighted (falls back to top).
static void build_view_restore(const retpos_t *rp) {
  bool ok;
  int16_t ti;
  int8_t  cl;
  scroll_off = 0;
  scroll_next = getticks() + MS_TO_TICKS(1000);
  if (rp->is_fat != browse_is_fat) { build_view_top(); return; }
  ok = rp->is_fat ? win_seed_fat_cluster(rp->cluster)
                  : win_seed_image_offset(rp->offset);
  if (!ok || win_count == 0) { build_view_top(); return; }
  sel_kind = SEL_FILE; bstate->anchor = bstate->win[0];   // the child, at window[0]
  top_v = win_at_start ? -(int8_t)nav_rows : 0;       // nav rows only at the true start
  win_cover();                                        // fill overhang; may slide the window
  ti = sel_index();                                   // where the child landed after covering
  if (ti < 0) { build_view_top(); return; }
  cl = (int8_t)ti - top_v;
  if (cl < 0)              cl = 0;
  if (cl > LCD_LINES - 1)  cl = LCD_LINES - 1;
  curs_line = cl;
}

// ---- wraparound targets ----
// Wrap DOWN past the last entry: rebuild the top view but land on "Exit menu".
static void wrap_to_top(void) {
  build_view_top();                              // window from the start, nav rows visible
  sel_set_coord(-(int8_t)nav_rows);              // highlight Exit (topmost nav row)
  curs_line = 0;                                 // Exit sits on LCD row 0 (top_v == -nav_rows)
}

// Wrap UP past Exit: jump to the dir end, last entry highlighted, marker below it.
static void build_view_bottom(void) {
  bool    at_start;
  int8_t  min_top, cl;
  int16_t ti;
  scroll_off = 0;
  scroll_next = getticks() + MS_TO_TICKS(1000);
  if (browse_is_fat) win_count = browse_fat_last(&browse_path, bstate->win, WIN_SIZE, &at_start);
  else               win_count = image_fill_last(bstate->win, WIN_SIZE, &at_start);
  if (win_count == 0) { build_view_top(); return; }   // empty dir -> top handles Exit/Parent
  win_at_end   = true;
  win_at_start = at_start;
  sel_kind = SEL_FILE; bstate->anchor = bstate->win[win_count - 1];   // last entry
  top_v = (int8_t)win_count - (LCD_LINES - 1);        // put the marker (coord win_count) on the last row
  min_top = win_at_start ? -(int8_t)nav_rows : 0;     // don't scroll above the start's nav rows
  if (top_v < min_top) top_v = min_top;
  win_cover();
  ti = sel_index();
  if (ti < 0) { build_view_top(); return; }
  cl = (int8_t)ti - top_v;
  if (cl < 0)              cl = 0;
  if (cl > LCD_LINES - 1)  cl = LCD_LINES - 1;
  curs_line = cl;
}

void menu_browse_files(void) {
  browse_state_t bs_local;
  memset(&bs_local, 0, sizeof(bs_local));   // zero = former static init
  bstate = &bs_local;
  ret_depth = 0;                 // fresh browse session: no descend history yet
  ret_pending_valid = false;
LOAD:
  jump_out_mainmenu = 0;
  lcd_clear();
  lcd_puts_P(PSTR("Reading..."));
  // Only the real FAT filesystem uses the browse_fat_* (l_opendir) path; every
  // other fop (image/d64, EEPROM-FS, m2i) must go through opendir/next_match,
  // so discriminate positively on &fatops rather than "anything but d64ops".
  browse_is_fat = (partition[current_part].fop == &fatops);
  browse_path.part = current_part;
  browse_path.dir  = partition[current_part].current_dir;
  // No "Change to parent dir" in the FAT card root (nothing above it).
  // Inside an image, ".." unmounts back to FAT, so keep the parent row there.
  nav_rows = (browse_is_fat && browse_path.dir.fat == 0) ? 1 : 2;
  if (ret_pending_valid) { build_view_restore(&bstate->rpending); ret_pending_valid = false; }
  else if (!browse_is_fat) wrap_to_top();   // entered an image: start the cursor on "Exit menu", not the first file
  else                     build_view_top();

  for (;;) {
    render_view();
    lcd_cursor(true);
    uint8_t act = input_loop();
    lcd_cursor(false);
    if (act == BR_ACT_PREV) { move_up();   continue; }   // move here, not in input_loop,
    if (act == BR_ACT_NEXT) { move_down(); continue; }   // so its scroll buffer is off the stack

    // act == BR_ACT_SEL
    if (sel_kind == NAV_EXIT) {
      jump_out_mainmenu = 1;
      return;
    }
    if (sel_kind == NAV_PARENT) {
      ustrcpy_P(command_buffer, PSTR("CD_"));
      command_length = 3;
      parse_doscommand();
      clear_command_buffer();
      if (current_error != ERROR_OK) { jump_out_mainmenu = 1; return; }
      if (ret_depth > 0) {                        // returning up one level
        ret_depth--;
        ret_pending_valid = (ret_depth < RET_MAX);   // was this level's position stored?
        if (ret_pending_valid) bstate->rpending = bstate->rstack[ret_depth];
      } else {
        ret_pending_valid = false;                // above where we started: no memory
      }
      goto LOAD;
    }
    // sel_kind == SEL_FILE
    if (bstate->anchor.cat == CAT_DIR || bstate->anchor.cat == CAT_IMAGE) {
      if (bstate->anchor.realname[0]) {              // FAT: load by unique reference
        cbmdirent_t dent;
        memset(&dent, 0, sizeof(dent));
        dent.opstype   = OPSTYPE_FAT;
        dent.typeflags = (bstate->anchor.cat == CAT_DIR) ? TYPE_DIR : TYPE_PRG;
        dent.pvt.fat.cluster = bstate->anchor.cluster;
        ustrcpy(dent.pvt.fat.realname, bstate->anchor.realname);
        ustrcpy(dent.name, bstate->anchor.realname);
        if (chdir(&browse_path, &dent)) {
          if (current_error != ERROR_OK) { jump_out_mainmenu = 1; return; }
        } else {
          update_current_dir(&browse_path);
          previous_file_dirent.name[0] = 0;   // mirror do_chdir(): invalidate LOAD"*"
        }
      } else {                                   // image entry: exact CBM name
        clear_command_buffer();
        ustrcpy_P(command_buffer, PSTR("CD:"));
        ustrncpy(command_buffer + 3, bstate->anchor.name, 16);
        command_length = ustrlen(command_buffer);
        parse_doscommand();
        clear_command_buffer();
        if (current_error != ERROR_OK) { jump_out_mainmenu = 1; return; }
      }
      // remember this child so returning up re-highlights it (FAT: cluster, image: offset)
      if (ret_depth < RET_MAX) {
        bstate->rstack[ret_depth].is_fat  = browse_is_fat;
        bstate->rstack[ret_depth].cluster = bstate->anchor.cluster;
        bstate->rstack[ret_depth].offset  = bstate->anchor.offset;
      }
      if (ret_depth < 255) ret_depth++;
      goto LOAD;
    }
    // plain file: no action
  }
}


#ifdef HAVE_DUAL_INTERFACE
#define MAIN_MENU_LAST_ENTRY 6
#else
#define MAIN_MENU_LAST_ENTRY 5
#endif

bool menu(void) {
  uint8_t mp = 0;
  uint8_t my = 0;
  uint8_t i;
  uint8_t old_bus;
  bool action;

  old_bus = active_bus;
  bus_sleep2(true);

  menu_select_status(); // disable splash screen if still active
  set_error(ERROR_OK);
  for (;;) {
    set_busy_led(false); set_dirty_led(true);
    lcd_clear();
    for (i = 0; i < LCD_LINES; i++) {
      lcd_locate(0, i);
      rom_menu_main(mp - my + i);
    }

    lcd_cursor(true);
    for (;;) {
      action = false;
      lcd_locate(0, my);
      //printf("mp: %u   my: %u\r\n", mp, my);
      //while (!get_key_state(KEY_ANY));
      if (get_key_autorepeat(KEY_PREV)) {
        if (mp > 0) {
          // Move up
          --mp;
          if (my > 0) {
            --my;               // Move within same page
          } else {
            mp = LCD_LINES - 1;
            my = LCD_LINES - 1; // page up
            break;
          }
        } else {
          // flip down to last menu entry
          my = MAIN_MENU_LAST_ENTRY % LCD_LINES;
          mp = MAIN_MENU_LAST_ENTRY;
          break;
        }
      }
      if (get_key_autorepeat(KEY_NEXT)) {
        if (mp < MAIN_MENU_LAST_ENTRY) {
          ++mp;
          if (my < (LCD_LINES - 1)) {
            ++my;               // Move within same page
          } else {
            my = 0;             // page down
            break;
          }
        } else {
          // flip up to first menu entry
          mp = 0;
          my = 0;
          break;
        }
      }
      if (get_key_press(KEY_SEL)) {
        action = true;
        break;
      }
    }
    lcd_cursor(false);
    if (!action) continue;
    lcd_clear();
    if      (mp == 1) {
       menu_browse_files();
       if (jump_out_mainmenu==1) {
          jump_out_mainmenu=0;
          break;
       }
    }
    else if (mp == 2) menu_device_number();
    else if (mp == 3) menu_set_clock();
    else if (mp == 4) menu_select_bus();
    else if (mp == 5) menu_adjust_contrast();
    else if (mp == 6) menu_adjust_brightness();
    else  break;
    if (current_error != ERROR_OK) break;
  }
  if(active_bus==old_bus) {
    bus_sleep2(false);
    lcd_draw_screen(SCRN_STATUS);
    update_leds();
    return false;
  } else {
    lcd_draw_screen(SCRN_STATUS);
    update_leds();
    return true;
  }
}

static void pwm_error(void) {
  lcd_locate(0, LCD_LINES - 2);
  lcd_puts_P(PSTR("Error:PWM controller\nnot found"));
  wait_anykey();
}

void menu_adjust_contrast(void) {
  uint8_t i;
  uint8_t min = 0;
  uint8_t max = LCD_COLS - 2;
  uint8_t res;

  lcd_clear();
  lcd_puts_P(PSTR("Adjust LCD contrast"));
  lcd_locate(0, 1);

  lcd_cursor(false);
  set_busy_led(true);
  for (;;) {
    lcd_locate(0, 1);
    lcd_putc('[');
    for (i = 0; i < LCD_COLS - 2; i++) {
      lcd_putc(i >= lcd_contrast ? ' ' : 0xFF);
    }
    lcd_putc(']');
    res = lcd_set_contrast(lcd_contrast);
    if (res) break;
    for (;;) {
      if (get_key_autorepeat(KEY_PREV)) {
        if (lcd_contrast <= min) lcd_contrast = max;
        else --lcd_contrast;
        break;
      }
      if (get_key_autorepeat(KEY_NEXT)) {
        if (lcd_contrast >= max) lcd_contrast = min;
        else ++lcd_contrast;
        break;
      }
      if (get_key_press(KEY_SEL)) {
        lcd_cursor(false);
        set_busy_led(false);
        menu_ask_store_settings();
        return;
      }
    }
  }
  pwm_error();
}


void menu_adjust_brightness(void) {
  uint8_t i;
  uint8_t min = 0;
  uint8_t max = 255;
  uint8_t res;
  uint8_t step;

  lcd_clear();
  lcd_puts_P(PSTR("Adjust brightness"));
  lcd_locate(0, 1);

  lcd_cursor(false);
  set_busy_led(true);
  for (;;) {
    lcd_locate(0, 1);
    lcd_putc('[');
    for (i = 0; i < 18; i++) {
      lcd_putc(i >= (lcd_brightness / 14) ? ' ' : 0xFF);
    }
    lcd_putc(']');
    res = lcd_set_brightness(lcd_brightness);
    if (res) break;
    for (;;) {
      step = 10;
      if (lcd_brightness < 20 || lcd_brightness > 235) step = 1;
      if (get_key_autorepeat(KEY_PREV)) {
        if (lcd_brightness <= min) lcd_brightness = max;
        else lcd_brightness -= step;
        break;
      }
      if (get_key_autorepeat(KEY_NEXT)) {
        if (lcd_brightness >= max) lcd_brightness = min;
        else lcd_brightness += step;
        break;
      }
      if (get_key_press(KEY_SEL)) {
        lcd_cursor(false);
        set_busy_led(false);
        menu_ask_store_settings();
        return;
      }
    }
  }
  pwm_error();
}
