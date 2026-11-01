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


   timer.c: System timer (and button debouncer)

*/

#include "config.h"
#include "diskchange.h"
#include "display.h"
#include "led.h"
#include "time.h"
#include "rtc.h"
#include "softrtc.h"
#include "timer.h"
#ifdef CONFIG_LCD_DISPLAY
#include "display_lcd.h"
#endif

#define DEBOUNCE_TICKS 4
#define SLEEP_TICKS    2*HZ

volatile tick_t ticks;

//Arduino only variable
#if CONFIG_HARDWARE_VARIANT == 11
static uint8_t sleep_flag;
// Explanation of sleep_flag:
// sleep_flag == 0   mormal operation
// sleep_flag == 170 prepare to sleep state
// sleep_flag == 255 sleep state, don't read analog keyboard anymore
#endif

// Logical buttons
volatile uint8_t active_keys;

// Physical buttons
rawbutton_t buttonstate;
tick_t      lastbuttonchange;

/* Called by the timer interrupt when the button state has changed */
static void buttons_changed(void) {
  /* Check if the previous state was stable for two ticks */
  if (time_after(ticks, lastbuttonchange + DEBOUNCE_TICKS)) {
    if (active_keys & IGNORE_KEYS) {
      active_keys &= ~IGNORE_KEYS;
    } else if (BUTTON_PREV && /* match only if PREV exists */
               !(buttonstate & (BUTTON_PREV|BUTTON_NEXT))) {
      /* Both buttons held down */
        active_keys |= KEY_HOME;
    } else if (!(buttonstate & BUTTON_NEXT) &&
               (buttons_read() & BUTTON_NEXT)) {
      /* "Next" button released */
      active_keys |= KEY_NEXT;
    } else if (BUTTON_PREV && /* match only if PREV exists */
               !(buttonstate & BUTTON_PREV) &&
               (buttons_read() & BUTTON_NEXT)) {
      active_keys |= KEY_PREV;
    }
  }

  lastbuttonchange = ticks;
  buttonstate = buttons_read();
}

#if CONFIG_HARDWARE_VARIANT == 11
// Arduino specific, analog keypad read
ISR(ADC_vect) // Reason for sleep_flag variable
{
static uint8_t adc_read;

if(sleep_flag != 255) {
//if(active_keys != KEY_SLEEP) {
adc_read = ADCH;

// Key [Right]
if(adc_read < 16)  {
    active_keys |= KEY_HOME;
    lastbuttonchange = ticks;
    buttonstate = 0;
    }
// Key [Down] or [Up]
if((adc_read > 15) && (adc_read < 84)) {

if (sleep_flag == 170) {
        sleep_flag = 255;
        /* Msg about Sleep state on the LCD screen */
#ifdef CONFIG_LCD_DISPLAY
        if (lcd_controller_type() != 0) {
        DS_SHOWP(0," zzz I sleep zzz");
        DS_SHOWP(1,"  zzz  zzz  zzz");
        }
#endif
    active_keys |= KEY_SLEEP | IGNORE_KEYS;
    lastbuttonchange = ticks;
    buttonstate = 3;
    }
if (sleep_flag == 0) {
    sleep_flag = 170;
    active_keys |= KEY_NEXT;
    lastbuttonchange = ticks;
    buttonstate = 2;
    }

}
// Key [Left]
if((adc_read > 83) && (adc_read < 140)) {
    active_keys |= KEY_PREV;
    lastbuttonchange = ticks;
    buttonstate = 4;
    }
// Key [Select]
if((adc_read > 139) && (adc_read < 212)) {
    active_keys |= KEY_NEXT;
    lastbuttonchange = ticks;
    buttonstate = 2;
    }


// No key pressed
if(adc_read > 213) {
if(sleep_flag != 255) {
    sleep_flag = 0;
    }
  }
 }
}
#endif

/* The main timer interrupt */
SYSTEM_TICK_HANDLER {
  rawbutton_t tmp = buttons_read();

  if (tmp != buttonstate) {
    buttons_changed();
  }

  ticks++;

#ifdef SINGLE_LED
  if (led_state & LED_ERROR) {
    if ((ticks & 15) == 0)
      toggle_led();
  } else {
    set_led((led_state & LED_BUSY) || (led_state & LED_DIRTY));
  }
#else
  if (led_state & LED_ERROR)
    if ((ticks & 15) == 0)
      toggle_dirty_led();
#endif

  /* Sleep button triggers when held down for 2sec */
  if (time_after(ticks, lastbuttonchange + DEBOUNCE_TICKS)) {
    if (!(buttonstate & BUTTON_NEXT) &&
        (!BUTTON_PREV || (buttonstate & BUTTON_PREV)) &&
        time_after(ticks, lastbuttonchange + SLEEP_TICKS) &&
        !key_pressed(KEY_SLEEP)) {
      /* Set ignore flag so the release doesn't trigger KEY_NEXT */
      active_keys |= KEY_SLEEP | IGNORE_KEYS;
      /* Avoid triggering for the next two seconds */
      lastbuttonchange = ticks;
/* No more read the Arduino's analog keyboard */
#if CONFIG_HARDWARE_VARIANT == 11
        sleep_flag = 255;
#endif
/* Msg about Sleep state on the LCD screen if LCD is present*/
#ifdef CONFIG_LCD_DISPLAY
        if (lcd_controller_type() != 0) {
        DS_SHOWP(0," zzz I sleep zzz");
        DS_SHOWP(1,"  zzz  zzz  zzz")
        }
#endif
    }
  }

  /* send tick to the software RTC emulation */
  softrtc_tick();

#ifdef CONFIG_REMOTE_DISPLAY
  /* Check if the display wants to be queried */
  if (display_intrq_active()) {
    active_keys |= KEY_DISPLAY;
  }
#endif
}
