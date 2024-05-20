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


   bus.h: Common IEC/IEEE bus definitions

*/

#ifndef BUS_H
#define BUS_H

#include <stdbool.h>

#include "config.h"
#include "iec.h"
#include "ieee.h"
#include "uart.h"

extern uint8_t device_address;

enum { IEC, IEEE488 };

void iec_interface_init(void);
void ieee_interface_init(void);

#ifdef HAVE_DUAL_INTERFACE
extern uint8_t active_bus;

static inline void bus_interface_init(void) {
  uart_puts_P(PSTR("bus_interface_init()\r\n"));
  if (active_bus == IEC) {
    uart_puts_P(PSTR("  ->iec_interface_init()\r\n"));
    iec_interface_init();
  } else { 
    uart_puts_P(PSTR("  ->ieee_interface_init()\r\n"));
    ieee_interface_init();
  }
}

static inline void bus_init(void) {
  uart_puts_P(PSTR("bus_init()\r\n"));
  if (active_bus == IEC) {
    uart_puts_P(PSTR("  ->iec_init()\r\n"));
    iec_init();
  } else {
    uart_puts_P(PSTR("  ->ieee488_Init()\r\n"));
    ieee488_Init();
  }
}

#ifdef IEC_SLOW_IEEE_FAST

#include <avr/power.h>
#include "uart.h"
#include "timer.h"
#include "spi.h"

static inline void set_clock_prescaler(uint8_t bus) {
  uart_flush();
  if (active_bus == IEC) clock_prescale_set(clock_div_2);    // Set clock to 16/2 =  8 MHz
  else                   clock_prescale_set(clock_div_1);    // Set clock to        16 MHz
  timer_init();
  uart_init();
  spi_set_speed(SPI_SPEED_FAST);
}
#else
static inline void set_clock_prescaler(uint8_t bus) {}
#endif

static inline void bus_mainloop(void) {
  uart_puts_P(PSTR("bus_mainloop()\r\n"));
  uart_puts_P(PSTR("  ->set_clock_prescaler()\r\n"));
  set_clock_prescaler(active_bus);
  if (active_bus == IEC) {
    uart_puts_P(PSTR("  ->iec_mainloop()\r\n"));
    iec_mainloop();
  } else {
    uart_puts_P(PSTR("  ->ieee_mainloop()\r\n"));
    ieee_mainloop();
  }
}

static inline void bus_sleep2(bool sleep) {
  uart_puts_P(PSTR("bus_sleep()\r\n"));
  if (active_bus == IEC) {
    if (sleep)
      uart_puts_P(PSTR("  ->iec_sleep(true)\r\n"));
    else
      uart_puts_P(PSTR("  ->iec_sleep(false)\r\n"));
     iec_sleep(sleep);
  } else {
    if(sleep)
      uart_puts_P(PSTR("  ->ieee488_BusSleep(true)\r\n"));
    else
      uart_puts_P(PSTR("  ->ieee488_BusSleep(false)\r\n"));
      ieee488_BusSleep(sleep);
  }
}
#else
#ifdef CONFIG_HAVE_IEC
#define active_bus IEC

static inline void bus_interface_init(void) {
  iec_interface_init();
}

static inline void bus_init(void) {
  iec_init();
}

static inline void bus_mainloop(void) {
  iec_mainloop();
}

static inline void bus_sleep2(bool sleep) {
  iec_sleep(sleep);
}
#else
#ifdef CONFIG_HAVE_IEEE
#define active_bus IEEE488

static inline void bus_interface_init(void) {
  ieee_interface_init();
}

static inline void bus_init(void) {
  ieee488_Init();
}

static inline void bus_mainloop(void) {
  ieee_mainloop();
}

static inline void bus_sleep2(bool sleep) {
  ieee488_BusSleep(sleep);
}
#endif // CONFIG_HAVE_IEEE
#endif // CONFIG_HAVE_IEC
#endif // HAVE_DUAL_INTERFACE

#endif
