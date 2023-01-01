/*
** Copyright (C) 2022-2023 Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This firmware is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this firmware. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <stdlib.h>
#include <string.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/wwdg.h>
#include "defs.h"
#include "config.h"

#define CNT(clk, rate) (((clk) + ((rate) >> 1)) / (rate)) // Rate-based clock count
#define KHZ(clk) ((clk) / 1000)
#define MHZ(clk) ((clk) / 1000000)

#define cm_wait_for_interrupt() __asm__("wfi")

extern struct cfg {
	const uint16_t id;
	const char revision;
	const char target_type;
	const char target_name[15];
	const char _null;
	char arm;
	char damp;
	char reverse;
	char timing;
	char freq_min;
	char freq_max;
	char duty_min;
	char duty_max;
	char duty_lim;
	char duty_drag;
	char throt_mode;
	char throt_cal;
	short throt_min;
	short throt_mid;
	short throt_max;
	char serial_mode;
	char serial_chid;
	char music1[64];
	char music2[64];
	char music3[64];
	char volume1;
	char volume2;
	char volume3;
	short last_error;
} cfg;

extern char _cfg[], _cfg_start[], _cfg_end[], _rom[], _ram[], _boot[], _vec[]; // Linker exports

extern int throt, telem, erpt, erpm;

void init(void);
void initio(void);
void inittelem(void);

void comp_in(int x);
void comp_out(int on);

void io_serial(void);
void io_pullup(void);

void sendtelem(void);
int execcmd(char *buf);

int clamp(int x, int a1, int a2);
int scale(int x, int a1, int a2, int b1, int b2);

void loadcfg(void);
void checkcfg(void);
int savecfg(void);
int playmusic(const char *str, int vol);
void reset(void) __attribute__((__noreturn__));
void error(int code) __attribute__((__noreturn__));
