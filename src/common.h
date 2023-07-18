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
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#ifdef USARTv1
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/usart.h>
#else
#include <libopencm3/stm32/usart.h>
#endif
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/wwdg.h>
#include "defs.h"
#include "config.h"

#define CLK_CNT(rate) ((CLK + ((rate) >> 1)) / (rate))
#define CLK_KHZ (CLK / 1000)
#define CLK_MHZ (CLK / 1000000)

#ifdef ANALOG
#define IO_ANALOG (ANALOG_MIN != ANALOG_MAX)
#elif defined IO_PA2 || defined IO_PA6
#define IO_ANALOG (cfg.input_mode == 1)
#else
#define IO_ANALOG 0
#endif

typedef struct {
	const uint16_t id;
	const char revision;
	const char target_type;
	const char target_name[15];
	const char _null;
	char arm;
	char damp;
	char revdir;
	char timing;
	char sine_range;
	char sine_power;
	char freq_min;
	char freq_max;
	char duty_min;
	char duty_max;
	char duty_spup;
	char duty_ramp;
	char duty_drag;
	char throt_mode;
	char throt_cal;
	short throt_min;
	short throt_mid;
	short throt_max;
	char input_mode;
	char input_chid;
	char telem_mode;
	char telem_phid;
	char telem_poles;
	char prot_temp;
	char prot_volt;
	char prot_cells;
	char prot_curr;
	char music[128];
	char volume;
	char beacon;
	char led;
} Cfg;

typedef struct {
	int Kp, Ki, Kd, Li, i, x;
} PID;

extern char _cfg[], _cfg_start[], _cfg_end[], _rom[], _ram[], _boot[], _vec[]; // Linker exports
extern const uint16_t sinedata[];
extern const Cfg cfgdata;
extern Cfg cfg;
extern int throt, ertm, erpm, temp, volt, curr, csum, dshotval, beepval;
extern char analog, telreq, flipdir, beacon, dshotext;
extern volatile uint32_t tickms;

void init(void);
void initio(void);
void initled(void);
void inittelem(void);
void ledctl(int x);
void hsictl(int x);
void compctl(int x);
void io_serial(void);
void io_analog(void);
void adc_trig(void);
void adc_data(int t, int v, int c, int x);
void sendtelem(void);
int execcmd(char *buf);
int clamp(int x, int a1, int a2);
int scale(int x, int a1, int a2, int b1, int b2);
int smooth(int *s, int x, int n);
int calcpid(PID *pid, int x, int y);
void checkcfg(void);
int savecfg(void);
int resetcfg(void);
int playmusic(const char *str, int vol);
void reset(void) __attribute__((__noreturn__));
