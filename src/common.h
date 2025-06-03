/*
** Copyright (C) Arseny Vakhrushev <arseny.vakhrushev@me.com>
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
#ifdef AT32F4
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/usart.h>
#else
#include <libopencm3/stm32/usart.h>
#endif
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/wwdg.h>
#include "config.h"
#include "defs.h"

#define CLK_CNT(rate) ((CLK + ((rate) >> 1)) / (rate))
#define CLK_KHZ (CLK / 1000)
#define CLK_MHZ (CLK / 1000000)

#ifdef ANALOG
#define IO_ANALOG (cfg.throt_set < 100)
#elif defined IO_PA2 || defined IO_PA6 || defined ANALOG_CHAN
#define IO_ANALOG (cfg.input_mode == 1)
#else
#define IO_ANALOG 0
#endif

#define GPIO(port, name) _GPIO(port, name)
#define _GPIO(port, name) __GPIO(port, name)
#define __GPIO(port, name) GPIO##port##_##name

typedef struct {
	const uint16_t id;
	const char revision;
	const char revpatch;
	const char name[15];
	const char _null;
	char arm;
	char damp;
	char revdir;
	char brushed;
	char timing;
	char sine_range;
	char sine_power;
	char freq_min;
	char freq_max;
	char duty_min;
	char duty_max;
	char duty_spup;
	char duty_ramp;
	char duty_rate;
	char duty_drag;
	char duty_lock;
	char throt_mode;
	char throt_rev;
	char throt_brk;
	char throt_set;
	char throt_cal;
	uint16_t throt_min;
	uint16_t throt_mid;
	uint16_t throt_max;
	uint16_t analog_min;
	uint16_t analog_max;
	char input_mode;
	char input_chid;
	char telem_mode;
	char telem_phid;
	char telem_poles;
	uint16_t prot_stall;
	char prot_temp;
	char prot_sens;
	char prot_volt;
	char prot_cells;
	uint16_t prot_curr;
	char music[256];
	char volume;
	char beacon;
	char bec;
	char led;
} Cfg;

typedef struct {
	int Kp, Ki, Kd, Li, i, x;
} PID;

typedef void (*Func)(void);

extern char _boot[], _cfg[], _cfg_start[], _cfg_end[], _rom[], _ram[], _eod[], _vec[]; // Linker exports
extern const uint16_t sinedata[];
extern const Cfg cfgdata;
extern Cfg cfg;
extern int throt, ertm, erpm, temp1, temp2, volt, curr, csum, dshotval, beepval;
extern char analog, telreq, telmode, flipdir, beacon, dshotext;

void init(void);
void initio(void);
void initgpio(void);
void initled(void);
void inittelem(void);
int hallcode(void);
void ledctl(int x);
void hsictl(int x);
void compctl(int x);
void io_serial(void);
void io_analog(void);
void adctrig(void);
void adcdata(int t, int u, int v, int c, int a);
void delay(int ms, Func f);
void kisstelem(void);
void autotelem(void);
int execcmd(char *str);
char crc8(const char *buf, int len);
char crc8dvbs2(const char *buf, int len);
int scale(int x, int a1, int a2, int b1, int b2);
int smooth(int *s, int x, int n);
void initpid(PID *pid, int x);
int calcpid(PID *pid, int x, int y);
void checkcfg(void);
int savecfg(void);
int resetcfg(void);
void resetcom(void);
int playmusic(const char *str, int vol);
void playsound(const char *buf, int vol);

static inline int min(int a, int b) {return a < b ? a : b;}
static inline int max(int a, int b) {return a > b ? a : b;}
static inline int clamp(int x, int a, int b) {return min(max(x, a), b);}

// Temperature sensors

static inline int NTC10K3455LO2K(int x) {
	if (x < 100) return 0;
	return (x < 2338 ? (x - 1650) * 46 + 74841 : (x - 2640) * 83 + 132044) >> 8;
}

static inline int NTC10K3455UP2K(int x) {
	if (x > 3200) return 0;
	return (x > 961 ? (x - 1650) * -46 + 74841 : (x - 660) * -83 + 132044) >> 8;
}

static inline int NTC10K3455LO10K(int x) {
	if (x < 100) return 0;
	return (x < 2762 ? (x - 1650) * 36 + 25600 : (x - 3036) * 151 + 107130) >> 8;
}

static inline int NTC10K3455UP10K(int x) {
	if (x > 3200) return 0;
	return (x > 537 ? (x - 1650) * -36 + 25600 : (x - 264) * -151 + 107130) >> 8;
}
