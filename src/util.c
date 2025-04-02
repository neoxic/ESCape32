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

#include "common.h"

#ifndef HALL_MAP
#elif HALL_MAP == 0xAFB35
#define HALL1_PORT A
#define HALL1_PIN1 15
#define HALL2_PORT B
#define HALL2_PIN2 3
#define HALL2_PIN3 5
#elif HALL_MAP == 0xB358
#define HALL_PORT B
#define HALL_PIN1 3
#define HALL_PIN2 5
#define HALL_PIN3 8
#endif

#ifndef BEC_MAP
#elif BEC_MAP == 0xB35
#define BEC_PORT B
#define BEC_PIN1 3
#define BEC_PIN2 5
#elif BEC_MAP == 0xCEF
#define BEC_PORT C
#define BEC_PIN1 14
#define BEC_PIN2 15
#endif

#if LED_MAP == 0xAF
#define LED1_PORT A
#define LED1_PIN 15
#elif LED_MAP == 0xB8
#define LED1_PORT B
#define LED1_PIN 8
#elif LED_MAP == 0xF2AF
#define LED1_PORT F
#define LED1_PIN 2
#define LED2_PORT A
#define LED2_PIN 15
#elif LED_MAP == 0xAFB3B4
#define LED1_PORT A
#define LED1_PIN 15
#define LED2_PORT B
#define LED2_PIN 3
#define LED3_PORT B
#define LED3_PIN 4
#elif LED_MAP == 0xAFB5B3
#define LED1_PORT A
#define LED1_PIN 15
#define LED2_PORT B
#define LED2_PIN 5
#define LED3_PORT B
#define LED3_PIN 3
#elif LED_MAP == 0xB5B3AF
#define LED1_PORT B
#define LED1_PIN 5
#define LED2_PORT B
#define LED2_PIN 3
#define LED3_PORT A
#define LED3_PIN 15
#elif LED_MAP == 0xB5B4B3
#define LED1_PORT B
#define LED1_PIN 5
#define LED2_PORT B
#define LED2_PIN 4
#define LED3_PORT B
#define LED3_PIN 3
#elif LED_MAP == 0xB8B5B3
#define LED1_PORT B
#define LED1_PIN 8
#define LED2_PORT B
#define LED2_PIN 5
#define LED3_PORT B
#define LED3_PIN 3
#endif

#ifdef LED_INV
#define LED1_INV
#define LED2_INV
#define LED3_INV
#endif

#ifdef LED_OD
#define LED1_OD
#define LED2_OD
#define LED3_OD
#endif

#ifndef FLASH_CR_STRT
#define FLASH_CR_STRT FLASH_CR_START
#endif

static char busy;

void initgpio(void) {
#ifdef HALL_MAP
#ifdef HALL1_PORT
#ifdef HALL1_PIN2
	GPIO(HALL1_PORT, PUPDR) |= 1 << HALL1_PIN1 * 2 | 1 << HALL1_PIN2 * 2;
	GPIO(HALL2_PORT, PUPDR) |= 1 << HALL2_PIN3 * 2;
	GPIO(HALL1_PORT, MODER) &= ~(3 << HALL1_PIN1 * 2 | 3 << HALL1_PIN2 * 2);
	GPIO(HALL2_PORT, MODER) &= ~(3 << HALL2_PIN3 * 2);
#else
	GPIO(HALL1_PORT, PUPDR) |= 1 << HALL1_PIN1 * 2;
	GPIO(HALL2_PORT, PUPDR) |= 1 << HALL2_PIN2 * 2 | 1 << HALL2_PIN3 * 2;
	GPIO(HALL1_PORT, MODER) &= ~(3 << HALL1_PIN1 * 2);
	GPIO(HALL2_PORT, MODER) &= ~(3 << HALL2_PIN2 * 2 | 3 << HALL2_PIN3 * 2);
#endif
#else
	GPIO(HALL_PORT, PUPDR) |= 1 << HALL_PIN1 * 2 | 1 << HALL_PIN2 * 2 | 1 << HALL_PIN3 * 2;
	GPIO(HALL_PORT, MODER) &= ~(3 << HALL_PIN1 * 2 | 3 << HALL_PIN2 * 2 | 3 << HALL_PIN3 * 2);
#endif
#endif
#ifdef BEC_MAP
	int x = cfg.bec;
#if BEC_MAP == 0xADE // SWD pins
	if (!(GPIOA_IDR & 0x6000)) { // External pull-down
		GPIOA_ODR |= x << 13;
		GPIOA_OSPEEDR &= ~0x3c000000; // A13,A14 (low speed)
		GPIOA_PUPDR &= ~0x3c000000; // A13,A14 (no pull-up/pull-down)
		GPIOA_MODER ^= 0x3c000000; // A13,A14 (output)
	}
#else
	GPIO(BEC_PORT, ODR) |= (x & 1) << BEC_PIN1 | (x & 2) << (BEC_PIN2 - 1);
	GPIO(BEC_PORT, MODER) &= ~(2 << BEC_PIN1 * 2 | 2 << BEC_PIN2 * 2);
#endif
#endif
#ifdef RPM_PIN
	GPIO(RPM_PORT, MODER) = (GPIO(RPM_PORT, MODER) & ~(3 << RPM_PIN * 2)) | 1 << RPM_PIN * 2;
#endif
}

__attribute__((__weak__))
void initled(void) {
#ifdef LED1_PORT
#ifdef LED1_INV
	GPIO(LED1_PORT, ODR) |= 1 << LED1_PIN;
#endif
#ifdef LED1_OD
	GPIO(LED1_PORT, OTYPER) |= 1 << LED1_PIN;
#endif
	GPIO(LED1_PORT, MODER) &= ~(2 << LED1_PIN * 2);
#endif
#ifdef LED2_PORT
#ifdef LED2_INV
	GPIO(LED2_PORT, ODR) |= 1 << LED2_PIN;
#endif
#ifdef LED2_OD
	GPIO(LED2_PORT, OTYPER) |= 1 << LED2_PIN;
#endif
	GPIO(LED2_PORT, MODER) &= ~(2 << LED2_PIN * 2);
#endif
#ifdef LED3_PORT
#ifdef LED3_INV
	GPIO(LED3_PORT, ODR) |= 1 << LED3_PIN;
#endif
#ifdef LED3_OD
	GPIO(LED3_PORT, OTYPER) |= 1 << LED3_PIN;
#endif
	GPIO(LED3_PORT, MODER) &= ~(2 << LED3_PIN * 2);
#endif
#ifdef LED4_PORT
#ifdef LED4_INV
	GPIO(LED4_PORT, ODR) |= 1 << LED4_PIN;
#endif
#ifdef LED4_OD
	GPIO(LED4_PORT, OTYPER) |= 1 << LED4_PIN;
#endif
	GPIO(LED4_PORT, MODER) &= ~(2 << LED4_PIN * 2);
#endif
}

#ifdef HALL_MAP
int hallcode(void) {
#ifdef HALL1_PORT
	int x1 = GPIO(HALL1_PORT, IDR);
	int x2 = GPIO(HALL2_PORT, IDR);
#ifdef HALL1_PIN2
	return (x1 & (1 << HALL1_PIN1)) >> HALL1_PIN1 | (x1 & (1 << HALL1_PIN2)) >> (HALL1_PIN2 - 1) | (x2 & (1 << HALL2_PIN3)) >> (HALL2_PIN3 - 2);
#else
	return (x1 & (1 << HALL1_PIN1)) >> HALL1_PIN1 | (x2 & (1 << HALL2_PIN2)) >> (HALL2_PIN2 - 1) | (x2 & (1 << HALL2_PIN3)) >> (HALL2_PIN3 - 2);
#endif
#else
	int x = GPIO(HALL_PORT, IDR);
	return (x & (1 << HALL_PIN1)) >> HALL_PIN1 | (x & (1 << HALL_PIN2)) >> (HALL_PIN2 - 1) | (x & (1 << HALL_PIN3)) >> (HALL_PIN3 - 2);
#endif
}
#endif

__attribute__((__weak__))
void ledctl(int x) {
#ifdef LED1_PORT
#ifdef LED1_INV
	GPIO(LED1_PORT, BSRR) = x & 1 ? 1 << (LED1_PIN + 16) : 1 << LED1_PIN;
#else
	GPIO(LED1_PORT, BSRR) = x & 1 ? 1 << LED1_PIN : 1 << (LED1_PIN + 16);
#endif
#endif
#ifdef LED2_PORT
#ifdef LED2_INV
	GPIO(LED2_PORT, BSRR) = x & 2 ? 1 << (LED2_PIN + 16) : 1 << LED2_PIN;
#else
	GPIO(LED2_PORT, BSRR) = x & 2 ? 1 << LED2_PIN : 1 << (LED2_PIN + 16);
#endif
#endif
#ifdef LED3_PORT
#ifdef LED3_INV
	GPIO(LED3_PORT, BSRR) = x & 4 ? 1 << (LED3_PIN + 16) : 1 << LED3_PIN;
#else
	GPIO(LED3_PORT, BSRR) = x & 4 ? 1 << LED3_PIN : 1 << (LED3_PIN + 16);
#endif
#endif
#ifdef LED4_PORT
#ifdef LED4_INV
	GPIO(LED4_PORT, BSRR) = x & 8 ? 1 << (LED4_PIN + 16) : 1 << LED4_PIN;
#else
	GPIO(LED4_PORT, BSRR) = x & 8 ? 1 << LED4_PIN : 1 << (LED4_PIN + 16);
#endif
#endif
}

__attribute__((__weak__))
void hsictl(int x) {
	int cr = RCC_CR;
	int tv = (cr & 0xf8) >> 3; // 5 bits
	RCC_CR = (cr & ~0xf8) | clamp(tv + x, 0, 0x1f) << 3;
}

char crc8(const char *buf, int len) {
	static const char tbl[] = {
		0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
		0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
		0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
		0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
		0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
		0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
		0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
		0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
		0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
		0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
		0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
		0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
		0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
		0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
		0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
		0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
	};
	char crc = 0;
	while (len--) crc = tbl[crc ^ *buf++];
	return crc;
}

char crc8dvbs2(const char *buf, int len) {
	static const char tbl[] = {
		0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d,
		0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f,
		0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9,
		0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b,
		0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0,
		0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2,
		0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44,
		0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16,
		0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92,
		0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0,
		0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36,
		0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64,
		0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f,
		0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d,
		0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab,
		0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9,
	};
	char crc = 0;
	while (len--) crc = tbl[crc ^ *buf++];
	return crc;
}

int scale(int x, int a1, int a2, int b1, int b2) {
	if (x <= a1) return b1;
	if (x >= a2) return b2;
	return b1 + (x - a1) * (b2 - b1) / (a2 - a1);
}

int smooth(int *s, int x, int n) {
	int q;
	if (x < 0) return 0; // Sanity check
	if ((q = *s) < 0) *s = q = x << n; // Initialize state
	return (*s = x + q - (q >> n)) >> n;
}

void initpid(PID *pid, int x) {
	pid->i = 0;
	pid->x = x;
}

int calcpid(PID *pid, int x, int y) {
	int p = x - y; // Proportional error
	int l = pid->Li; // Integral error limit
	int i = clamp(pid->i + p, -l, l); // Integral error
	int d = x - pid->x; // Derivative error
	pid->i = i;
	pid->x = x;
	return pid->Kp * p + pid->Ki * i + pid->Kd * d;
}

void checkcfg(void) {
#ifndef ANALOG
#ifndef ANALOG_CHAN
	if (IO_ANALOG) cfg.arm = 1; // Ensure low level on startup
	else
#endif
	cfg.arm = !!cfg.arm;
#else
	cfg.arm = 0;
#endif
#ifdef PWM_ENABLE
	cfg.damp = 1;
#else
	cfg.damp = !!cfg.damp;
#endif
	cfg.revdir = !!cfg.revdir;
	cfg.brushed = !!cfg.brushed;
	cfg.timing = clamp(cfg.timing, 1, 31);
	cfg.sine_range = cfg.sine_range && cfg.damp && !cfg.brushed ? clamp(cfg.sine_range, 5, 25) : 0;
	cfg.sine_power = clamp(cfg.sine_power, 1, 15);
	cfg.freq_min = clamp(cfg.freq_min, 16, 48);
	cfg.freq_max = clamp(cfg.freq_max, cfg.freq_min, 96);
	cfg.duty_min = clamp(cfg.duty_min, 1, 100);
	cfg.duty_max = clamp(cfg.duty_max, cfg.duty_min, 100);
	cfg.duty_spup = clamp(cfg.duty_spup, 1, 100);
	cfg.duty_ramp = clamp(cfg.duty_ramp, 0, 100);
	cfg.duty_rate = clamp(cfg.duty_rate, 1, 100);
	cfg.duty_drag = clamp(cfg.duty_drag, 0, 100);
	cfg.duty_lock = cfg.duty_lock && cfg.damp && !cfg.brushed;
	cfg.throt_mode = clamp(cfg.throt_mode, 0, IO_ANALOG ? 0 : cfg.duty_lock ? 1 : 3);
	cfg.throt_rev = clamp(cfg.throt_rev, 0, 3);
	cfg.throt_brk = clamp(cfg.throt_brk, cfg.duty_drag, 100);
	cfg.throt_set = clamp(cfg.throt_set, 0, cfg.arm ? 0 : 100);
	cfg.throt_cal = !!cfg.throt_cal;
	cfg.throt_min = clamp(cfg.throt_min, 900, 1900);
	cfg.throt_max = clamp(cfg.throt_max, cfg.throt_min + 200, 2100);
	cfg.throt_mid = clamp(cfg.throt_mid, cfg.throt_min + 100, cfg.throt_max - 100);
	cfg.analog_min = clamp(cfg.analog_min, 0, 3200);
	cfg.analog_max = clamp(cfg.analog_max, cfg.analog_min + 200, 3400);
#ifdef IO_PA2
	cfg.input_mode = clamp(cfg.input_mode, 0, 5);
	cfg.input_chid = cfg.input_mode >= 3 ? clamp(cfg.input_chid, 1, cfg.input_mode == 3 ? 14 : 16) : 0;
#else
#if defined IO_PA6 || defined ANALOG_CHAN
	cfg.input_mode = clamp(cfg.input_mode, 0, 1);
#else
	cfg.input_mode = 0;
#endif
	cfg.input_chid = 0;
#endif
	cfg.telem_mode = clamp(cfg.telem_mode, 0, 4);
	cfg.telem_phid =
		cfg.telem_mode == 2 ? clamp(cfg.telem_phid, 1, 2):
		cfg.telem_mode == 3 ? clamp(cfg.telem_phid, 1, 28):
		cfg.input_mode == 4 ? clamp(cfg.telem_phid, 0, 4) : 0;
	cfg.telem_poles = clamp(cfg.telem_poles & ~1, 2, 100);
	cfg.prot_stall = cfg.prot_stall && !cfg.brushed ? clamp(cfg.prot_stall, 1500, 3500) : 0;
	cfg.prot_temp = cfg.prot_temp ? clamp(cfg.prot_temp, 60, 140) : 0;
#if SENS_CNT >= 3
	cfg.prot_sens = clamp(cfg.prot_sens, 0, 2);
#else
	cfg.prot_sens = 0;
#endif
#if SENS_CNT >= 1 && VOLT_MUL > 0
	cfg.prot_volt = cfg.prot_volt ? clamp(cfg.prot_volt, 28, 38) : 0;
	cfg.prot_cells = clamp(cfg.prot_cells, 0, 24);
#else
	cfg.prot_volt = 0;
	cfg.prot_cells = 0;
#endif
#if SENS_CNT >= 2 && CURR_MUL > 0
	cfg.prot_curr = clamp(cfg.prot_curr, 0, 500);
#else
	cfg.prot_curr = 0;
#endif
	cfg.volume = clamp(cfg.volume, 0, 100);
	cfg.beacon = clamp(cfg.beacon, 0, 100);
#ifdef BEC_MAP
	cfg.bec = clamp(cfg.bec, 0, 3);
#else
	cfg.bec = 0;
#endif
#if LED_CNT > 0
	cfg.led &= (1 << LED_CNT) - 1;
#else
	cfg.led = 0;
#endif
}

int savecfg(void) {
	if (ertm || busy) return 0;
	__disable_irq();
	FLASH_KEYR = FLASH_KEYR_KEY1;
	FLASH_KEYR = FLASH_KEYR_KEY2;
	FLASH_SR = -1; // Clear errors
	FLASH_CR = FLASH_CR_PER;
#ifdef STM32F0
	FLASH_AR = (uint32_t)_cfg;
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT; // Erase page
#else
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT | ((uint32_t)(_cfg - _boot) >> 11) << FLASH_CR_PNB_SHIFT; // Erase page
#endif
	while (FLASH_SR & FLASH_SR_BSY);
	FLASH_CR = FLASH_CR_PG;
#ifdef STM32F0
#define T uint16_t
#else
#define T uint32_t
#endif
	T *dst = (T *)_cfg;
	T *src = (T *)_cfg_start;
	T *end = (T *)_cfg_end;
#undef T
	while (src < end) { // Write data
		*dst++ = *src++;
#ifndef STM32F0
		*dst++ = *src++;
#endif
		while (FLASH_SR & FLASH_SR_BSY);
	}
	FLASH_CR = FLASH_CR_LOCK;
	__enable_irq();
#ifdef STM32F0
	if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return 0;
#else
	if (FLASH_SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR)) return 0;
#endif
	return !memcmp(_cfg, _cfg_start, _cfg_end - _cfg_start);
}

int resetcfg(void) {
	__disable_irq();
	memcpy(&cfg, &cfgdata, sizeof cfgdata);
	__enable_irq();
	return savecfg();
}

void resetcom(void) {
#ifdef PWM_ENABLE
	TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_HIGH | TIM_CCMR1_OC2M_FORCE_HIGH;
	TIM1_CCMR2 = TIM_CCMR2_OC3M_FORCE_HIGH;
#else
	TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW | TIM_CCMR1_OC2M_FORCE_LOW;
	TIM1_CCMR2 = TIM_CCMR2_OC3M_FORCE_LOW;
#endif
	int er = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
#ifdef INVERTED_HIGH
	er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
	er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	TIM1_CCER = er;
	TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
}

static void delayf(void) {
	TIM6_EGR = TIM_EGR_UG; // Reset arming timeout
	if (!(TIM1_SR & TIM_SR_UIF)) return;
	TIM1_SR = ~TIM_SR_UIF;
	int a = TIM1_CCR1;
	int b = TIM1_CCR3;
	TIM1_CCR1 = b;
	TIM1_CCR3 = a;
}

int playmusic(const char *str, int vol) {
	static const uint16_t arr[] = {15287, 14429, 13619, 12856, 12133, 11452, 10810, 10203, 9630, 9090, 8579, 8097, 7643};
	char *end;
	int tmp = strtol(str, &end, 10); // Tempo
	if (str == end) tmp = 125; // 120 BPM by default
	else {
		if (tmp < 10 || tmp > 999) return 0; // Sanity check
		tmp = 15000 / tmp;
		str = end;
	}
	if (!vol || ertm || busy) return 0;
	busy = 1;
	resetcom();
#ifdef PWM_ENABLE
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_LOW;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
	int er = TIM_CCER_CC1E | TIM_CCER_CC2NE | TIM_CCER_CC3E;
#else
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_HIGH;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
	int er = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
#endif
#ifdef INVERTED_HIGH
	er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
	er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	TIM1_CCER = er;
	TIM1_PSC = CLK_MHZ / 8 - 1; // 125ns resolution
	for (int a, b, c = 0; (a = *str++);) {
		if (a >= 'a' && a <= 'g') a -= 'c', b = 0; // Low note
		else if (a >= 'A' && a <= 'G') a -= 'C', b = 1; // High note
		else if (a == '_') { // Pause
			TIM1_CCR1 = 0;
			TIM1_CCR3 = 0;
			goto update;
		} else {
			if (a == '+' && !c++) continue; // Octave up
			if (a == '-' && c--) continue; // Octave down
			break; // Invalid specifier
		}
		a = (a + 7) % 7 << 1;
		if (a > 4) --a;
		if (*str == '#') ++a, ++str;
		TIM1_ARR = arr[a] >> (b + c); // Frequency
		TIM1_CCR1 = (DEAD_TIME + CLK_MHZ / 8 - 1) * 8 / CLK_MHZ + vol; // Volume
		TIM1_CCR3 = 0;
	update:
		TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
		a = strtol(str, &end, 10); // Duration
		if (str == end) a = 1;
		else {
			if (a < 1 || a > 99) break; // Sanity check
			str = end;
		}
		delay(tmp * a, delayf);
	}
	TIM1_PSC = 0;
	TIM1_ARR = CLK_KHZ / 24 - 1;
	resetcom();
	busy = 0;
	return !str[-1];
}

void playsound(const char *buf, int vol) { // AU file format, 8-bit linear PCM, mono
	const uint32_t *hdr = (const uint32_t *)buf;
	if (hdr[0] != 0x646e732e || hdr[3] != 0x2000000 || hdr[5] != 0x1000000 || !vol || ertm || busy) return;
	busy = 1;
	resetcom();
#ifdef PWM_ENABLE
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_LOW;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
	int er = TIM_CCER_CC1E | TIM_CCER_CC2NE | TIM_CCER_CC3E;
#else
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_HIGH;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
	int er = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
#endif
#ifdef INVERTED_HIGH
	er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
	er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	TIM1_CCER = er;
	TIM1_CCR1 = 0;
	TIM1_CCR3 = 0;
	TIM1_ARR = CLK_KHZ / 24 - 1;
	TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
	TIM6_PSC = 0;
	TIM6_ARR = CLK_CNT(__builtin_bswap32(hdr[4])) - 1;
	TIM6_EGR = TIM_EGR_UG;
	TIM6_CR1 = TIM_CR1_CEN;
	buf += __builtin_bswap32(hdr[1]);
	for (int len = __builtin_bswap32(hdr[2]);;) {
		if (!(TIM6_SR & TIM_SR_UIF)) continue;
		TIM6_SR = ~TIM_SR_UIF;
		if (len-- <= 0) break;
		int8_t x = *buf++;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_CCR1 = DEAD_TIME + ((x + 128) * vol * CLK_MHZ >> 13);
		TIM1_CCR3 = DEAD_TIME + ((127 - x) * vol * CLK_MHZ >> 13);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	}
	TIM6_CR1 = 0;
	resetcom();
	busy = 0;
}
