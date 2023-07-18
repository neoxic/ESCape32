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

#include "common.h"

__attribute__((__weak__))
void hsictl(int x) {
	int cr = RCC_CR;
	int tv = (cr & 0xf8) >> 3; // 5 bits
	RCC_CR = (cr & ~0xf8) | ((tv + x) & 0x1f) << 3;
}

int clamp(int x, int a1, int a2) {
	if (x <= a1) return a1;
	if (x >= a2) return a2;
	return x;
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
	cfg.arm = !!cfg.arm;
	cfg.damp = !!cfg.damp;
	cfg.revdir = !!cfg.revdir;
#ifdef SENSORED
	cfg.timing = 0;
	cfg.sine_range = 0;
	cfg.sine_power = 0;
#else
	cfg.timing = clamp(cfg.timing, 1, 7);
	cfg.sine_range = cfg.damp && cfg.sine_range ? clamp(cfg.sine_range, 5, 25) : 0;
	cfg.sine_power = clamp(cfg.sine_power, 1, 15);
#endif
	cfg.freq_min = clamp(cfg.freq_min, 16, 48);
	cfg.freq_max = clamp(cfg.freq_max, cfg.freq_min, 96);
	cfg.duty_min = clamp(cfg.duty_min, 1, 100);
	cfg.duty_max = clamp(cfg.duty_max, cfg.duty_min, 100);
	cfg.duty_spup = clamp(cfg.duty_spup, 1, 25);
	cfg.duty_ramp = clamp(cfg.duty_ramp, 1, 100);
	cfg.duty_drag = clamp(cfg.duty_drag, 0, 100);
	cfg.throt_mode = clamp(cfg.throt_mode, 0, 2);
	cfg.throt_cal = !!cfg.throt_cal;
	cfg.throt_min = clamp(cfg.throt_min, 900, 1900);
	cfg.throt_max = clamp(cfg.throt_max, cfg.throt_min + 200, 2100);
	cfg.throt_mid = clamp(cfg.throt_mid, cfg.throt_min + 100, cfg.throt_max - 100);
#ifdef IO_PA2
	cfg.input_mode = clamp(cfg.input_mode, 0, 4);
	cfg.input_chid = cfg.input_mode >= 3 ? clamp(cfg.input_chid, 1, cfg.input_mode == 4 ? 16 : 14) : 0;
#else
#ifdef IO_PA6
	cfg.input_mode = clamp(cfg.input_mode, 0, 1);
#else
	cfg.input_mode = 0;
#endif
	cfg.input_chid = 0;
#endif
	cfg.telem_mode = clamp(cfg.telem_mode, 0, 3);
	cfg.telem_phid = cfg.telem_mode == 3 ? clamp(cfg.telem_phid, 1, 28) : 0;
	cfg.telem_poles = clamp(cfg.telem_poles & ~1, 2, 100);
	cfg.prot_temp = cfg.prot_temp ? clamp(cfg.prot_temp, 60, 140) : 0;
#if SENS_CNT >= 1
	cfg.prot_volt = cfg.prot_volt ? clamp(cfg.prot_volt, 28, 38) : 0;
	cfg.prot_cells = clamp(cfg.prot_cells, 0, 16);
#else
	cfg.prot_volt = 0;
	cfg.prot_cells = 0;
#endif
#if SENS_CNT < 2
	cfg.prot_curr = 0;
#endif
	cfg.volume = clamp(cfg.volume, 0, 100);
	cfg.beacon = clamp(cfg.beacon, 0, 100);
#if LED_CNT > 0
	cfg.led &= (1 << LED_CNT) - 1;
#else
	cfg.led = 0;
#endif
#ifndef ANALOG
	if (!IO_ANALOG) return;
	cfg.arm = 1; // Ensure low level on startup
	cfg.throt_mode = 0;
#endif
}

int savecfg(void) {
	if (ertm) return 0;
	__disable_irq();
	WWDG_CFR = ~WWDG_CFR_EWI; // Slow down watchdog
	WWDG_CR = 0xff;
	FLASH_KEYR = FLASH_KEYR_KEY1;
	FLASH_KEYR = FLASH_KEYR_KEY2;
	FLASH_SR = -1; // Clear errors
	FLASH_CR = FLASH_CR_PER;
#ifdef STM32G0
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT | ((uint32_t)(_cfg - _boot) >> 11) << FLASH_CR_PNB_SHIFT; // Erase page
#else
	FLASH_AR = (uint32_t)_cfg;
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT; // Erase page
#endif
	while (FLASH_SR & FLASH_SR_BSY) WWDG_CR = 0xff;
	FLASH_CR = FLASH_CR_PG;
#ifdef STM32G0
#define T uint32_t
#else
#define T uint16_t
#endif
	T *dst = (T *)_cfg;
	T *src = (T *)_cfg_start;
	T *end = (T *)_cfg_end;
#undef T
	while (src < end) { // Write data
		*dst++ = *src++;
#ifdef STM32G0
		*dst++ = *src++;
#endif
		while (FLASH_SR & FLASH_SR_BSY) WWDG_CR = 0xff;
	}
	FLASH_CR = FLASH_CR_LOCK;
	WWDG_CFR = 0x7f; // Resume watchdog
	__enable_irq();
#ifdef STM32G0
	if (FLASH_SR & (FLASH_SR_PROGERR | FLASH_SR_WRPERR)) return 0;
#else
	if (FLASH_SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) return 0;
#endif
	return !memcmp(_cfg, _cfg_start, _cfg_end - _cfg_start);
}

int resetcfg(void) {
	__disable_irq();
	memcpy(&cfg, &cfgdata, sizeof cfgdata);
	__enable_irq();
	return savecfg();
}

int playmusic(const char *str, int vol) {
	static const uint16_t arr[] = {30575, 28859, 27240, 25713, 24268, 22906, 21621, 20407, 19261, 18181, 17160, 16196, 15287};
	static char flag;
	if (!vol || ertm || flag) return 0;
	flag = 1;
	vol <<= 1;
	TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_HIGH | TIM_CCMR1_OC2M_FORCE_LOW;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
#ifdef INVERTED_HIGH
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC3E | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#else
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC3E;
#endif
	TIM1_PSC = CLK_MHZ / 8 - 1; // 8MHz
	for (int a, b; (a = *str++);) {
		if (a >= 'a' && a <= 'g') a -= 'c', b = 0; // Low note
		else if (a >= 'A' && a <= 'G') a -= 'C', b = 1; // High note
		else if (a == '_') goto set; // Pause
		else break;
		a = (a + 7) % 7 << 1;
		if (a > 4) --a;
		if (*str == '#') ++a, ++str;
		TIM1_ARR = arr[a] >> b; // Frequency
		TIM1_CCR3 = vol; // Volume
	set:
		TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
		a = *str;
		if (a >= '1' && a <= '8') a -= '0', ++str;
		else a = 1;
		for (uint32_t t = tickms + a * 125; t != tickms;) { // Duration 125*X ms
			if (TIM14_CR1 & TIM_CR1_CEN) TIM14_EGR = TIM_EGR_UG; // Reset arming timeout
			IWDG_KR = IWDG_KR_RESET;
			WWDG_CR = 0xff;
		}
		TIM1_CCR3 = 0; // Preload silence
	}
	TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW | TIM_CCMR1_OC2M_FORCE_LOW;
	TIM1_CCMR2 = TIM_CCMR2_OC3M_FORCE_LOW;
	TIM1_PSC = 0;
	TIM1_ARR = CLK_KHZ / 24 - 1;
	TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
	flag = 0;
	return !str[-1];
}

void reset(void) {
	WWDG_CR = WWDG_CR_WDGA; // Trigger watchdog reset
	for (;;); // Never return
}
