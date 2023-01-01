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

void loadcfg(void) {
	memcpy(_cfg_start, _cfg, _cfg_end - _cfg_start);
	checkcfg();
}

void checkcfg(void) {
	cfg.arm = !!cfg.arm;
	cfg.damp = !!cfg.damp;
	cfg.reverse = !!cfg.reverse;
#ifdef SENSORED
	cfg.timing = 0;
#else
	cfg.timing = clamp(cfg.timing, 0, 7);
#endif
	cfg.freq_min = clamp(cfg.freq_min, 16, 96);
	cfg.freq_max = clamp(cfg.freq_max, cfg.freq_min, 96);
	cfg.duty_min = clamp(cfg.duty_min, 0, 100);
	cfg.duty_max = clamp(cfg.duty_max, cfg.duty_min, 100);
	cfg.duty_lim = clamp(cfg.duty_lim, 1, 200);
	cfg.duty_drag = clamp(cfg.duty_drag, 0, 100);
	cfg.throt_mode = clamp(cfg.throt_mode, 0, 2);
	cfg.throt_cal = !!cfg.throt_cal;
	cfg.throt_min = clamp(cfg.throt_min, 900, 1900);
	cfg.throt_max = clamp(cfg.throt_max, cfg.throt_min + 200, 2100);
	cfg.throt_mid = clamp(cfg.throt_mid, cfg.throt_min + 100, cfg.throt_max - 100);
#ifdef IO_PA2
	cfg.serial_mode = clamp(cfg.serial_mode, 0, 3);
	cfg.serial_chid = cfg.serial_mode >= 2 ? clamp(cfg.serial_chid, 1, cfg.serial_mode == 3 ? 16 : 14) : 0;
#else
	cfg.serial_mode = 0;
	cfg.serial_chid = 0;
#endif
	cfg.volume1 = clamp(cfg.volume1, 0, 100);
	cfg.volume2 = clamp(cfg.volume2, 0, 100);
	cfg.volume3 = clamp(cfg.volume3, 0, 100);
}

int savecfg(void) {
	FLASH_KEYR = FLASH_KEYR_KEY1;
	FLASH_KEYR = FLASH_KEYR_KEY2;
	FLASH_CR = FLASH_CR_PER;
#ifdef STM32G0
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT | ((uint32_t)(_cfg - _boot) >> 11) << FLASH_CR_PNB_SHIFT; // Erase page
#else
	FLASH_AR = (uint32_t)_cfg;
	FLASH_CR = FLASH_CR_PER | FLASH_CR_STRT; // Erase page
#endif
	while (FLASH_SR & FLASH_SR_BSY);
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
		while (FLASH_SR & FLASH_SR_BSY);
	}
	FLASH_CR = FLASH_CR_LOCK;
	return !memcmp(_cfg, _cfg_start, _cfg_end - _cfg_start);
}

int playmusic(const char *str, int vol) {
	static const uint16_t arr[] = {
		61152, 57719, 54480, 51426, 48537, 45813, 43242, 40815, 38524, 36363, 34322, 32393,
		30575, 28859, 27240, 25713, 24268, 22906, 21621, 20407, 19261, 18181, 17160, 16196,
		15287,
	};
	static int flag;
	if (!vol || erpt || flag) return 0;
	flag = 1;
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1;
	TIM1_CCMR2 = TIM_CCMR2_OC3M_FORCE_HIGH;
	TIM1_CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC3NE;
#ifdef INVERTED_HIGH
	TIM1_CCER |= TIM_CCER_CC1P;
#endif
#ifdef INVERTED_LOW
	TIM1_CCER |= TIM_CCER_CC3NP;
#endif
	TIM1_PSC = MHZ(PCLK2) / 16 - 1; // 16 MHz
	TIM16_PSC = KHZ(PCLK2) / 8 - 1; // Tempo (125us resolution)
	TIM16_EGR = TIM_EGR_UG;
	for (int a, b; (a = *str++);) {
		if (a >= 'a' && a <= 'g') a -= 'c', b = 0; // Low note
		else if (a >= 'A' && a <= 'G') a -= 'C', b = 12; // High note
		else if (a == '_') goto set; // Pause
		else break;
		a = (a + 7) % 7 << 1;
		if (a > 4) --a;
		if (*str == '#') ++a, ++str;
		TIM1_ARR = arr[a + b]; // Frequency
		TIM1_CCR1 = vol << 2; // Volume
	set:
		TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
		a = *str;
		if (a >= '1' && a <= '8') a -= '0', ++str;
		else a = 1;
		TIM16_ARR = a * 1000 - 1; // Duration
		TIM16_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
		while (TIM16_CR1 & TIM_CR1_CEN) { // Keep watchdogs happy
			IWDG_KR = IWDG_KR_RESET;
			if (WWDG_CR & WWDG_CR_WDGA) WWDG_CR = 0xff;
		}
		TIM1_CCR1 = 0; // Preload silence
	}
	TIM1_CCMR1 = 0;
	TIM1_CCMR2 = 0;
	TIM1_CCER = 0;
	TIM1_PSC = 0;
	TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
	flag = 0;
	return !str[-1];
}

void reset(void) {
	WWDG_CR = WWDG_CR_WDGA; // Reset via watchdog
	for (;;); // Never return
}

void error(int code) {
	TIM1_EGR = TIM_EGR_BG; // Shut down bridge
	cfg.last_error = code;
	savecfg();
	reset();
}
