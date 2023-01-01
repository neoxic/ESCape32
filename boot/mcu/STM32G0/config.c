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

void init(void) {
	rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

	RCC_AHBENR = RCC_AHBENR_FLASHEN | RCC_AHBENR_CRCEN;
	RCC_APBENR1 = RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN;

#ifdef IO_PA6
	RCC_IOPENR = 0x01; // GPIOAEN=1
	GPIOA_AFRL = 0x01000000; // A6 (TIM3_CH1)
	GPIOA_PUPDR = 0x24001000; // A6 (pull-up)
	GPIOA_MODER = 0xebffefff; // A6 (TIM3_CH1)
#else
	RCC_IOPENR = 0x02; // GPIOBEN=1
	GPIOB_AFRL = 0x00010000; // B4 (TIM3_CH1)
	GPIOB_PUPDR = 0x00000100; // B4 (pull-up)
	GPIOB_MODER = 0xfffffeff; // B4 (TIM3_CH1)
#endif
}
