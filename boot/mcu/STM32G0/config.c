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

void init(void) {
	RCC_IOPENR = 0x3; // GPIOAEN=1, GPIOBEN=1
	RCC_AHBENR = RCC_AHBENR_FLASHEN | RCC_AHBENR_CRCEN;
	RCC_APBENR2 = RCC_APBENR2_TIM1EN;

	FLASH_ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DBG_SWEN;
	RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI16 | 8 << RCC_PLLCFGR_PLLN_SHIFT | RCC_PLLCFGR_PLLREN | 1 << RCC_PLLCFGR_PLLR_SHIFT;
	RCC_CR |= RCC_CR_PLLON;
	while (!(RCC_CR & RCC_CR_PLLRDY));
	RCC_CFGR = RCC_CFGR_SW_PLLRCLK;

#ifdef IO_PA2
	RCC_APBENR1 |= RCC_APBENR1_USART2EN;
	GPIOA_AFRL |= 0x100; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x10000000; // A15 (USART2_RX)
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (USART2_TX)
#else
	RCC_APBENR1 |= RCC_APBENR1_TIM3EN;
#ifdef IO_PA6
	GPIOA_AFRL |= 0x1000000; // A6 (TIM3_CH1)
	GPIOA_PUPDR |= 0x1000; // A6 (pull-up)
	GPIOA_MODER &= ~0x1000; // A6 (TIM3_CH1)
#else
	GPIOB_AFRL |= 0x10000; // B4 (TIM3_CH1)
	GPIOB_PUPDR |= 0x100; // B4 (pull-up)
	GPIOB_MODER &= ~0x100; // B4 (TIM3_CH1)
#endif
#endif
}
