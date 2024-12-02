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
	RCC_AHB1ENR = RCC_AHB1ENR_FLASHEN | RCC_AHB1ENR_CRCEN;
	RCC_AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	RCC_APB1ENR1 = RCC_APB1ENR1_USART2EN;
	RCC_APB2ENR = RCC_APB2ENR_TIM1EN;

	FLASH_ACR = 4 | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_DBG_SWEN; // LATENCY=4
	RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI16 | 10 << RCC_PLLCFGR_PLLN_SHIFT | RCC_PLLCFGR_PLLREN;
	RCC_CR |= RCC_CR_HSION | RCC_CR_PLLON;
	while (!(RCC_CR & (RCC_CR_HSIRDY | RCC_CR_PLLRDY)));
	RCC_CFGR = RCC_CFGR_SWx_PLL;

	GPIOA_AFRL = 0x00000700; // A2 (USART2_TX)
	GPIOA_AFRH = 0x70000000; // A15 (USART2_RX)
	GPIOA_PUPDR = 0x24000010; // A2 (pull-up)
	GPIOB_PUPDR = 0x00000000;
	GPIOA_MODER = 0xebffffef; // A2 (USART2_TX)
	GPIOB_MODER = 0xffffffff;
}
