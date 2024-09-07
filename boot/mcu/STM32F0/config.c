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
	RCC_AHBENR = RCC_AHBENR_CRCEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	RCC_APB2ENR = RCC_APB2ENR_TIM1EN;

	FLASH_ACR = FLASH_ACR_LATENCY_1WS | FLASH_ACR_PRFTEN;
	RCC_CFGR = RCC_CFGR_PLLMUL_MUL12;
	RCC_CR |= RCC_CR_PLLON;
	while (!(RCC_CR & RCC_CR_PLLRDY));
	RCC_CFGR |= RCC_CFGR_SW_PLL;

	GPIOA_MODER = 0xebffffff;
	GPIOB_MODER = 0xffffffff;
#ifdef IO_PA2
	RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA_AFRL |= 0x100; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x10000000; // A15 (USART2_RX)
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (USART2_TX)
#else
	RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOB_AFRL |= 0x10000; // B4 (TIM3_CH1)
	GPIOB_PUPDR |= 0x100; // B4 (pull-up)
	GPIOB_MODER &= ~0x100; // B4 (TIM3_CH1)
#endif
}
