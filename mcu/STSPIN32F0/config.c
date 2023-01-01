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

#include <libopencm3/stm32/syscfg.h>
#include "common.h"

void init(void) {
	RCC_APB2RSTR = -1;
	RCC_APB1RSTR = -1;
	RCC_APB2RSTR = 0;
	RCC_APB1RSTR = 0;
	RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;
	RCC_APB2ENR = RCC_APB2ENR_SYSCFGCOMPEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM16EN;
	RCC_APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_WWDGEN;
	memcpy(_vec, _rom, _ram - _vec); // Copy vector table to SRAM
	SYSCFG_CFGR1 = SYSCFG_CFGR1_MEM_MODE_SRAM; // Map SRAM at 0x00000000

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x01000222; // A0 (TIM2_CH1), A1 (TIM2_CH2), A2 (TIM2_CH3), A6 (TIM3_CH1)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000000;
	GPIOB_AFRH = 0x22200000; // B13 (TIM1_CH1N), B14 (TIM1_CH2N), B15 (TIM1_CH3N)
	GPIOF_ODR = 0x00c0; // F7,F6=11 (maximum over-current threshold)
	GPIOA_PUPDR = 0x24002015; // A0,A1,A2 (pull-up), A6 (pull-down)
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeaefea; // A0 (TIM2_CH1), A1 (TIM2_CH2), A2 (TIM2_CH3), A6 (TIM3_CH1), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xabffefff; // B6 (USART1_TX), B13 (TIM1_CH1N), B14 (TIM1_CH2N), B15 (TIM1_CH3N)
	GPIOF_MODER = 0xffff5fff; // F6,F7 (output)

	WWDG_CFR = 0x1ff; // Watchdog timeout 4096*8*64/PCLK=~43ms

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40);

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);
}

void io_pullup() {
	GPIOA_PUPDR &= ~0x00003000;
	GPIOA_PUPDR |= 0x00001000; // A6 (pull-up)
}
