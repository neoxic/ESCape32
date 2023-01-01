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

#define COMP_CSR MMIO32(SYSCFG_COMP_BASE + 0x1c)

void init(void) {
	RCC_APB2RSTR = -1;
	RCC_APB1RSTR = -1;
	RCC_APB2RSTR = 0;
	RCC_APB1RSTR = 0;
	RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	RCC_APB2ENR = RCC_APB2ENR_SYSCFGCOMPEN | RCC_APB2ENR_ADCEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM16EN;
	RCC_APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_WWDGEN;
	SYSCFG_CFGR1 = SYSCFG_CFGR1_MEM_MODE_SRAM; // Map SRAM at 0x00000000
	memcpy(_vec, _rom, _ram - _vec); // Copy vector table to SRAM

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x20000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000022; // B0 (TIM1_CH2N), B1 (TIM1_CH3N)
	GPIOB_AFRH = 0x00000000;
	GPIOA_PUPDR = 0x24000000;
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffa; // B0 (TIM1_CH2N), B1 (TIM1_CH3N), B6 (USART1_TX)
#ifdef IO_PA2
	RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
	GPIOA_PUPDR |= 0x00000020; // A2 (pull-down)
	GPIOA_MODER &=~ 0x00000010; // A2 (TIM15_CH1)
#else
	RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOB_AFRL |= 0x00010000; // B4 (TIM3_CH1)
	GPIOB_PUPDR |= 0x00000200; // B4 (pull-down)
	GPIOB_MODER &=~ 0x00000100; // B4 (TIM3_CH1)
#endif

	WWDG_CFR = 0x1ff; // Watchdog timeout 4096*8*64/PCLK=~43ms

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x40);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40);

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_TIM15_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);
}

void comp_in(int x) {
	int cr = 0;
	switch (x & 3) {
		case COMP_IN1:
			cr = 0x61; // A0
			break;
		case COMP_IN2:
			cr = 0x41; // A4
			break;
		case COMP_IN3:
			cr = 0x51; // A5
			break;
	}
	if (x & 4) cr |= 0x800; // Polarity
	COMP_CSR = cr;
}

void comp_out(int on) {
	if (on) COMP_CSR |= 0x400; // On
	else COMP_CSR &= ~0x700; // Off
}

#ifdef IO_PA2
void io_serial(void) {
	TIM15_DIER = 0;
	nvic_clear_pending_irq(NVIC_TIM15_IRQ);
	RCC_APB2ENR &= ~RCC_APB2ENR_TIM15EN;
	RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA_AFRL |= 0x00000100; // A2 (USART2_TX)
}

void io_pullup() {
	GPIOA_PUPDR &= ~0x00000030;
	GPIOA_PUPDR |= 0x00000010; // A2 (pull-up)
}
#else
void io_pullup() {
	GPIOB_PUPDR &= ~0x00000300;
	GPIOB_PUPDR |= 0x00000100; // B4 (pull-up)
}
#endif

void tim1_brk_up_trg_com_isr(void) {
	int sr = TIM1_SR;
	if (sr & TIM_SR_UIF) tim1_up_isr();
	if (sr & TIM_SR_COMIF) tim1_com_isr();
}

void dma1_channel4_7_dma2_channel3_5_isr(void) {
#ifdef IO_PA2
	if (RCC_APB1ENR & RCC_APB1ENR_USART2EN) { // TIM15 and USART2_RX share channel 5
		usart2_dma_isr();
		return;
	}
#endif
	iotim_dma_isr();
}
