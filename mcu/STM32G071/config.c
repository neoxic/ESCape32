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
#include <libopencm3/stm32/dmamux.h>
#include "common.h"

#define COMP1_CSR MMIO32(COMP_BASE + 0x00)
#define COMP2_CSR MMIO32(COMP_BASE + 0x04)

static volatile uint32_t *comp_csr = &COMP1_CSR;

void init(void) {
	RCC_APBRSTR2 = -1;
	RCC_APBRSTR1 = -1;
	RCC_APBRSTR2 = 0;
	RCC_APBRSTR1 = 0;
	RCC_IOPENR = 0x03; // GPIOAEN=1, GPIOBEN=1
	RCC_AHBENR = RCC_AHBENR_FLASHEN | RCC_AHBENR_DMAEN;
	RCC_APBENR2 = RCC_APBENR2_SYSCFGEN | RCC_APBENR2_TIM1EN | RCC_APBENR2_USART1EN | RCC_APBENR2_TIM16EN | RCC_APBENR2_ADCEN;
	RCC_APBENR1 = RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM3EN | RCC_APBENR1_WWDGEN;
	SYSCFG_CFGR1 = SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP; // A11->A9, A12->A10
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x20000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000002; // B0 (TIM1_CH2N)
	GPIOB_AFRH = 0x00000000;
	GPIOA_PUPDR = 0x24000000;
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffe; // B0 (TIM1_CH2N), B6 (USART1_TX)
#ifdef IO_PA6
	GPIOA_AFRL |= 0x01000000; // A6 (TIM3_CH1)
	GPIOB_AFRH |= 0x20000000; // B15 (TIM1_CH3N)
	GPIOA_PUPDR |= 0x00002000; // A6 (pull-down)
	GPIOA_MODER &=~ 0x00001000; // A6 (TIM3_CH1)
	GPIOB_MODER &=~ 0x40000000; // B15 (TIM1_CH3N)
#else
	GPIOB_AFRL |= 0x00010020; // B1 (TIM1_CH3N), B4 (TIM3_CH1)
	GPIOB_PUPDR |= 0x00000200; // B4 (pull-down)
	GPIOB_MODER &=~ 0x00000104; // B1 (TIM1_CH3N), B4 (TIM3_CH1)
#endif

	WWDG_CFR = 0x207f; // Watchdog timeout 4096*16*64/PCLK=~66ms

	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_TIM3_CH1;
	DMAMUX1_CxCR(2) = DMAMUX_CxCR_DMAREQ_ID_USART1_TX;
	DMAMUX1_CxCR(3) = DMAMUX_CxCR_DMAREQ_ID_USART1_RX;

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0x40);

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
}

void comp_in(int x) {
	int cr = 0;
	comp_out(0); // Disable output before switching comparators
	switch (x & 3) {
#ifdef IO_PA6
		case COMP_IN1:
			comp_csr = &COMP1_CSR;
			cr = 0x281; // A0
			break;
		case COMP_IN2:
			comp_csr = &COMP2_CSR;
			cr = 0x281; // A2
			break;
#else
		case COMP_IN1:
			comp_csr = &COMP2_CSR;
			cr = 0x281; // A2
			break;
		case COMP_IN2:
			comp_csr = &COMP2_CSR;
			cr = 0x261; // B3
			break;
#endif
		case COMP_IN3:
			comp_csr = &COMP2_CSR;
			cr = 0x271; // B7
			break;
	}
	if (x & 4) cr |= 0x8000; // Polarity
	*comp_csr = cr;
}

void comp_out(int on) {
	TIM2_TISEL = on ? comp_csr == &COMP2_CSR ? 0x100 : 0x01 : 0;
}

#ifdef IO_PA6
void io_pullup() {
	GPIOA_PUPDR &= ~0x00003000;
	GPIOA_PUPDR |= 0x00001000; // A6 (pull-up)
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
