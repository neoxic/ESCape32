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

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/adc.h>
#include "common.h"

#if SENS_MAP == 0xA3 // A3 (volt)
#define SENS_CHAN 0x3
#elif SENS_MAP == 0xA6A0 // A6 (volt), A0 (curr)
#define SENS_CHAN 0xc0
#endif

#define ADC1_BASE ADC_BASE
#define COMP_CSR MMIO32(SYSCFG_COMP_BASE + 0x1c)
#ifdef USE_COMP2
#define COMP_SHIFT 16
#else
#define COMP_SHIFT 0
#endif

static char len, ain;
static uint16_t buf[5];

void init(void) {
	RCC_APB2RSTR = -1;
	RCC_APB1RSTR = -1;
	RCC_APB2RSTR = 0;
	RCC_APB1RSTR = 0;
	RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	RCC_APB2ENR = RCC_APB2ENR_SYSCFGCOMPEN | RCC_APB2ENR_ADCEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
	RCC_APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM6EN | RCC_APB1ENR_WWDGEN;
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

	RCC_CFGR &= ~RCC_CFGR_SW_PLL;
	while (RCC_CFGR & RCC_CFGR_SWS_PLL);
	RCC_CR &= ~RCC_CR_PLLON;
	RCC_CFGR = 0x8240000; // PLLMUL=11001 (x26)
	RCC_CR |= RCC_CR_PLLON;
	while (!(RCC_CR & RCC_CR_PLLRDY));
	RCC_CFGR |= RCC_CFGR_SW_PLL;

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x20000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000022; // B0 (TIM1_CH2N), B1 (TIM1_CH3N)
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffa; // B0 (TIM1_CH2N), B1 (TIM1_CH3N), B6 (USART1_TX)
#ifndef ANALOG
#ifdef IO_PA2
	RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (TIM15_CH1)
#else
	RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOB_AFRL |= 0x10000; // B4 (TIM3_CH1)
	GPIOB_PUPDR |= 0x100; // B4 (pull-up)
	GPIOB_MODER &= ~0x100; // B4 (TIM3_CH1)
#endif
#endif

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x80);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x80); // ADC
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x80); // USART1_TX
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40); // TIM3 or TIM15 or USART2_RX

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_TIM15_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);

	RCC_CR2 |= RCC_CR2_HSI14ON; // Enable IRC28M
	while (!(RCC_CR2 & RCC_CR2_HSI14RDY));
	ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
	TIM6_ARR = CLK_MHZ - 1;
	TIM6_SR = ~TIM_SR_UIF;
	TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
	while (TIM6_CR1 & TIM_CR1_CEN); // Wait for 1us (RM 11.4.1)
	ADC1_CR2 |= ADC_CR2_CAL;
	while (ADC1_CR2 & ADC_CR2_CAL);
	ADC1_CR1 = ADC_CR1_SCAN;
	ADC1_SMPR1 = -1; // Sampling time ~17us @ IRC28M/2=14Mhz
	ADC1_SMPR2 = -1;
	ADC1_SQR3 = SENS_CHAN;
	len = SENS_CNT;
	if (IO_ANALOG) {
		ADC1_SQR3 |= AIN_CHAN << (len++ * 5);
		ain = 1;
	}
	ADC1_SQR3 |= 0x230 << (len * 5); // CH17 (vref), CH16 (temp)
	len += 2;
	ADC1_SQR1 = (len - 1) << ADC_SQR1_L_LSB;
	DMA1_CPAR(1) = (uint32_t)&ADC1_DR;
	DMA1_CMAR(1) = (uint32_t)buf;

	TIM1_DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC4IE; // Software comparator blanking, ADC trigger
	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2
	TIM2_CR2 = TIM_CR2_MMS_COMPARE_OC1REF; // TRGO=OC1REF
	TIM2_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2; // Inverted PWM on OC1
	TIM2_CCMR2 = TIM_CCMR2_CC4S_IN_TI4;
	TIM2_CCER = TIM_CCER_CC4E; // IC4 on rising edge on TI4 (COMP_OUT)
}

void compctl(int x) {
	int cr = 0;
	switch (x & 3) {
		case COMP_IN1:
			cr = 0x61; // A1>A0
			break;
		case COMP_IN2:
			cr = 0x41; // A1>A4
			break;
		case COMP_IN3:
			cr = 0x51; // A1>A5
			break;
	}
	if (x & 4) cr |= 0x800; // Change polarity
	COMP_CSR = cr << COMP_SHIFT;
}

void io_serial(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM15_IRQ);
	RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
	GPIOA_AFRL |= 0x100; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x10000000; // A15 (USART2_RX)
}

void io_analog(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM15_IRQ);
	GPIOA_PUPDR &= ~0x30; // A2 (no pull-up/pull-down)
	GPIOA_MODER |= 0x30; // A2 (analog)
}

void adctrig(void) {
	if (DMA1_CCR(1) & DMA_CCR_EN) return;
	DMA1_CNDTR(1) = len;
	DMA1_CCR(1) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE | ADC_CR2_DMA | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_SWSTART;
}

void tim1_brk_up_trg_com_isr(void) {
	int sr = TIM1_SR;
	if (sr & TIM_SR_UIF) {
		TIM1_SR = ~TIM_SR_UIF;
		if (TIM1_CCR4) COMP_CSR &= ~(0x700 << COMP_SHIFT); // COMP_OUT off
	}
	if (sr & TIM_SR_COMIF) tim1_com_isr();
}

void tim1_cc_isr(void) {
	int sr = TIM1_SR;
	if (sr & TIM_SR_CC4IF) {
		TIM1_SR = ~TIM_SR_CC4IF;
		COMP_CSR |= 0x400 << COMP_SHIFT; // COMP_OUT=TIM2_IC4
	}
	if (!(sr & TIM_SR_CC1IF)) return;
	TIM1_SR = ~TIM_SR_CC1IF;
	if (!(ADC1_CR2 & ADC_CR2_DMA)) return;
	ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE | ADC_CR2_DMA | ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_SWSTART | ADC_CR2_SWSTART;
}

void dma1_channel1_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(1);
	DMA1_CCR(1) = 0;
	ADC1_CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
	int i = 0, v = 0, c = 0, x = 0;
#if SENS_CNT == 2
	c = buf[i++];
#endif
#if SENS_CNT > 0
	v = buf[i++];
#endif
	if (ain) x = buf[i++];
	int r = 4914000 / buf[i + 1];
	int t = (1440 - (buf[i] * r >> 12)) * 400 / 408 + 100;
	adcdata(t, v * r >> 12, c * r >> 12, x * r >> 12);
}
