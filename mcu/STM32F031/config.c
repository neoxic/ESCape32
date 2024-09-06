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

#include <libopencm3/stm32/adc.h>
#include "common.h"

#if SENS_MAP == 0xA5 // A5 (volt)
#define SENS_CHAN 0x20
#elif SENS_MAP == 0xA5A4 // A5 (volt), A4 (curr)
#define SENS_CHAN 0x30
#endif

static char len, ain;
static uint16_t buf[5];

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
	GPIOA_AFRL = 0x00000222; // A0 (TIM2_CH1), A1 (TIM2_CH2), A2 (TIM2_CH3)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOA_PUPDR = 0x24000015; // A0,A1,A2 (pull-up)
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeaffea; // A0 (TIM2_CH1), A1 (TIM2_CH2), A2 (TIM2_CH3), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffefff; // B6 (USART1_TX)
#ifdef STSPIN32F0
	RCC_AHBENR |= RCC_AHBENR_GPIOFEN;
	GPIOB_AFRH |= 0x22200000; // B13 (TIM1_CH1N), B14 (TIM1_CH2N), B15 (TIM1_CH3N)
	GPIOB_MODER &= ~0x54000000; // B13 (TIM1_CH1N), B14 (TIM1_CH2N), B15 (TIM1_CH3N)
	GPIOF_ODR = 0xc0; // F7,F6=11 (maximum overcurrent threshold)
	GPIOF_MODER = 0xffff5fff; // F6,F7 (output)
#else
	GPIOA_AFRL |= 0x20000000; // A7 (TIM1_CH1N)
	GPIOB_AFRL |= 0x22; // B0 (TIM1_CH2N), B1 (TIM1_CH3N)
	GPIOA_MODER &= ~0x4000; // A7 (TIM1_CH1N)
	GPIOB_MODER &= ~0x5; // B0 (TIM1_CH2N), B1 (TIM1_CH3N)
#endif
#ifndef ANALOG
	RCC_APB1ENR |= RCC_APB1ENR_TIM3EN;
	GPIOA_AFRL |= 0x1000000; // A6 (TIM3_CH1)
	GPIOA_PUPDR |= 0x1000; // A6 (pull-up)
	GPIOA_MODER &= ~0x1000; // A6 (TIM3_CH1)
#endif

	nvic_set_priority(NVIC_TIM3_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x80);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x80); // ADC
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ, 0x80); // USART1_TX
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ, 0x40); // TIM3

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM3_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_DMA2_CHANNEL1_2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMA2_CHANNEL3_5_IRQ);

	ADC1_CR = ADC_CR_ADCAL;
	while (ADC1_CR & ADC_CR_ADCAL);
	while (ADC1_CR = ADC_CR_ADEN, !(ADC1_ISR & ADC_ISR_ADRDY)); // Keep powering on until ready (Errata 2.5.3)
	ADC1_CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_EXTEN_RISING_EDGE;
	ADC1_SMPR = ADC_SMPR_SMP_239DOT5; // Sampling time ~17us @ HSI14
	ADC1_CCR = ADC_CCR_VREFEN | ADC_CCR_TSEN;
	ADC1_CHSELR = SENS_CHAN | 0x30000; // CH17 (vref), CH16 (temp)
	len = SENS_CNT + 2;
	if (IO_ANALOG) {
		ADC1_CHSELR |= 1 << AIN_CHAN;
		ain = 1;
		++len;
	}
	DMA1_CPAR(1) = (uint32_t)&ADC1_DR;
	DMA1_CMAR(1) = (uint32_t)buf;

	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2
	TIM2_CR2 = TIM_CR2_TI1S | TIM_CR2_MMS_COMPARE_PULSE; // TI1=CH1^CH2^CH3, TRGO=CC1IF
	TIM2_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM2_CCER = TIM_CCER_CC1E; // IC1 on any edge on TI1
}

void io_analog(void) {
	RCC_APB1RSTR = RCC_APB1RSTR_TIM3RST;
	RCC_APB1RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM3_IRQ);
	GPIOA_PUPDR &= ~0x3000; // A6 (no pull-up/pull-down)
	GPIOA_MODER |= 0x3000; // A6 (analog)
}

void adctrig(void) {
	if (DMA1_CCR(1) & DMA_CCR_EN) return;
	DMA1_CNDTR(1) = len;
	DMA1_CCR(1) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC1_CR = ADC_CR_ADSTART;
}

void dma1_channel1_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(1);
	DMA1_CCR(1) = 0;
	int i = 0, v = 0, c = 0, x = 0;
#if SENS_CNT == 2
	c = buf[i++];
#endif
#if SENS_CNT > 0
	v = buf[i++];
#endif
	if (ain) x = buf[i++];
	int r = ST_VREFINT_CAL * 3300 / buf[i + 1];
	int t = (buf[i] * r / 3300 - ST_TSENSE_CAL1_30C) * 320 / (ST_TSENSE_CAL2_110C - ST_TSENSE_CAL1_30C) + 120;
	adcdata(t, v * r >> 12, c * r >> 12, x * r >> 12);
}
