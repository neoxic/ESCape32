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
#include <libopencm3/stm32/g4/opamp.h>
#include "common.h"

#if SENS_MAP == 0xA6 // A6 (volt)
#define SENS_CHAN 0xb
#elif SENS_MAP == 0xA6A3 // A6 (volt), A3 (curr)
#define SENS_CHAN 0x2c8
#endif

#ifndef ANALOG_CHAN
#define ANALOG_CHAN 0x7 // ADC_IN7 (A2)
#endif

#ifdef TEMP_CHAN
#define TEMP_SHIFT 12
#else
#define TEMP_SHIFT 0
#define TEMP_CHAN 0x11 // ADC_IN17 (temp)
#define TEMP_FUNC(x) (((x) / 3000 - ST_TSENSE_CAL1_30C) * 400 / (ST_TSENSE_CAL2_110C - ST_TSENSE_CAL1_30C) + 120)
#endif

#ifdef USE_COMP2
#define COMP_CSR MMIO32(COMP_BASE + 0x4)
#else
#define COMP_CSR MMIO32(COMP_BASE + 0x0)
#define TIM1_CCMR3 MMIO32(TIM1_BASE + 0x54)
#endif
#define TIM2_OR1 MMIO32(TIM2_BASE + 0x50)

static char len, ain;
static uint16_t buf[6];

void init(void) {
	RCC_AHB1RSTR = -1;
	RCC_AHB2RSTR = -1;
	RCC_AHB3RSTR = -1;
	RCC_APB1RSTR1 = -1;
	RCC_APB1RSTR2 = -1;
	RCC_APB2RSTR = -1;
	RCC_AHB1RSTR = 0;
	RCC_AHB2RSTR = 0;
	RCC_AHB3RSTR = 0;
	RCC_APB1RSTR1 = 0;
	RCC_APB1RSTR2 = 0;
	RCC_APB2RSTR = 0;
	RCC_AHB1ENR = RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_FLASHEN;
	RCC_AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_ADCEN;
	RCC_APB1ENR1 = RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM6EN | 0x800; // WWDGEN=1
	RCC_APB2ENR = RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x10000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000111; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x07000011; // B0 (TIM1_CH2N), B1 (TIM1_CH3N), B6 (USART1_TX)
	GPIOA_PUPDR = 0x24000000;
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffa; // B0 (TIM1_CH2N), B1 (TIM1_CH3N), B6 (USART1_TX)
#ifndef ANALOG
	RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
	GPIOA_AFRL |= 0xe00; // A2 (TIM15_CH1)
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (TIM15_CH1)
	nvic_set_priority(NVIC_TIM1_BRK_TIM15_IRQ, 0x40);
#endif
	nvic_set_priority(NVIC_USART1_IRQ, 0x80);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x80); // ADC
	nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0x40); // TIM15
	nvic_set_priority(NVIC_DMA1_CHANNEL6_IRQ, 0x40); // USART2_RX
	nvic_set_priority(NVIC_DMA2_CHANNEL6_IRQ, 0x80); // USART1_TX

	nvic_enable_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	nvic_enable_irq(NVIC_TIM1_UP_TIM16_IRQ);
	nvic_enable_irq(NVIC_TIM1_TRG_COM_TIM17_IRQ);
	nvic_enable_irq(NVIC_TIM1_CC_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ);
	nvic_enable_irq(NVIC_DMA2_CHANNEL6_IRQ);

	DMA1_CSELR = 0x2270000; // C5S=TIM15, C6S=USART2_RX, C7S=USART2_TX
	DMA2_CSELR = 0x2200000; // C6S=USART1_TX, C7S=USART1_RX
#ifndef USE_COMP2
	TIM1_CCMR3 = 0x68; // OC5PE=1, OC5M=PWM1
	TIM2_OR1 = 0x4; // TI4=COMP1_OUT
#elif defined USE_OPAMP
	RCC_APB1ENR1 |= RCC_APB1ENR1_OPAMPEN;
	OPAMP1_CSR = 0x39; // OPAEN=1, OPAMODE=PGA, PGA_GAIN=16
#endif
	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2
	TIM2_CR2 = TIM_CR2_MMS_COMPARE_OC1REF; // TRGO=OC1REF
	TIM2_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2; // Inverted PWM on OC1
	TIM2_CCMR2 = TIM_CCMR2_CC4S_IN_TI4;
	TIM2_CCER = TIM_CCER_CC4E; // IC4 on rising edge on TI4 (COMPx_OUT)

	ADC_CCR(ADC1) = ADC_CCR_VREFEN | ADC_CCR_TSEN | 0x20000; // CKMODE=HCLK/2
	ADC_CR(ADC1) = 0; // DEEPPWD=0
	ADC_CR(ADC1) = ADC_CR_ADVREGEN;
	TIM6_ARR = CLK_KHZ / 50 - 1;
	TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
	while (TIM6_CR1 & TIM_CR1_CEN); // Wait for 20us (RM 16.4.6)
	ADC_CR(ADC1) = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
	while (ADC_CR(ADC1) & ADC_CR_ADCAL);
	while (ADC_CR(ADC1) = ADC_CR_ADEN | ADC_CR_ADVREGEN, !(ADC_ISR(ADC1) & ADC_ISR_ADRDY)); // Keep powering on until ready (RM 16.4.9)
	ADC_CFGR1(ADC1) = ADC_CFGR1_DMAEN | ADC_CFGR1_EXTSEL_VAL(9) | ADC_CFGR1_EXTEN_RISING_EDGE; // EXTSEL=TIM1_TRGO
	ADC_SMPR1(ADC1) = -1; // Sampling time ~16us @ HCLK/2=40Mhz
	ADC_SMPR2(ADC1) = -1;
	uint64_t seq = SENS_CHAN;
	len = SENS_CNT;
	if (IO_ANALOG) {
		seq |= ANALOG_CHAN << (len++ * 6);
		ain = 1;
	}
	seq |= TEMP_CHAN << (len * 6); // ADC_IN0 (vref)
	len += 2;
	ADC_SQR1(ADC1) = seq << 6 | (len - 1);
	ADC_SQR2(ADC1) = seq >> 24;
	DMA1_CPAR(1) = (uint32_t)&ADC_DR(ADC1);
	DMA1_CMAR(1) = (uint32_t)buf;
}

void hsictl(int x) {
	int cr = RCC_ICSCR;
	int tv = (cr & 0x7f000000) >> 24; // 7 bits
	RCC_ICSCR = (cr & ~0x7f000000) | ((tv + x) & 0x7f) << 24;
}

void compctl(int x) {
	int cr = 0;
	switch (x & 3) {
#ifdef USE_COMP2
		case COMP_IN1:
			cr = 0x4000071; // B4>A4
			break;
		case COMP_IN2:
			cr = 0x6000071; // B4>A5
			break;
		case COMP_IN3:
			cr = 0x71; // B4>B7
			break;
#else
		case COMP_IN1:
			cr = 0x2000071; // A1>A0
			break;
		case COMP_IN2:
			cr = 0x4000071; // A1>A4
			break;
		case COMP_IN3:
			cr = 0x6000071; // A1>A5
			break;
#endif
	}
	if (x & 4) cr |= 0x8000; // Change polarity
	COMP_CSR = cr;
}

void io_serial(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	RCC_APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	GPIOA_AFRL = (GPIOA_AFRL & ~0xf00) | 0x700; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x30000000; // A15 (USART2_RX)
}

void io_analog(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	GPIOA_PUPDR &= ~0x30; // A2 (no pull-up/pull-down)
	GPIOA_MODER |= 0x30; // A2 (analog)
}

void adctrig(void) {
	if (DMA1_CCR(1) & DMA_CCR_EN) return;
	DMA1_CNDTR(1) = len;
	DMA1_CCR(1) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC_CR(ADC1) = ADC_CR_ADSTART | ADC_CR_ADVREGEN;
}

#ifdef USE_COMP2
void tim1_up_tim16_isr(void) {
	TIM1_SR = ~TIM_SR_UIF;
	if (TIM1_CCR4) TIM2_OR1 = 0;
}

void tim1_cc_isr(void) {
	TIM1_SR = ~TIM_SR_CC4IF;
	TIM2_OR1 = 0x8; // TI4=COMP2_OUT
}
#endif

void dma1_channel6_isr(void) {
	dma1_channel5_isr();
}

void dma1_channel1_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(1);
	DMA1_CCR(1) = 0;
	int i = 0, u = 0, v = 0, c = 0, a = 0;
#if SENS_CNT >= 2
	c = buf[i++];
#endif
#if SENS_CNT >= 1
	v = buf[i++];
#endif
#if SENS_CNT >= 3
	u = buf[i++];
#endif
	if (ain) a = buf[i++];
	int r = ST_VREFINT_CAL * 3000 / buf[i + 1];
	adcdata(TEMP_FUNC(buf[i] * r >> TEMP_SHIFT), u * r >> 12, v * r >> 12, c * r >> 12, a * r >> 12);
}
