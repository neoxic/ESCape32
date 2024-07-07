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

#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/adc.h>
#include "common.h"

#if SENS_MAP == 0xA6 // A6 (volt)
#define SENS_CHAN 0x3
#elif SENS_MAP == 0xA6F1 // A6 (volt), F1 (curr)
#define SENS_CHAN 0xca
#elif SENS_MAP == 0xBFA6 // B15 (volt), A6 (curr)
#define SENS_CHAN 0x3c3
#endif

#define COMP1_CSR MMIO32(COMP_BASE + 0x0)
#define COMP2_CSR MMIO32(COMP_BASE + 0x4)
#define TIM1_CCMR3 MMIO32(TIM1_BASE + 0x50)
#define TIM2_TISEL MMIO32(TIM2_BASE + 0x5c)

static char len1, len2, ain;
static uint16_t buf[10];

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
	RCC_AHB1ENR = RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN | RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_FLASHEN;
	RCC_AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOFEN | RCC_AHB2ENR_ADC12EN;
	RCC_APB1ENR1 = RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM6EN | RCC_APB1ENR1_WWDGEN;
	RCC_APB2ENR = RCC_APB2ENR_SYSCFGEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN;
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

#ifdef USE_HSE
	if (!cfg.throt_cal) {
		RCC_CFGR = RCC_CFGR_SWx_HSI16;
		while ((RCC_CFGR & RCC_CFGR_SWS_MASK << RCC_CFGR_SWS_SHIFT) != RCC_CFGR_SWx_HSI16 << RCC_CFGR_SWS_SHIFT);
		RCC_CR = (RCC_CR & ~RCC_CR_PLLON) | RCC_CR_HSEON;
		while (!(RCC_CR & RCC_CR_HSERDY));
		RCC_PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | (336 / USE_HSE) << RCC_PLLCFGR_PLLN_SHIFT | RCC_PLLCFGR_PLLREN;
		RCC_CR |= RCC_CR_PLLON;
		while (!(RCC_CR & RCC_CR_PLLRDY));
		RCC_CFGR = RCC_CFGR_SWx_PLL | RCC_CFGR_HPRE_DIV2 << RCC_CFGR_HPRE_SHIFT;
		TIM6_ARR = CLK_MHZ / 2 - 1;
		TIM6_SR = ~TIM_SR_UIF;
		TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
		while (TIM6_CR1 & TIM_CR1_CEN); // Ensure 1us HCLK/2 transition period (RM 7.2.7)
		RCC_CFGR = RCC_CFGR_SWx_PLL;
	}
#endif

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x60000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000666; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x07000006; // B0 (TIM1_CH2N), B6 (USART1_TX)
	GPIOA_PUPDR = 0x24000000;
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffe; // B0 (TIM1_CH2N), B6 (USART1_TX)
#ifdef USE_PB1 // 48-pin package
	GPIOB_AFRL |= 0x60; // B1 (TIM1_CH3N)
	GPIOB_MODER &= ~0x4; // B1 (TIM1_CH3N)
#else
	GPIOF_AFRL |= 0x6; // F0 (TIM1_CH3N)
	GPIOF_MODER &= ~0x1; // F0 (TIM1_CH3N)
#endif
#ifndef ANALOG
	RCC_APB2ENR |= RCC_APB2ENR_TIM15EN;
	GPIOA_AFRL |= 0x900; // A2 (TIM15_CH1)
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (TIM15_CH1)
#endif

	nvic_set_priority(NVIC_TIM1_BRK_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x80);
	nvic_set_priority(NVIC_USART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x40); // TIM15 or USART2_RX
	nvic_set_priority(NVIC_DMA1_CHANNEL3_IRQ, 0x80); // USART1_TX
	nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0x80); // ADC1
	nvic_set_priority(NVIC_DMA1_CHANNEL5_IRQ, 0x80); // ADC2

	nvic_enable_irq(NVIC_TIM1_TRG_TIM17_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ);

	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_TIM15_CH1;
	DMAMUX1_CxCR(2) = DMAMUX_CxCR_DMAREQ_ID_UART1_RX;
	DMAMUX1_CxCR(3) = DMAMUX_CxCR_DMAREQ_ID_UART1_TX;
	DMAMUX1_CxCR(4) = DMAMUX_CxCR_DMAREQ_ID_ADC1;
	DMAMUX1_CxCR(5) = DMAMUX_CxCR_DMAREQ_ID_ADC2;

	ADC_CCR(ADC1) = ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_CKMODE_DIV4;
	ADC_CR(ADC1) = 0; // DEEPPWD=0
	ADC_CR(ADC2) = 0; // DEEPPWD=0
	ADC_CR(ADC1) = ADC_CR_ADVREGEN;
	ADC_CR(ADC2) = ADC_CR_ADVREGEN;
	TIM6_ARR = CLK_KHZ / 50 - 1;
	TIM6_SR = ~TIM_SR_UIF;
	TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
	while (TIM6_CR1 & TIM_CR1_CEN); // Wait for 20us (RM 21.4.6)
	ADC_CR(ADC1) = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
	ADC_CR(ADC2) = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
	while (ADC_CR(ADC1) & ADC_CR_ADCAL);
	while (ADC_CR(ADC2) & ADC_CR_ADCAL);
	while (ADC_CR(ADC1) = ADC_CR_ADEN | ADC_CR_ADVREGEN, !(ADC_ISR(ADC1) & ADC_ISR_ADRDY)); // Keep powering on until ready (RM 21.4.9)
	while (ADC_CR(ADC2) = ADC_CR_ADEN | ADC_CR_ADVREGEN, !(ADC_ISR(ADC2) & ADC_ISR_ADRDY));
	ADC_CFGR1(ADC1) = ADC_CFGR1_DMAEN | ADC12_CFGR1_EXTSEL_TIM1_TRGO | ADC_CFGR1_EXTEN_RISING_EDGE;
	ADC_CFGR1(ADC2) = ADC_CFGR1_DMAEN | ADC12_CFGR1_EXTSEL_TIM1_TRGO | ADC_CFGR1_EXTEN_RISING_EDGE;
	ADC_SMPR1(ADC1) = -1; // Sampling time ~15us @ PCLK/4=42Mhz
	ADC_SMPR1(ADC2) = -1;
	ADC_SMPR2(ADC1) = -1;
	ADC_SMPR2(ADC2) = -1;
	int val1 = 0x490; // CH18 (vref), CH16 (temp)
	int val2 = SENS_CHAN;
	len1 = 2;
	len2 = SENS_CNT;
	if (IO_ANALOG) {
#ifdef ANALOG_CHAN
		val2 |= ANALOG_CHAN << (len2++ * 6);
#else
		val1 = val1 << 6 | 0x3; // ADC1_IN3 (PA2)
		++len1;
#endif
		ain = 1;
	}
	ADC_SQR1(ADC1) = val1 << 6 | (len1 - 1);
	ADC_SQR1(ADC2) = val2 << 6 | (len2 - 1);
	DMA1_CPAR(4) = (uint32_t)&ADC_DR(ADC1);
	DMA1_CMAR(4) = (uint32_t)(buf + len2);
	DMA1_CPAR(5) = (uint32_t)&ADC_DR(ADC2);
	DMA1_CMAR(5) = (uint32_t)buf;

	TIM1_CCMR3 = 0x68; // OC5PE=1, OC5M=PWM1
	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2
	TIM2_CR2 = TIM_CR2_MMS_COMPARE_OC3REF; // TRGO=OC3REF
	TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TI1;
	TIM2_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM2; // Inverted PWM on OC3
	TIM2_CCER = TIM_CCER_CC1E; // IC1 on rising edge on TI1 (COMPx_OUT)
}

#ifdef LED_WS2812
void initled(void) {
	RCC_APB2ENR |= RCC_APB2ENR_TIM16EN;
	GPIOB_AFRH |= 0x1; // B8 (TIM16_CH1)
	GPIOB_MODER &= ~0x10000; // B8 (TIM16_CH1)
	TIM16_BDTR = TIM_BDTR_MOE;
	TIM16_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1;
	TIM16_CCER = TIM_CCER_CC1E;
	TIM16_CR2 = TIM_CR2_CCDS; // CC1 DMA request on UEV
	TIM16_ARR = CLK_CNT(800000) - 1;
	TIM16_RCR = 7;
	nvic_set_priority(NVIC_DMA2_CHANNEL1_IRQ, 0x80);
	nvic_enable_irq(NVIC_DMA2_CHANNEL1_IRQ);
	DMAMUX1_CxCR(9) = DMAMUX_CxCR_DMAREQ_ID_TIM16_CH1;
	DMA2_CPAR(1) = (uint32_t)&TIM16_CCR1;
	DMA2_CMAR(1) = (uint32_t)(buf + 5);
}

void ledctl(int x) {
	if (DMA2_CCR(1) & DMA_CCR_EN) return;
	buf[5] = x & 2 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Green
	buf[6] = x & 1 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Red
	buf[7] = x & 4 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Blue
	buf[8] = 0;
	buf[9] = 0;
	DMA2_CNDTR(1) = 5;
	DMA2_CCR(1) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	TIM16_DIER = TIM_DIER_CC1DE;
	TIM16_EGR = TIM_EGR_UG;
	TIM16_CR1 = TIM_CR1_CEN;
}
#endif

void hsictl(int x) {
	int cr = RCC_ICSCR;
	int tv = (cr & 0x7f000000) >> 24; // 7 bits
	RCC_ICSCR = (cr & ~0x7f000000) | ((tv + x) & 0x7f) << 24;
}

void compctl(int x) {
	int id = 0;
	int cr = 0;
	switch (x & 3) {
		case COMP_IN1:
			id = 1;
			cr = 0x80071; // A1>A0
			break;
		case COMP_IN2:
			id = 1;
			cr = 0x80061; // A1>A4
			break;
		case COMP_IN3:
			id = 2;
			cr = 0x80161; // A3>A5
			break;
	}
	if (x & 4) cr |= 0x8000; // Change polarity
	switch (id) {
		case 0:
			COMP1_CSR = 0;
			COMP2_CSR = 0;
			TIM2_TISEL = 0;
			break;
		case 1:
			COMP1_CSR = cr;
			TIM2_TISEL = 0x1; // TI1=COMP1_OUT
			break;
		case 2:
			COMP2_CSR = cr;
			TIM2_TISEL = 0x2; // TI1=COMP2_OUT
			break;
	}
}

void io_serial(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	RCC_APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	GPIOA_AFRL = (GPIOA_AFRL & ~0xf00) | 0x700; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x70000000; // A15 (USART2_RX)
	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_UART2_RX;
	DMAMUX1_CxCR(6) = DMAMUX_CxCR_DMAREQ_ID_UART2_TX;
}

void io_analog(void) {
	RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
	RCC_APB2RSTR = 0;
	nvic_clear_pending_irq(NVIC_TIM1_BRK_TIM15_IRQ);
	GPIOA_PUPDR &= ~0x30; // A2 (no pull-up/pull-down)
	GPIOA_MODER |= 0x30; // A2 (analog)
}

void adctrig(void) {
	if ((DMA1_CCR(4) & DMA_CCR_EN) || (DMA1_CCR(5) & DMA_CCR_EN)) return;
	DMA1_CNDTR(4) = len1;
	DMA1_CCR(4) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC_CR(ADC1) = ADC_CR_ADSTART | ADC_CR_ADVREGEN;
	if (!len2) return;
	DMA1_CNDTR(5) = len2;
	DMA1_CCR(5) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC_CR(ADC2) = ADC_CR_ADSTART | ADC_CR_ADVREGEN;
}

static void adcdma(void) {
	int i = 0, v = 0, c = 0, x = 0;
#if SENS_CNT == 2
	c = buf[i++];
#endif
#if SENS_CNT > 0
	v = buf[i++];
#endif
	if (ain) x = buf[i++];
	int r = ST_VREFINT_CAL * 3000 / buf[i + 1];
	int t = (buf[i] * r / 3000 - ST_TSENSE_CAL1_30C) * 400 / (ST_TSENSE_CAL2_130C - ST_TSENSE_CAL1_30C) + 120;
	adcdata(t, v * r >> 12, c * r >> 12, x * r >> 12);
}

void dma1_channel4_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(4);
	DMA1_CCR(4) = 0;
	if (DMA1_CCR(5) & DMA_CCR_EN) return;
	adcdma();
}

void dma1_channel5_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(5);
	DMA1_CCR(5) = 0;
	if (DMA1_CCR(4) & DMA_CCR_EN) return;
	adcdma();
}

#ifdef LED_WS2812
void dma2_channel1_isr(void) {
	DMA2_IFCR = DMA_IFCR_CTCIF(1);
	DMA2_CCR(1) = 0;
	TIM16_DIER = 0;
	TIM16_CR1 = 0;
}
#endif
