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

#if SENS_MAP == 0xA0 // A0 (volt)
#define SENS_CHAN 0x0
#elif SENS_MAP == 0xA5 // A5 (volt)
#define SENS_CHAN 0x5
#elif SENS_MAP == 0xA6 // A6 (volt)
#define SENS_CHAN 0x6
#elif SENS_MAP == 0xA5A4 // A5 (volt), A4 (curr)
#define SENS_CHAN 0x54
#elif SENS_MAP == 0xA6A4 // A6 (volt), A4 (curr)
#define SENS_CHAN 0x64
#endif

#define COMP1_CSR MMIO32(COMP_BASE + 0x0)
#define COMP2_CSR MMIO32(COMP_BASE + 0x4)

static char len, ain;
static uint16_t buf[10];

void init(void) {
	RCC_AHBRSTR = -1;
	RCC_APBRSTR1 = -1;
	RCC_APBRSTR2 = -1;
	RCC_AHBRSTR = 0;
	RCC_APBRSTR1 = 0;
	RCC_APBRSTR2 = 0;
	RCC_IOPENR = 0x3; // GPIOAEN=1, GPIOBEN=1
	RCC_AHBENR = RCC_AHBENR_DMAEN | RCC_AHBENR_FLASHEN;
	RCC_APBENR1 = RCC_APBENR1_TIM2EN | RCC_APBENR1_TIM6EN | RCC_APBENR1_WWDGEN;
	RCC_APBENR2 = RCC_APBENR2_SYSCFGEN | RCC_APBENR2_TIM1EN | RCC_APBENR2_USART1EN | RCC_APBENR2_ADCEN;
	SYSCFG_CFGR1 = SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP; // A11->A9, A12->A10
	SCB_VTOR = (uint32_t)_rom; // Set vector table address

	// Default GPIO state - analog input
	GPIOA_AFRL = 0x20000000; // A7 (TIM1_CH1N)
	GPIOA_AFRH = 0x00000222; // A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_AFRL = 0x00000002; // B0 (TIM1_CH2N)
	GPIOB_PUPDR = 0x00001000; // B6 (pull-up)
	GPIOA_MODER = 0xebeabfff; // A7 (TIM1_CH1N), A8 (TIM1_CH1), A9 (TIM1_CH2), A10 (TIM1_CH3)
	GPIOB_MODER = 0xffffeffe; // B0 (TIM1_CH2N), B6 (USART1_TX)
#ifdef IO_PA6 // N version
	GPIOB_AFRH |= 0x20000000; // B15 (TIM1_CH3N)
	GPIOB_MODER &= ~0x40000000; // B15 (TIM1_CH3N)
#else
	GPIOB_AFRL |= 0x20; // B1 (TIM1_CH3N)
	GPIOB_MODER &= ~0x4; // B1 (TIM1_CH3N)
#endif
#ifndef ANALOG
#ifdef IO_PA2
	RCC_APBENR2 |= RCC_APBENR2_TIM15EN;
	GPIOA_AFRL |= 0x500; // A2 (TIM15_CH1)
	GPIOA_PUPDR |= 0x10; // A2 (pull-up)
	GPIOA_MODER &= ~0x10; // A2 (TIM15_CH1)
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
#endif

	nvic_set_priority(NVIC_TIM34_IRQ, 0x40);
	nvic_set_priority(NVIC_TIM15_IRQ, 0x40);
	nvic_set_priority(NVIC_USART1_IRQ, 0x80);
	nvic_set_priority(NVIC_USART2_LPUART2_IRQ, 0x40);
	nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 0x40); // TIM3 or TIM15 or USART2_RX
	nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, 0x80); // USART1_TX
	nvic_set_priority(NVIC_DMA1_CHANNEL4_7_DMAMUX_IRQ, 0x80); // ADC

	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_enable_irq(NVIC_TIM34_IRQ);
	nvic_enable_irq(NVIC_TIM15_IRQ);
	nvic_enable_irq(NVIC_USART1_IRQ);
	nvic_enable_irq(NVIC_USART2_LPUART2_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);
	nvic_enable_irq(NVIC_DMA1_CHANNEL4_7_DMAMUX_IRQ);

#ifdef IO_PA2
	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_TIM15_CH1;
#else
	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_TIM3_CH1;
#endif
	DMAMUX1_CxCR(2) = DMAMUX_CxCR_DMAREQ_ID_USART1_RX;
	DMAMUX1_CxCR(3) = DMAMUX_CxCR_DMAREQ_ID_USART1_TX;
	DMAMUX1_CxCR(4) = DMAMUX_CxCR_DMAREQ_ID_ADC;

	ADC_CFGR2(ADC1) = ADC_CFGR2_CKMODE_PCLK_DIV4 << ADC_CFGR2_CKMODE_SHIFT;
	ADC_CCR(ADC1) = ADC_CCR_VREFEN | ADC_CCR_TSEN;
	ADC_CR(ADC1) = ADC_CR_ADVREGEN;
	TIM6_ARR = CLK_KHZ / 50 - 1;
	TIM6_SR = ~TIM_SR_UIF;
	TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
	while (TIM6_CR1 & TIM_CR1_CEN); // Wait for 20us (RM 15.3.2)
	ADC_CR(ADC1) = ADC_CR_ADVREGEN | ADC_CR_ADCAL;
	while (ADC_CR(ADC1) & ADC_CR_ADCAL);
	ADC_CR(ADC1) = ADC_CR_ADEN | ADC_CR_ADVREGEN;
	while (!(ADC_ISR(ADC1) & ADC_ISR_ADRDY));
	ADC_CFGR1(ADC1) = ADC_CFGR1_DMAEN | ADC_CFGR1_EXTEN_RISING_EDGE | ADC_CFGR1_CHSELRMOD;
	ADC_SMPR1(ADC1) = ADC_SMPR_SMPx_160DOT5CYC; // Sampling time ~10us @ PCLK/4=16Mhz
	ADC_CHSELR(ADC1) = SENS_CHAN;
	len = SENS_CNT;
	if (IO_ANALOG) {
		ADC_CHSELR(ADC1) |= AIN_CHAN << (len++ << 2);
		ain = 1;
	}
	ADC_CHSELR(ADC1) |= 0xfdc << (len << 2); // CH13 (vref), CH12 (temp)
	len += 2;
	while (!(ADC_ISR(ADC1) & ADC_ISR_CCRDY));
	DMA1_CPAR(4) = (uint32_t)&ADC_DR(ADC1);
	DMA1_CMAR(4) = (uint32_t)buf;

	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2
	TIM2_CR2 = TIM_CR2_MMS_COMPARE_OC3REF; // TRGO=OC3REF
	TIM2_CCMR1 = TIM_CCMR1_CC2S_IN_TI2;
	TIM2_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM2; // Inverted PWM on OC3
	TIM2_CCER = TIM_CCER_CC2E; // IC2 on rising edge on TI2 (COMP2_OUT)
#if defined IO_PA2 || defined IO_PA6
	TIM2_CCMR1 |= TIM_CCMR1_CC1S_IN_TI1;
	TIM2_CCER |= TIM_CCER_CC1E; // IC1 on rising edge on TI1 (COMP1_OUT)
#endif
}

#ifdef LED_WS2812
void initled(void) {
	RCC_APBENR2 |= RCC_APBENR2_TIM16EN;
	GPIOB_AFRH |= 0x2; // B8 (TIM16_CH1)
	GPIOB_MODER &= ~0x10000; // B8 (TIM16_CH1)
	TIM16_BDTR = TIM_BDTR_MOE;
	TIM16_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1;
	TIM16_CCER = TIM_CCER_CC1E;
	TIM16_CR2 = TIM_CR2_CCDS; // CC1 DMA request on UEV
	TIM16_ARR = CLK_CNT(800000) - 1;
	TIM16_RCR = 7;
	DMAMUX1_CxCR(6) = DMAMUX_CxCR_DMAREQ_ID_TIM16_CH1;
	DMA1_CPAR(6) = (uint32_t)&TIM16_CCR1;
	DMA1_CMAR(6) = (uint32_t)(buf + 5);
}

void ledctl(int x) {
	if (DMA1_CCR(6) & DMA_CCR_EN) return;
	buf[5] = x & 2 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Green
	buf[6] = x & 1 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Red
	buf[7] = x & 4 ? CLK_CNT(1250000) : CLK_CNT(2500000); // Blue
	buf[8] = 0;
	buf[9] = 0;
	DMA1_CNDTR(6) = 5;
	DMA1_CCR(6) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	TIM16_DIER = TIM_DIER_CC1DE;
	TIM16_EGR = TIM_EGR_UG;
	TIM16_CR1 = TIM_CR1_CEN;
}
#endif

void hsictl(int x) {
	int cr = RCC_ICSCR;
	int tv = (cr & 0x7f00) >> 8; // 7 bits
	RCC_ICSCR = (cr & ~0x7f00) | ((tv + x) & 0x7f) << 8;
}

void compctl(int x) {
	int id = 0;
	int cr = 0;
	switch (x & 3) {
#ifdef IO_PA2
		case COMP_IN1:
			id = 1;
			cr = 0x108281; // A0>A1
			break;
		case COMP_IN2:
			id = 2;
			cr = 0x108271; // B7>A3
			break;
		case COMP_IN3:
			id = 2;
			cr = 0x108071; // B7>B4
			break;
#else
		case COMP_IN1:
			id = 2;
			cr = 0x100281; // A3>A2
			break;
		case COMP_IN2:
#ifdef IO_PA6 // N version
			id = 1;
			cr = 0x100281; // A1>A0
#else
			id = 2;
			cr = 0x100261; // A3>B3
#endif
			break;
		case COMP_IN3:
			id = 2;
			cr = 0x100271; // A3>B7
			break;
#endif
	}
	if (x & 4) cr ^= 0x8000; // Change polarity
	switch (id) {
		case 0:
			COMP1_CSR = 0;
			COMP2_CSR = 0;
			TIM2_TISEL = 0;
			break;
#if defined IO_PA2 || defined IO_PA6
		case 1:
			COMP1_CSR = cr;
			TIM2_TISEL = 0x1; // TI1=COMP1_OUT
			break;
#endif
		case 2:
			COMP2_CSR = cr;
			TIM2_TISEL = 0x100; // TI2=COMP2_OUT
			break;
	}
}

void io_serial(void) {
	RCC_APBRSTR2 = RCC_APBRSTR2_TIM15RST;
	RCC_APBRSTR2 = 0;
	nvic_clear_pending_irq(NVIC_TIM15_IRQ);
	RCC_APBENR1 |= RCC_APBENR1_USART2EN;
	GPIOA_AFRL = (GPIOA_AFRL & ~0xf00) | 0x100; // A2 (USART2_TX)
	GPIOA_AFRH |= 0x10000000; // A15 (USART2_RX)
	DMAMUX1_CxCR(1) = DMAMUX_CxCR_DMAREQ_ID_USART2_RX;
	DMAMUX1_CxCR(5) = DMAMUX_CxCR_DMAREQ_ID_USART2_TX;
}

#ifdef IO_PA6
void io_analog(void) {
	RCC_APBRSTR1 = RCC_APBRSTR1_TIM3RST;
	RCC_APBRSTR1 = 0;
	nvic_clear_pending_irq(NVIC_TIM34_IRQ);
	GPIOA_PUPDR &= ~0x3000; // A6 (no pull-up/pull-down)
	GPIOA_MODER |= 0x3000; // A6 (analog)
}
#else
void io_analog(void) {
	RCC_APBRSTR2 = RCC_APBRSTR2_TIM15RST;
	RCC_APBRSTR2 = 0;
	nvic_clear_pending_irq(NVIC_TIM15_IRQ);
	GPIOA_PUPDR &= ~0x30; // A2 (no pull-up/pull-down)
	GPIOA_MODER |= 0x30; // A2 (analog)
}
#endif

void adctrig(void) {
	if (DMA1_CCR(4) & DMA_CCR_EN) return;
	DMA1_CNDTR(4) = len;
	DMA1_CCR(4) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	ADC_CR(ADC1) = ADC_CR_ADSTART | ADC_CR_ADVREGEN;
}

void dma1_channel4_7_dmamux_isr(void) {
#ifdef LED_WS2812
	if (DMA1_ISR & DMA_ISR_TCIF(6)) {
		DMA1_IFCR = DMA_IFCR_CTCIF(6);
		DMA1_CCR(6) = 0;
		TIM16_DIER = 0;
		TIM16_CR1 = 0;
		return;
	}
#endif
	DMA1_IFCR = DMA_IFCR_CTCIF(4);
	DMA1_CCR(4) = 0;
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
