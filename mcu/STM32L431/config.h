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

#pragma once

#define CLK 80000000
#define IO_PA2

#define IFTIM TIM2
#define IFTIM_XRES 2
#define IFTIM_ICFL 128
#define IFTIM_ICMR TIM2_CCMR2
#define IFTIM_ICM1 (TIM_CCMR2_CC4S_IN_TI4 | TIM_CCMR2_IC4F_DTF_DIV_16_N_8)
#define IFTIM_ICM2 (TIM_CCMR2_CC4S_IN_TI4 | TIM_CCMR2_IC4F_DTF_DIV_8_N_8)
#define IFTIM_ICM3 (TIM_CCMR2_CC4S_IN_TI4 | TIM_CCMR2_IC4F_DTF_DIV_4_N_8)
#define IFTIM_ICIE TIM_DIER_CC4IE
#define IFTIM_ICR TIM2_CCR4
#define IFTIM_OCR TIM2_CCR1
#define iftim_isr tim2_isr

#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 5
#define iotim_isr tim1_brk_tim15_isr
#define iodma_isr dma1_channel5_isr

#define USART1_DMA_BASE DMA2_BASE
#define USART1_RX_DMA 7
#define USART1_TX_DMA 6
#define usart1_tx_dma_isr dma2_channel6_isr

#define USART2_RX_DMA 6
#define USART2_TX_DMA 7

#define tim1_com_isr tim1_trg_com_tim17_isr

#ifdef USE_COMP2
#define SW_BLANKING
#else
#define TIM1_CCR5 MMIO32(TIM1_BASE + 0x58)
#define TIM_CCER_CC5E 0x10000
#endif
