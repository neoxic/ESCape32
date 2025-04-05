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

#define CLK 64000000

#define IFTIM TIM2
#define IFTIM_XRES 2
#define IFTIM_ICFL 64
#define IFTIM_ICMR TIM2_CCMR1
#if defined IO_PA2 || defined IO_PA6
#define IFTIM_ICM1 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8)
#define IFTIM_ICM2 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_4_N_8 | TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_4_N_8)
#define IFTIM_ICM3 (TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_2_N_8 | TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_2_N_8)
#define IFTIM_ICIE (TIM_DIER_CC1IE | TIM_DIER_CC2IE)
#define IFTIM_ICR (sr & TIM_SR_CC1IF ? TIM2_CCR1 : TIM2_CCR2)
#else
#define IFTIM_ICM1 (TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8)
#define IFTIM_ICM2 (TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_4_N_8)
#define IFTIM_ICM3 (TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_2_N_8)
#define IFTIM_ICIE TIM_DIER_CC2IE
#define IFTIM_ICR TIM2_CCR2
#endif
#define IFTIM_OCR TIM2_CCR3
#define iftim_isr tim2_isr

#ifdef IO_PA2
#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define iotim_isr tim15_isr
#else
#define IOTIM TIM3
#ifdef IO_PA6
#define IOTIM_IDR (GPIOA_IDR & 0x40) // A6
#else
#define IOTIM_IDR (GPIOB_IDR & 0x10) // B4
#endif
#define iotim_isr tim3_isr
#endif
#define IOTIM_DMA 1
#define iodma_isr dma1_channel1_isr

#define USART1_RX_DMA 2
#define USART1_TX_DMA 3
#define usart1_tx_dma_isr dma1_channel2_3_isr

#define USART2_RX_DMA 1
#define USART2_TX_DMA 5
#define usart2_isr usart2_lpuart2_isr

#define tim1_com_isr tim1_brk_up_trg_com_isr
#define tim3_isr tim34_isr
