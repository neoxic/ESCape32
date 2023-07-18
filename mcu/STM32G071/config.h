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

#pragma once

#if SENS_MAP == 0xA0 // A0 (volt)
#define SENS_CNT 1
#define SENS_CHAN 0x0
#elif SENS_MAP == 0xA6 // A6 (volt)
#define SENS_CNT 1
#define SENS_CHAN 0x6
#elif SENS_MAP == 0xA5A4 // A5 (volt), A4 (curr)
#define SENS_CNT 2
#define SENS_CHAN 0x54
#elif SENS_MAP == 0xA6A4 // A6 (volt), A4 (curr)
#define SENS_CNT 2
#define SENS_CHAN 0x64
#endif

#ifndef THROT_CHAN
#ifdef IO_PA2
#define THROT_CHAN 2 // A2
#else
#define THROT_CHAN 6 // A6
#endif
#endif

#define CLK 64000000

#define IFTIM TIM2
#define IFTIM_ICF 64
#if defined IO_PA2 || defined IO_PA6
#define IFTIM_ICE (TIM_DIER_CC1IE | TIM_DIER_CC2IE)
#define IFTIM_ICR (sr & TIM_SR_CC1IF ? TIM2_CCR1 : TIM2_CCR2)
#else
#define IFTIM_ICE TIM_DIER_CC2IE
#define IFTIM_ICR TIM2_CCR2
#endif
#define IFTIM_OCR TIM2_CCR3
#define iftim_isr tim2_isr

#ifdef IO_PA2
#define IO_TYPE 0
#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define iotim_isr tim15_isr
#else
#define IOTIM TIM3
#define iotim_isr tim34_isr
#ifdef IO_PA6
#define IO_TYPE 1
#define IOTIM_IDR (GPIOA_IDR & 0x40) // A6
#else
#define IO_TYPE 2
#define IOTIM_IDR (GPIOB_IDR & 0x10) // B4
#endif
#endif
#define IOTIM_DMA 1
#define iodma_isr dma1_channel1_isr

#define USART1_TX_DMA 2
#define USART1_RX_DMA 3
#define usart1_dma_isr dma1_channel2_3_isr

#define USART2_RX_DMA 1
#define usart2_isr usart2_lpuart2_isr

#define tim1_com_isr tim1_brk_up_trg_com_isr
