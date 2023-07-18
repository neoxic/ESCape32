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

#if SENS_MAP == 0xA3 // A3 (volt)
#define SENS_CNT 1
#define SENS_CHAN 0x3
#elif SENS_MAP == 0xA3A6 // A3 (volt), A6 (curr)
#define SENS_CNT 2
#define SENS_CHAN 0x66
#endif

#ifndef THROT_CHAN
#define THROT_CHAN 2 // A2
#endif

#define CLK 120000000
#define IO_PA2
#define IO_TYPE 0

#define IFTIM TIM3
#define IFTIM_ICF 128
#define IFTIM_ICE TIM_DIER_CC1IE
#define IFTIM_ICR TIM3_CCR1
#define IFTIM_OCR TIM3_CCR3
#define iftim_isr tim3_isr

#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 5
#define iotim_isr tim15_isr
#define iodma_isr dma1_channel4_7_dma2_channel3_5_isr

#define USART1_TX_DMA 2
#define USART1_RX_DMA 3
#define usart1_dma_isr dma1_channel2_3_dma2_channel1_2_isr

#define USART2_RX_DMA 5

#define tim1_com_isr tim1_brk_up_trg_com_isr
