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
#define SENS_CHAN 0x8
#elif SENS_MAP == 0xA6 // A6 (volt)
#define SENS_CNT 1
#define SENS_CHAN 0x40
#elif SENS_MAP == 0xA3A6 // A3 (volt), A6 (curr)
#define SENS_CNT 2
#define SENS_CHAN 0x48
#define SENS_SWAP
#endif

#ifndef THROT_CHAN
#define THROT_CHAN 0x4 // A2
#endif

#define CLK 48000000

#define IFTIM TIM2
#define IFTIM_ICF 64
#define IFTIM_ICE TIM_DIER_CC4IE
#define IFTIM_ICR TIM2_CCR4
#define IFTIM_OCR TIM2_CCR1
#define iftim_isr tim2_isr

#ifdef IO_PA2
#define IO_TYPE 0
#define IOTIM TIM15
#define IOTIM_IDR (GPIOA_IDR & 0x4) // A2
#define IOTIM_DMA 5
#define iotim_isr tim15_isr
#else
#define IO_TYPE 2
#define IOTIM TIM3
#define IOTIM_IDR (GPIOB_IDR & 0x10) // B4
#define IOTIM_DMA 4
#define iotim_isr tim3_isr
#endif
#define iodma_isr dma1_channel4_7_dma2_channel3_5_isr

#define USART1_TX_DMA 2
#define USART1_RX_DMA 3
#define usart1_dma_isr dma1_channel2_3_dma2_channel1_2_isr

#define USART2_RX_DMA 5

void tim1_com_isr(void);
