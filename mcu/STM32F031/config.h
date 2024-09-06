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

#define CLK 48000000
#define IO_PA6
#define IO_TYPE 1
#define GPTIM TIM16

#define IFTIM TIM2
#define IFTIM_XRES 2
#define IFTIM_IDR (GPIOA_IDR & 0x7) // A0|A1|A2
#define iftim_isr tim2_isr

#define IOTIM TIM3
#define IOTIM_IDR (GPIOA_IDR & 0x40) // A6
#define IOTIM_DMA 4
#define iotim_isr tim3_isr
#define iodma_isr dma1_channel4_7_dma2_channel3_5_isr

#define USART1_RX_DMA 3
#define USART1_TX_DMA 2
#define usart1_tx_dma_isr dma1_channel2_3_dma2_channel1_2_isr

#define tim1_com_isr tim1_brk_up_trg_com_isr
