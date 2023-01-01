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

#define PCLK1 48000000
#define PCLK2 48000000

#ifdef IO_PA2
#define TARGET_TYPE 2
#define IOTIM TIM15
#define IOTIM_CLK PCLK2
#define IOTIM_IDR GPIOA_IDR
#define IOTIM_PIN 0x04 // A2
#define IOTIM_DMA 5
#define iotim_isr tim15_isr
#else
#define TARGET_TYPE 1
#define IOTIM TIM3
#define IOTIM_CLK PCLK1
#define IOTIM_IDR GPIOB_IDR
#define IOTIM_PIN 0x10 // B4
#define IOTIM_DMA 4
#define iotim_isr tim3_isr
#endif

#define USART1_TX_DMA 2
#define USART1_RX_DMA 3
#define usart1_dma_isr dma1_channel2_3_dma2_channel1_2_isr

#define USART2_RX_DMA 5

void tim1_up_isr(void);
void tim1_com_isr(void);
void iotim_dma_isr(void);
void usart2_dma_isr(void);
