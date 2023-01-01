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

#include "common.h"

static char iobuf[10];

void inittelem(void) {
	USART1_BRR = CNT(PCLK2, 115200);
	USART1_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_OVRDIS;
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE;
	DMA1_CPAR(USART1_TX_DMA) = (uint32_t)&USART1_TDR;
	DMA1_CMAR(USART1_TX_DMA) = (uint32_t)iobuf;
	DMA1_CCR(USART1_TX_DMA) = DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

void usart1_dma_isr(void) {
	DMA1_IFCR = DMA_IFCR_CTCIF(USART1_TX_DMA);
	DMA1_CCR(USART1_TX_DMA) &= ~DMA_CCR_EN;
}

static char crc8(const char *buf, int len) {
	char crc = 0;
	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; ++i) crc = crc & 0x80 ? (crc << 1) ^ 0x07 : crc << 1;
	}
	return crc;
}

void sendtelem(void) { // KISS telemetry
	if (DMA1_CCR(USART1_TX_DMA) & DMA_CCR_EN) return;
	int v1 = 0; // Temperature (C)
	int v2 = 0; // Voltage (V/100)
	int v3 = 0; // Current (A/100)
	int v4 = 0; // Consumption (mAh)
	int v5 = erpm / 100; // Hundreds of ERPM
	iobuf[0] = v1;
	iobuf[1] = v2 >> 8;
	iobuf[2] = v2;
	iobuf[3] = v3 >> 8;
	iobuf[4] = v3;
	iobuf[5] = v4 >> 8;
	iobuf[6] = v4;
	iobuf[7] = v5 >> 8;
	iobuf[8] = v5;
	iobuf[9] = crc8(iobuf, 9);
	DMA1_CNDTR(USART1_TX_DMA) = 10;
	DMA1_CCR(USART1_TX_DMA) |= DMA_CCR_EN;
	telem = 0;
}
