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

#ifdef USARTv1
#define USART1_TDR USART1_DR
#define USART1_RDR USART1_DR
#define USART_CR2_RXINV 0
#define USART_CR2_TXINV 0
#endif

static int ibusdma(void);
static int sportdma(void);

static int (*iodma)(void);

static char rxlen, iobuf[16];

void inittelem(void) {
	USART1_BRR = CLK_CNT(115200);
	switch (cfg.telem_mode) {
		case 2: // iBUS
			iodma = ibusdma;
			rxlen = 4;
			break;
		case 3: // S.Port
			iodma = sportdma;
			rxlen = 2;
			USART1_BRR = CLK_CNT(57600);
			USART1_CR2 = USART_CR2_RXINV | USART_CR2_TXINV;
			break;
	}
	if (iodma) {
		USART1_CR3 = USART_CR3_HDSEL;
		USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE;
	} else { // KISS
		USART1_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT;
		USART1_CR1 = USART_CR1_UE | USART_CR1_TE;
	}
	DMA1_CPAR(USART1_TX_DMA) = (uint32_t)&USART1_TDR;
	DMA1_CMAR(USART1_TX_DMA) = (uint32_t)iobuf;
	DMA1_CPAR(USART1_RX_DMA) = (uint32_t)&USART1_RDR;
	DMA1_CMAR(USART1_RX_DMA) = (uint32_t)iobuf;
}

void usart1_isr(void) {
	if (USART1_CR1 & USART_CR1_TCIE) {
#ifdef USARTv1
		USART1_SR = ~USART_SR_TC;
#else
		USART1_ICR = USART_ICR_TCCF;
#endif
		USART1_CR1 = USART_CR1_UE | USART_CR1_RE;
		return;
	}
#ifdef USARTv1
	USART1_SR, USART1_DR; // Clear flags
#else
	USART1_RQR = USART_RQR_RXFRQ; // Clear RXNE
	USART1_ICR = USART_ICR_IDLECF | USART_ICR_ORECF;
#endif
	USART1_CR1 = USART_CR1_UE | USART_CR1_RE;
	USART1_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;
	DMA1_CNDTR(USART1_RX_DMA) = rxlen;
	DMA1_CCR(USART1_RX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

void usart1_dma_isr(void) {
	if (DMA1_ISR & DMA_ISR_TCIF(USART1_TX_DMA)) {
		DMA1_IFCR = DMA_IFCR_CTCIF(USART1_TX_DMA);
		DMA1_CCR(USART1_TX_DMA) = 0;
		return;
	}
	DMA1_IFCR = DMA_IFCR_CTCIF(USART1_RX_DMA);
	int txlen = iodma();
	if (!txlen) return;
#ifdef USARTv1
	USART1_SR = ~USART_SR_TC;
#else
	USART1_ICR = USART_ICR_TCCF;
#endif
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE; // TE=0->TE=1 generates idle frame
	DMA1_CNDTR(USART1_TX_DMA) = txlen;
	DMA1_CCR(USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

static void resync(void) {
#ifdef USARTv1
	USART1_SR, USART1_DR; // Clear flags
#else
	USART1_ICR = USART_ICR_IDLECF;
#endif
	USART1_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE;
	USART1_CR3 = USART_CR3_HDSEL;
	DMA1_CCR(USART1_RX_DMA) = 0;
}

static int ibustype(int n) {
	switch (n) {
		case 1: return 0x201; // Temperature
		case 2: return 0x202; // RPM
#if SENS_CNT >= 1
		case 3: return 0x203; // Voltage
#endif
#if SENS_CNT >= 2
		case 4: return 0x205; // Current
		case 5: return 0x206; // Consumption
#endif
	}
	return 0;
}

static int ibusval(int n) {
	switch (n) {
		case 1: return (temp + 40) * 10;
		case 2: return erpm / (cfg.telem_poles >> 1);
#if SENS_CNT >= 1
		case 3: return volt;
#endif
#if SENS_CNT >= 2
		case 4: return curr;
		case 5: return csum;
#endif
	}
	return 0;
}

static int ibusresp2(char a, int x) {
	char b = x, c = x >> 8;
	int u = 0xfff9 - a - b - c;
	iobuf[0] = 6;
	iobuf[1] = a;
	iobuf[2] = b;
	iobuf[3] = c;
	iobuf[4] = u;
	iobuf[5] = u >> 8;
	return 6;
}

static int ibusresp4(char a, int x) {
	char b = x, c = x >> 8, d = x >> 16, e = x >> 24;
	int u = 0xfff7 - a - b - c - d - e;
	iobuf[0] = 8;
	iobuf[1] = a;
	iobuf[2] = b;
	iobuf[3] = c;
	iobuf[4] = d;
	iobuf[5] = e;
	iobuf[6] = u;
	iobuf[7] = u >> 8;
	return 8;
}

static int ibusdma(void) {
	if (iobuf[0] != 4) { // Invalid frame
		resync();
		return 0;
	}
	char a = iobuf[1];
	if (a + (iobuf[2] | iobuf[3] << 8) != 0xfffb) { // Invalid checksum
		resync();
		return 0;
	}
	int n = a & 0xf;
	int t = ibustype(n);
	if (!t) return 0;
	switch (a & 0xf0) {
		case 0x80: return 4; // Probe
		case 0x90: return ibusresp2(a, t); // Type
		case 0xa0: { // Value
			int v = ibusval(n);
			switch (t >> 8) {
				case 2: return ibusresp2(a, v);
				case 4: return ibusresp4(a, v);
			}
		}
	}
	return 0;
}

static int sporttype(int n) {
	switch (n) {
		case 1: return 0xb70; // Temperature
		case 2: return 0x500; // RPM
#if SENS_CNT >= 1
		case 3: return 0x210; // Voltage
#endif
#if SENS_CNT >= 2
		case 4: return 0x200; // Current
		case 5: return 0x600; // Consumption
#endif
	}
	return 0;
}

static int sportval(int n) {
	switch (n) {
		case 1: return temp;
		case 2: return erpm / (cfg.telem_poles >> 1);
#if SENS_CNT >= 1
		case 3: return volt;
#endif
#if SENS_CNT >= 2
		case 4: return curr / 10;
		case 5: return csum;
#endif
	}
	return 0;
}

static void sportbyte(int *crc, int *pos, char b) {
	*crc += b;
	*crc += *crc >> 8;
	*crc &= 0xff;
	if (b == 0x7d || b == 0x7e) { // Byte stuffing
		b &= ~0x20;
		iobuf[(*pos)++] = 0x7d;
	}
	iobuf[(*pos)++] = b;
}

static int sportresp(int t, int v) {
	int crc = 0, pos = 0;
	sportbyte(&crc, &pos, 0x10);
	sportbyte(&crc, &pos, t);
	sportbyte(&crc, &pos, t >> 8);
	sportbyte(&crc, &pos, v);
	sportbyte(&crc, &pos, v >> 8);
	sportbyte(&crc, &pos, v >> 16);
	sportbyte(&crc, &pos, v >> 24);
	sportbyte(&crc, &pos, 0xff - crc);
	return pos;
}

static int sportdma(void) {
	static int n;
	if (iobuf[0] != 0x7e) { // Invalid frame
		resync();
		return 0;
	}
	if ((iobuf[1] & 0x1f) != cfg.telem_phid - 1) return 0;
	int t;
	while (!(t = sporttype(++n))) n = 0;
	return sportresp(t, sportval(n));
}

static char crc8(const char *buf, int len) {
	char crc = 0;
	while (len--) {
		crc ^= *buf++;
		for (int i = 0; i < 8; ++i) crc = crc & 0x80 ? (crc << 1) ^ 0x7 : crc << 1;
	}
	return crc;
}

void sendtelem(void) { // KISS
	if (DMA1_CCR(USART1_TX_DMA) & DMA_CCR_EN) return;
	int erpx = erpm / 100;
	iobuf[0] = temp;
	iobuf[1] = volt >> 8;
	iobuf[2] = volt;
	iobuf[3] = curr >> 8;
	iobuf[4] = curr;
	iobuf[5] = csum >> 8;
	iobuf[6] = csum;
	iobuf[7] = erpx >> 8;
	iobuf[8] = erpx;
	iobuf[9] = crc8(iobuf, 9);
	DMA1_CNDTR(USART1_TX_DMA) = 10;
	DMA1_CCR(USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}
