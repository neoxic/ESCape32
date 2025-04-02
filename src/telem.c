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

#include "common.h"

#ifndef USART1_DMA_BASE
#define USART1_DMA_BASE DMA1_BASE
#endif

#ifdef AT32F4
#define USART1_TDR USART1_DR
#define USART1_RDR USART1_DR
#endif

static int ibusfunc(int len);
static int sportfunc(int len);
static int (*iofunc)(int len);
static char iobuf[16];

void inittelem(void) {
	USART1_BRR = CLK_CNT(115200);
	switch (telmode) {
		case 2: // iBUS
			iofunc = ibusfunc;
#ifndef AT32F4
			USART1_RTOR = 12; // TX delay ~100us
			USART1_CR2 = USART_CR2_RTOEN;
#endif
			break;
		case 3: // S.Port
			iofunc = sportfunc;
			USART1_BRR = CLK_CNT(57600);
#ifndef AT32F4
			USART1_RTOR = 26; // TX delay ~450us
			USART1_CR2 = USART_CR2_RTOEN | USART_CR2_RXINV | USART_CR2_TXINV;
			GPIOB_PUPDR = (GPIOB_PUPDR & ~0x3000) | 0x2000; // B6 (pull-down)
#endif
			break;
		case 4: // CRSF
			USART1_BRR = CLK_CNT(416666);
			break;
	}
	if (iofunc) {
		USART1_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;
		USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
	} else {
		USART1_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT;
		USART1_CR1 = USART_CR1_UE | USART_CR1_TE;
	}
	DMA_CPAR(USART1_DMA_BASE, USART1_RX_DMA) = (uint32_t)&USART1_RDR;
	DMA_CMAR(USART1_DMA_BASE, USART1_RX_DMA) = (uint32_t)iobuf;
	DMA_CPAR(USART1_DMA_BASE, USART1_TX_DMA) = (uint32_t)&USART1_TDR;
	DMA_CMAR(USART1_DMA_BASE, USART1_TX_DMA) = (uint32_t)iobuf;
}

void usart1_isr(void) {
	int cr = USART1_CR1;
	if (cr & USART_CR1_TCIE) goto reading;
#ifdef AT32F4
	USART1_SR, USART1_DR; // Clear flags
#else
	USART1_RQR = USART_RQR_RXFRQ; // Clear RXNE
	USART1_ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_RTOCF;
#endif
	int len = iofunc(sizeof iobuf - DMA_CNDTR(USART1_DMA_BASE, USART1_RX_DMA));
	if (len) {
#ifdef AT32F4
		USART1_SR = ~USART_SR_TC;
#else
		USART1_ICR = USART_ICR_TCCF;
#endif
		USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
		DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = 0;
		DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = len;
		DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
		return;
	}
reading:
#ifdef AT32F4
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
#else
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RTOIE;
#endif
	DMA_CCR(USART1_DMA_BASE, USART1_RX_DMA) = 0;
	DMA_CNDTR(USART1_DMA_BASE, USART1_RX_DMA) = sizeof iobuf;
	DMA_CCR(USART1_DMA_BASE, USART1_RX_DMA) = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

void usart1_tx_dma_isr(void) {
	DMA_IFCR(USART1_DMA_BASE) = DMA_IFCR_CTCIF(USART1_TX_DMA);
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = 0;
}

static int ibusresp(char a, int x) {
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

static int ibusfunc(int len) {
	static const uint16_t type[] = {0x201, 0x201, 0x203, 0x205, 0x206, 0x202};
	if (len != 4 || iobuf[0] != 4) return 0; // Invalid frame
	char a = iobuf[1];
	if (a + (iobuf[2] | iobuf[3] << 8) != 0xfffb) return 0; // Invalid checksum
	int n = (a & 0xf) - (cfg.telem_phid - 1) * 6 - 1;
	if (n < 0 || n > 5) return 0;
	switch (a >> 4) {
		case 0x8: return 4; // Probe
		case 0x9: return ibusresp(a, type[n]); // Type
		case 0xa: // Value
			switch (n) {
				case 0: return ibusresp(a, (temp1 + 40) * 10);
				case 1: return ibusresp(a, (temp2 + 40) * 10);
				case 2: return ibusresp(a, volt);
				case 3: return ibusresp(a, curr);
				case 4: return ibusresp(a, csum);
				case 5: return ibusresp(a, min(erpm / (cfg.telem_poles >> 1), 0xffff));
			}
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

static int sportfunc(int len) {
	static const uint16_t type[] = {0xb70, 0x400, 0x210, 0x200, 0xb30, 0x500};
	static int n;
	if (len != 2 || iobuf[0] != 0x7e) return 0; // Invalid frame
	if ((iobuf[1] & 0x1f) != cfg.telem_phid - 1) return 0;
	if (n == 6) n = 0;
	int t = type[n];
	switch (n++) {
		case 0: return sportresp(t, temp1);
		case 1: return sportresp(t, temp2);
		case 2: return sportresp(t, volt);
		case 3: return sportresp(t, curr * 205 >> 11);
		case 4: return sportresp(t, csum);
		case 5: return sportresp(t, erpm / (cfg.telem_poles >> 1));
	}
	return 0;
}

void kisstelem(void) {
	if (DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) & DMA_CCR_EN) return;
	int r = erpm * 41 >> 12;
	iobuf[0] = temp1;
	iobuf[1] = volt >> 8;
	iobuf[2] = volt;
	iobuf[3] = curr >> 8;
	iobuf[4] = curr;
	iobuf[5] = csum >> 8;
	iobuf[6] = csum;
	iobuf[7] = r >> 8;
	iobuf[8] = r;
	iobuf[9] = crc8(iobuf, 9);
	DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = 10;
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

static void crsftelem(void) {
	if (DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) & DMA_CCR_EN) return;
	int v = volt * 205 >> 11;
	int c = curr * 205 >> 11;
	iobuf[0] = 0xc8;
	iobuf[1] = 0x0a;
	iobuf[2] = 0x08;
	iobuf[3] = v >> 8;
	iobuf[4] = v;
	iobuf[5] = c >> 8;
	iobuf[6] = c;
	iobuf[7] = csum >> 16;
	iobuf[8] = csum >> 8;
	iobuf[9] = csum;
	iobuf[10] = 0;
	iobuf[11] = crc8dvbs2(iobuf + 2, 9);
	DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = 12;
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

void autotelem(void) {
	static int n;
	int x = 0;
	switch (telmode) {
		case 1: // KISS
			kisstelem();
			break;
		case 4: // CRSF
			crsftelem();
			break;
	}
	if (!dshotext) return;
	switch (n++) { // Extended DSHOT telemetry
		case 0:
			x = 0x200 | (temp1 & 0xff); // ESC temperature (C)
			break;
		case 1:
			x = 0x800 | (temp2 & 0xff); // Motor temperature (C)
			break;
		case 2:
			x = 0x400 | ((volt * 41 >> 10) & 0xff); // Voltage (V/4)
			break;
		case 3:
			x = 0x600 | ((curr * 41 >> 12) & 0xff); // Current (A)
			n = 0;
			break;
	}
	__disable_irq();
	if (!dshotval) dshotval = x;
	__enable_irq();
}
