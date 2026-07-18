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
static int msbfunc(int len);
static int hottfunc(int len);
static int (*iofunc)(int len);
static char iocnt, iobuf[128] __attribute__((aligned(2)));
static char *iopos = iobuf;
static char *ioend = iobuf;

void inittelem(void) {
#ifndef AT32F4
	USART1_RTOR = 10;
	USART1_CR2 = USART_CR2_RTOEN;
#endif
	switch (telmode) {
		case 0: // KISS
		case 1: // KISS auto
			USART1_BRR = CLK_CNT(115200);
			break;
		case 2: // iBUS
			iofunc = ibusfunc;
			USART1_BRR = CLK_CNT(115200);
			break;
		case 3: // S.Port
			iofunc = sportfunc;
			USART1_BRR = CLK_CNT(57600);
#ifndef AT32F4
			USART1_RTOR = 26; // ~450us
			USART1_CR2 |= USART_CR2_RXINV | USART_CR2_TXINV;
			GPIOB_PUPDR = (GPIOB_PUPDR & ~0x3000) | 0x2000; // B6 (pull-down)
#endif
			break;
		case 4: // CRSF
			USART1_BRR = CLK_CNT(416666);
			break;
#ifndef DISABLE_MSB
		case 5: // MSB
			iofunc = msbfunc;
			USART1_BRR = CLK_CNT(38400);
			break;
#endif
#ifndef DISABLE_HOTT
		case 6: // HoTT
			iofunc = hottfunc;
			USART1_BRR = CLK_CNT(19200);
			break;
#endif
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
	DMA_CNDTR(USART1_DMA_BASE, USART1_RX_DMA) = sizeof iobuf;
}

void usart1_isr(void) {
	if (USART1_CR1 & USART_CR1_TCIE) {
		DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = 0;
		goto reading;
	}
#ifdef AT32F4
	USART1_SR, USART1_DR; // Clear flags
#else
	USART1_RQR = USART_RQR_RXFRQ; // Clear RXNE
	USART1_ICR = USART_ICR_IDLECF | USART_ICR_ORECF | USART_ICR_RTOCF;
#endif
	DMA_CCR(USART1_DMA_BASE, USART1_RX_DMA) = 0;
	int len = iofunc(sizeof iobuf - DMA_CNDTR(USART1_DMA_BASE, USART1_RX_DMA));
	if (len) {
		if (len < 0) return;
#ifdef AT32F4
		USART1_SR = ~USART_SR_TC;
#else
		USART1_ICR = USART_ICR_TCCF;
#endif
		USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
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
	DMA_CNDTR(USART1_DMA_BASE, USART1_RX_DMA) = sizeof iobuf;
	DMA_CCR(USART1_DMA_BASE, USART1_RX_DMA) = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

void usart1_tx_dma_isr(void) {
	DMA_IFCR(USART1_DMA_BASE) = DMA_IFCR_CTCIF(USART1_TX_DMA);
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = 0;
	__disable_irq();
	if (iopos > ioend) {
		DMA_CMAR(USART1_DMA_BASE, USART1_TX_DMA) = (uint32_t)ioend;
		DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = iopos - ioend;
		DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
		ioend = iopos;
	} else {
		iopos = iobuf;
		ioend = iobuf;
	}
	__enable_irq();
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
	if (len != 4 || iobuf[0] != 4) return 0;
	int a = iobuf[1];
	if (a + (iobuf[2] | iobuf[3] << 8) != 0xfffb) return 0; // Invalid checksum
	int n = (a & 0xf) - (telphid - 1) * 6 - 1;
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
	static uint8_t n;
	if (len != 2 || iobuf[0] != 0x7e || (iobuf[1] & 0x1f) != telphid - 1) return 0;
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

static int msbresp(char a, int x) {
	iobuf[0] = a;
	iobuf[1] = x << 1;
	iobuf[2] = x >> 7;
	return 3;
}

static int msbfunc(int len) {
	static const char type[] = {6, 6, 1, 2, 11, 5};
	if (len != 1) return 0;
	int a = iobuf[0];
	if (a == 0x5a) { // "Clear" command
		csum = 0;
		return 0;
	}
	int n = a - (telphid - 1) * 6 - 4;
	if (n < 0 || n > 5) return 0;
	a = a << 4 | type[n];
	switch (n) {
		case 0: return msbresp(a, temp1 * 10);
		case 1: return msbresp(a, temp2 * 10);
		case 2: return msbresp(a, volt * 205 >> 11);
		case 3: return msbresp(a, curr * 205 >> 11);
		case 4: return msbresp(a, csum);
		case 5: return msbresp(a, erpm / (cfg.telem_poles * 50));
	}
	return 0;
}

static int hottfunc(int len) {
	if (len != 2 || iobuf[0] != 0x80 || iobuf[1] != 0x8c) return 0;
	static int t, u, v, c, r;
	int a, b;
	iobuf[0] = 0x7c;
	iobuf[1] = 0x8c;
	iobuf[2] = 0x00;
	iobuf[3] = 0xc0;
	iobuf[4] = 0x00;
	iobuf[5] = 0x00;
	iobuf[6] = a = volt * 205 >> 11;
	iobuf[7] = a >> 8;
	iobuf[8] = v = min(v, a);
	iobuf[9] = v >> 8;
	iobuf[10] = a = csum * 205 >> 11;
	iobuf[11] = a >> 8;
	iobuf[12] = a = temp1 + 20;
	iobuf[13] = t = max(t, a);
	iobuf[14] = a = curr * 205 >> 11;
	iobuf[15] = a >> 8;
	iobuf[16] = c = max(c, a);
	iobuf[17] = c >> 8;
	iobuf[18] = a = min(erpm / (cfg.telem_poles * 5), 0xffff);
	iobuf[19] = a >> 8;
	iobuf[20] = r = max(r, a);
	iobuf[21] = r >> 8;
	iobuf[22] = a = temp2 + 20;
	iobuf[23] = u = max(u, a);
	memset(iobuf + 24, 0, 19);
	iobuf[43] = 0x7d;
	for (a = 0, b = 0; a < 44; b += iobuf[a++]);
	iobuf[44] = b;
	iocnt = 5; // Output delay
	iopos = iobuf;
	ioend = iobuf + 45;
	USART1_CR1 = USART_CR1_UE | USART_CR1_TE;
	return -1;
}

static void sendkiss(void) {
	if (DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) & DMA_CCR_EN) return;
	int a = erpm * 41 >> 12;
	iobuf[0] = temp1;
	iobuf[1] = volt >> 8;
	iobuf[2] = volt;
	iobuf[3] = curr >> 8;
	iobuf[4] = curr;
	iobuf[5] = csum >> 8;
	iobuf[6] = csum;
	iobuf[7] = a >> 8;
	iobuf[8] = a;
	iobuf[9] = crc8(iobuf, 9);
	DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = 10;
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

static void sendcrsf(void) {
	static char n;
	int a = 0, b;
	char buf[12] = {0xc8};
	switch (n++) {
		case 0: // Temperature
			a = temp1 * 10;
			b = temp2 * 10;
			buf[2] = 0x0d;
			buf[3] = telphid - 1;
			buf[4] = a >> 8;
			buf[5] = a;
			buf[6] = b >> 8;
			buf[7] = b;
			a = 6;
			break;
		case 1: // Battery or voltage
			// if (telphid > 1) {
			// 	a = min(volt * 10, 0xffff);
			// 	buf[2] = 0x0e;
			// 	buf[3] = 0x80 + telphid - 2;
			// 	buf[4] = a >> 8;
			// 	buf[5] = a;
			// 	a = 4;
			// 	break;
			// }
			a = volt * 205 >> 11;
			b = curr * 205 >> 11;
			buf[2] = 0x08;
			buf[3] = a >> 8;
			buf[4] = a;
			buf[5] = b >> 8;
			buf[6] = b;
			buf[7] = csum >> 16;
			buf[8] = csum >> 8;
			buf[9] = csum;
			buf[10] = 0;
            a = 9;
            if (telphid > 0) {
                buf[11] = telphid;
                a = 10; 
            }
			break;
		case 2: // RPM
			a = erpm / (cfg.telem_poles >> 1);
			buf[2] = 0x0c;
			buf[3] = telphid - 1;
			buf[4] = a >> 16;
			buf[5] = a >> 8;
			buf[6] = a;
			a = 5;
			n = 0;
			break;
	}
	buf[1] = a + 1;
	buf[a + 2] = crc8dvbs2(buf + 2, a);
	sendtelemdata(buf, a + 3);
}

void sendtelem(void) {
	if (telreq && !telmode) { // KISS
		sendkiss();
		telreq = 0;
	}
	if (tick & 0xf) return; // 16kHz -> 1kHz
	if (iopos < ioend) {
		if (iocnt) { // Output delay
			--iocnt;
			return;
		}
		USART1_TDR = *iopos++; // Clear TXE+TC
		if (iopos == ioend) USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
	}
	if (tick & 0x1f0) return; // 1ms -> 32ms
	switch (telmode) {
		case 1: // KISS auto
			sendkiss();
			break;
		case 4: // CRSF
			sendcrsf();
			break;
	}
	if (!dshotext) return;
	static char n;
	int x = 0;
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

void sendtelemdata(const char *buf, int len) {
	__disable_irq();
	char *pos = iopos;
	if (pos + len > iobuf + sizeof iobuf) len = 0; // Packet too big
	iopos += len;
	if (pos == iobuf) ioend = iopos;
	__enable_irq();
	if (!len) return;
	memcpy(pos, buf, len);
	if (pos != iobuf) return;
	DMA_CMAR(USART1_DMA_BASE, USART1_TX_DMA) = (uint32_t)iobuf;
	DMA_CNDTR(USART1_DMA_BASE, USART1_TX_DMA) = len;
	DMA_CCR(USART1_DMA_BASE, USART1_TX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}
