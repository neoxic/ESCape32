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

static void entryirq(void);
static void calibirq(void);
static void servoirq(void);
static void dshotirq(void);
static void dshotdma(void);
#ifdef IO_PA2
static void idleirq(void);
static void throtdma(void);
static void ibusdma(void);
static void sbusdma(void);
#endif
static void cliirq(void);

static void (*irqhandler)(void) = entryirq;
static void (*dmahandler)(void);

static char iobuf[1024], dshotinv;

void initio(void) {
	TIM_PSC(IOTIM) = MHZ(IOTIM_CLK) - 1; // 1us resolution
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_ARPE;
	TIM_BDTR(IOTIM) = TIM_BDTR_MOE;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1FP1; // Reset on rising edge on TI1
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_UIE | TIM_DIER_CC1IE;
}

void iotim_isr(void) {
	irqhandler();
}

void iotim_dma_isr(void) {
	dmahandler();
}

#ifdef IO_PA2
void usart2_isr(void) {
	irqhandler();
}

void usart2_dma_isr(void) {
	dmahandler();
}
#endif

static void entryirq(void) { // Signal detection
	static int m, n, u = 5;
	if (TIM_SR(IOTIM) & TIM_SR_UIF) { // Timeout ~65.5ms
		TIM_SR(IOTIM) = ~TIM_SR_UIF;
		n = 0;
		if (!(IOTIM_IDR & IOTIM_PIN) || ++m < 4) return;
		irqhandler = cliirq;
		io_pullup();
#ifdef IO_PA2
		io_serial();
		USART2_BRR = CNT(PCLK1, 38400);
		USART2_CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
		USART2_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
#else
		TIM3_CCER = 0;
		TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM3_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_CK_INT_N_8;
		TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
		TIM3_SR = ~TIM_SR_CC2IF;
		TIM3_DIER = TIM_DIER_CC2IE;
		TIM3_PSC = 0;
		TIM3_ARR = CNT(PCLK1, 38400) - 1; // Bit time
		TIM3_CCR1 = CNT(PCLK1, 76800); // Half-bit time
		TIM3_EGR = TIM_EGR_UG;
		TIM3_CR1 = TIM_CR1_CEN;
#endif
		return;
	}
	WWDG_CR = 0xff; // Start watchdog
#ifdef IO_PA2
	int mode = cfg.serial_mode;
	if (mode) {
		irqhandler = idleirq;
		io_serial();
		int brr, len;
		switch (mode) {
			case 1: // Serial throttle
				dmahandler = throtdma;
				brr = CNT(PCLK1, 115200);
				len = 2;
				break;
			case 2: // iBUS
				dmahandler = ibusdma;
				brr = CNT(PCLK1, 115200);
				len = 32;
				break;
			case 3: // S.BUS (8E2)
				dmahandler = sbusdma;
				brr = CNT(PCLK1, 100000);
				len = 25;
				USART2_CR1 = USART_CR1_PCE | USART_CR1_M0;
				USART2_CR2 = USART_CR2_STOPBITS_2 | USART_CR2_RXINV;
				break;
		}
		USART2_BRR = brr;
		USART2_CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
		USART2_CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE; // Wait for idle line
		DMA1_CPAR(USART2_RX_DMA) = (uint32_t)&USART2_RDR;
		DMA1_CMAR(USART2_RX_DMA) = (uint32_t)iobuf;
		DMA1_CNDTR(USART2_RX_DMA) = len;
		DMA1_CCR(USART2_RX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
		return;
	}
#endif
	int t = TIM_CCR1(IOTIM); // Time between two rising edges
	if (!n++) return; // First capture is always invalid
	if (t >= 2800) { // Servo PWM
		irqhandler = calibirq;
		calibirq();
		return;
	}
	if (t >= 5 || n <= 4) return;
	if (u != t) {
		u = t;
		n = 1;
		return;
	}
	irqhandler = dshotirq;
	dmahandler = dshotdma;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM_CCER(IOTIM) = TIM_CCER_CC1E | TIM_CCER_CC1P; // IC1 on falling edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_UIE; // Wait for idle line
	TIM_PSC(IOTIM) = u >= 2; // 0 - DSHOT 600, 1 - DSHOT 300
	TIM_ARR(IOTIM) = CNT(IOTIM_CLK, 300000) - 1; // Minimum blank time between frames
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	DMA1_CPAR(IOTIM_DMA) = (uint32_t)&TIM_CCR1(IOTIM);
	DMA1_CMAR(IOTIM_DMA) = (uint32_t)iobuf;
	DMA1_CNDTR(IOTIM_DMA) = 16;
	DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_8BIT;
}

static void calibirq(void) { // Align pulse period to the nearest millisecond via HSI trimming
	static int n, q, x, y;
	if (!cfg.throt_cal) goto done;
	int p = TIM_CCR1(IOTIM); // Pulse period
	if (p < 2800) return; // Invalid signal
	WWDG_CR = 0xff; // Reset watchdog
	q += p - ((p + 500) / 1000) * 1000; // Cumulative error
	if (++n & 3) return;
#ifdef STM32G0
	int cr = RCC_ICSCR;
	int ht = (cr & 0x7f00) >> 8;
#else
	int cr = RCC_CR;
	int ht = (cr & 0xf8) >> 3;
#endif
	if (q > x) { // Slow down
		y = -q;
		--ht;
	} else if (q < y) { // Speed up
		x = -q;
		++ht;
	} else {
	done:
		irqhandler = servoirq;
		TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8;
		TIM_CCER(IOTIM) = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P; // IC1 on rising edge on TI1, IC2 on falling edge on TI1
		TIM_SR(IOTIM) = ~TIM_SR_CC2IF;
		TIM_DIER(IOTIM) = TIM_DIER_CC2IE;
		return;
	}
#ifdef STM32G0
	RCC_ICSCR = (cr & ~0x7f00) | (ht & 0x7f) << 8;
#else
	RCC_CR = (cr & ~0xf8) | (ht & 0x1f) << 3;
#endif
	q = 0;
}

static void servoval(int x) {
	throt = cfg.throt_mode ?
		x < cfg.throt_mid - 50 ? scale(x, cfg.throt_min, cfg.throt_mid - 50, -2000, 0):
		x > cfg.throt_mid + 50 ? scale(x, cfg.throt_mid + 50, cfg.throt_max, 0, 2000): 0:
		x > cfg.throt_min + 50 ? scale(x, cfg.throt_min + 50, cfg.throt_max, 0, 2000): 0;
}

static void servoirq(void) { // Servo PWM
	int p = TIM_CCR1(IOTIM); // Pulse period
	int w = TIM_CCR2(IOTIM); // Pulse width
	if (p < 2800) return; // Invalid signal
	if (w >= 28 && w <= 32) { // Telemetry request
		telem = 1;
		goto done;
	}
	if (w < 800 || w > 2200) return; // Invalid signal
	servoval(w);
done:
	WWDG_CR = 0xff; // Reset watchdog
}

static void dshotirq(void) { // DSHOT sync
	if (!(TIM_DIER(IOTIM) & TIM_DIER_UIE) || !(TIM_SR(IOTIM) & TIM_SR_UIF)) return; // Fall through exactly once
	TIM_SR(IOTIM) = ~TIM_SR_UIF;
	if (IOTIM_IDR & IOTIM_PIN) { // Bidirectional DSHOT is inverted
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
		io_pullup();
		dshotinv = 1;
	}
	TIM_ARR(IOTIM) = -1;
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE;
	TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
}

static int dshotcrc(int x, int inv) {
	int a = x;
	for (int b = x; b >>= 4; a ^= b);
	if (inv) a = ~a;
	return a & 0x0f;
}

static void dshottelem(void) { // DSHOT telemetry
	static const char gcr[] = {0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17, 0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f};
	int a = erpt ? erpt : 65408;
	int b = 0;
	if (a > 65408) a = 65408;
	while (a > 511) a >>= 1, ++b;
	b = b << 13 | a << 4;
	b |= dshotcrc(b, 1);
	a = 0;
	for (int i = 0, j = 0; i < 16; i += 4, j += 5) a |= gcr[b >> i & 0x0f] << j;
	for (int p = -1, i = 19; i >= 0; --i) {
		if (a >> i & 1) p = ~p;
		iobuf[20 - i] = p;
	}
	iobuf[0] = -1;
	iobuf[21] = 0;
}

static void dshotdma(void) { // DSHOT frame
	DMA1_IFCR = DMA_IFCR_CTCIF(IOTIM_DMA);
	if (DMA1_CCR(IOTIM_DMA) & DMA_CCR_DIR) { // Switch back to RX
		TIM_DIER(IOTIM) = 0;
		TIM_CCER(IOTIM) = 0;
		DMA1_CCR(IOTIM_DMA) = 0;
		DMA1_CNDTR(IOTIM_DMA) = 16;
		DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_8BIT;
		TIM_CR2(IOTIM) = 0;
		TIM_ARR(IOTIM) = -1;
		TIM_EGR(IOTIM) = TIM_EGR_UG;
		TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_8;
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
		TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
		return;
	}
	int x = 0;
	for (int i = 0; i < 16; ++i) {
		x <<= 1;
		if (iobuf[i] >= CNT(IOTIM_CLK, 1066666)) x |= 1;
	}
	if (dshotcrc(x, dshotinv)) return; // Invalid checksum
	WWDG_CR = 0xff; // Reset watchdog
	if (dshotinv) { // Switch to TX
		TIM_DIER(IOTIM) = 0;
		TIM_CCER(IOTIM) = 0;
		TIM_SMCR(IOTIM) = 0;
		DMA1_CCR(IOTIM_DMA) = 0;
		DMA1_CNDTR(IOTIM_DMA) = 22;
		DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_8BIT;
		TIM_CR2(IOTIM) = TIM_CR2_CCDS; // CC1 DMA request on UEV using the same DMA channel
		TIM_CCMR1(IOTIM) = 0; // Disable OC before enabling PWM to force OC1REF update (RM: Note #2)
		TIM_CCMR1(IOTIM) = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2;
		TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
		TIM_CCR1(IOTIM) = 0; // Preload high level
		dshottelem(); // Prepare data
		int arr = (CNT(IOTIM_CLK, 40000) >> TIM_PSC(IOTIM)) - TIM_CNT(IOTIM) - 1; // Calculate 25us output delay
		if (arr < 99) arr = 99; // Sanity check
		TIM_ARR(IOTIM) = arr; // Preload output delay
		TIM_EGR(IOTIM) = TIM_EGR_UG; // Update registers and trigger DMA to preload the first bit
		TIM_ARR(IOTIM) = CNT(IOTIM_CLK, 750000) - 1; // Preload bit time
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // Enable output
	}
	if (x & 0x10) telem = 1; // Telemetry request
	x >>= 5;
	if (x > 47) {
		throt = cfg.throt_mode ? (x > 1047 ? x - 1047 : 47 - x) << 1 : x - 47;
		return;
	}
	throt = 0;
	static int cmd, cnt;
	if (cmd != x) {
		cmd = x;
		cnt = 0;
	}
	if (cnt < 10) ++cnt;
	if (erpt) return; // Motor must be stopped
	switch (cmd) {
		case 1: // DIGITAL_CMD_BEEP1
			playmusic(cfg.music1, cfg.volume1);
			break;
		case 2: // DIGITAL_CMD_BEEP2
			playmusic(cfg.music2, cfg.volume2);
			break;
		case 3: // DIGITAL_CMD_BEEP3
			playmusic(cfg.music3, cfg.volume3);
			break;
		case 7: // DIGITAL_CMD_SPIN_DIRECTION_1
		case 20: // DIGITAL_CMD_SPIN_DIRECTION_NORMAL
			if (cnt != 6) break;
			cfg.reverse = 0;
			break;
		case 8: // DIGITAL_CMD_SPIN_DIRECTION_2
		case 21: // DIGITAL_CMD_SPIN_DIRECTION_REVERSED
			if (cnt != 6) break;
			cfg.reverse = 1;
			break;
		case 9: // DIGITAL_CMD_3D_MODE_OFF
			if (cnt != 6) break;
			cfg.throt_mode = 0;
			break;
		case 10: // DIGITAL_CMD_3D_MODE_ON
			if (cnt != 6) break;
			cfg.throt_mode = 1;
			break;
		case 12: // DIGITAL_CMD_SAVE_SETTINGS
			if (cnt != 6) break;
			savecfg();
			break;
	}
}

#ifdef IO_PA2
static void idleirq(void) { // Serial sync
	USART2_ICR = USART_ICR_IDLECF;
	USART2_RQR = USART_RQR_RXFRQ;
	if (USART2_ISR & USART_ISR_FE) { // Data error
		USART2_ICR = USART_ICR_FECF;
		return;
	}
	USART2_CR1 &= ~USART_CR1_IDLEIE;
	USART2_CR3 |= USART_CR3_DMAR;
}

static void throtdma(void) { // Serial frame
	DMA1_IFCR = DMA_IFCR_CTCIF(USART2_RX_DMA);
	int x = iobuf[0] | iobuf[1] << 8;
	if (dshotcrc(x, 0)) return; // Invalid checksum
	x &= 0xffffff;
	if (x & 0x800000) x -= 0x1000000; // Propagate sign
	throt = x;
	WWDG_CR = 0xff; // Reset watchdog
}

static void ibusdma(void) { // iBUS frame
	DMA1_IFCR = DMA_IFCR_CTCIF(USART2_RX_DMA);
	if (iobuf[0] != 0x20 || iobuf[1] != 0x40) return; // Invalid frame
	int n = cfg.serial_chid;
	int x = 1500;
	int u = 0xff9f;
	for (int i = 1;; ++i) {
		int j = i << 1;
		char a = iobuf[j];
		char b = iobuf[j + 1];
		int v = a | b << 8;
		if (i == 15) {
			if (u != v) return; // Invalid checksum
			break;
		}
		u -= a + b;
		if (i == n) x = v & 0x0fff;
	}
	servoval(x);
	WWDG_CR = 0xff; // Reset watchdog
}

static void sbusdma(void) { // S.BUS frame
	DMA1_IFCR = DMA_IFCR_CTCIF(USART2_RX_DMA);
	if (iobuf[0] != 0x0f || iobuf[24] != 0x00) return; // Invalid frame
	int n = cfg.serial_chid - 1;
	int i = (n * 11) >> 3;
	int a = (n * 3) & 7;
	int b = ((n + 1) * 3) & 7;
	int x = a < b ?
		iobuf[i + 1] >> a | (iobuf[i + 2] & ((1 << b) - 1)) << (8 - a):
		iobuf[i + 1] >> a | iobuf[i + 2] << (8 - a) | (iobuf[i + 3] & ((1 << b) - 1)) << a;
	throt = cfg.throt_mode ?
		x < 946 ? scale(x, 240, 946, -2000, 0):
		x > 1101 ? scale(x, 1101, 1807, 0, 2000): 0:
		x > 318 ? scale(x, 318, 1807, 0, 2000): 0;
	WWDG_CR = 0xff; // Reset watchdog
}

static void cliirq(void) { // CLI on USART2
	static int i, j;
	int cr = USART2_CR1;
	int sr = USART2_ISR; // A read from SR followed by a read/write access to DR properly clears status bits
	if ((cr & USART_CR1_TXEIE) && (sr & USART_ISR_TXE)) { // Ready for next byte
		if (i < j) {
			USART2_TDR = iobuf[i++];
			return;
		}
		i = 0;
		j = 0;
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
		return;
	}
	if ((cr & USART_CR1_TCIE) && (sr & USART_ISR_TC)) { // Transmission complete
		USART2_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
		return;
	}
	if (sr & USART_ISR_FE || i == sizeof iobuf - 1) reset(); // Data error
	char b = USART2_RDR;
	iobuf[i++] = b;
	if (i == 2 && iobuf[0] == 0x00 && iobuf[1] == 0xff) scb_reset_system(); // Reboot into bootloader
	if (b != '\n') return;
	iobuf[i] = '\0';
	i = 0;
	j = execcmd(iobuf);
	USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TXEIE;
}
#else
static void cliirq(void) { // CLI on semi-software UART using TIM3
	static int i, j, b, n;
	int er = TIM3_DIER;
	int sr = TIM3_SR;
	if ((er & TIM_DIER_UIE) && (sr & TIM_SR_UIF)) {
		TIM3_SR = ~TIM_SR_UIF;
		if (i < j) { // Output
			int p = 0;
			if (++n == 10) { // Stop bit
				n = 0;
				if (++i == j) { // End of data
					i = 0;
					j = 0;
				}
			} else if (n == 1 || !(iobuf[i] & (1 << (n - 2)))) p = -1; // Start bit or data bit Low
			TIM3_CCR1 = p;
			return;
		}
		TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
		TIM3_DIER = TIM_DIER_CC2IE;
		TIM3_CCR1 = CNT(PCLK1, 76800); // Half-bit time
		TIM3_EGR = TIM_EGR_UG;
	}
	if ((er & TIM_DIER_CC1IE) && (sr & TIM_SR_CC1IF)) { // Half-bit time
		TIM3_SR = ~TIM_SR_CC1IF;
		int p = IOTIM_IDR & IOTIM_PIN; // Signal level
		if (!n++) { // Start bit
			if (p) reset(); // Data error
			b = 0;
			return;
		}
		if (n < 10) { // Data bit
			if (p) b |= 1 << (n - 2);
			return;
		}
		if (!p || i == sizeof iobuf - 1) reset(); // Data error
		n = 0;
		TIM3_SR = ~TIM_SR_CC2IF;
		TIM3_DIER = TIM_DIER_CC2IE;
		iobuf[i++] = b;
		if (i == 2 && iobuf[0] == 0x00 && iobuf[1] == 0xff) scb_reset_system(); // Reboot into bootloader
		if (b != '\n') return;
		iobuf[i] = '\0';
		i = 0;
		j = execcmd(iobuf);
		TIM3_SMCR = 0;
		TIM3_CCR1 = 0; // Preload high level
		TIM3_EGR = TIM_EGR_UG; // Update registers and trigger UEV
		TIM3_CCER = TIM_CCER_CC1E; // Enable output
		TIM3_DIER = TIM_DIER_UIE;
	}
	if ((er & TIM_DIER_CC2IE) && (sr & TIM_SR_CC2IF)) { // Falling edge
		TIM3_SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
		TIM3_DIER = TIM_DIER_CC1IE;
	}
}
#endif
