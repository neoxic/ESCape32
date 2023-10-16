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
#define USART2_TDR USART2_DR
#define USART2_RDR USART2_DR
#define USART2_ISR USART2_SR
#define USART_CR1_M0 USART_CR1_M
#define USART_CR2_RXINV 0
#define USART_CR2_TXINV 0
#define USART_ISR_FE USART_SR_FE
#endif

static void entryirq(void);
static void calibirq(void);
static void servoirq(void);
static void dshotirq(void);
static void dshotdma(void);
static void cliirq(void);
#ifdef IO_PA2
static void syncirq(void);
static void throtdma(void);
static void ibusdma(void);
static void sbusdma(void);
static char rxlen;
#endif

static void (*ioirq)(void);
static void (*iodma)(void);

static char dshotinv, iobuf[1024];
static uint16_t dshotarr1, dshotarr2, dshotbuf[32];

void initio(void) {
	ioirq = entryirq;
	TIM_BDTR(IOTIM) = TIM_BDTR_MOE;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1FP1; // Reset on rising edge on TI1
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_UIE | TIM_DIER_CC1IE;
	TIM_PSC(IOTIM) = CLK_MHZ - 1; // 1us resolution
	TIM_ARR(IOTIM) = -1;
	TIM_CR1(IOTIM) = TIM_CR1_URS;
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
}

static void entryirq(void) {
	static int n, c, d;
	if (TIM_SR(IOTIM) & TIM_SR_UIF) { // Timeout ~66ms
		TIM_SR(IOTIM) = ~TIM_SR_UIF;
		if (!IOTIM_IDR) { // Low level
			if (IO_ANALOG) goto analog;
			n = 0;
			return;
		}
		if (++c < 8) return; // Wait for ~500ms before entering CLI
		ioirq = cliirq;
#ifdef IO_PA2
		io_serial();
		USART2_BRR = CLK_CNT(38400);
		if (!(GPIOA_IDR & 0x8000)) USART2_CR3 = USART_CR3_HDSEL; // A15 low
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
#else
		TIM3_CCER = 0;
		TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM3_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_CK_INT_N_8;
		TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
		TIM3_SR = ~TIM_SR_CC2IF;
		TIM3_DIER = TIM_DIER_CC2IE;
		TIM3_PSC = 0;
		TIM3_ARR = CLK_CNT(38400) - 1; // Bit time
		TIM3_CCR1 = CLK_CNT(76800); // Half-bit time
		TIM3_EGR = TIM_EGR_UG;
		TIM3_CR1 = TIM_CR1_CEN;
#endif
		return;
	}
	if (IO_ANALOG) {
	analog:
		io_analog();
		analog = 1;
		return;
	}
	IWDG_KR = IWDG_KR_START;
#ifdef IO_PA2
	if (cfg.input_mode >= 2) {
		ioirq = syncirq;
		io_serial();
		USART2_BRR = CLK_CNT(115200);
		switch (cfg.input_mode) {
			case 2: // Serial throttle
				iodma = throtdma;
				rxlen = 2;
				break;
			case 3: // iBUS
				iodma = ibusdma;
				rxlen = 32;
				break;
			case 4: // SBUS
				iodma = sbusdma;
				rxlen = 25;
				USART2_BRR = CLK_CNT(100000);
				USART2_CR1 = USART_CR1_PCE | USART_CR1_M0;
				USART2_CR2 = USART_CR2_STOPBITS_2 | USART_CR2_RXINV | USART_CR2_TXINV;
				break;
		}
		USART2_CR3 = USART_CR3_HDSEL;
		USART2_CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_IDLEIE;
		DMA1_CPAR(USART2_RX_DMA) = (uint32_t)&USART2_RDR;
		DMA1_CMAR(USART2_RX_DMA) = (uint32_t)iobuf;
		return;
	}
#endif
	int t = TIM_CCR1(IOTIM); // Time between two rising edges
	if (!n++) return; // First capture is always invalid
	if (TIM_PSC(IOTIM)) {
		if (t > 2000) { // Servo/Oneshot125
			ioirq = calibirq;
			calibirq();
			return;
		}
		TIM_PSC(IOTIM) = TIM_PSC(IOTIM) == CLK_MHZ - 1 ? CLK_MHZ / 8 - 1 : 0;
		TIM_EGR(IOTIM) = TIM_EGR_UG;
		n = 0;
		return;
	}
	int m = 3;
	while (t >= CLK_CNT(800000)) t >>= 1, --m;
	if (d != m) {
		d = m;
		n = 1;
		return;
	}
	if (m < 1 || n < 4) return;
	ioirq = dshotirq;
	iodma = dshotdma;
	dshotarr1 = CLK_CNT(m * 150000) - 1;
	dshotarr2 = CLK_CNT(m * 375000) - 1;
	TIM_CCER(IOTIM) = 0;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_DIER(IOTIM) = TIM_DIER_UIE;
	TIM_ARR(IOTIM) = dshotarr1; // Frame reset timeout
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	DMA1_CPAR(IOTIM_DMA) = (uint32_t)&TIM_CCR1(IOTIM);
	DMA1_CMAR(IOTIM_DMA) = (uint32_t)dshotbuf;
}

static void calibirq(void) { // Align pulse period to the nearest millisecond via HSI trimming within 6.25% margin
	static int n, q, x, y;
	if (!cfg.throt_cal) goto done;
	int p = TIM_CCR1(IOTIM); // Pulse period
	if (p < 2000) return; // Invalid signal
	IWDG_KR = IWDG_KR_RESET;
	q += p - ((p + 500) / 1000) * 1000; // Cumulative error
	if (++n & 3) return;
	if (q > x) { // Slow down
		if ((q << 3) > p) goto done;
		y = -q;
		hsictl(-1);
	} else if (q < y) { // Speed up
		if ((q << 3) < -p) goto done;
		x = -q;
		hsictl(1);
	} else goto done;
	q = 0;
	return;
done:
	ioirq = servoirq;
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8;
	TIM_CCER(IOTIM) = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P; // IC1 on rising edge on TI1, IC2 on falling edge on TI1
	TIM_SR(IOTIM) = ~TIM_SR_CC2IF;
	TIM_DIER(IOTIM) = TIM_DIER_CC2IE;
}

static void servoval(int x) {
	throt = cfg.throt_mode ?
		x < cfg.throt_mid - 50 ? scale(x, cfg.throt_min, cfg.throt_mid - 50, -2000, 0):
		x > cfg.throt_mid + 50 ? scale(x, cfg.throt_mid + 50, cfg.throt_max, 0, 2000): 0:
		x > cfg.throt_min + 50 ? scale(x, cfg.throt_min + 50, cfg.throt_max, 0, 2000): 0;
}

static void servoirq(void) {
	int p = TIM_CCR1(IOTIM); // Pulse period
	int w = TIM_CCR2(IOTIM); // Pulse width
	if (p < 2000) return; // Invalid signal
	if (w >= 28 && w <= 32) { // Telemetry request
		IWDG_KR = IWDG_KR_RESET;
		telreq = 1;
		return;
	}
	if (w < 800 || w > 2200) return; // Invalid signal
	IWDG_KR = IWDG_KR_RESET;
	servoval(w);
}

static void dshotirq(void) {
	if (!(TIM_DIER(IOTIM) & TIM_DIER_UIE) || !(TIM_SR(IOTIM) & TIM_SR_UIF)) return; // Fall through exactly once
	TIM_SR(IOTIM) = ~TIM_SR_UIF;
	if (!TIM_CCER(IOTIM)) { // Detect DSHOT polarity
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on any edge on TI1
		dshotinv = IOTIM_IDR; // Inactive level
	}
	DMA1_CNDTR(IOTIM_DMA) = 32;
	DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	TIM_ARR(IOTIM) = -1;
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE;
	TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
}

static int dshotcrc(int x, int inv) {
	int a = x;
	for (int b = x; b >>= 4; a ^= b);
	if (inv) a = ~a;
	return a & 0xf;
}

static void dshotdma(void) {
	static const char gcr[] = {0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17, 0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f};
	static int cmd, cnt, rep;
	if (DMA1_CCR(IOTIM_DMA) & DMA_CCR_DIR) {
#ifdef AT32F421 // Errata 1.5.1
		RCC_APB2RSTR = RCC_APB2RSTR_TIM15RST;
		RCC_APB2RSTR = 0;
		TIM15_BDTR = TIM_BDTR_MOE;
		TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#else
		TIM_CCER(IOTIM) = 0;
		TIM_DIER(IOTIM) = 0;
		TIM_CR2(IOTIM) = 0;
#endif
		DMA1_CCR(IOTIM_DMA) = 0;
		DMA1_CNDTR(IOTIM_DMA) = 32;
		DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
		TIM_ARR(IOTIM) = -1;
		TIM_EGR(IOTIM) = TIM_EGR_UG;
		TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_CK_INT_N_8;
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on any edge on TI1
		TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
		return;
	}
	int x = 0;
	int y = dshotarr1 + 1; // Two bit time
	int z = y >> 2; // Half-bit time
	for (int i = 0; i < 32; ++i) {
		if (i && dshotbuf[i] >= y) goto resync; // Invalid pulse timing
		x <<= 1;
		if (dshotbuf[++i] >= z) x |= 1;
	}
	if (dshotcrc(x, dshotinv)) { // Invalid checksum
	resync:
		DMA1_CCR(IOTIM_DMA) = 0;
		TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
		TIM_ARR(IOTIM) = dshotarr1; // Frame reset timeout
		TIM_EGR(IOTIM) = TIM_EGR_UG;
		TIM_SR(IOTIM) = ~TIM_SR_UIF;
		TIM_DIER(IOTIM) = TIM_DIER_UIE;
		return;
	}
	IWDG_KR = IWDG_KR_RESET;
	if (dshotinv) { // Bidirectional DSHOT
		TIM_CCER(IOTIM) = 0;
		TIM_SMCR(IOTIM) = 0;
		TIM_CCMR1(IOTIM) = 0; // Disable OC before enabling PWM to force OC1REF update (RM: OC1M, note #2)
		TIM_CCMR1(IOTIM) = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2;
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // Enable output as soon as possible (GD32F350 makes a twitch)
		TIM_CR2(IOTIM) = TIM_CR2_CCDS; // CC1 DMA request on UEV using the same DMA channel
		DMA1_CCR(IOTIM_DMA) = 0;
		DMA1_CNDTR(IOTIM_DMA) = 23;
		DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
		if (!dshotval) {
			int a = ertm ? ertm : 65408;
			int b = 0;
			if (a > 65408) a = 65408;
			while (a > 511) a >>= 1, ++b;
			dshotval = a | b << 9;
		}
		int a = dshotval << 4 | dshotcrc(dshotval, 1);
		int b = 0;
		for (int i = 0, j = 0; i < 16; i += 4, j += 5) b |= gcr[a >> i & 0xf] << j;
		for (int p = -1, i = 19; i >= 0; --i) {
			if (b >> i & 1) p = ~p;
			dshotbuf[20 - i] = p;
		}
		dshotbuf[0] = -1;
		dshotbuf[21] = 0;
		dshotbuf[22] = 0;
		__disable_irq();
		int arr = CLK_CNT(33333) - TIM_CNT(IOTIM) - 1; // Calculate 30us output delay
		if (arr < 99) arr = 99; // Sanity check
		TIM_ARR(IOTIM) = arr; // Preload output delay
		TIM_CCR1(IOTIM) = 0; // Preload high level
		TIM_EGR(IOTIM) = TIM_EGR_UG; // Update registers and trigger DMA to preload the first bit
		TIM_ARR(IOTIM) = dshotarr2; // Preload bit time
		__enable_irq();
		if (!rep || !--rep) dshotval = 0;
	}
	int tlm = x & 0x10;
	x >>= 5;
	if (!x || x > 47) {
		if (tlm) telreq = 1; // Telemetry request
		throt = x ? cfg.throt_mode ? (x > 1047 ? x - 1047 : 47 - x) << 1 : x - 47 : 0;
		cmd = 0;
		return;
	}
	if (!tlm || ertm) return; // Telemetry bit must be set, motor must be stopped
	if (cmd != x) {
		cmd = x;
		cnt = 0;
	}
	if (cnt < 10) ++cnt;
	switch (cmd) {
		case 1: // DSHOT_CMD_BEACON1
		case 2: // DSHOT_CMD_BEACON2
		case 3: // DSHOT_CMD_BEACON3
		case 4: // DSHOT_CMD_BEACON4
		case 5: // DSHOT_CMD_BEACON5
			beacon = cmd;
			break;
		case 7: // DSHOT_CMD_SPIN_DIRECTION_1
			if (cnt != 6) break;
			cfg.revdir = 0;
			break;
		case 8: // DSHOT_CMD_SPIN_DIRECTION_2
			if (cnt != 6) break;
			cfg.revdir = 1;
			break;
		case 9: // DSHOT_CMD_3D_MODE_OFF
			if (cnt != 6) break;
			cfg.throt_mode = 0;
			break;
		case 10: // DSHOT_CMD_3D_MODE_ON
			if (cnt != 6) break;
			cfg.throt_mode = 1;
			break;
		case 12: // DSHOT_CMD_SAVE_SETTINGS
			if (cnt != 6) break;
			beepval = savecfg();
			break;
		case 13: // DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE
			if (cnt != 6) break;
			dshotext = 1;
			dshotval = 0xe00;
			rep = 10;
			break;
		case 14: // DSHOT_CMD_EXTENDED_TELEMETRY_DISABLE
			if (cnt != 6) break;
			dshotext = 0;
			dshotval = 0xeff;
			rep = 10;
			break;
		case 20: // DSHOT_CMD_SPIN_DIRECTION_NORMAL
			if (cnt != 6) break;
			flipdir = 0;
			break;
		case 21: // DSHOT_CMD_SPIN_DIRECTION_REVERSED
			if (cnt != 6) break;
			flipdir = 1;
			break;
#if LED_CNT >= 1
		case 22: // DSHOT_CMD_LED0_ON
			cfg.led |= 1;
			break;
		case 26: // DSHOT_CMD_LED0_OFF
			cfg.led &= ~1;
			break;
#endif
#if LED_CNT >= 2
		case 23: // DSHOT_CMD_LED1_ON
			cfg.led |= 2;
			break;
		case 27: // DSHOT_CMD_LED1_OFF
			cfg.led &= ~2;
			break;
#endif
#if LED_CNT >= 3
		case 24: // DSHOT_CMD_LED2_ON
			cfg.led |= 4;
			break;
		case 28: // DSHOT_CMD_LED2_OFF
			cfg.led &= ~4;
			break;
#endif
#if LED_CNT >= 4
		case 25: // DSHOT_CMD_LED3_ON
			cfg.led |= 8;
			break;
		case 29: // DSHOT_CMD_LED3_OFF
			cfg.led &= ~8;
			break;
#endif
		case 40: // Select motor timing
			if (cnt != 6) break;
			if ((x = cfg.timing + 1) > 7) x = 1;
			beepval = cfg.timing = x;
			break;
		case 41: // Select PWM frequency
			if (cnt != 6) break;
			if ((x = (cfg.freq_min >> 2) + 1) > 12 || x < 6) x = 6;
			cfg.freq_min = x << 2;
			cfg.freq_max = x << 3;
			beepval = x - 5;
			break;
		case 42: // Select maximum duty cycle ramp
			if (cnt != 6) break;
			if ((x = cfg.duty_ramp / 10 + 1) > 10) x = 0;
			cfg.duty_ramp = x * 10;
			beepval = x;
			break;
		case 43: // Increase acceleration slew rate
			if (cnt != 6) break;
			if ((x = cfg.duty_rate) < 100 && ++x > 10) x = (x + 4) / 5 * 5;
			beepval = cfg.duty_rate = x;
			break;
		case 44: // Decrease acceleration slew rate
			if (cnt != 6) break;
			if ((x = cfg.duty_rate) > 1 && --x > 10) x = x / 5 * 5;
			beepval = cfg.duty_rate = x;
			break;
		case 47: // Reset settings
			if (cnt != 6) break;
			beepval = resetcfg();
			break;
	}
}

void iotim_isr(void) {
	ioirq();
}

void iodma_isr(void) {
#ifdef IO_PA2
	DMA1_IFCR = DMA_IFCR_CTCIF(IOTIM_DMA) | DMA_IFCR_CTCIF(USART2_RX_DMA);
#else
	DMA1_IFCR = DMA_IFCR_CTCIF(IOTIM_DMA);
#endif
	iodma();
}

#ifdef IO_PA2
void usart2_isr(void) {
	ioirq();
}

static void syncirq(void) {
#ifdef USARTv1
	USART2_SR, USART2_DR; // Clear flags
#else
	USART2_RQR = USART_RQR_RXFRQ; // Clear RXNE
	USART2_ICR = USART_ICR_IDLECF | USART_ICR_ORECF;
#endif
	USART2_CR1 &= ~USART_CR1_IDLEIE;
	USART2_CR3 = USART_CR3_HDSEL | USART_CR3_DMAR;
	DMA1_CNDTR(USART2_RX_DMA) = rxlen;
	DMA1_CCR(USART2_RX_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

static void resync(void) {
#ifdef USARTv1
	USART2_SR, USART2_DR; // Clear flags
#else
	USART2_ICR = USART_ICR_IDLECF;
#endif
	USART2_CR1 |= USART_CR1_IDLEIE;
	USART2_CR3 = USART_CR3_HDSEL;
	DMA1_CCR(USART2_RX_DMA) = 0;
}

static void throtdma(void) {
	int x = iobuf[0] | iobuf[1] << 8;
	if (dshotcrc(x, 0)) { // Invalid checksum
		resync();
		return;
	}
	IWDG_KR = IWDG_KR_RESET;
	x &= 0xfff;
	if (x & 0x800) x -= 0x1000; // Propagate sign
	throt = x;
}

static void ibusdma(void) {
	if (iobuf[0] != 0x20 || iobuf[1] != 0x40) { // Invalid frame
		resync();
		return;
	}
	int n = cfg.input_chid;
	int x = 1500;
	int u = 0xff9f;
	for (int i = 1;; ++i) {
		int j = i << 1;
		char a = iobuf[j];
		char b = iobuf[j + 1];
		int v = a | b << 8;
		if (i == 15) {
			if (u != v) { // Invalid checksum
				resync();
				return;
			}
			break;
		}
		u -= a + b;
		if (i == n) x = v & 0xfff;
	}
	IWDG_KR = IWDG_KR_RESET;
	servoval(x);
}

static void sbusdma(void) {
	if (iobuf[0] != 0x0f) { // Invalid frame
		resync();
		return;
	}
	IWDG_KR = IWDG_KR_RESET;
	int n = cfg.input_chid - 1;
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
}

static void cliirq(void) {
	static int i, j;
	int cr = USART2_CR1;
	if (cr & USART_CR1_TXEIE) {
		USART2_TDR = iobuf[i++]; // Clear TXE+TC
		if (i < j) return;
#ifdef USARTv1
		USART2_SR = ~USART_SR_TC;
#endif
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
		i = 0;
		j = 0;
		return;
	}
	if (cr & USART_CR1_TCIE) {
#ifdef USARTv1
		USART2_SR = ~USART_SR_TC;
#else
		USART2_ICR = USART_ICR_TCCF;
#endif
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
		return;
	}
	if (USART2_ISR & USART_ISR_FE || i == sizeof iobuf - 1) WWDG_CR = WWDG_CR_WDGA; // Data error
	char b = USART2_RDR; // Clear RXNE
	if (b == '\b' || b == 0x7f) { // Backspace
		if (i) --i;
		return;
	}
	iobuf[i++] = b;
	if (i == 2 && iobuf[0] == 0x00 && iobuf[1] == 0xff) scb_reset_system(); // Reboot into bootloader
	if (b != '\n') return;
	iobuf[i] = '\0';
	i = 0;
	if (!(j = execcmd(iobuf))) return;
	USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TXEIE;
}
#else
static void cliirq(void) {
	static int i, j, n, b;
	switch (TIM3_DIER) {
		case TIM_DIER_UIE: // Output
			TIM3_SR = ~TIM_SR_UIF;
			if (i < j) {
				int p = -1;
				if (!n++) b = iobuf[i]; // Start bit
				else if (n < 10) { // Data bit
					if (b & 1) p = 0;
					b >>= 1;
				} else { // Stop bit
					if (++i == j) { // End of data
						i = 0;
						j = 0;
					}
					n = 0;
					p = 0;
				}
				TIM3_CCR1 = p;
				break;
			}
			TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
			TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
			TIM3_SR = ~TIM_SR_CC2IF;
			TIM3_DIER = TIM_DIER_CC2IE;
			TIM3_CCR1 = CLK_CNT(76800); // Half-bit time
			TIM3_EGR = TIM_EGR_UG;
			break;
		case TIM_DIER_CC1IE: // Half-bit time
			TIM3_SR = ~TIM_SR_CC1IF;
			int p = IOTIM_IDR; // Signal level
			if (!n++) { // Start bit
				if (p) WWDG_CR = WWDG_CR_WDGA; // Data error
				b = 0;
				break;
			}
			if (n < 10) { // Data bit
				b >>= 1;
				if (p) b |= 0x80;
				break;
			}
			if (!p || i == sizeof iobuf - 1) WWDG_CR = WWDG_CR_WDGA; // Data error
			TIM3_SR = ~TIM_SR_CC2IF;
			TIM3_DIER = TIM_DIER_CC2IE;
			n = 0;
			if (b == '\b' || b == 0x7f) { // Backspace
				if (i) --i;
				break;
			}
			iobuf[i++] = b;
			if (i == 2 && iobuf[0] == 0x00 && iobuf[1] == 0xff) scb_reset_system(); // Reboot into bootloader
			if (b != '\n') break;
			iobuf[i] = '\0';
			i = 0;
			if (!(j = execcmd(iobuf))) break;
			TIM3_SMCR = 0;
			TIM3_CCR1 = 0; // Preload high level
			TIM3_EGR = TIM_EGR_UG; // Update registers and trigger UEV
			TIM3_CCER = TIM_CCER_CC1E; // Enable output
			TIM3_DIER = TIM_DIER_UIE;
			break;
		case TIM_DIER_CC2IE: // Falling edge
			TIM3_SR = ~(TIM_SR_CC1IF | TIM_SR_CC2IF);
			TIM3_DIER = TIM_DIER_CC1IE;
			break;
	}
}
#endif
