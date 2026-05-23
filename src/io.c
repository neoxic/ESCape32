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

#ifdef AT32F4
#define USART2_TDR USART2_DR
#define USART2_RDR USART2_DR
#define USART2_ISR USART2_SR
#define USART_CR1_M0 USART_CR1_M
#define USART_ISR_FE USART_SR_FE
#define USART_ISR_NF USART_SR_NE
#endif

static void entryirq(void);
static void calibirq(void);
static void servoirq(void);
static void dshotirq(void);
static void cliirq(void);
#ifdef IO_PA2
static void serialirq(void);
static int serialfunc(int len);
static int ibusfunc(int len);
static int sbusfunc(int len);
static int crsffunc(int len);
static int exbusfunc(int len);
static int hottfunc(int len);
static int (*iofunc)(int len);
#endif
static void (*ioirq)(void);
static char dshotinv, iobuf[1024] __attribute__((aligned(2)));
static uint16_t dshotarr1, dshotarr2, dshotbuf[23] = {-1, -1, 0, -1, 0, -1, -1, 0, -1, 0, -1, -1, 0, -1, 0, -1, 0, -1, -1, -1};

static void setthrot(int x) {
	if (x < 0) return;
	throt = cfg.throt_mode ?
		x < cfg.throt_mid - 50 ? scale(x, cfg.throt_min, cfg.throt_mid - 50, -2000, 0):
		x > cfg.throt_mid + 50 ? scale(x, cfg.throt_mid + 50, cfg.throt_max, 0, 2000): 0:
		x > cfg.throt_min + 50 ? scale(x, cfg.throt_min + 50, cfg.throt_max, 0, 2000): 0;
}

#if defined IO_PA2 || defined IO_AUX
static void setbrake(int x) {
	if (x < 0) return;
	brake = scale(x, 1100, 1900, 0, cfg.duty_drag);
	auxup = 0;
}
#endif

#ifdef IO_AUX
void iotim2_isr(void) {
#if IOTIM2 == TIM16 || IOTIM2 == TIM17
	static uint16_t t1;
	uint16_t t2 = TIM_CCR1(IOTIM2);
	uint16_t er = TIM_CCER(IOTIM2);
	TIM_CCER(IOTIM2) = er ^ TIM_CCER_CC1P;
	if (!(er & TIM_CCER_CC1P)) { // Rising edge
		t1 = t2;
		return;
	}
	int x = t2 - t1;
#else
	int x = TIM_CCR2(IOTIM2);
#endif
	if (x < 800 || x > 2200) return; // Invalid signal
	setbrake(x);
}
#endif

void initio(void) {
	ioirq = entryirq;
	TIM_BDTR(IOTIM) = TIM_BDTR_MOE;
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1FP1; // Reset on rising edge on TI1
	TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_UIE | TIM_DIER_CC1IE;
	TIM_PSC(IOTIM) = CLK_MHZ - 1; // 1us resolution
	TIM_ARR(IOTIM) = -1;
	TIM_CR1(IOTIM) = TIM_CR1_URS;
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
#ifdef IO_AUX
#if IOTIM2 == TIM16 || IOTIM2 == TIM17
	TIM_CCMR1(IOTIM2) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8;
	TIM_CCER(IOTIM2) = TIM_CCER_CC1E; // IC1 on rising edge on TI1
	TIM_DIER(IOTIM2) = TIM_DIER_CC1IE;
#else
	TIM_CCMR1(IOTIM2) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8;
	TIM_SMCR(IOTIM2) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1FP1; // Reset on rising edge on TI1
	TIM_CCER(IOTIM2) = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
	TIM_DIER(IOTIM2) = TIM_DIER_CC2IE;
#endif
	TIM_PSC(IOTIM2) = CLK_MHZ - 1; // 1us resolution
	TIM_ARR(IOTIM2) = -1;
	TIM_EGR(IOTIM2) = TIM_EGR_UG;
	TIM_CR1(IOTIM2) = TIM_CR1_CEN;
#endif
}

static void entryirq(void) {
	static char n, c, d;
	if (TIM_SR(IOTIM) & TIM_SR_UIF) { // Timeout ~66ms
		TIM_SR(IOTIM) = ~TIM_SR_UIF;
		if (!IOTIM_IDR) { // Low level
			if (IO_ANALOG) goto analog;
			n = 0;
			return;
		}
		if (++c < 16) return; // Wait for ~1s before entering CLI
		ioirq = cliirq;
#ifdef IO_PA2
		io_serial();
#ifdef IO_RXTX
		GPIOA_PUPDR |= 0x80000000; // A15 (pull-down)
		GPIOA_MODER &= ~0x40000000; // A15 (USART2_RX)
		TIM15_ARR = CLK_CNT(20000) - 1;
		TIM15_EGR = TIM_EGR_UG;
		TIM15_SR = ~TIM_SR_UIF;
		TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
		while (TIM15_CR1 & TIM_CR1_CEN) { // Wait for 50us high level on A15
			if (!(GPIOA_IDR & 0x8000)) { // A15 low
				USART2_CR3 = USART_CR3_HDSEL;
				break;
			}
		}
#else
		USART2_CR3 = USART_CR3_HDSEL;
#endif
		USART2_BRR = CLK_CNT(38400);
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
#else
		TIM3_CCER = 0;
		TIM3_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_CK_INT_N_8;
		TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
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
	int t = TIM_CCR1(IOTIM); // Time between two rising edges
	if (IO_ANALOG) {
	analog:
#ifndef ANALOG_CHAN
		io_analog();
		analog = 1;
#endif
		return;
	}
	if (!n++) return; // First capture is always invalid
	IWDG_KR = IWDG_KR_START;
#ifdef IO_PA2
	if (cfg.input_mode >= 2) {
		ioirq = serialirq;
		io_serial();
		switch (cfg.input_mode) {
			case 2: // Serial
				iofunc = serialfunc;
				USART2_BRR = CLK_CNT(SERIAL_BR);
				break;
			case 3: // iBUS
				iofunc = ibusfunc;
				USART2_BRR = CLK_CNT(115200);
				break;
			case 4: // SBUS/SBUS2
				iofunc = sbusfunc;
				USART2_BRR = CLK_CNT(100000);
				USART2_CR1 = USART_CR1_PCE | USART_CR1_M0;
				USART2_CR2 = USART_CR2_STOPBITS_2;
#ifndef AT32F4
				USART2_CR2 |= USART_CR2_RXINV | USART_CR2_TXINV;
				GPIOA_PUPDR = (GPIOA_PUPDR & ~0x30) | 0x20; // A2 (pull-down)
#endif
				TIM15_PSC = CLK_MHZ / 8 - 1; // 125ns resolution
				TIM15_ARR = -1;
				TIM15_EGR = TIM_EGR_UG;
				TIM15_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
				break;
			case 5: // CRSF
				iofunc = crsffunc;
				USART2_BRR = CLK_CNT(416666);
				break;
			case 6: // EXBUS
				iofunc = exbusfunc;
				USART2_BRR = CLK_CNT(125000);
				break;
			case 7: // HoTT
				iofunc = hottfunc;
				USART2_BRR = CLK_CNT(115200);
				break;
		}
		USART2_CR3 = USART_CR3_HDSEL | USART_CR3_DMAT | USART_CR3_DMAR;
		USART2_CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
		DMA1_CPAR(USART2_RX_DMA) = (uint32_t)&USART2_RDR;
		DMA1_CMAR(USART2_RX_DMA) = (uint32_t)iobuf;
		DMA1_CPAR(USART2_TX_DMA) = (uint32_t)&USART2_TDR;
		DMA1_CMAR(USART2_TX_DMA) = (uint32_t)iobuf;
		DMA1_CNDTR(USART2_RX_DMA) = sizeof iobuf;
		return;
	}
#endif
	if (TIM_PSC(IOTIM)) {
		if (t > 2000) { // Servo/Oneshot125
			ioirq = calibirq;
			TIM_DIER(IOTIM) = TIM_DIER_CC1IE;
			TIM_CR1(IOTIM) = TIM_CR1_CEN;
			calibirq();
			return;
		}
		TIM_PSC(IOTIM) = TIM_PSC(IOTIM) == CLK_MHZ - 1 ? CLK_MHZ / 8 - 1 : 0;
		TIM_EGR(IOTIM) = TIM_EGR_UG;
		n = 0;
		return;
	}
	int m = 2;
	while (t >= CLK_CNT(800000)) t >>= 1, --m;
	if (d != m) {
		d = m;
		n = 1;
		return;
	}
	if (m < 0 || n < 4) return;
	ioirq = dshotirq;
	dshotarr1 = CLK_CNT(150000 << m) - 1;
	dshotarr2 = CLK_CNT(375000 << m) - 1;
	TIM_CCER(IOTIM) = 0;
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_UIE;
	TIM_ARR(IOTIM) = dshotarr1; // Frame reset timeout
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	DMA1_CPAR(IOTIM_DMA) = (uint32_t)&TIM_CCR1(IOTIM);
	DMA1_CMAR(IOTIM_DMA) = (uint32_t)iobuf;
}

static void calibirq(void) {
	static uint8_t n;
	static int16_t q, x, y;
	int p = TIM_CCR1(IOTIM); // Pulse period
	if (cfg.throt_cal && // 50/100/125/200/250/333/500Hz 11/22ms servo PWM within 8% margin
		((p < 22880 && p > 21120) || (p < 20800 && p > 19200) || (p < 11440 && p > 10560) || (p < 10400 && p > 9600) || (p < 8320 && p > 7680) ||
		(p < 5200 && p > 4800) || (p < 4160 && p > 3840) || (p < 3120 && p > 2880) || (p < 2080 && p > 1920))) {
		IWDG_KR = IWDG_KR_RESET;
		q += p - ((p + 500) / 1000) * 1000; // Cumulative error
		if (++n & 3) return;
		if (q > x) { // Slow down
			y = -q;
			q = 0;
			hsictl(-1);
			return;
		}
		if (q < y) { // Speed up
			x = -q;
			q = 0;
			hsictl(1);
			return;
		}
	}
	ioirq = servoirq;
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8;
	TIM_CCER(IOTIM) = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
	TIM_SR(IOTIM) = ~TIM_SR_CC2IF;
	TIM_DIER(IOTIM) = TIM_DIER_CC2IE;
}

static void servoirq(void) {
	int x = TIM_CCR2(IOTIM);
	if (x >= 28 && x <= 32) { // Telemetry request
		IWDG_KR = IWDG_KR_RESET;
		telreq = 1;
		return;
	}
	if (x < 800 || x > 2200) return; // Invalid signal
	IWDG_KR = IWDG_KR_RESET;
	setthrot(x);
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

static void dshotreset(void) {
#ifdef AT32F4 // Errata 1.5.1
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
	DMA1_CMAR(IOTIM_DMA) = (uint32_t)iobuf;
	DMA1_CNDTR(IOTIM_DMA) = 32;
	DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
	TIM_ARR(IOTIM) = -1;
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_CCMR1(IOTIM) = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_CK_INT_N_8;
	TIM_SMCR(IOTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM_CCER(IOTIM) = TIM_CCER_CC1E; // IC1 on any edge on TI1
	TIM_DIER(IOTIM) = TIM_DIER_CC1DE;
}

static void dshotresync(void) {
	if (dshotinv) dshotreset();
	DMA1_CCR(IOTIM_DMA) = 0;
	TIM_CR1(IOTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	TIM_ARR(IOTIM) = dshotarr1; // Frame reset timeout
	TIM_EGR(IOTIM) = TIM_EGR_UG;
	TIM_SR(IOTIM) = ~TIM_SR_UIF;
	TIM_DIER(IOTIM) = TIM_DIER_UIE;
}

static int dshotcrc(int x, int inv) {
	int a = x;
	for (int b = x; b >>= 4; a ^= b);
	if (inv) a = ~a;
	return a & 0xf;
}

void iotim_dma_isr(void) { // DSHOT
	static const char gcr[] = {0x19, 0x1b, 0x12, 0x13, 0x1d, 0x15, 0x16, 0x17, 0x1a, 0x09, 0x0a, 0x0b, 0x1e, 0x0d, 0x0e, 0x0f};
	static char cmd, cnt, rep;
	DMA1_IFCR = DMA_IFCR_CTCIF(IOTIM_DMA);
	if (DMA1_CCR(IOTIM_DMA) & DMA_CCR_DIR) {
		dshotreset();
		if (!dshotval) {
			int a = ertm ? min(ertm, 65408) : 65408;
			int b = 0;
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
		if (!rep || !--rep) dshotval = 0;
		return;
	}
	if (dshotinv) { // Bidirectional DSHOT
		TIM_CCER(IOTIM) = 0;
		TIM_SMCR(IOTIM) = 0;
		TIM_CCMR1(IOTIM) = 0; // Disable OC before enabling PWM to force OC1REF update (RM: OC1M, note #2)
		TIM_CCMR1(IOTIM) = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2;
		TIM_CR2(IOTIM) = TIM_CR2_CCDS; // CC1 DMA request on UEV using the same DMA channel
		DMA1_CCR(IOTIM_DMA) = 0;
		DMA1_CMAR(IOTIM_DMA) = (uint32_t)dshotbuf;
		DMA1_CNDTR(IOTIM_DMA) = 23;
		DMA1_CCR(IOTIM_DMA) = DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_16BIT | DMA_CCR_MSIZE_16BIT;
		TIM_CCR1(IOTIM) = 0; // Preload high level
		__disable_irq();
		TIM_ARR(IOTIM) = max(CLK_CNT(33333) - TIM_CNT(IOTIM) - 1, 19); // 30us output delay
		TIM_EGR(IOTIM) = TIM_EGR_UG; // Update registers and trigger DMA to preload the first bit
		TIM_EGR(IOTIM); // Ensure UEV has happened
		TIM_ARR(IOTIM) = dshotarr2; // Preload bit time
		TIM_CCER(IOTIM) = TIM_CCER_CC1E; // Enable output
		__enable_irq();
	}
	int x = 0;
	int y = dshotarr1 + 1; // Two bit time
	int z = y >> 2; // Half-bit time
	uint16_t *buf = (uint16_t *)iobuf;
	for (int i = 0; i < 32; i += 2) {
		if (i && buf[i] >= y) { // Invalid pulse timing
			dshotresync();
			return;
		}
		x <<= 1;
		if (buf[i + 1] >= z) x |= 1;
	}
	if (dshotcrc(x, dshotinv)) { // Invalid checksum
		dshotresync();
		return;
	}
	IWDG_KR = IWDG_KR_RESET;
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
			if ((x = (cfg.timing >> 1) + 1) > 15 || x < 8) x = 8;
			cfg.timing = x << 1;
			beepval = x - 7;
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
		case 43: // Select duty cycle slew rate
			if (cnt != 6) break;
			if ((x = cfg.duty_rate / 10 + 1) > 10) x = 1;
			cfg.duty_rate = x * 10;
			beepval = x;
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

#ifdef IO_PA2
void usart2_isr(void) {
	ioirq();
}

static int getchan1(const char *buf, int n) { // SBUS/CRSF
	if (n < 0 || n >= 16) return -1;
	int i = (n * 11) >> 3;
	int a = (n * 3) & 7;
	int b = ((n + 1) * 3) & 7;
	int x = a < b ?
		buf[i] >> a | (buf[i + 1] & ((1 << b) - 1)) << (8 - a):
		buf[i] >> a | buf[i + 1] << (8 - a) | (buf[i + 2] & ((1 << b) - 1)) << (16 - a);
	return (x * 5 >> 3) + 880;
}

static int getchan2(const char *buf, int m, int n, int s) { // EXBUS/HoTT
	if (s > 0 && n >= 8 && (n -= s) < 8) return -1;
	if (n < 0 || n >= m) return -1;
	buf += n << 1;
	int a = buf[0];
	int b = buf[1];
	return (s < 0 ? a | b << 8 : a << 8 | b) >> 3;
}

static void serialirq(void) {
	if (USART2_CR1 & USART_CR1_TCIE) {
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
		goto reading;
	}
#ifdef AT32F4
	USART2_SR, USART2_DR; // Clear flags
#else
	USART2_RQR = USART_RQR_RXFRQ; // Clear RXNE
	USART2_ICR = USART_ICR_IDLECF | USART_ICR_ORECF;
#endif
	int len = iofunc(sizeof iobuf - DMA1_CNDTR(USART2_RX_DMA));
	if (len) {
		if (len < 0) return;
#ifdef AT32F4
		USART2_SR = ~USART_SR_TC;
#else
		USART2_ICR = USART_ICR_TCCF;
#endif
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
		DMA1_CCR(USART2_TX_DMA) = 0;
		DMA1_CNDTR(USART2_TX_DMA) = len;
		DMA1_CCR(USART2_TX_DMA) = DMA_CCR_EN | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
		return;
	}
reading:
	DMA1_CCR(USART2_RX_DMA) = 0;
	DMA1_CNDTR(USART2_RX_DMA) = sizeof iobuf;
	DMA1_CCR(USART2_RX_DMA) = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
}

static int serialresp1(int a, int b) {
	iobuf[0] = 4;
	iobuf[1] = a;
	iobuf[2] = b;
	iobuf[3] = crc8(iobuf, 3);
	return 4;
}

static int serialresp2(int x) {
	return serialresp1(x, x >> 8);
}

static int serialfunc(int len) {
	if (len < 2 || iobuf[0] != len || crc8(iobuf, len)) return 0;
	IWDG_KR = IWDG_KR_RESET;
	len -= 2;
	for (char *buf = iobuf + 1; len--;) {
		int val = *buf++;
		char *pos = buf;
		switch (val) {
			case 0x00: // Throttle
				if (len < 2) return 0;
				val = buf[0] | buf[1] << 8;
				buf += 2;
				if (val & 0x8000) val -= 0x10000; // Propagate sign
				if (val < -2000 || val > 2000) break;
				throt = val;
				break;
			case 0x01: // Drag brake adjustment
				if (len < 1) return 0;
				brake = min(*buf++, 100) * cfg.duty_drag * 41 >> 12;
				auxup = 0;
				break;
			case 0x02: // Normal motor direction
				flipdir = 0;
				break;
			case 0x03: // Reversed motor direction
				flipdir = 1;
				break;
			case 0x04: // LED on/off bits
				if (len < 1) return 0;
				cfg.led = *buf++ & ((1 << LED_CNT) - 1);
				break;
			case 0x80: // Combined telemetry
				iobuf[0] = 12;
				iobuf[1] = temp1;
				iobuf[2] = temp2;
				iobuf[3] = volt;
				iobuf[4] = volt >> 8;
				iobuf[5] = curr;
				iobuf[6] = curr >> 8;
				iobuf[7] = csum;
				iobuf[8] = csum >> 8;
				iobuf[9] = val = min(ertm, 0xffff);
				iobuf[10] = val >> 8;
				iobuf[11] = crc8(iobuf, 11);
				return 12;
			case 0x81: return serialresp2(min(ertm, 0xffff)); // Electrical revolution time (us)
			case 0x82: return serialresp1(temp1, temp2); // Temperature (C)
			case 0x83: return serialresp2(volt); // Voltage (V/100)
			case 0x84: return serialresp2(curr); // Current (A/100)
			case 0x85: return serialresp2(csum); // Consumption (mAh)
			default:
				return 0;
		}
		len -= buf - pos;
	}
	return 0;
}

static int ibusfunc(int len) {
	if (len != 32 || iobuf[0] != 0x20 || iobuf[1] != 0x40) return 0;
	int n1 = cfg.input_ch1;
	int n2 = cfg.input_ch2;
	int x1 = -1;
	int x2 = -1;
	int u = 0xff9f;
	for (int i = 1;; ++i) {
		int j = i << 1;
		int a = iobuf[j];
		int b = iobuf[j + 1];
		int v = a | b << 8;
		if (i == 15) {
			if (u != v) return 0; // Invalid checksum
			break;
		}
		u -= a + b;
		if (i == n1) x1 = v & 0xfff;
		else if (i == n2) x2 = v & 0xfff;
	}
	IWDG_KR = IWDG_KR_RESET;
	setthrot(x1);
	setbrake(x2);
	return 0;
}

static void sbusirq1(void) {
	TIM15_SR = ~TIM_SR_UIF;
	TIM15_DIER = 0;
	TIM15_ARR = -1;
	TIM15_EGR = TIM_EGR_UG;
	USART2_CR1 = USART_CR1_PCE | USART_CR1_M0 | USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE;
	DMA1_CCR(USART2_RX_DMA) = 0;
	DMA1_CNDTR(USART2_RX_DMA) = sizeof iobuf;
	DMA1_CCR(USART2_RX_DMA) = DMA_CCR_EN | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
	ioirq = serialirq;
}

static void sbusirq2(void) {
	static const char slot[] = {
		0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, // 2..7
		0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3, // 10..15
		0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb, // 18..23
		0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb, // 26..31
	};
	static char n;
	int a = 0, b = 0;
	TIM15_SR = ~TIM_SR_UIF;
	switch (n) {
		case 0: // SBS-01T (ESC temperature)
			a = temp1 + 100;
			b = a >> 8 | 0x80;
			break;
		case 1: // SBS-01T (motor temperature)
			a = temp2 + 100;
			b = a >> 8 | 0x80;
			break;
		case 2: // SBS-01C (current)
			b = curr;
			a = b >> 8 | 0x40;
			break;
		case 3: // SBS-01C (voltage)
			b = volt;
			a = b >> 8;
			break;
		case 4: // SBS-01C (consumption)
			b = csum;
			a = b >> 8;
			break;
		case 5: // SBS-01R (RPM)
			a = min(erpm / (cfg.telem_poles * 3), 0xffff);
			b = a >> 8;
			break;
	}
	iobuf[0] = slot[n + (cfg.telem_phid - 1) * 6];
	iobuf[1] = a;
	iobuf[2] = b;
	DMA1_CCR(USART2_TX_DMA) = 0;
	DMA1_CNDTR(USART2_TX_DMA) = 3;
	DMA1_CCR(USART2_TX_DMA) = DMA_CCR_EN | DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_8BIT | DMA_CCR_MSIZE_8BIT;
	if (++n < 6) return;
	ioirq = sbusirq1;
	n = 0;
}

static int sbusfunc(int len) {
	if (len != 25 || iobuf[0] != 0x0f) return 0;
	TIM15_EGR = TIM_EGR_UG;
	IWDG_KR = IWDG_KR_RESET;
	setthrot(getchan1(iobuf + 1, cfg.input_ch1 - 1));
	setbrake(getchan1(iobuf + 1, cfg.input_ch2 - 1));
	int a = iobuf[24];
	if ((a & 0xf) != 0x4) return 0; // No telemetry
	if (telmode == 2 || telmode == 3 || telmode == 5 || a >> 4 != cfg.telem_phid - 1) {
		ioirq = sbusirq1;
		__disable_irq();
		TIM15_ARR = 57279 - TIM15_CNT; // Disable RX for 2000+660*8-120=7160us
		TIM15_EGR = TIM_EGR_UG;
		__enable_irq();
	} else {
		ioirq = sbusirq2;
		__disable_irq();
		TIM15_ARR = 25599 - TIM15_CNT; // Delay TX for 2000+660*2-120=3200us
		TIM15_EGR = TIM_EGR_UG;
		TIM15_EGR; // Ensure UEV has happened
		TIM15_ARR = 5279; // Preload slot interval
		__enable_irq();
	}
	TIM15_SR = ~TIM_SR_UIF;
	TIM15_DIER = TIM_DIER_UIE;
	USART2_CR1 = USART_CR1_PCE | USART_CR1_M0 | USART_CR1_UE | USART_CR1_TE;
	return -1;
}

static int crsffunc(int len) {
	if (len != 26 || iobuf[1] != 0x18 || iobuf[2] != 0x16 || crc8dvbs2(iobuf + 2, 24)) return 0;
	IWDG_KR = IWDG_KR_RESET;
	setthrot(getchan1(iobuf + 3, cfg.input_ch1 - 1));
	setbrake(getchan1(iobuf + 3, cfg.input_ch2 - 1));
	return 0;
}

static int exbusresp(int x) {
	static uint8_t n;
	int a = DESIG_UNIQUE_ID0;
	iobuf[0] = 0x3b;
	iobuf[1] = 0x01;
	iobuf[3] = x;
	iobuf[4] = 0x3a;
	iobuf[6] = 0x0f;
	iobuf[8] = a & 0x1f;
	iobuf[9] = 0xa4;
	iobuf[10] = a >> 8;
	iobuf[11] = a >> 16;
	iobuf[12] = 0x00;
	if (n < 7) { // Send description
		static const char *const str1[] = {"ESCape32", "Temp1", "Temp2", "Volt", "Curr", "Csum", "RPM"};
		static const char *const str2[] = {"", "C", "C", "V", "A", "mAh", ""};
		char *p = iobuf + 15;
		char *q = stpcpy(p, str1[n]);
		int b = q - p; // len1
		p = stpcpy(q, str2[n]);
		int c = p - q; // len2
		a = b + c;
		iobuf[2] = a + 18;
		iobuf[5] = a + 10;
		iobuf[7] = a + 8;
		iobuf[13] = n++;
		iobuf[14] = b << 3 | c;
		p[0] = crc8(iobuf + 7, a + 8);
		p[1] = b = crc16ccitt(iobuf, a + 16);
		p[2] = b >> 8;
		return a + 18;
	}
	iobuf[2] = 0x26;
	iobuf[5] = 0x1e;
	iobuf[7] = 0x5c;
	iobuf[13] = 0x11;
	iobuf[14] = temp1;
	iobuf[15] = temp1 >> 8;
	iobuf[16] = 0x21;
	iobuf[17] = temp2;
	iobuf[18] = temp2 >> 8;
	iobuf[19] = 0x34;
	iobuf[20] = volt;
	iobuf[21] = volt >> 8;
	iobuf[22] = volt >> 16 | 0x40;
	iobuf[23] = 0x44;
	iobuf[24] = curr;
	iobuf[25] = curr >> 8;
	iobuf[26] = curr >> 16 | 0x40;
	iobuf[27] = 0x54;
	iobuf[28] = csum;
	iobuf[29] = csum >> 8;
	iobuf[30] = csum >> 16;
	iobuf[31] = 0x64;
	iobuf[32] = a = erpm / (cfg.telem_poles >> 1);
	iobuf[33] = a >> 8;
	iobuf[34] = a >> 16;
	iobuf[35] = crc8(iobuf + 7, 28);
	iobuf[36] = a = crc16ccitt(iobuf, 36);
	iobuf[37] = a >> 8;
	return 38;
}

static int exbusfunc(int len) {
	static char n = 10;
	for (char *buf = iobuf; len >= 8;) {
		int pos = buf[2];
		int pfx = buf[5];
		if ((len -= pos) < 0 || pos != pfx + 8 || crc16ccitt(buf, pos)) break;
		IWDG_KR = IWDG_KR_RESET;
		switch (buf[4]) {
			case 0x31: // Channel data
				setthrot(getchan2(buf + 6, pfx >> 1, cfg.input_ch1 - 1, -1));
				setbrake(getchan2(buf + 6, pfx >> 1, cfg.input_ch2 - 1, -1));
				n = 0;
				break;
			case 0x3a: // Telemetry request
				if (len || pfx) break;
				return exbusresp(buf[3]);
		}
		if (!len) return 0;
		buf += pos;
	}
	if (!n) return 0;
	USART2_BRR = --n & 1 ? CLK_CNT(250000) : CLK_CNT(125000);
	return 0;
}

static int hottfunc(int len) {
	if (len < 5 || iobuf[0] != 0xa8 || crc16xmodem(iobuf, len - 2) != (iobuf[len - 2] << 8 | iobuf[len - 1])) return 0;
	IWDG_KR = IWDG_KR_RESET;
	int num = iobuf[2];
	int pfx = num << 1;
	if (pfx != len - 5) return 0;
	switch (iobuf[1] & 0x7f) {
		case 1: // SUMDv1
			setthrot(getchan2(iobuf + 3, num, cfg.input_ch1 - 1, 0));
			setbrake(getchan2(iobuf + 3, num, cfg.input_ch2 - 1, 0));
			break;
		case 3: { // SUMDv3
			static const char map[] = {0x60, 0x61, 0x80, 0x82, 0x84, 0x60}; // Function code mapping
			int code = iobuf[pfx - 1];
			if (code > 5) break;
			int a = map[code];
			int b = a >> 3;
			int c = (a & 7) << 2;
			if ((num -= 2) > b) num = b;
			setthrot(getchan2(iobuf + 3, num, cfg.input_ch1 - 1, c));
			setbrake(getchan2(iobuf + 3, num, cfg.input_ch2 - 1, c));
			break;
		}
	}
	return 0;
}

static void cliirq(void) {
	static uint16_t i, j;
	int cr = USART2_CR1;
	if (cr & USART_CR1_TXEIE) {
		USART2_TDR = iobuf[i++]; // Clear TXE+TC
		if (i < j) return;
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_TCIE;
		i = 0;
		j = 0;
		return;
	}
	if (cr & USART_CR1_TCIE) {
		USART2_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
		return;
	}
	if (USART2_ISR & (USART_ISR_FE | USART_ISR_NF) || i == sizeof iobuf - 1) WWDG_CR = WWDG_CR_WDGA; // Data error
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
	static uint16_t i, j;
	static uint8_t n, b;
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
