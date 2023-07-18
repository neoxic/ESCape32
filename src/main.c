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

#define REVISION 5

const Cfg cfgdata = {
	.id = 0x32ea,
	.revision = REVISION,
	.target_type = IO_TYPE | SENS_CNT << 2,
	.target_name = TARGET_NAME,
	.arm = ARM,                 // Wait for 250ms zero throttle on startup
	.damp = DAMP,               // Complementary PWM (active freewheeling)
	.revdir = REVDIR,           // Reversed motor direction
	.timing = TIMING,           // Motor timing (3.75*X degrees) [1..7]
	.sine_range = SINE_RANGE,   // Sine startup range (%) [0 - off, 5..25]
	.sine_power = SINE_POWER,   // Sine startup power (%) [1..15]
	.freq_min = FREQ_MIN,       // Minimum PWM frequency (kHz) [16..48]
	.freq_max = FREQ_MAX,       // Maximum PWM frequency (kHz) [16..96]
	.duty_min = DUTY_MIN,       // Minimum duty cycle (%) [1..100]
	.duty_max = DUTY_MAX,       // Maximum duty cycle (%) [1..100]
	.duty_spup = DUTY_SPUP,     // Maximum power during spin-up (%) [1..25]
	.duty_ramp = DUTY_RAMP,     // Acceleration ramping (0.1*X %/ms) [1..100]
	.duty_drag = DUTY_DRAG,     // Drag brake amount (%) [0..100]
	.throt_mode = THROT_MODE,   // Throttle mode (0 - forward, 1 - forward/reverse, 2 - forward/brake/reverse)
	.throt_cal = THROT_CAL,     // Throttle calibration
	.throt_min = THROT_MIN,     // Minimum throttle (us)
	.throt_mid = THROT_MID,     // Middle throttle (us)
	.throt_max = THROT_MAX,     // Maximum throttle (us)
	.input_mode = INPUT_MODE,   // Input mode (0 - servo/Oneshot125/DSHOT, 1 - analog, 2 - serial, 3 - iBUS, 4 - SBUS)
	.input_chid = INPUT_CHID,   // iBUS/SBUS channel ID [0 - off, 1..14 - iBUS, 1..16 - SBUS]
	.telem_mode = TELEM_MODE,   // Telemetry mode (0 - KISS, 1 - KISS auto, 2 - iBUS, 3 - S.Port)
	.telem_phid = TELEM_PHID,   // S.Port physical ID [0 - off, 1..28]
	.telem_poles = TELEM_POLES, // Number of motor poles for RPM telemetry [2..100]
	.prot_temp = PROT_TEMP,     // Temperature threshold (C) [0 - off, 60..140]
	.prot_volt = PROT_VOLT,     // Low voltage cutoff per battery cell (V/10) [0 - off, 28..38]
	.prot_cells = PROT_CELLS,   // Number of battery cells [0 - auto, 1..12]
	.prot_curr = PROT_CURR,     // Maximum current (A) [0..255]
	.music = MUSIC,             // Startup music
	.volume = VOLUME,           // Sound volume (%) [0..100]
	.beacon = BEACON,           // Beacon volume (%) [0..100]
	.led = LED,                 // LED bits
};

__attribute__((__section__(".cfg")))
Cfg cfg = cfgdata;

int throt, ertm, erpm, temp, volt, curr, csum, dshotval, beepval = -1;
char analog, telreq, flipdir, beacon, dshotext;
volatile uint32_t tickms;

static int step, sine, sync, ival;
static char prep, accl, tick, reverse, ready;

#ifdef ANALOG
#define reset() { \
	__disable_irq(); \
	TIM1_CCMR1 = TIM_CCMR1_OC1M_FORCE_LOW | TIM_CCMR1_OC2M_FORCE_LOW; \
	TIM1_CCMR2 = TIM_CCMR2_OC3M_FORCE_LOW; \
	TIM1_EGR = TIM_EGR_COMG; \
	for (;;) WWDG_CR = 0xff; \
}
#endif

/*
6-step commutation sequence:
 #  +|-  mask  code
 1  C|B   110   101
 2  A|B   011   001
 3  A|C   101   011
 4  B|C   110   010
 5  B|A   011   110
 6  C|A   101   100
*/

static void nextstep(void) {
#ifndef SENSORED
	if (sine) { // Sine startup
		IFTIM_OCR = sine;
		TIM_ARR(IFTIM) = sine;
		TIM_EGR(IFTIM) = TIM_EGR_UG;
		if (!prep && step) step = step * 60 - 59; // Switch over from 6-step
		if (reverse) {
			if (--step < 1) step = 360;
		} else {
			if (++step > 360) step = 1;
		}
		int a = step - 1;
		int b = a < 120 ? a + 240 : a - 120;
		int c = a < 240 ? a + 120 : a - 240;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_ARR = CLK_KHZ / 24 - 1;
		TIM1_CCR1 = DEAD_TIME + (sinedata[a] * cfg.sine_power >> 4);
		TIM1_CCR2 = DEAD_TIME + (sinedata[b] * cfg.sine_power >> 4);
		TIM1_CCR3 = DEAD_TIME + (sinedata[c] * cfg.sine_power >> 4);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
		if (prep) return;
		TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
		TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_PWM1;
#ifdef INVERTED_HIGH
		TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#else
		TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
#endif
		TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
		TIM_DIER(IFTIM) = 0;
		compctl(0);
		prep = 1;
		return;
	}
#else
	static const char map[] = {3, 5, 4, 1, 2, 6}; // map[code] -> step
	int code = IFTIM_IDR;
	if (code < 1 || code > 6) reset(); // Invalid Hall sensor code
	step = map[code - 1];
#endif
	static const char seq[] = {0x35, 0x19, 0x2b, 0x32, 0x1e, 0x2c}; // Commutation sequence
	static int buf[6];
	if (reverse) {
		if (--step < 1) step = 6;
	} else {
		if (++step > 6) step = 1;
	}
	int x = seq[step - 1];
	int m = x >> 3; // Energized phase mask
	int p = x & m; // Positive phase
	int n = ~x & m; // Negative phase
	int z = (step & 1) ^ reverse; // BEMF rising
	int m1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	int m2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM1;
#ifdef INVERTED_HIGH
	int er = TIM_CCER_CC4E | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#else
	int er = TIM_CCER_CC4E;
#endif
#ifndef SENSORED
	int cc = z ? 4 : 0;
#endif
	if (p & 1) {
		m1 |= TIM_CCMR1_OC1M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC1NE | TIM_CCER_CC1E : TIM_CCER_CC1E;
	} else if (n & 1) {
		m1 |= TIM_CCMR1_OC1M_FORCE_HIGH;
		er |= TIM_CCER_CC1NE;
	} else {
		if (!z) {
			m1 |= TIM_CCMR1_OC1M_FORCE_LOW;
			er |= TIM_CCER_CC1NE;
		}
#ifndef SENSORED
		cc |= 1;
#endif
	}
	if (p & 2) {
		m1 |= TIM_CCMR1_OC2M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC2NE | TIM_CCER_CC2E : TIM_CCER_CC2E;
	} else if (n & 2) {
		m1 |= TIM_CCMR1_OC2M_FORCE_HIGH;
		er |= TIM_CCER_CC2NE;
	} else {
		if (!z) {
			m1 |= TIM_CCMR1_OC2M_FORCE_LOW;
			er |= TIM_CCER_CC2NE;
		}
#ifndef SENSORED
		cc |= 2;
#endif
	}
	if (p & 4) {
		m2 |= TIM_CCMR2_OC3M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC3NE | TIM_CCER_CC3E : TIM_CCER_CC3E;
	} else if (n & 4) {
		m2 |= TIM_CCMR2_OC3M_FORCE_HIGH;
		er |= TIM_CCER_CC3NE;
	} else {
		if (!z) {
			m2 |= TIM_CCMR2_OC3M_FORCE_LOW;
			er |= TIM_CCER_CC3NE;
		}
#ifndef SENSORED
		cc |= 3;
#endif
	}
	TIM1_CCMR1 = m1;
	TIM1_CCMR2 = m2;
	TIM1_CCER = er;
	buf[step - 1] = ival;
	if (sync == 6) ertm = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5]) >> 1; // Electrical revolution time (us)
#ifndef SENSORED
	static int pcc, a, b;
	compctl(pcc);
	pcc = cc;
	if (ival > 1000) {
		a = 1000;
		b = 0;
	} else if (++b == 6) {
		if (a - ival > ival >> 1) { // Desync
			TIM_DIER(IFTIM) = TIM_DIER_UIE;
			return;
		}
		a = ival;
		b = 0;
	}
	if (ertm < 1000) { // 60K+ ERPM
		TIM1_CCR4 = 0;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	} else if (ertm < 2000) { // 30K+ ERPM
		TIM1_CCR4 = 0;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_2;
	} else { // Minimum PWM frequency
		TIM1_CCR4 = IFTIM_ICF << 2;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_4;
	}
	TIM_SR(IFTIM) = 0; // Clear BEMF events before enabling interrupts
	TIM_DIER(IFTIM) = TIM_DIER_UIE | IFTIM_ICE;
#endif
}

static void laststep(void) {
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_PWM1;
#ifdef INVERTED_HIGH
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#else
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
#endif
}

void tim1_com_isr(void) {
	if (!(TIM1_DIER & TIM_DIER_COMIE)) return;
	TIM1_SR = ~TIM_SR_COMIF;
	nextstep();
}

#ifdef SENSORED
void iftim_isr(void) { // Any change on Hall sensor inputs
	if (!(TIM_DIER(IFTIM) & TIM_DIER_CC1IE)) return;
	ival = (ival * 3 + TIM_CCR1(IFTIM)) >> 2;
	if (sync < 6) ++sync;
}
#else
void iftim_isr(void) { // BEMF zero-crossing
	int er = TIM_DIER(IFTIM);
	int sr = TIM_SR(IFTIM);
	if ((er & TIM_DIER_UIE) && (sr & TIM_SR_UIF)) { // Timeout
		TIM_SR(IFTIM) = ~TIM_SR_UIF;
		TIM_DIER(IFTIM) = 0;
		sync = 0;
		accl = 0;
		ival = 10000;
		ertm = 100000000;
		return;
	}
	if (!(er & IFTIM_ICE)) return;
	int t = IFTIM_ICR; // Time since last zero-crossing
	if (t < ival >> 1) return;
	int u = ival * 3;
	accl = t < u >> 2 && sync == 6; // Rapid acceleration
	ival = (t + u) >> 2; // Commutation interval
	IFTIM_OCR = ((ival >> 1) - (ival * cfg.timing >> 4)) >> accl; // Commutation delay
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_DIER(IFTIM) = 0;
	if (sync < 6) ++sync;
}
#endif

void adc_data(int t, int v, int c, int x) {
	static int z = 5000, st = -1, sv = -1, sc = -1, sx = -1;
	if ((c -= z) >= 0) ready = 1;
	else {
		if (!ready) z += c >> 1;
		c = 0;
	}
	temp = smooth(&st, t > 0 ? t : 0, 10); // C
	volt = smooth(&sv, v * VOLT_MUL / 100, 7); // V/100
	curr = smooth(&sc, c * CURR_MUL / 10, 4); // A/100
	if (!analog) return;
	throt = scale(smooth(&sx, x, 4), 10, 4085, ANALOG_MIN * 20, ANALOG_MAX * 20); // Analog throttle
}

void sys_tick_handler(void) {
	SCB_ICSR = SCB_ICSR_PENDSVSET; // Continue with low priority
	SCB_SCR = 0; // Resume main loop
	if (++tick & 15) return; // 16kHz -> 1kHz
	++tickms;
}

void pend_sv_handler(void) {
	static char d, led = -1;
	static int i, n, q;
	if (telreq && !cfg.telem_mode) { // Telemetry request
		sendtelem();
		telreq = 0;
	}
	if (tick & 15) return; // 16kHz -> 1kHz
	adc_trig();
	if (!(tickms & 31)) { // Telemetry every 32ms
		if (dshotext) {
			int v = 0;
			switch (++d) {
				case 1:
					v = 0x200 | (temp & 0xff);
					break;
				case 2:
					v = 0x400 | ((volt / 25) & 0xff);
					break;
				case 3:
					v = 0x600 | ((curr / 100) & 0xff);
					d = 0;
					break;
			}
			__disable_irq();
			if (!dshotval) dshotval = v;
			__enable_irq();
		}
		if (cfg.telem_mode == 1) sendtelem();
	}
	if (led != cfg.led) ledctl(led = cfg.led); // Update LEDs
	i += curr;
	if (++n < 1000) return; // 1ms -> 1s
	csum = (q += i / 1000) / 360; // mAh
	i = 0;
	n = 0;
}

static void beep(void) {
	static const char *const beacons[] = {"EG", "FA", "GB", "AB#", "aDGE"};
	static const char *const values[] = {"c6", "C2", "D2C2", "E2D2C2", "F#2E2D2C2", "G#A#G#A#G#2", "G#A#G#A#F#2G#2", "G#A#G#A#E2F#2G#2", "G#A#G#A#D2E2F#2G#2", "G#A#G#A#C2D2E2F#2G#2", 0};
	if (beacon) {
		playmusic(beacons[beacon - 1], cfg.beacon);
		beacon = 0;
	}
	if (beepval < 0) return;
	int i[10], n = 0, x = beepval, vol = cfg.volume;
	while (i[n++] = x % 10, x /= 10);
	if (vol < 25) vol = 25;
	while (n--) {
		if (x++) playmusic("_4", vol);
		playmusic(values[i[n]], vol);
	}
	beepval = -1;
}

void main(void) {
	memcpy(_cfg_start, _cfg, _cfg_end - _cfg_start); // Copy configuration to SRAM
	checkcfg();
	init();
	initled();
	inittelem();
#ifndef ANALOG
	initio();
#else
	throt = ANALOG_MIN * 20;
	analog = ANALOG_MIN != ANALOG_MAX;
#endif

	TIM1_BDTR = TIM_DTG | TIM_BDTR_OSSR | TIM_BDTR_MOE;
	TIM1_ARR = CLK_KHZ / 24 - 1;
	TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#ifdef STM32G0
	TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE << 16; // TRGO2=OC1
#else
	TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE; // TRGO=OC1
#endif

	TIM_PSC(IFTIM) = CLK_MHZ / 2 - 1; // 0.5us resolution
	TIM_ARR(IFTIM) = 0;
	TIM_CR1(IFTIM) = TIM_CR1_URS;
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
#ifdef SENSORED
	TIM_CR2(IFTIM) = TIM_CR2_TI1S | TIM_CR2_MMS_UPDATE; // TI1=CH1^CH2^CH3, TRGO=UPDATE
	TIM_SMCR(IFTIM) = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM_CCMR1(IFTIM) = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_DTF_DIV_8_N_8;
	TIM_CCER(IFTIM) = TIM_CCER_CC1E; // IC1 on any edge on TI1
#endif

	nvic_set_priority(NVIC_PENDSV_IRQ, 0x80);
	STK_RVR = CLK_KHZ / 16 - 1; // 16kHz
	STK_CVR = 0;
	STK_CSR = STK_CSR_ENABLE | STK_CSR_TICKINT | STK_CSR_CLKSOURCE_AHB;

	int cells = cfg.prot_cells;
#if SENS_CNT > 0
	while (!ready) { // Wait for sensors
		WWDG_CR = 0xff;
		__WFI();
	}
	if (!cells) cells = (volt + 439) / 440; // Assume maximum 4.4V per battery cell
#endif
#ifndef ANALOG
	int csr = RCC_CSR;
	RCC_CSR = RCC_CSR_RMVF; // Clear reset flags
	if (!(csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))) { // Power-on
		playmusic(cfg.music, cfg.volume);
		if (cfg.prot_volt) for (int i = 0; i < cells; ++i) playmusic("_2D", cfg.volume); // Number of battery cells
	}
	if (cfg.arm || (csr & RCC_CSR_WWDGRSTF)) { // Arming required
		TIM14_PSC = CLK_KHZ / 10 - 1; // 0.1ms resolution
		TIM14_ARR = 2499; // 250ms
		TIM14_CR1 = TIM_CR1_URS;
		TIM14_EGR = TIM_EGR_UG;
		TIM14_CR1 = TIM_CR1_CEN | TIM_CR1_URS;
		throt = 1;
		while (!(TIM14_SR & TIM_SR_UIF)) { // Wait for 250ms zero throttle
			WWDG_CR = 0xff;
			__WFI();
			beep();
			if (!throt) continue;
			TIM14_EGR = TIM_EGR_UG;
		}
		throt = 0;
		TIM14_CR1 = 0;
		playmusic("GC", cfg.volume);
	}
#endif
	laststep();
	TIM1_EGR = TIM_EGR_COMG;
	PID curpid = {.Kp = 400, .Ki = 0, .Kd = 600};
	for (int curduty = 0, running = 0, braking = 2, choke = 0, r = 0, v = 0;;) {
		SCB_SCR = SCB_SCR_SLEEPONEXIT; // Suspend main loop
		WWDG_CR = 0xff;
		__WFI();
		int input = throt;
		int range = cfg.sine_range * 20;
		int margin = range && sine ? 20 : 0;
		int newduty = 0;
		if (!running) curduty = 0;
		if (input > 0) { // Forward
			if (range + margin < input) newduty = scale(input, range, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
			else sine = scale(input, 0, range, 1000, 145);
			reverse = cfg.revdir ^ flipdir;
			running = 1;
			braking = 0;
		} else if (input < 0) { // Reverse
			if (cfg.throt_mode == 2 && braking != 2) { // Proportional brake
				curduty = scale(-input, 0, 2000, cfg.duty_drag * 20, 2000);
				running = 0;
				braking = 1;
			} else {
				if (range + margin < -input) newduty = scale(-input, range, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
				else sine = scale(-input, 0, range, 1000, 145);
				reverse = !cfg.revdir ^ flipdir;
				running = 1;
			}
		} else { // Neutral
			curduty = cfg.duty_drag * 20;
			running = 0;
			if (braking == 1) braking = 2; // Reverse after braking
		}
#ifndef SENSORED
		if (running && sine) { // Sine startup
			if (!newduty) {
				if (!ertm) goto skip_duty;
				ertm = sine * 180;
				erpm = 60000000 / ertm;
				goto skip_duty;
			}
			__disable_irq();
			int a = step - 1;
			int b = a / 60;
			int c = b * 60;
			IFTIM_OCR = sine * (reverse ? c - a + 60 : a - c + 1); // Commutation delay
			TIM_ARR(IFTIM) = 0xffff;
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			step = b + 1; // Switch over to 6-step
			sine = 0;
			prep = 0;
			sync = 0;
			accl = 0;
			ival = 10000;
			nextstep();
			__enable_irq();
		}
#endif
		int arr = CLK_KHZ / cfg.freq_min;
		if (running) {
			if (ertm) {
				erpm = 60000000 / ertm;
				arr = scale(erpm, 30000, 60000, arr, CLK_KHZ / cfg.freq_max); // Variable PWM frequency
			}
			if ((newduty -= choke) < 0) newduty = 0;
			if (sync < 6) { // Power cap during spin-up
				int maxduty = cfg.duty_spup * 20;
				if (newduty > maxduty) newduty = maxduty;
			}
			int a = accl ? 0 : cfg.duty_ramp;
			int b = a >> 3;
			if (r < (a & 7)) ++b;
			if (++r == 8) r = 0;
			if (curduty >= newduty || (curduty += b) > newduty) curduty = newduty; // Acceleration ramping
		}
		int ccr = scale(curduty, 0, 2000, running && cfg.damp ? DEAD_TIME : 0, arr);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_ARR = arr;
		TIM1_CCR1 = ccr;
		TIM1_CCR2 = ccr;
		TIM1_CCR3 = ccr;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	skip_duty:
		if (running && !step) { // Start motor
			__disable_irq();
			ival = 10000;
			ertm = 100000000;
			nextstep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
			TIM1_DIER |= TIM_DIER_COMIE;
#ifdef SENSORED
			TIM_SR(IFTIM) = ~TIM_SR_CC1IF;
			TIM_DIER(IFTIM) = TIM_DIER_CC1IE;
			TIM_ARR(IFTIM) = -1;
#else
			IFTIM_OCR = 0xffff;
			TIM_ARR(IFTIM) = 0xffff;
#endif
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			__enable_irq();
		} else if (!running && step) { // Stop motor
			__disable_irq();
			laststep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
			TIM1_DIER &= ~TIM_DIER_COMIE;
			TIM_ARR(IFTIM) = 0;
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			TIM_DIER(IFTIM) = 0;
#ifndef SENSORED
			compctl(0);
#endif
			step = 0;
			sine = 0;
			prep = 0;
			sync = 0;
			accl = 0;
			ertm = 0;
			erpm = 0;
			__enable_irq();
		}
		beep();
		if (tick & 15) continue; // 16kHz -> 1kHz
		if (volt >= cfg.prot_volt * cells * 10) v = 0;
		else if (++v == 3000) reset(); // Low voltage cutoff after 3s
		int t = cfg.prot_temp ? clamp((temp - cfg.prot_temp) * 100, 0, 1500) : 0; // 25% power cap @ 15C above threshold
		int u = cfg.prot_curr ? calcpid(&curpid, curr, cfg.prot_curr * 100) >> 10 : 0; // Current PID control
		choke = clamp(choke + u, t, 2000);
	}
}
