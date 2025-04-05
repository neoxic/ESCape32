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

#define REVISION 14
#define REVPATCH 1

const Cfg cfgdata = {
	.id = 0x32ea,
	.revision = REVISION,
	.revpatch = REVPATCH,
	.name = TARGET_NAME,
	.arm = ARM,                 // Wait for 250ms zero throttle on startup
	.damp = DAMP,               // Complementary PWM (active freewheeling)
	.revdir = REVDIR,           // Reversed motor direction
	.brushed = BRUSHED,         // Brushed mode
	.timing = TIMING,           // Motor timing (15/16 deg) [1..31]
	.sine_range = SINE_RANGE,   // Sine startup range (%) [0 - off, 5..25]
	.sine_power = SINE_POWER,   // Sine startup power (%) [1..15]
	.freq_min = FREQ_MIN,       // Minimum PWM frequency (kHz) [16..48]
	.freq_max = FREQ_MAX,       // Maximum PWM frequency (kHz) [16..96]
	.duty_min = DUTY_MIN,       // Minimum duty cycle (%) [1..100]
	.duty_max = DUTY_MAX,       // Maximum duty cycle (%) [1..100]
	.duty_spup = DUTY_SPUP,     // Maximum duty cycle during spin-up (%) [1..25]
	.duty_ramp = DUTY_RAMP,     // Maximum duty cycle ramp (kERPM) [0..100]
	.duty_rate = DUTY_RATE,     // Duty cycle slew rate (0.1%/ms) [1..100]
	.duty_drag = DUTY_DRAG,     // Drag brake power (%) [0..100]
	.duty_lock = DUTY_LOCK,     // Active drag brake (motor lock)
	.throt_mode = THROT_MODE,   // Throttle mode (0 - forward, 1 - forward/reverse, 2 - forward/brake/reverse, 3 - forward/brake)
	.throt_rev = THROT_REV,     // Maximum reverse throttle (0 - 100%, 1 - 75%, 2 - 50%, 3 - 25%)
	.throt_brk = THROT_BRK,     // Maximum brake power (%) [0..100]
	.throt_set = THROT_SET,     // Preset throttle (%) [0..100]
	.throt_cal = THROT_CAL,     // Automatic throttle calibration
	.throt_min = THROT_MIN,     // Minimum throttle setpoint (us)
	.throt_mid = THROT_MID,     // Middle throttle setpoint (us)
	.throt_max = THROT_MAX,     // Maximum throttle setpoint (us)
	.analog_min = ANALOG_MIN,   // Minimum analog throttle setpoint (mV)
	.analog_max = ANALOG_MAX,   // Maximum analog throttle setpoint (mV)
	.input_mode = INPUT_MODE,   // Input mode (0 - servo/Oneshot125/DSHOT, 1 - analog, 2 - serial, 3 - iBUS, 4 - SBUS/SBUS2, 5 - CRSF)
	.input_chid = INPUT_CHID,   // Serial channel ID [0 - off, 1..14 - iBUS, 1..16 - SBUS/SBUS2/CRSF]
	.telem_mode = TELEM_MODE,   // Telemetry mode (0 - KISS, 1 - KISS auto, 2 - iBUS, 3 - S.Port, 4 - CRSF)
	.telem_phid = TELEM_PHID,   // Telemetry physical ID [0 - off, 1..2 - iBUS, 1..28 - S.Port, 1..4 - SBUS2]
	.telem_poles = TELEM_POLES, // Number of motor poles for RPM telemetry [2..100]
	.prot_stall = PROT_STALL,   // Stall protection (ERPM) [0 - off, 1800..3200]
	.prot_temp = PROT_TEMP,     // Temperature threshold (C) [0 - off, 60..140]
	.prot_sens = PROT_SENS,     // Temperature sensor (0 - ESC, 1 - motor, 2 - both)
	.prot_volt = PROT_VOLT,     // Low voltage cutoff per battery cell (V/10) [0 - off, 28..38]
	.prot_cells = PROT_CELLS,   // Number of battery cells [0 - auto, 1..24]
	.prot_curr = PROT_CURR,     // Maximum current (A) [0..500]
	.music = MUSIC,             // Startup music
	.volume = VOLUME,           // Sound volume (%) [0..100]
	.beacon = BEACON,           // Beacon volume (%) [0..100]
	.bec = BEC,                 // BEC voltage control [0..3]
	.led = LED,                 // LED on/off bits [0..15]
};

__attribute__((__section__(".cfg")))
Cfg cfg = cfgdata;

int throt, ertm, erpm, temp1, temp2, volt, curr, csum, dshotval, beepval = -1;
char analog, telreq, telmode, flipdir, beacon, dshotext;

static int oldstep, step, sine, sync, ival, cutback, led;
static char prep, fast, lock, tick, ready, reverse;
static uint32_t tickms, tickmsv;
static volatile char tickmsf;
#ifndef HALL_MAP
static const int hall;
#else
static int hall;

static int getcode(void) {
	int x = -1;
	for (int i = 0, j = 0; j < 4; ++j) {
		int y = hallcode();
		if (x == y) continue;
		if (++i == 20) hard_fault_handler(); // Unstable signal
		x = y;
		j = 0;
	}
	return x;
}
#endif

/*
6-step commutation sequence:
 #  +|-  MASK  BEMF
 1  C|B   110   101
 2  A|B   011   001
 3  A|C   101   011
 4  B|C   110   010
 5  B|A   011   110
 6  C|A   101   100
*/

static void nextstep(void) {
	if (sine) { // Sine startup
		TIM_ARR(IFTIM) = IFTIM_OCR = sine;
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
		int p = min(cfg.sine_power << 3, 120 - cutback); // 50% cutback at 15C above prot_temp
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_ARR = CLK_KHZ / 24 - 1;
		TIM1_CCR1 = DEAD_TIME + (sinedata[a] * p >> 7);
		TIM1_CCR2 = DEAD_TIME + (sinedata[b] * p >> 7);
		TIM1_CCR3 = DEAD_TIME + (sinedata[c] * p >> 7);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#ifdef RPM_PIN
		if (step == 1) GPIO(RPM_PORT, BSRR) = 1 << (RPM_PIN + 16);
		else if (step == 181) GPIO(RPM_PORT, BSRR) = 1 << RPM_PIN;
#endif
		if (prep) return;
		TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_PWM1;
		TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
#ifdef PWM_ENABLE
		int er = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
#else
		int er = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
#endif
#ifdef INVERTED_HIGH
		er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
		er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
		TIM1_CCER = er;
		TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
		TIM_DIER(IFTIM) = 0;
		compctl(0);
		sync = 0;
		prep = 1;
		return;
	}
#ifdef HALL_MAP
	static const char map[][2] = {{2, 4}, {4, 6}, {3, 5}, {6, 2}, {1, 3}, {5, 1}}; // Hall sensor code mapping
	if (hall > 4000) {
		int code = getcode();
		if (code < 1 || code > 6) hard_fault_handler(); // Invalid Hall sensor code
		step = map[code - 1][!!reverse];
	} else
#endif
	if (reverse) {
		if (--step < 1) step = 6;
	} else {
		if (++step > 6) step = 1;
	}
	static const char seq[] = {0x35, 0x19, 0x2b, 0x32, 0x1e, 0x2c}; // Commutation sequence
	static int pcc, val, cnt, buf[6];
	int x = seq[step - 1];
	int m = x >> 3; // Energized phase mask
	int p = x & m; // Positive phase
	int n = ~x & m; // Negative phase
	int m1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
#ifdef TIM1_CCR5
	int m2 = TIM_CCMR2_OC3PE;
	int er = TIM_CCER_CC5E;
#else
	int m2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM1;
	int er = TIM_CCER_CC4E;
#endif
	int cc = (step & 1) ^ reverse ? 4 : 0; // BEMF rising/falling
	if (p & 1) {
		m1 |= TIM_CCMR1_OC1M_PWM1;
#ifdef PWM_ENABLE
		er |= TIM_CCER_CC1E;
#else
		er |= cfg.damp ? TIM_CCER_CC1E | TIM_CCER_CC1NE : TIM_CCER_CC1E;
#endif
	} else if (n & 1) {
#ifdef PWM_ENABLE
		m1 |= TIM_CCMR1_OC1M_FORCE_LOW;
#else
		m1 |= TIM_CCMR1_OC1M_FORCE_HIGH;
#endif
		er |= TIM_CCER_CC1NE;
	} else {
#ifdef PWM_ENABLE
		m1 |= TIM_CCMR1_OC1M_FORCE_HIGH;
#else
		m1 |= TIM_CCMR1_OC1M_FORCE_LOW;
#endif
		er |= TIM_CCER_CC1NE;
		cc |= 1;
	}
	if (p & 2) {
		m1 |= TIM_CCMR1_OC2M_PWM1;
#ifdef PWM_ENABLE
		er |= TIM_CCER_CC2E;
#else
		er |= cfg.damp ? TIM_CCER_CC2E | TIM_CCER_CC2NE : TIM_CCER_CC2E;
#endif
	} else if (n & 2) {
#ifdef PWM_ENABLE
		m1 |= TIM_CCMR1_OC2M_FORCE_LOW;
#else
		m1 |= TIM_CCMR1_OC2M_FORCE_HIGH;
#endif
		er |= TIM_CCER_CC2NE;
	} else {
#ifdef PWM_ENABLE
		m1 |= TIM_CCMR1_OC2M_FORCE_HIGH;
#else
		m1 |= TIM_CCMR1_OC2M_FORCE_LOW;
#endif
		er |= TIM_CCER_CC2NE;
		cc |= 2;
	}
	if (p & 4) {
		m2 |= TIM_CCMR2_OC3M_PWM1;
#ifdef PWM_ENABLE
		er |= TIM_CCER_CC3E;
#else
		er |= cfg.damp ? TIM_CCER_CC3E | TIM_CCER_CC3NE : TIM_CCER_CC3E;
#endif
	} else if (n & 4) {
#ifdef PWM_ENABLE
		m2 |= TIM_CCMR2_OC3M_FORCE_LOW;
#else
		m2 |= TIM_CCMR2_OC3M_FORCE_HIGH;
#endif
		er |= TIM_CCER_CC3NE;
	} else {
#ifdef PWM_ENABLE
		m2 |= TIM_CCMR2_OC3M_FORCE_HIGH;
#else
		m2 |= TIM_CCMR2_OC3M_FORCE_LOW;
#endif
		er |= TIM_CCER_CC3NE;
		cc |= 3;
	}
#ifdef INVERTED_HIGH
	er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
	er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	TIM1_CCMR1 = m1;
	TIM1_CCMR2 = m2;
	TIM1_CCER = er;
	compctl(pcc);
	pcc = cc;
	if (ival > 1000 << IFTIM_XRES) {
		val = 1000 << IFTIM_XRES;
		cnt = 0;
	} else if (++cnt == 6) {
		if (abs(val - ival) > ival >> 1) { // Probably desync
			sync = 0;
			fast = 0;
			ival = 5000 << IFTIM_XRES;
			ertm = 100000000;
		}
		val = ival;
		cnt = 0;
	}
	if (ertm < 100) { // 600K+ ERPM
#ifdef TIM1_CCR5
		TIM1_CCR5 = 0;
#else
		TIM1_CCR4 = 0;
#endif
		IFTIM_ICMR = IFTIM_ICM3;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	} else if (ertm < 200) { // 300K+ ERPM
#ifdef TIM1_CCR5
		TIM1_CCR5 = 0;
#else
		TIM1_CCR4 = 0;
#endif
		IFTIM_ICMR = IFTIM_ICM2;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	} else if (ertm < 1000) { // 60K+ ERPM
#ifdef TIM1_CCR5
		TIM1_CCR5 = 0;
#else
		TIM1_CCR4 = 0;
#endif
		IFTIM_ICMR = IFTIM_ICM1;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	} else if (ertm < 2000) { // 30K+ ERPM
#ifdef TIM1_CCR5
		TIM1_CCR5 = 0;
#else
		TIM1_CCR4 = 0;
#endif
		IFTIM_ICMR = IFTIM_ICM1;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_2;
	} else { // Minimum PWM frequency
#ifdef TIM1_CCR5
		TIM1_CCR5 = IFTIM_ICFL << 2;
#else
		TIM1_CCR4 = IFTIM_ICFL << 2;
#endif
		IFTIM_ICMR = IFTIM_ICM1;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_4;
	}
	TIM_SR(IFTIM) = 0; // Clear BEMF events before enabling interrupts
	TIM_DIER(IFTIM) = TIM_DIER_UIE | IFTIM_ICIE;
	buf[step - 1] = hall > 4000 ? hall << IFTIM_XRES : ival;
	if (sync < 6) return;
	ertm = (buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5]) >> (IFTIM_XRES + 1); // Electrical revolution time (us)
#ifdef RPM_PIN
	if (step == 1) GPIO(RPM_PORT, BSRR) = 1 << (RPM_PIN + 16);
	else if (step == 4) GPIO(RPM_PORT, BSRR) = 1 << RPM_PIN;
#endif
}

static void laststep(void) {
	resetcom();
	if (sine && prep) { // Switch over to 6-step
		step = (step + 29) / 60 + 1;
		if (step > 6) step = 1;
	}
	sine = 0;
	prep = 0;
	if (lock) nextstep();
	else {
#ifdef PWM_ENABLE
		TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_PWM2;
		TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM2;
#else
		TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_PWM1;
		TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM1;
#endif
	}
	TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
	compctl(0);
	oldstep = step;
	step = 0;
}

void tim1_com_isr(void) {
	if (!(TIM1_DIER & TIM_DIER_COMIE)) return;
#if !defined STM32G4 && !defined AT32F4
	int m1 = TIM1_CCMR1;
	int m2 = TIM1_CCMR2;
#ifdef PWM_ENABLE
	if ((m1 & TIM_CCMR1_OC1M_MASK) == TIM_CCMR1_OC1M_FORCE_HIGH) m1 &= ~TIM_CCMR1_OC1M_MASK;
	if ((m1 & TIM_CCMR1_OC2M_MASK) == TIM_CCMR1_OC2M_FORCE_HIGH) m1 &= ~TIM_CCMR1_OC2M_MASK;
	if ((m2 & TIM_CCMR2_OC3M_MASK) == TIM_CCMR2_OC3M_FORCE_HIGH) m2 &= ~TIM_CCMR2_OC3M_MASK;
#else
	if ((m1 & TIM_CCMR1_OC1M_MASK) == TIM_CCMR1_OC1M_FORCE_LOW) m1 &= ~TIM_CCMR1_OC1M_MASK;
	if ((m1 & TIM_CCMR1_OC2M_MASK) == TIM_CCMR1_OC2M_FORCE_LOW) m1 &= ~TIM_CCMR1_OC2M_MASK;
	if ((m2 & TIM_CCMR2_OC3M_MASK) == TIM_CCMR2_OC3M_FORCE_LOW) m2 &= ~TIM_CCMR2_OC3M_MASK;
#endif
	TIM1_CCMR1 = m1;
	TIM1_CCMR2 = m2;
	TIM1_EGR = TIM_EGR_COMG;
#endif
	TIM1_SR = ~TIM_SR_COMIF;
	nextstep();
}

void iftim_isr(void) { // BEMF zero-crossing
	int er = TIM_DIER(IFTIM);
	int sr = TIM_SR(IFTIM);
	if ((er & TIM_DIER_UIE) && (sr & TIM_SR_UIF)) { // Timeout
		TIM_SR(IFTIM) = ~TIM_SR_UIF;
		TIM_DIER(IFTIM) = 0;
		sync = 0;
		fast = 0;
		ival = 10000 << IFTIM_XRES;
		ertm = 100000000;
		return;
	}
	if (!(er & IFTIM_ICIE)) return;
	int t = IFTIM_ICR; // Time since last zero-crossing
	if (t < ival >> 1) return;
	int u = ival * 3;
	fast = (t < u >> 2 || t > u >> 1) && ertm < 2000; // Fast acceleration/deceleration
	ival = (t + u) >> 2; // Commutation interval
	IFTIM_OCR = max((ival - (ival * cfg.timing >> 5)) >> 1, 1); // Commutation delay
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_DIER(IFTIM) = 0;
	if (sync < 6) ++sync;
}

#ifdef HALL_MAP
void tim3_isr(void) { // Any change on Hall sensor inputs
	if (TIM3_SR & TIM_SR_UIF) { // Timeout
		TIM3_SR = ~TIM_SR_UIF;
		hall = 0x10000;
		if (sine || !step) return;
		sync = 0;
		fast = 0;
		ival = 10000 << IFTIM_XRES;
		ertm = 100000000;
		return;
	}
	hall = (TIM3_CCR1 + hall * 3) >> 2;
	if (hall < 5000 || sine || !step) return;
	ival = hall << IFTIM_XRES;
	TIM1_EGR = TIM_EGR_COMG;
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_DIER(IFTIM) = 0;
	if (sync < 6) ++sync;
}
#endif

void adcdata(int t, int u, int v, int c, int a) {
	static int z = 3300, i, q, st = -1, su = -1, sv = -1, sc = -1, sa = -1;
	if ((c -= z) >= 0) ready = 1;
	else {
		if (!ready) z += c >> 1;
		c = 0;
	}
	temp1 = max((t = smooth(&st, t, 10)) >> 2, 0); // C
	temp2 = max((u = smooth(&su, TEMP_SENS(u), 10)) >> 2, 0); // C
	volt = smooth(&sv, v * VOLT_MUL * 131 >> 17, 7); // V/100
	curr = smooth(&sc, c * CURR_MUL * 205 >> 11, 4); // A/100
	i += curr; // Current integral
	if (!(tickms & 0x3ff)) {
		csum = (q += i >> 10) * 91 >> 15; // mAh
		i = 0;
	}
	if (cfg.prot_temp) { // Temperature protection
		int x = -(cfg.prot_temp << 2);
		switch (cfg.prot_sens) {
			case 0:
				x += t;
				break;
			case 1:
				x += u;
				break;
			case 2:
				x += max(t, u);
				break;
		}
		cutback = clamp(x, 0, 60);
	}
	if (!analog) return;
	throt = scale(smooth(&sa, a, 5), cfg.analog_min, cfg.analog_max, cfg.throt_set * 20, 2000);
}

void sys_tick_handler(void) {
	SCB_ICSR = SCB_ICSR_PENDSVSET; // Continue with low priority
	SCB_SCR = 0; // Resume main loop
	if (++tick & 15) return; // 16kHz -> 1kHz
	if (++tickms == tickmsv) tickmsf = 0;
}

void delay(int ms, Func f) {
	__disable_irq();
	tickmsv = tickms + ms;
	tickmsf = 1;
	__enable_irq();
	while (tickmsf) f();
}

void pend_sv_handler(void) {
	static int a = -1;
	if (telreq && !telmode) { // Telemetry request
		kisstelem();
		telreq = 0;
	}
	if (tick & 15) return; // 16kHz -> 1kHz
	adctrig();
	if (!(tickms & 31)) autotelem(); // Telemetry every 32ms
	int b = cfg.led ? cfg.led : led;
	if (a != b) ledctl(a = b); // Update LED
}

void hard_fault_handler(void) {
	ledctl(1); // Indicate error
	TIM1_EGR = TIM_EGR_BG;
	TIM6_PSC = CLK_KHZ / 10 - 1; // 0.1ms resolution
	TIM6_ARR = 9999;
	TIM6_EGR = TIM_EGR_UG;
	TIM6_SR = ~TIM_SR_UIF;
	TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_OPM;
	while (TIM6_CR1 & TIM_CR1_CEN); // Wait for 1s
	WWDG_CR = WWDG_CR_WDGA; // Trigger watchdog reset
	for (;;); // Never return
}

static void delayf(void) {
	TIM6_EGR = TIM_EGR_UG; // Reset arming timeout
}

static void beep(void) {
	static const char *const beacons[] = {"EG", "FA", "GB", "AB#", "aDGE"};
	static const char *const values[] = {"c6", "C2", "D2C2", "E2D2C2", "F#2E2D2C2", "G#A#G#A#G#2", "G#A#G#A#F#2G#2", "G#A#G#A#E2F#2G#2", "G#A#G#A#D2E2F#2G#2", "G#A#G#A#C2D2E2F#2G#2", 0};
	if (beacon) {
		playmusic(beacons[beacon - 1], cfg.beacon);
		beacon = 0;
	}
	if (beepval < 0) return;
	int i[10], n = 0, x = beepval, vol = max(cfg.volume, 25);
	while (i[n++] = x % 10, x /= 10);
	while (n--) {
		if (x++) delay(500, delayf);
		playmusic(values[i[n]], vol);
	}
	beepval = -1;
}

void main(void) {
	memcpy(_cfg_start, _cfg, _cfg_end - _cfg_start); // Copy configuration to SRAM
	checkcfg();
	const int brushed = cfg.brushed;
	const int mode = cfg.throt_mode;
	lock = cfg.duty_lock;
	throt = cfg.throt_set * 20;
	telmode = cfg.telem_mode;
#if defined ANALOG || defined ANALOG_CHAN
	analog = IO_ANALOG;
#endif
	init();
	initgpio();
	initled();
	inittelem();
#ifndef ANALOG
	initio();
#endif
	TIM1_BDTR = TIM_DTG | TIM_BDTR_OSSR | TIM_BDTR_MOE;
	TIM1_ARR = CLK_KHZ / 24 - 1;
	TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
#ifdef STM32G0
	TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE << 16; // TRGO2=OC1
#else
	TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS | TIM_CR2_MMS_COMPARE_PULSE; // TRGO=OC1
#endif
	TIM_PSC(IFTIM) = (CLK_MHZ >> (IFTIM_XRES + 1)) - 1; // 125/250/500ns resolution
	TIM_ARR(IFTIM) = 0;
	TIM_CR1(IFTIM) = TIM_CR1_URS;
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
#ifdef HALL_MAP
	if (!brushed && getcode() != 7) { // Hybrid mode
		TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
		TIM3_CCMR1 = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_DTF_DIV_8_N_8;
		TIM3_CCER = TIM_CCER_CC1E; // IC1 on any edge on TI1
		TIM3_DIER = TIM_DIER_UIE | TIM_DIER_CC1IE;
		TIM3_PSC = CLK_MHZ / 2 - 1; // 500ns resolution
		TIM3_ARR = -1;
		TIM3_CR1 = TIM_CR1_URS;
		TIM3_EGR = TIM_EGR_UG;
		TIM3_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
		hall = 0x10000;
	}
#endif
	nvic_set_priority(NVIC_PENDSV_IRQ, 0x80);
	STK_RVR = CLK_KHZ / 16 - 1; // 16kHz
	STK_CVR = 0;
	STK_CSR = STK_CSR_ENABLE | STK_CSR_TICKINT | STK_CSR_CLKSOURCE_AHB;
#ifndef ANALOG
	int cells = cfg.prot_cells;
#if SENS_CNT > 0
	while (!ready) __WFI(); // Wait for sensors
	if (!cells) cells = (volt + 439) / 440; // Assume maximum 4.4V per battery cell
#endif
	int csr = RCC_CSR;
	RCC_CSR = RCC_CSR_RMVF; // Clear reset flags
	if (!(csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))) { // Power-on
		const char *str = cfg.music;
		if (str[0] == '~') playsound(_eod, clamp(atoi(str + 1), 0, 100));
		else playmusic(str, cfg.volume);
		if (cfg.prot_volt) { // Report the number of battery cells
			beepval = cells;
			delay(250, delayf);
			beep();
		}
	}
	if (cfg.arm || (csr & RCC_CSR_WWDGRSTF)) { // Arming required
	rearm:
		TIM6_PSC = CLK_KHZ / 10 - 1; // 0.1ms resolution
		TIM6_ARR = 2499; // 250ms
		TIM6_CR1 = TIM_CR1_URS;
		TIM6_EGR = TIM_EGR_UG;
		TIM6_CR1 = TIM_CR1_CEN | TIM_CR1_URS;
		TIM6_SR = ~TIM_SR_UIF;
		throt = 1;
		while (!(TIM6_SR & TIM_SR_UIF)) { // Wait for 250ms zero throttle
			__WFI();
			beep();
			if (!throt) continue;
			TIM6_EGR = TIM_EGR_UG;
		}
		throt = 0;
		TIM6_CR1 = 0;
		playmusic(hall ? "G_GC" : "GC", cfg.volume);
	}
#endif
	laststep();
	PID bpid = {.Kp = 50, .Ki = 0, .Kd = 1000}; // Stall protection
	PID cpid = {.Kp = 400, .Ki = 0, .Kd = 600}; // Overcurrent protection
	for (int curduty = 0, running = 0, braking = 2, cutoff = 0, boost = 0, choke = 0, n = 0;;) {
		int ccr, arr = CLK_KHZ / cfg.freq_min;
		int input = cutoff == 3000 ? 0 : throt;
		int range = cfg.sine_range * 20;
		int delta = range ? 10 : 0;
		int newduty = 0;
		if (!running) curduty = 0;
		if (!input) { // Neutral
			if (braking == 1) braking = 2; // Reverse after braking
			if (lock) { // Active drag brake
				curduty = min(cfg.duty_drag, 100 - cutback); // 60% cutback at 15C above prot_temp
				running = 0;
				goto setduty;
			}
			if (sync < 6 || erpm < 800) { // Passive drag brake
				curduty = cfg.duty_drag * 20;
				running = 0;
				goto setduty;
			}
			boost = 0; // Coasting
			goto calcduty;
		}
		if (input < 0) { // Reverse
			if ((mode == 2 && braking != 2) || mode == 3) { // Proportional brake
				curduty = scale(input, -2000, 0, cfg.throt_brk * 20, cfg.duty_drag * 20);
				running = 0;
				braking = 1;
				goto setduty;
			}
			input = input * (cfg.throt_rev - 4) >> 2;
			reverse = !cfg.revdir ^ flipdir;
			running = 1;
		} else { // Forward
			reverse = cfg.revdir ^ flipdir;
			running = 1;
			braking = 0;
		}
		if (range + (sine ? delta : -delta) < input) newduty = scale(input, range + delta, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
		else sine = scale(input, 0, range - delta, 1000 << IFTIM_XRES, cfg.prot_stall ? (333333 << IFTIM_XRES) / cfg.prot_stall : 145 << IFTIM_XRES);
		if (sine) { // Sine startup
			if (!newduty) {
				if (!ertm) goto skipduty;
				ertm = sine * (180 >> IFTIM_XRES);
				erpm = 60000000 / ertm;
				goto skipduty;
			}
			__disable_irq();
			if (prep) { // Switch over to 6-step
				int a = step - 1;
				int b = a / 60;
				int c = b * 60;
				if (reverse && ++b == 6) b = 0;
				IFTIM_OCR = sine * (reverse ? a - c + 1 : c - a + 60); // Commutation delay
				TIM_ARR(IFTIM) = (1 << (IFTIM_XRES + 16)) - 1;
				TIM_EGR(IFTIM) = TIM_EGR_UG;
				step = b + 1;
			}
			sine = 0;
			prep = 0;
			sync = 0;
			fast = 0;
			ival = 10000 << IFTIM_XRES;
			nextstep();
			__enable_irq();
			initpid(&bpid, 10000 << IFTIM_XRES);
			curduty = 0;
			boost = 0;
		}
	calcduty:
		if (brushed && step != reverse + 1) step = 0; // Change brushed direction
		if ((newduty += boost - choke) < 0) newduty = 0;
		if (ertm) { // Variable PWM frequency
			erpm = 60000000 / ertm;
			arr = scale(erpm, 30000, 60000, arr, CLK_KHZ / cfg.freq_max);
		}
		int maxduty = min(scale(erpm, 0, cfg.duty_ramp * 1000, cfg.duty_spup * 20, 2000), 2000 - cutback * 25); // 75% cutback at 15C above prot_temp
		if (newduty > maxduty) newduty = maxduty;
		int a = fast ? 0 : cfg.duty_rate;
		int b = a >> 3;
		if (n < (a & 7)) ++b;
		if (++n == 8) n = 0;
		if (curduty > newduty ? sync < 6 || (curduty -= b) < newduty : (curduty += b) > newduty) curduty = newduty; // Duty cycle slew rate limiting
	setduty:
#ifdef FULL_DUTY // Allow 100% duty cycle
		ccr = scale(curduty, 0, 2000, lock || (running && cfg.damp) ? DEAD_TIME : 0, arr--);
#else
		ccr = scale(curduty, 0, 2000, lock || (running && cfg.damp) ? DEAD_TIME : 0, brushed ? arr - (CLK_MHZ * 3 >> 1) : arr);
#endif
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_ARR = arr;
		TIM1_CCR1 = ccr;
		TIM1_CCR2 = ccr;
		TIM1_CCR3 = ccr;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	skipduty:
		if (running && !step) { // Start motor
			if (brushed) {
				int m1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
				int m2 = TIM_CCMR2_OC3PE;
#ifdef PWM_ENABLE
				int er = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
				if (reverse) {
					m1 |= TIM_CCMR1_OC1M_FORCE_LOW | TIM_CCMR1_OC2M_PWM1;
					m2 |= TIM_CCMR2_OC3M_FORCE_LOW;
				} else {
					m1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_LOW;
					m2 |= TIM_CCMR2_OC3M_PWM1;
				}
#else
				int er = 0;
				if (reverse) {
					m1 |= TIM_CCMR1_OC1M_FORCE_HIGH | TIM_CCMR1_OC2M_PWM1;
					m2 |= TIM_CCMR2_OC3M_FORCE_HIGH;
					er |= TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC3NE;
					if (cfg.damp) er |= TIM_CCER_CC2NE;
				} else {
					m1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_FORCE_HIGH;
					m2 |= TIM_CCMR2_OC3M_PWM1;
					er |= TIM_CCER_CC1E | TIM_CCER_CC2NE | TIM_CCER_CC3E;
					if (cfg.damp) er |= TIM_CCER_CC1NE | TIM_CCER_CC3NE;
				}
#endif
#ifdef INVERTED_HIGH
				er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef PWM_ENABLE
				er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
				TIM1_CCMR1 = m1;
				TIM1_CCMR2 = m2;
				TIM1_CCER = er;
				TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
				step = reverse + 1;
				ertm = 600; // 100K ERPM (freq_min/duty_spup/duty_ramp have no effect)
				goto tick;
			}
			__disable_irq();
			step = oldstep;
			ival = 10000 << IFTIM_XRES;
			ertm = 100000000;
			nextstep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
#ifdef SW_BLANKING
			TIM1_DIER |= TIM_DIER_COMIE | TIM_DIER_UIE | TIM_DIER_CC4IE;
#else
			TIM1_DIER |= TIM_DIER_COMIE;
#endif
			TIM_ARR(IFTIM) = IFTIM_OCR = (1 << (IFTIM_XRES + 16)) - 1;
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			__enable_irq();
			initpid(&bpid, 10000 << IFTIM_XRES);
			boost = 0;
		} else if (!running && step) { // Stop motor
			__disable_irq();
#ifdef SW_BLANKING
			TIM1_DIER &= ~(TIM_DIER_COMIE | TIM_DIER_UIE | TIM_DIER_CC4IE);
#else
			TIM1_DIER &= ~TIM_DIER_COMIE;
#endif
			TIM_DIER(IFTIM) = 0;
			TIM_ARR(IFTIM) = 0;
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			laststep();
			sync = 0;
			fast = 0;
			ertm = 0;
			erpm = 0;
			__enable_irq();
		}
	tick:
		SCB_SCR = SCB_SCR_SLEEPONEXIT; // Suspend main loop
		__WFI();
		if (tick & 15) continue; // 16kHz -> 1kHz
#ifndef ANALOG
		if (cutoff < 3000) cutoff = volt < cfg.prot_volt * cells * 10 ? cutoff + 1 : 0;
		else if (!running) goto rearm; // Low voltage cutoff after 3s
#endif
		boost = cfg.prot_stall ? clamp(boost + (calcpid(&bpid, hall > 4000 ? hall : ival >> IFTIM_XRES, 20000000 / cfg.prot_stall - 800) >> 16), 0, 160) : 0; // Up to 8%
		choke = cfg.prot_curr ? clamp(choke + (calcpid(&cpid, curr, cfg.prot_curr * 100) >> 10), 0, 2000) : 0;
		beep();
#ifdef LED_STAT
		int x = 0;
		if (running) {
			if ((tickms << (throt < 0) & 0x1ff) < (sine ? 0x180 : 0x100)) x = LED_CNT >= 2 ? 2 : 1;
		} else if (curduty) {
			if ((tickms & (lock ? 0x2ff : 0x3ff)) < 0x40) x = LED_CNT >= 3 ? 4 : LED_CNT;
		}
		if (cutback || cutoff || choke) x |= 1;
		led = x;
#endif
	}
}
