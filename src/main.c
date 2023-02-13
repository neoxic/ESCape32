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

#define REVISION 2

const Cfg cfgdata = {
	.id = 0x32ea,
	.revision = REVISION,
	.target_type = IO_TYPE | SENS_CNT << 2,
	.target_name = TARGET_NAME,
	.arm = 1,               // Wait for 250ms zero throttle on startup
	.damp = 1,              // Complementary PWM (active freewheeling)
	.revdir = 0,            // Reversed motor direction
	.timing = 4,            // Motor timing (3.75*X degrees) [1..7]
	.freq_min = 24,         // Minimum PWM frequency (kHz)
	.freq_max = 48,         // Maximum PWM frequency (kHz)
	.duty_min = 0,          // Minimum duty cycle (%)
	.duty_max = 100,        // Maximum duty cycle (%)
	.duty_spup = 15,        // Maximum power during spin-up (%)
	.duty_ramp = 10,        // Acceleration ramping (0.5*X %/ms)
	.duty_drag = 0,         // Drag brake amount (%)
	.throt_mode = 0,        // Throttle mode (0 - forward, 1 - forward/reverse, 2 - forward/brake/reverse)
	.throt_cal = 1,         // Throttle calibration
	.throt_min = 1000,      // Minimum throttle (us)
	.throt_mid = 1500,      // Middle throttle (us)
	.throt_max = 2000,      // Maximum throttle (us)
	.input_mode = 0,        // Input mode (0 - servo/DSHOT, 1 - analog, 2 - serial, 3 - iBUS, 4 - SBUS)
	.input_chid = 0,        // iBUS/SBUS channel ID
	.telem_mode = 0,        // Telemetry mode (0 - KISS, 1 - KISS auto, 2 - iBUS, 3 - S.Port)
	.telem_phid = 0,        // S.Port physical ID
	.telem_poles = 14,      // Number of motor poles for RPM telemetry
	.prot_temp = TEMP_MAX,  // Temperature threshold (C)
	.prot_volt = 0,         // Low voltage cutoff per battery cell (V/10)
	.prot_cells = 0,        // Number of battery cells
	.prot_curr = CURR_MAX,  // Maximum current (A)
	.music = "dfa#",        // Startup music
	.volume = 25,           // Sound volume (%)
	.beacon = 50,           // Beacon volume (%)
	.led = 0,               // LED bits
};

__attribute__((__section__(".cfg")))
Cfg cfg = cfgdata;

int throt, erpt, erpm, temp, volt, curr, csum, dshotval, beepval = -1;
char analog, telreq, flipdir, beacon, dshotext;
volatile uint32_t tick;

static int step, period, pbuf[6], nsyn;
static char reverse, telmode, cells;

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
#ifdef SENSORED
	static const char map[] = {3, 5, 4, 1, 2, 6}; // map[code] -> step
	int code = IFTIM_IDR;
	if (code < 1 || code > 6) reset(); // Invalid Hall sensor code
	step = map[code - 1];
#endif
	if (reverse) {
		if (--step < 1) step = 6;
	} else {
		if (++step > 6) step = 1;
	}

	static const char seq[] = {0x35, 0x19, 0x2b, 0x32, 0x1e, 0x2c}; // Commutation sequence
	int x = seq[step - 1];
	int m = x >> 3; // Energized phase mask
	int p = x & m; // Positive phase
	int n = ~x & m; // Negative phase
	int m1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	int m2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4M_PWM1;
	int er = TIM_CCER_CC4E;
#ifndef SENSORED
	int cc = 0;
#endif

	// Phase A
	if (p & 1) {
		m1 |= TIM_CCMR1_OC1M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC1E | TIM_CCER_CC1NE : TIM_CCER_CC1E;
	} else if (n & 1) {
		m1 |= TIM_CCMR1_OC1M_FORCE_HIGH;
		er |= TIM_CCER_CC1NE;
	}
#ifndef SENSORED
	else cc = 1;
#endif

	// Phase B
	if (p & 2) {
		m1 |= TIM_CCMR1_OC2M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC2E | TIM_CCER_CC2NE : TIM_CCER_CC2E;
	} else if (n & 2) {
		m1 |= TIM_CCMR1_OC2M_FORCE_HIGH;
		er |= TIM_CCER_CC2NE;
	}
#ifndef SENSORED
	else cc = 2;
#endif

	// Phase C
	if (p & 4) {
		m2 |= TIM_CCMR2_OC3M_PWM1;
		er |= cfg.damp ? TIM_CCER_CC3E | TIM_CCER_CC3NE : TIM_CCER_CC3E;
	} else if (n & 4) {
		m2 |= TIM_CCMR2_OC3M_FORCE_HIGH;
		er |= TIM_CCER_CC3NE;
	}
#ifndef SENSORED
	else cc = 3;
#endif

	TIM1_CCMR1 = m1;
	TIM1_CCMR2 = m2;
	TIM1_CCER = er;

#ifndef SENSORED
	static int pcc;
	if ((step & 1) ^ reverse) cc |= 4; // BEMF rising
	compctl(pcc);
	pcc = cc;
#endif
}

static void laststep(void) {
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_PWM1;
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
	step = 0;
#ifndef SENSORED
	compctl(0);
#endif
}

void tim1_com_isr(void) {
	if (!(TIM1_DIER & TIM_DIER_COMIE)) return;
	TIM1_SR = ~TIM_SR_COMIF;
	pbuf[step - 1] = period;
	if (nsyn >= 6) erpt = (pbuf[0] + pbuf[1] + pbuf[2] + pbuf[3] + pbuf[4] + pbuf[5]) >> 1; // Electrical revolution period (us)
	nextstep();
#ifndef SENSORED
	if (erpt < 400) { // 150K+ ERPM
		TIM1_CCR4 = IFTIM_ICF + 8;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS;
	} else if (erpt < 2000) { // 30K+ ERPM
		TIM1_CCR4 = (IFTIM_ICF << 1) + 8;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_2;
	} else { // Minimum PWM frequency range
		TIM1_CCR4 = (IFTIM_ICF << 2) + 8;
		TIM_CR1(IFTIM) = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CKD_CK_INT_MUL_4;
	}
	TIM_SR(IFTIM) = 0; // Clear BEMF events before enabling interrupts
	TIM_DIER(IFTIM) = TIM_DIER_UIE | IFTIM_ICE;
#endif
}

#ifdef SENSORED
void iftim_isr(void) { // Any change on Hall sensor inputs
	if (!(TIM_DIER(IFTIM) & TIM_DIER_CC1IE)) return;
	period = (period * 3 + TIM_CCR1(IFTIM)) >> 2;
	if (nsyn < 6) ++nsyn;
}
#else
void iftim_isr(void) { // BEMF zero-crossing
	int er = TIM_DIER(IFTIM);
	int sr = TIM_SR(IFTIM);
	if ((er & TIM_DIER_UIE) && (sr & TIM_SR_UIF)) { // Timeout
		TIM_SR(IFTIM) = ~TIM_SR_UIF;
		goto desync;
	}
	if (!(er & IFTIM_ICE)) return;
	int t = IFTIM_ICR;
	if (t < (period >> 1)) { // Possible desync
		if (t < 150 || nsyn < 6) return;
		nsyn = 6;
		return;
	}
	period = (period * 3 + t) >> 2; // Blend time since last zero-crossing into commutation period
	if (nsyn < 60) { // Check for desync
		if (period < 150) goto desync;
		++nsyn;
	}
	IFTIM_OCR = (period >> 1) - (period * cfg.timing >> 4); // Commutation delay
	TIM_EGR(IFTIM) = TIM_EGR_UG;
	TIM_DIER(IFTIM) = 0;
	return;
desync:
	TIM_DIER(IFTIM) = TIM_DIER_UIE;
	period = 10000;
	erpt = 100000000;
	nsyn = 0;
}
#endif

void adc_data(int t, int v, int c, int x) {
	static int z = 5000, st = -1, sv = -1, sc = -1, sx = -1;
	if (t < 0) t = 0;
	if ((c -= z) < 0) { // Adjust zero current
		z += c >> 1;
		c = 0;
	}
	temp = smooth(&st, t, 10); // C
	volt = smooth(&sv, v * VOLT_MUL / 100, 7); // V/100
	curr = smooth(&sc, c * CURR_MUL / 10, 4); // A/100
	if (!analog) return;
	x = smooth(&sx, x, 4);
	throt = x > 10 ? scale(x, 10, 4085, 0, 2000) : 0; // Analog throttle
}

void sys_tick_handler(void) {
	static int n;
	SCB_ICSR = SCB_ICSR_PENDSVSET; // Continue with low priority
	if (++n < 10) return; // 10kHz -> 1kHz
	++tick;
	n = 0;
}

void pend_sv_handler(void) {
	static int n, d, i, m, q;
	static char led = -1;
	SCB_SCR = 0; // Resume main loop
	if (telreq && !telmode) { // Telemetry request
		sendtelem();
		telreq = 0;
	}
	if (++n < 10) return; // 10kHz -> 1kHz
	adc_trig();
	if (!(tick & 31)) { // Telemetry every 32ms
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
		if (telmode == 1) sendtelem();
	}
	if (led != cfg.led) ledctl(led = cfg.led); // Update LEDs
	if (!cells && volt && m == 100) cells = (volt + 424) / 425; // Assume maximum 4.25V per cell @ 100ms uptime
	i += curr;
	n = 0;
	if (++m < 1000) return; // 1ms -> 1s
	csum = (q += i / 1000) / 360; // mAh
	i = 0;
	m = 0;
}

static void beep(void) {
	static const char *const beacons[] = {"EG", "FA", "GB", "AB#", "aDGE"};
	static const char *const vals[] = {"a4", "C2", "C2D2", "C2D2E2", "C2D2E2F2", "GAGAG2", "GAGAG2F2", "GAGAG2F2E2", "GAGAG2F2E2D2", "GAGAG2F2E2D2C2", 0};
	if (beacon) {
		playmusic(beacons[beacon - 1], cfg.beacon);
		beacon = 0;
	}
	if (beepval < 0) return;
	int i[10], n = 0, x = beepval, vol = cfg.volume;
	while (i[n++] = x % 10, x /= 10);
	if (vol < 25) vol = 25;
	while (n--) {
		if (x++) playmusic("_2", vol);
		playmusic(vals[i[n]], vol);
	}
	beepval = -1;
}

void main(void) {
	memcpy(_cfg_start, _cfg, _cfg_end - _cfg_start); // Copy configuration to SRAM
	checkcfg();

	init();
	initled();
	initio();
	inittelem();

	telmode = cfg.telem_mode;
	cells = cfg.prot_cells;

	TIM1_BDTR = (DEAD_TIME & TIM_BDTR_DTG_MASK) | TIM_BDTR_MOE;
	TIM1_ARR = CLK_KHZ / 16;
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
	STK_RVR = CLK_KHZ / 10 - 1; // 10kHz
	STK_CVR = 0;
	STK_CSR = STK_CSR_ENABLE | STK_CSR_TICKINT | STK_CSR_CLKSOURCE_AHB;

	if (!(RCC_CSR & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF))) { // Power-on
		playmusic(cfg.music, cfg.volume); // Play startup music
		if (cfg.prot_volt) { // Beep the number of battery cells
			while (!cells) { // Wait for autodetection
				WWDG_CR = 0xff;
				__WFI();
			}
			for (int i = 0; i < cells; ++i) playmusic("_2D", cfg.volume);
		}
	}
	if (cfg.arm) {
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
		playmusic("GC", cfg.volume); // Arming beep
	}

	PID curpid = {.Kp = 400, .Ki = 0, .Kd = 600};

	for (int curduty = 0, running = 0, braking = 2, choke = 0, n = 0, v = 0;;) {
		SCB_SCR = SCB_SCR_SLEEPONEXIT; // Suspend main loop
		WWDG_CR = 0xff;
		__WFI();

		int newduty, throtval = throt;
		if (!running) curduty = 0; // Reset duty ramp
		if (throtval > 0) { // Forward throttle
			newduty = scale(throtval, 0, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
			reverse = cfg.revdir ^ flipdir;
			running = 1;
			braking = 0;
		} else if (throtval < 0) { // Reverse throttle
			if (cfg.throt_mode == 2 && braking != 2) { // Proportional brake
				curduty = scale(-throtval, 0, 2000, cfg.duty_drag * 20, 2000);
				running = 0;
				braking = 1;
			} else {
				newduty = scale(-throtval, 0, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
				reverse = !cfg.revdir ^ flipdir;
				running = 1;
			}
		} else { // Neutral throttle
			curduty = cfg.duty_drag * 20;
			running = 0;
			if (braking == 1) braking = 2; // Reverse after braking
		}

		int arr = CLK_KHZ / cfg.freq_min;
		if (running) {
			if ((newduty -= choke) < 0) newduty = 0;
			if (nsyn < 6) { // Power cap during spin-up
				int maxduty = cfg.duty_spup * 20;
				if (newduty > maxduty) newduty = maxduty;
			}
			if (curduty < newduty) { // Acceleration ramping
				curduty += cfg.duty_ramp;
				if (curduty > newduty) curduty = newduty;
			} else curduty = newduty;
			if (erpt) { // Variable PWM frequency
				erpm = 60000000 / erpt;
				arr = scale(erpm, 30000, 60000, arr, CLK_KHZ / cfg.freq_max);
			}
		}

		int ccr = scale(curduty, 0, 2000, running && cfg.damp ? DEAD_TIME : 0, arr);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_UDIS;
		TIM1_ARR = arr;
		TIM1_CCR1 = ccr;
		TIM1_CCR2 = ccr;
		TIM1_CCR3 = ccr;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;

		if (running && !step) { // Start motor
			__disable_irq();
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
			period = 10000;
			erpt = 100000000;
			__enable_irq();
		} else if (!running && step) { // Stop motor
			__disable_irq();
			laststep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
			TIM1_DIER &= ~TIM_DIER_COMIE;
			TIM_DIER(IFTIM) = 0;
			TIM_ARR(IFTIM) = 0;
			TIM_EGR(IFTIM) = TIM_EGR_UG;
			period = 0;
			erpt = 0;
			erpm = 0;
			nsyn = 0;
			memset(&pbuf, 0, sizeof pbuf);
			__enable_irq();
		}

		beep();
		if (++n < 10) continue; // 10kHz -> 1kHz
		if (volt >= cfg.prot_volt * cells * 10) v = 0;
		else if (++v == 3000) reset(); // Low voltage cutoff after 3s
		int t = cfg.prot_temp ? clamp((temp - cfg.prot_temp) * 100, 0, 1500) : 0; // 25% power cap @ 15C above threshold
		int u = cfg.prot_curr ? calcpid(&curpid, curr, cfg.prot_curr * 100) >> 10 : 0; // Current PID control
		choke = clamp(choke + u, t, 2000);
		n = 0;
	}
}
