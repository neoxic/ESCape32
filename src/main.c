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

#define REVISION 1

__attribute__((__section__(".cfg")))
struct cfg cfg = {
	.id = 0x32ea,
	.revision = REVISION,
	.target_type = TARGET_TYPE,
	.target_name = TARGET_NAME,
	.arm = 1,               // Wait for zero throttle on startup
	.damp = 1,              // Complementary PWM
	.reverse = 0,           // Reverse motor direction
	.timing = 4,            // Motor timing (3.75*X degrees) [0..7]
	.freq_min = 24,         // Minimum PWM frequency (kHz)
	.freq_max = 48,         // Maximum PWM frequency (kHz)
	.duty_min = 0,          // Minimum duty cycle (%)
	.duty_max = 100,        // Maximum duty cycle (%)
	.duty_lim = 6,          // Acceleration/deceleration speed (0.5*X %/ms)
	.duty_drag = 0,         // Drag brake amount (%)
	.throt_mode = 0,        // Throttle mode (0 - forward, 1 - forward/reverse, 2 - forward/brake/reverse)
	.throt_cal = 1,         // Throttle calibration
	.throt_min = 1000,      // Minimum throttle (us)
	.throt_mid = 1500,      // Middle throttle (us)
	.throt_max = 2000,      // Maximum throttle (us)
	.serial_mode = 0,       // Serial input mode (0 - off, 1 - on, 2 - iBUS, 3 - S.BUS)
	.serial_chid = 0,       // iBUS/S.BUS channel ID
	.music1 = "dfa#",       // Music #1 (startup)
	.music2 = "bgeCgeDgeC", // Music #2
	.music3 = "a2bC#_e",    // Music #3
	.volume1 = 25,          // Music #1 volume (%)
	.volume2 = 50,          // Music #1 volume (%)
	.volume3 = 50,          // Music #1 volume (%)
};

int throt, telem, erpt, erpm;

static int step, reverse, period, pbuf[6], nsyn;

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
	int code = GPIOA_IDR & 7; // A2|A1|A0
	if (code < 1 || code > 6) error(1001); // Invalid Hall sensor code
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
	int m2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	int er = 0;
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

#ifdef INVERTED_HIGH
	er |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef INVERTED_LOW
	er |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	TIM1_CCMR1 = m1;
	TIM1_CCMR2 = m2;
	TIM1_CCER = er;

#ifndef SENSORED
	static int _cc;
	if ((step & 1) ^ reverse) cc |= 4;
	comp_in(_cc);
	_cc = cc;
#endif
}

static void laststep(void) {
	TIM1_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;
	TIM1_CCMR2 = TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_PWM1;
	TIM1_CCER = TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE;
#ifdef INVERTED_HIGH
	TIM1_CCER |= TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P;
#endif
#ifdef INVERTED_LOW
	TIM1_CCER |= TIM_CCER_CC1NP | TIM_CCER_CC2NP | TIM_CCER_CC3NP;
#endif
	step = 0;
#ifndef SENSORED
	comp_in(0);
#endif
}

void tim1_com_isr(void) {
	if (!(TIM1_DIER & TIM_DIER_COMIE)) return;
	TIM1_SR = ~TIM_SR_COMIF;
	pbuf[step - 1] = period;
	if (nsyn >= 6) erpt = (pbuf[0] + pbuf[1] + pbuf[2] + pbuf[3] + pbuf[4] + pbuf[5]) >> 1; // Electrical revolution period (us)
	nextstep();
#ifndef SENSORED
	if (erpm > 200000) {
		TIM1_CCR4 = 72;
		TIM2_CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_ARPE;
	} else if (erpm > 100000) {
		TIM1_CCR4 = 136;
		TIM2_CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_ARPE | TIM_CR1_CKD_CK_INT_MUL_2;
	} else {
		TIM1_CCR4 = 264;
		TIM2_CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_ARPE | TIM_CR1_CKD_CK_INT_MUL_4;
	}
	TIM2_SR = 0; // Clear BEMF events before enabling interrupts
	TIM2_DIER = TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC4IE;
#endif
}

#ifdef SENSORED
void tim2_isr(void) { // Any change on Hall sensor inputs
	if (!(TIM2_DIER & TIM_DIER_CC1IE)) return;
	period = (period * 3 + TIM2_CCR1) >> 2;
	if (nsyn < 6) ++nsyn;
}
#else
void tim1_up_isr(void) { // COMP_OUT off
	if (!(TIM1_DIER & TIM_DIER_UIE)) return;
	TIM1_SR = ~TIM_SR_UIF;
	comp_out(0);
}

void tim1_cc_isr(void) { // COMP_OUT on
	if (!(TIM1_DIER & TIM_DIER_CC4IE)) return;
	TIM1_SR = ~TIM_SR_CC4IF;
	comp_out(1);
}

void tim2_isr(void) { // BEMF zero-crossing
	int sr = TIM2_SR;
	if (sr & TIM_SR_UIF) { // Timeout
		TIM2_SR = ~TIM_SR_UIF;
		goto desync;
	}
	if (!(TIM2_DIER & (TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC4IE))) return;
	int t =
		sr & TIM_SR_CC1IF ? TIM2_CCR1:
		sr & TIM_SR_CC2IF ? TIM2_CCR2:
		TIM2_CCR4;
	if (t < (period >> 1)) { // Possible desync
		if (t < 150 || nsyn < 12) return;
		nsyn = 12;
		return;
	}
	if (t > (period << 1) && nsyn >= 6) { // Possible desync
		if (nsyn < 12) goto desync; // Recurring deviation
		nsyn = 6;
	}
	period = (period * 3 + t) >> 2; // Blend time since last zero-crossing into commutation period
	if (nsyn < 60) { // Check for desync
		if (period < 150) goto desync;
		++nsyn;
	}
	TIM2_CCR3 = (period >> 1) - (period * cfg.timing >> 4); // Commutation delay
	TIM2_EGR = TIM_EGR_UG;
	TIM2_DIER = 0;
	return;
desync:
	TIM2_DIER = TIM_DIER_UIE;
	period = 10000;
	erpt = 100000000;
	nsyn = 0;
}
#endif

void sys_tick_handler(void) { // 10kHz
	SCB_SCR = 0; // Resume main loop
}

void main(void) {
	init();
	loadcfg();

	TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	TIM1_CR2 = TIM_CR2_CCPC | TIM_CR2_CCUS; // Preloaded CC control, commutation event on TRGI
	TIM1_BDTR = (DEAD_TIME & TIM_BDTR_DTG_MASK) | TIM_BDTR_MOE;
	TIM1_SMCR = TIM_SMCR_TS_ITR1; // TRGI=TIM2

	TIM2_PSC = MHZ(PCLK1) / 2 - 1; // 0.5us resolution
	TIM2_ARR = 0;
	TIM2_EGR = TIM_EGR_UG;
	TIM2_CR1 = TIM_CR1_CEN | TIM_CR1_URS | TIM_CR1_ARPE;
#ifdef SENSORED
	TIM2_CR2 = TIM_CR2_TI1S | TIM_CR2_MMS_UPDATE; // TI1=CH1^CH2^CH3, TRGO=UPDATE
	TIM2_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TRC | TIM_CCMR1_IC1F_DTF_DIV_8_N_8;
	TIM2_CCER = TIM_CCER_CC1E; // IC1 on any edge on TI1
#else
	TIM2_CR2 = TIM_CR2_MMS_COMPARE_OC3REF; // TRGO=OC3REF
	TIM2_CCMR1 = TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_IC1F_DTF_DIV_8_N_8 | TIM_CCMR1_CC2S_IN_TI2 | TIM_CCMR1_IC2F_DTF_DIV_8_N_8;
	TIM2_CCMR2 = TIM_CCMR2_CC4S_IN_TI4 | TIM_CCMR2_IC4F_DTF_DIV_8_N_8 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_PWM2; // Inverted PWM on OC3
	TIM2_CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC4E; // ICx on rising edge on TIx (COMP_OUT)
#endif

	STK_RVR = KHZ(PCLK2) / 10 - 1; // 10kHz
	STK_CVR = 0;
	STK_CSR = STK_CSR_ENABLE | STK_CSR_TICKINT | STK_CSR_CLKSOURCE_AHB;

	IWDG_KR = IWDG_KR_UNLOCK;
	IWDG_RLR = 999; // 100ms @ 40kHz LSI
	IWDG_KR = IWDG_KR_RESET;

	initio();
	inittelem();
	if (cfg.arm) { // Wait for zero throttle
		if (!(RCC_CSR & RCC_CSR_WWDGRSTF)) playmusic(cfg.music1, cfg.volume1); // Play startup music on power-on
		throt = 1;
		while (throt) cm_wait_for_interrupt();
		playmusic("_2GC", cfg.volume1); // Arming beep
	}
	int curduty = 0;
	int running = 0;
	int braking = 0;
	IWDG_KR = IWDG_KR_START;
	for (;;) {
		IWDG_KR = IWDG_KR_RESET;
		SCB_SCR = SCB_SCR_SLEEPONEXIT; // Suspend main loop
		cm_wait_for_interrupt();
		int newduty, throtval = throt;
		if (!running) curduty = 0; // Reset duty ramp
		if (throtval > 0) { // Forward throttle
			newduty = scale(throtval, 0, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
			reverse = cfg.reverse;
			running = 1;
			braking = 0;
		} else if (throtval < 0) { // Reverse throttle
			if (cfg.throt_mode == 2 && braking != -1) {
				curduty = scale(-throtval, 0, 2000, cfg.duty_drag * 20, 2000);
				running = 0;
				braking = 1;
			} else {
				newduty = scale(-throtval, 0, 2000, cfg.duty_min * 20, cfg.duty_max * 20);
				reverse = !cfg.reverse;
				running = 1;
			}
		} else { // Neutral throttle
			curduty = cfg.duty_drag * 20;
			running = 0;
			if (braking == 1) braking = -1; // Reverse after braking
		}
		int arr = KHZ(PCLK2) / cfg.freq_min;
		if (running) {
			if (nsyn < 6 && newduty > 300) newduty = 300; // 15% duty cycle cap during spin-up
			if (curduty < newduty) { // Acceleration
				curduty += cfg.duty_lim;
				if (curduty > newduty) curduty = newduty;
			} else if (curduty > newduty) { // Deceleration
				curduty -= cfg.duty_lim;
				if (curduty < newduty) curduty = newduty;
			}
			if (erpt) {
				erpm = 60000000 / erpt;
				arr = scale(erpm, 20000, 40000, arr, KHZ(PCLK2) / cfg.freq_max); // Variable PWM frequency
			}
		}
		int ccr = scale(curduty, 0, 2000, running && cfg.damp ? DEAD_TIME : 0, arr);
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_UDIS | TIM_CR1_ARPE;
		TIM1_ARR = arr;
		TIM1_CCR1 = ccr;
		TIM1_CCR2 = ccr;
		TIM1_CCR3 = ccr;
		TIM1_CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
		if (running && !step) { // Start motor
			cm_disable_interrupts();
			nextstep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
#ifdef SENSORED
			TIM1_DIER = TIM_DIER_COMIE;
			TIM2_DIER = TIM_DIER_CC1IE;
			TIM2_ARR = -1;
#else
			TIM1_DIER = TIM_DIER_UIE | TIM_DIER_CC4IE | TIM_DIER_COMIE;
			TIM2_ARR = 0xffff;
			TIM2_CCR3 = 0xffff;
#endif
			TIM2_EGR = TIM_EGR_UG;
			period = 10000;
			erpt = 100000000;
			cm_enable_interrupts();
		} else if (!running && step) { // Stop motor
			cm_disable_interrupts();
			laststep();
			TIM1_EGR = TIM_EGR_UG | TIM_EGR_COMG;
			TIM1_DIER = 0;
			TIM2_DIER = 0;
			TIM2_ARR = 0;
			TIM2_EGR = TIM_EGR_UG;
			period = 0;
			erpt = 0;
			erpm = 0;
			nsyn = 0;
			memset(&pbuf, 0, sizeof pbuf);
			cm_enable_interrupts();
		}
		if (telem) sendtelem(); // Telemetry request
	}
}
