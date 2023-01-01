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

void initio(void) {
#ifdef IO_PA2
	USART2_BRR = CNT(PCLK1, 38400);
	USART2_CR3 = USART_CR3_HDSEL | USART_CR3_OVRDIS;
	USART2_CR1 = USART_CR1_UE | USART_CR1_RE;
#else
	TIM3_ARR = CNT(PCLK1, 38400) - 1; // Bit time
	TIM3_CCR1 = CNT(PCLK1, 76800); // Half-bit time
	TIM3_EGR = TIM_EGR_UG;
	TIM3_CR1 = TIM_CR1_CEN;
	TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM3_CCMR1 = TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_PWM2 | TIM_CCMR1_CC2S_IN_TI1 | TIM_CCMR1_IC2F_CK_INT_N_8;
	TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
#endif
	TIM2_PSC = PCLK1 / 10000 - 1; // 0.1ms resolution
	TIM2_ARR = 4999; // 500ms I/O timeout
	TIM2_EGR = TIM_EGR_UG;
	TIM2_CR1 = TIM_CR1_CEN | TIM_CR1_URS;
}

#ifdef IO_PA2
int recv(char *buf, int len) {
	TIM2_EGR = TIM_EGR_UG;
	TIM2_SR = 0;
	USART2_ICR = USART_ICR_FECF;
	for (int i = 0; i < len; ++i) {
		while (!(USART2_ISR & USART_ISR_RXNE)) {
			if (TIM2_SR & TIM_SR_UIF) return 0; // Timeout
		}
		buf[i] = USART2_RDR;
		if (USART2_ISR & USART_ISR_FE) return 0; // Data error
		TIM2_EGR = TIM_EGR_UG;
	}
	return 1;
}

void send(const char *buf, int len) {
	USART2_CR1 = USART_CR1_UE | USART_CR1_TE;
	for (int i = 0; i < len; ++i) {
		while (!(USART2_ISR & USART_ISR_TXE));
		USART2_TDR = buf[i];
	}
	while (!(USART2_ISR & USART_ISR_TC));
	USART2_CR1 = USART_CR1_UE | USART_CR1_RE;
}
#else
int recv(char *buf, int len) {
	TIM2_EGR = TIM_EGR_UG;
	TIM2_SR = 0;
	int i = 0, n = 10, b = 0;
	for (;;) {
		int sr = TIM3_SR;
		if (sr & TIM_SR_CC1IF) { // Half-bit time
			TIM3_SR = ~TIM_SR_CC1IF;
			if (n == 10) continue;
			int p = TIM3_IDR & TIM3_PIN; // Signal level
			if (!n++) { // Start bit
				if (p) return 0; // Data error
				b = 0;
				continue;
			}
			if (n < 10) { // Data bit
				if (p) b |= 1 << (n - 2);
				continue;
			}
			if (!p) return 0; // Data error
			buf[i++] = b;
			if (i == len) return 1;
			TIM2_EGR = TIM_EGR_UG;
		}
		if (sr & TIM_SR_CC2IF) { // Falling edge
			TIM3_SR = ~TIM_SR_CC2IF;
			if (n == 10) n = 0;
		}
		if (TIM2_SR & TIM_SR_UIF) return 0; // Timeout
	}
}

void send(const char *buf, int len) {
	TIM3_SMCR = 0;
	TIM3_CCR1 = 0; // Preload high level
	TIM3_EGR = TIM_EGR_UG; // Update registers and trigger UEV
	TIM3_CCER = TIM_CCER_CC1E; // Enable output
	int i = 0, n = 0;
	for (;;) {
		if (!(TIM3_SR & TIM_SR_UIF)) continue;
		TIM3_SR = ~TIM_SR_UIF;
		if (i == len) break;
		int p = 0;
		if (++n == 10) { // Stop bit
			n = 0;
			++i;
		} else if (n == 1 || !(buf[i] & (1 << (n - 2)))) p = -1; // Start bit or data bit Low
		TIM3_CCR1 = p;
	}
	TIM3_SMCR = TIM_SMCR_SMS_RM | TIM_SMCR_TS_TI1F_ED; // Reset on any edge on TI1
	TIM3_CCER = TIM_CCER_CC2E | TIM_CCER_CC2P; // IC2 on falling edge on TI1
	TIM3_CCR1 = CNT(PCLK1, 76800); // Half-bit time
	TIM3_EGR = TIM_EGR_UG;
}
#endif

int recvval(void) {
	char buf[2];
	return recv(buf, 2) && (buf[0] ^ buf[1]) == 0xff ? buf[0] : -1;
}

void sendval(int val) {
	char buf[2] = {val, ~val};
	send(buf, 2);
}

int recvdata(char *buf) {
	int cnt = recvval();
	if (cnt == -1) return -1;
	int len = (cnt + 1) << 2;
	uint32_t crc;
	return recv(buf, len) && recv((char *)&crc, 4) && crc32(buf, len) == crc ? len : -1;
}

void senddata(const char *buf, int len) {
	uint32_t crc = crc32(buf, len);
	sendval((len >> 2) - 1);
	send(buf, len);
	send((char *)&crc, 4);
}
