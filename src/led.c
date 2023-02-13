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

#define GPIO(port, name) _GPIO(port, name)
#define _GPIO(port, name) __GPIO(port, name)
#define __GPIO(port, name) GPIO##port##_##name

__attribute__((__weak__))
void initled(void) {
#ifdef LED1_PORT
#ifdef LED1_INV
	GPIO(LED1_PORT, ODR) |= 1 << LED1_PIN;
#endif
	GPIO(LED1_PORT, MODER) &= ~(1 << ((LED1_PIN << 1) + 1));
#endif
#ifdef LED2_PORT
#ifdef LED2_INV
	GPIO(LED2_PORT, ODR) |= 1 << LED2_PIN;
#endif
	GPIO(LED2_PORT, MODER) &= ~(1 << ((LED2_PIN << 1) + 1));
#endif
#ifdef LED3_PORT
#ifdef LED3_INV
	GPIO(LED3_PORT, ODR) |= 1 << LED3_PIN;
#endif
	GPIO(LED3_PORT, MODER) &= ~(1 << ((LED3_PIN << 1) + 1));
#endif
#ifdef LED4_PORT
#ifdef LED4_INV
	GPIO(LED4_PORT, ODR) |= 1 << LED4_PIN;
#endif
	GPIO(LED4_PORT, MODER) &= ~(1 << ((LED4_PIN << 1) + 1));
#endif
}

__attribute__((__weak__))
void ledctl(int x) {
#ifdef LED1_PORT
#ifdef LED1_INV
	GPIO(LED1_PORT, BSRR) = x & 1 ? 1 << (LED1_PIN + 16) : 1 << LED1_PIN;
#else
	GPIO(LED1_PORT, BSRR) = x & 1 ? 1 << LED1_PIN : 1 << (LED1_PIN + 16);
#endif
#endif
#ifdef LED2_PORT
#ifdef LED2_INV
	GPIO(LED2_PORT, BSRR) = x & 2 ? 1 << (LED2_PIN + 16) : 1 << LED2_PIN;
#else
	GPIO(LED2_PORT, BSRR) = x & 2 ? 1 << LED2_PIN : 1 << (LED2_PIN + 16);
#endif
#endif
#ifdef LED3_PORT
#ifdef LED3_INV
	GPIO(LED3_PORT, BSRR) = x & 4 ? 1 << (LED3_PIN + 16) : 1 << LED3_PIN;
#else
	GPIO(LED3_PORT, BSRR) = x & 4 ? 1 << LED3_PIN : 1 << (LED3_PIN + 16);
#endif
#endif
#ifdef LED4_PORT
#ifdef LED4_INV
	GPIO(LED4_PORT, BSRR) = x & 8 ? 1 << (LED4_PIN + 16) : 1 << LED4_PIN;
#else
	GPIO(LED4_PORT, BSRR) = x & 8 ? 1 << LED4_PIN : 1 << (LED4_PIN + 16);
#endif
#endif
}
