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

#pragma once

#if !defined COMP_MAP || COMP_MAP == 123
#define COMP_IN1 1
#define COMP_IN2 2
#define COMP_IN3 3
#elif COMP_MAP == 231
#define COMP_IN1 2
#define COMP_IN2 3
#define COMP_IN3 1
#elif COMP_MAP == 312
#define COMP_IN1 3
#define COMP_IN2 1
#define COMP_IN3 2
#elif COMP_MAP == 132
#define COMP_IN1 1
#define COMP_IN2 3
#define COMP_IN3 2
#elif COMP_MAP == 321
#define COMP_IN1 3
#define COMP_IN2 2
#define COMP_IN3 1
#elif COMP_MAP == 213
#define COMP_IN1 2
#define COMP_IN2 1
#define COMP_IN3 3
#endif

#ifndef SENS_MAP
#define SENS_MAP 0
#define SENS_CNT 0
#define SENS_CHAN 0
#endif

#ifdef LED_WS2812
#define LED_CNT 3
#elif !defined LED_MAP
#define LED_CNT 0
#elif LED_MAP == 0xA15B3B4
#define LED_CNT 3
#define LED1_PORT A
#define LED1_PIN 15
#define LED2_PORT B
#define LED2_PIN 3
#define LED3_PORT B
#define LED3_PIN 4
#elif LED_MAP == 0xA15B5B3
#define LED_CNT 3
#define LED1_PORT A
#define LED1_PIN 15
#define LED2_PORT B
#define LED2_PIN 5
#define LED3_PORT B
#define LED3_PIN 3
#elif LED_MAP == 0xB5B3A15
#define LED_CNT 3
#define LED1_PORT B
#define LED1_PIN 5
#define LED2_PORT B
#define LED2_PIN 3
#define LED3_PORT A
#define LED3_PIN 15
#elif LED_MAP == 0xB8B5B3
#define LED_CNT 3
#define LED1_PORT B
#define LED1_PIN 8
#define LED2_PORT B
#define LED2_PIN 5
#define LED3_PORT B
#define LED3_PIN 3
#endif

#ifdef LED_INV
#define LED1_INV
#define LED2_INV
#define LED3_INV
#endif

#ifndef TEMP_MAX
#define TEMP_MAX 0 // C
#endif
#ifndef VOLT_MUL
#define VOLT_MUL 0 // %/10
#endif
#ifndef CURR_MUL
#define CURR_MUL 0 // mA/mV
#endif
#ifndef CURR_MAX
#define CURR_MAX 0 // A
#endif
