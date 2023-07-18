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

#if DEAD_TIME < 128
#define TIM_DTG DEAD_TIME
#elif DEAD_TIME < 256
#define TIM_DTG (((DEAD_TIME - 128) >> 1) | 0x80)
#elif DEAD_TIME < 512
#define TIM_DTG (((DEAD_TIME - 256) >> 3) | 0xc0)
#elif DEAD_TIME < 1024
#define TIM_DTG (((DEAD_TIME - 512) >> 4) | 0xe0)
#endif

#ifndef COMP_MAP
#elif COMP_MAP == 123
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
#elif LED_MAP == 0xB8
#define LED_CNT 1
#define LED1_PORT B
#define LED1_PIN 8
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
#elif LED_MAP == 0xB5B4B3
#define LED_CNT 3
#define LED1_PORT B
#define LED1_PIN 5
#define LED2_PORT B
#define LED2_PIN 4
#define LED3_PORT B
#define LED3_PIN 3
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

#ifndef VOLT_MUL
#define VOLT_MUL 0 // %/10
#endif
#ifndef CURR_MUL
#define CURR_MUL 0 // mA/mV
#endif
#ifndef ANALOG_MIN
#define ANALOG_MIN 0 // %
#endif
#ifndef ANALOG_MAX
#define ANALOG_MAX 100 // %
#endif

// Default settings

#ifndef ARM
#define ARM 1
#endif
#ifndef DAMP
#define DAMP 1
#endif
#ifndef REVDIR
#define REVDIR 0
#endif
#ifndef TIMING
#define TIMING 4
#endif
#ifndef SINE_RANGE
#define SINE_RANGE 0
#endif
#ifndef SINE_POWER
#define SINE_POWER 8
#endif
#ifndef FREQ_MIN
#define FREQ_MIN 24
#endif
#ifndef FREQ_MAX
#define FREQ_MAX 48
#endif
#ifndef DUTY_MIN
#define DUTY_MIN 1
#endif
#ifndef DUTY_MAX
#define DUTY_MAX 100
#endif
#ifndef DUTY_SPUP
#define DUTY_SPUP 10
#endif
#ifndef DUTY_RAMP
#define DUTY_RAMP 25
#endif
#ifndef DUTY_DRAG
#define DUTY_DRAG 0
#endif
#ifndef THROT_MODE
#define THROT_MODE 0
#endif
#ifndef THROT_CAL
#ifdef USE_HSE
#define THROT_CAL 0
#else
#define THROT_CAL 1
#endif
#endif
#ifndef THROT_MIN
#define THROT_MIN 1000
#endif
#ifndef THROT_MID
#define THROT_MID 1500
#endif
#ifndef THROT_MAX
#define THROT_MAX 2000
#endif
#ifndef INPUT_MODE
#define INPUT_MODE 0
#endif
#ifndef INPUT_CHID
#define INPUT_CHID 0
#endif
#ifndef TELEM_MODE
#define TELEM_MODE 0
#endif
#ifndef TELEM_PHID
#define TELEM_PHID 0
#endif
#ifndef TELEM_POLES
#define TELEM_POLES 14
#endif
#ifndef PROT_TEMP
#define PROT_TEMP 0
#endif
#ifndef PROT_VOLT
#define PROT_VOLT 0
#endif
#ifndef PROT_CELLS
#define PROT_CELLS 0
#endif
#ifndef PROT_CURR
#define PROT_CURR 0
#endif
#ifndef MUSIC
#define MUSIC "dfa#"
#endif
#ifndef VOLUME
#define VOLUME 25
#endif
#ifndef BEACON
#define BEACON 50
#endif
#ifndef LED
#define LED 0
#endif
