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
