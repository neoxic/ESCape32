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

#define CFG_MAP(XX) \
	XX( 0, val, arm) \
	XX( 1, val, damp) \
	XX( 2, val, reverse) \
	XX( 3, val, timing) \
	XX( 4, val, freq_min) \
	XX( 5, val, freq_max) \
	XX( 6, val, duty_min) \
	XX( 7, val, duty_max) \
	XX( 8, val, duty_lim) \
	XX( 9, val, duty_drag) \
	XX(10, val, throt_mode) \
	XX(11, val, throt_cal) \
	XX(12, val, throt_min) \
	XX(13, val, throt_mid) \
	XX(14, val, throt_max) \
	XX(15, val, serial_mode) \
	XX(16, val, serial_chid) \
	XX(17, str, music1) \
	XX(18, str, music2) \
	XX(19, str, music3) \
	XX(20, val, volume1) \
	XX(21, val, volume2) \
	XX(22, val, volume3) \
	XX(23, val, last_error) \

static const char *const cmds[] = {"help", "info", "show", "get", "set", "play", "save", "_throt", "_telem", 0};
static const char *const keys[] = {
#define XX(idx, type, key) #key,
CFG_MAP(XX)
#undef XX
0};

static int split(char *str, char **vec, int len, const char *sep) {
	int idx = 0;
	for (char *val; (val = strsep(&str, sep));) {
		if (!*val) continue;
		vec[idx++] = val;
		if (idx == len) break;
	}
	return idx;
}

static int getidx(const char *str, const char *const vec[]) {
	int idx = 0;
	const char *const *pos = vec;
	for (const char *val; (val = *pos++) && strcasecmp(val, str); ++idx);
	return idx;
}

static int getval(const char *str, int *val) {
	char *end;
	int res = strtol(str, &end, 10);
	if (*end) return 0;
	*val = res;
	return 1;
}

static void appendstr(char **pos, const char *str) {
	*pos = stpcpy(*pos, str);
}

static void appendval(char **pos, int val) {
	char buf[12];
	itoa(val, buf, 10);
	appendstr(pos, buf);
}

#define appendpair(pos, type, key) \
	appendstr(pos, #key); \
	appendstr(pos, ": "); \
	append##type(pos, cfg.key); \
	appendstr(pos, "\n"); \

#define setstr(str, key) strlcpy(cfg.key, str, sizeof cfg.key)
#define setval(str, key) \
	if (!getval(str, &val)) goto error; \
	cfg.key = val; \

int execcmd(char *buf) {
	char *args[10];
	int narg = split(buf, args, 10, " \t\r\n");
	if (!narg) return 0;
	int val;
	char *pos = buf;
	switch (getidx(args[0], cmds)) {
		case 0: // 'help'
			appendstr(&pos,
				"Usage:\n"
				"info\n"
				"show\n"
				"get <param>\n"
				"set <param> <value>\n"
				"play [<music>] [<volume>]\n"
				"save\n"
			);
			break;
		case 1: // 'info'
			if (narg != 1) goto error;
			appendstr(&pos, "ESCape32 rev");
			appendval(&pos, cfg.revision);
			appendstr(&pos, " [");
			appendstr(&pos, cfg.target_name);
			appendstr(&pos, "]\n");
			break;
		case 2: // 'show'
			if (narg != 1) goto error;
#define XX(idx, type, key) \
			appendpair(&pos, type, key);
CFG_MAP(XX)
#undef XX
			break;
		case 3: // 'get <key>'
			if (narg != 2) goto error;
			switch (getidx(args[1], keys)) {
#define XX(idx, type, key) \
				case idx: \
					appendpair(&pos, type, key); \
					break;
CFG_MAP(XX)
#undef XX
				default:
					goto error;
			}
			break;
		case 4: // 'set <key> <val>'
			if (narg != 3) goto error;
			switch (getidx(args[1], keys)) {
#define XX(idx, type, key) \
				case idx: \
					set##type(args[2], key); \
					checkcfg(); \
					appendpair(&pos, type, key); \
					break;
CFG_MAP(XX)
#undef XX
				default:
					goto error;
			}
			break;
		case 5: // 'play [<music>] [<volume>]'
			if (narg < 1 || narg > 3) goto error;
			val = cfg.volume1;
			if (narg == 3 && (!getval(args[2], &val) || val < 1 || val > 100)) goto error;
			if (!playmusic(narg >= 2 ? args[1] : cfg.music1, val)) goto error;
			break;
		case 6: // 'save'
			if (narg != 1 || erpt || !savecfg()) goto error;
			break;
		case 7: // '_throt <val>'
			if (narg != 2 || !getval(args[1], &val) || val < -2000 || val > 2000) goto error;
			throt = val;
			break;
		case 8: // '_telem'
			if (narg != 1) goto error;
			telem = 1;
			break;
		default:
			goto error;
	}
	appendstr(&pos, "OK\n");
	return pos - buf;
error:
	appendstr(&pos, "ERROR\n");
	return pos - buf;
}
