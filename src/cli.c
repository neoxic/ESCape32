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

#define CFG_MAP(XX) \
	XX( 0, val, arm) \
	XX( 1, val, damp) \
	XX( 2, val, revdir) \
	XX( 3, val, brushed) \
	XX( 4, val, timing) \
	XX( 5, val, sine_range) \
	XX( 6, val, sine_power) \
	XX( 7, val, freq_min) \
	XX( 8, val, freq_max) \
	XX( 9, val, duty_min) \
	XX(10, val, duty_max) \
	XX(11, val, duty_spup) \
	XX(12, val, duty_ramp) \
	XX(13, val, duty_rate) \
	XX(14, val, duty_drag) \
	XX(15, val, throt_mode) \
	XX(16, val, throt_set) \
	XX(17, val, throt_cal) \
	XX(18, val, throt_min) \
	XX(19, val, throt_mid) \
	XX(20, val, throt_max) \
	XX(21, val, analog_min) \
	XX(22, val, analog_max) \
	XX(23, val, input_mode) \
	XX(24, val, input_chid) \
	XX(25, val, telem_mode) \
	XX(26, val, telem_phid) \
	XX(27, val, telem_poles) \
	XX(28, val, prot_stall) \
	XX(29, val, prot_temp) \
	XX(30, val, prot_volt) \
	XX(31, val, prot_cells) \
	XX(32, val, prot_curr) \
	XX(33, str, music) \
	XX(34, val, volume) \
	XX(35, val, beacon) \
	XX(36, val, bec) \
	XX(37, val, led) \

static int beep = -1;

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
	appendstr(pos, itoa(val, buf, 10));
}

static void appenddec(char **pos, int val) {
	char buf[12];
	appendstr(pos, itoa(val / 100, buf, 10));
	appendstr(pos, ".");
	for (int i = strlen(itoa(val % 100, buf, 10)); i < 2; ++i) appendstr(pos, "0");
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

#define setbeepstr(str)

static int setbeepval(int val) {
	if (beep < 0 || val < 0) return val;
	beep = beepval = val;
	return val;
}

int execcmd(char *buf) {
	static const char *const cmds[] = {"help", "info", "show", "get", "set", "save", "reset", "play", "throt", "beep", 0};
	static const char *const keys[] = {
#define XX(idx, type, key) #key,
CFG_MAP(XX)
#undef XX
	0};
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
				"save\n"
				"reset\n"
				"play <music> [<volume>]\n"
				"throt <value>\n"
				"beep\n"
			);
			break;
		case 1: // 'info'
			if (narg != 1) goto error;
			appendstr(&pos, "ESCape32 rev");
			appendval(&pos, setbeepval(cfg.revision));
			appendstr(&pos, " [");
			appendstr(&pos, cfg.target_name);
			appendstr(&pos, "]\nTemp: ");
			appendval(&pos, temp);
			appendstr(&pos, "C\nVolt: ");
			appenddec(&pos, volt);
			appendstr(&pos, "V\nCurr: ");
			appenddec(&pos, curr);
			appendstr(&pos, "A\nCsum: ");
			appendval(&pos, csum);
			appendstr(&pos, "mAh\nERPM: ");
			appendval(&pos, erpm);
			appendstr(&pos, "\n");
			break;
		case 2: // 'show'
			if (narg != 1) goto error;
#define XX(idx, type, key) \
			appendpair(&pos, type, key);
CFG_MAP(XX)
#undef XX
			break;
		case 3: // 'get <param>'
			if (narg != 2) goto error;
			switch (getidx(args[1], keys)) {
#define XX(idx, type, key) \
				case idx: \
					appendpair(&pos, type, key); \
					setbeep##type(cfg.key); \
					break;
CFG_MAP(XX)
#undef XX
				default:
					goto error;
			}
			break;
		case 4: // 'set <param> <value>'
			if (narg != 3) goto error;
			switch (getidx(args[1], keys)) {
#define XX(idx, type, key) \
				case idx: \
					set##type(args[2], key); \
					checkcfg(); \
					appendpair(&pos, type, key); \
					setbeep##type(cfg.key); \
					break;
CFG_MAP(XX)
#undef XX
				default:
					goto error;
			}
			break;
		case 5: // 'save'
			if (narg != 1 || !setbeepval(savecfg())) goto error;
			break;
		case 6: // 'reset'
			if (narg != 1 || !setbeepval(resetcfg())) goto error;
			break;
		case 7: // 'play <music> [<volume>]'
			if (narg < 2 || narg > 3) goto error;
			val = cfg.volume;
			if (narg == 3 && (!getval(args[2], &val) || val < 1 || val > 100)) goto error;
			if (!playmusic(args[1], val)) goto error;
			break;
		case 8: // 'throt <value>'
			if (narg != 2 || !getval(args[1], &val) || val < -2000 || val > 2000) goto error;
			throt = val;
			analog = 0;
			break;
		case 9: // 'beep'
			if (narg != 1) goto error;
			if (beep < 0) beep = 0;
			beepval = beep;
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
