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
	XX( 0, val, arm,         ARM,         "off;on",                                        -1,    0,    1,  0, 0, 0) \
	XX( 1, val, damp,        DAMP,        "off;on",                                        -1,    0,    1,  0, 0, 0) \
	XX( 2, val, revdir,      REVDIR,      "off;on",                                        -1,    0,    1,  0, 0, 0) \
	XX( 3, val, brushed,     BRUSHED,     "off;on",                                        -1,    0,    1,  0, 0, 0) \
	XX( 4, val, timing,      TIMING,      "",                                               0,    1,   31,  0, cfg.brushed, 0) \
	XX( 5, val, sine_range,  SINE_RANGE,  "%",                                              0,    0,   25,  0, cfg.brushed, 0) \
	XX( 6, val, sine_power,  SINE_POWER,  "%",                                              0,    1,   15,  0, cfg.brushed, 0) \
	XX( 7, val, freq_min,    FREQ_MIN,    "kHz",                                            0,   16,   48,  0, cfg.brushed, 0) \
	XX( 8, val, freq_max,    FREQ_MAX,    "kHz",                                            0,   16,   96,  0, 0, 0) \
	XX( 9, val, duty_min,    DUTY_MIN,    "%",                                              0,    1,  100,  0, 0, 0) \
	XX(10, val, duty_max,    DUTY_MAX,    "%",                                              0,    1,  100,  0, 0, 0) \
	XX(11, val, duty_spup,   DUTY_SPUP,   "%",                                              0,    1,  100,  0, cfg.brushed, 0) \
	XX(12, val, duty_ramp,   DUTY_RAMP,   "kERPM",                                          0,    0,  100,  0, cfg.brushed, 0) \
	XX(13, val, duty_rate,   DUTY_RATE,   "%/ms",                                           1,    1,  100,  1, 0, 0) \
	XX(14, val, duty_drag,   DUTY_DRAG,   "%",                                              0,    0,  100,  0, 0, 0) \
	XX(15, val, duty_lock,   DUTY_LOCK,   "off;soft;hard",                                 -1,    0,    2,  0, cfg.brushed, 0) \
	XX(16, val, throt_mode,  THROT_MODE,  "fwd;fwd/rev;fwd/brk/rev;fwd/brk",               -1,    0,    3,  0, 0, rearm = 1) \
	XX(17, val, throt_rev,   THROT_REV,   "100%;75%;50%;25%",                              -1,    0,    3,  0, 0, 0) \
	XX(18, val, throt_brk,   THROT_BRK,   "%",                                              0,    0,  100,  0, 0, 0) \
	XX(19, val, throt_set,   THROT_SET,   "%",                                              0,    0,  100,  0, cfg.arm, 0) \
	XX(20, val, throt_ztc,   THROT_ZTC,   "off;on",                                        -1,    0,    1,  0, cfg.brushed, 0) \
	XX(21, val, throt_cal,   THROT_CAL,   "off;on",                                        -1,    0,    1,  0, 0, 0) \
	XX(22, val, throt_min,   THROT_MIN,   "us",                                             0,  900, 1900, 10, 0, 0) \
	XX(23, val, throt_mid,   THROT_MID,   "us",                                             0, 1000, 2000, 10, 0, 0) \
	XX(24, val, throt_max,   THROT_MAX,   "us",                                             0, 1100, 2100, 10, 0, 0) \
	XX(25, val, analog_min,  ANALOG_MIN,  "mV",                                             0,    0, 3200, 10, 0, 0) \
	XX(26, val, analog_max,  ANALOG_MAX,  "mV",                                             0,  200, 3400, 10, 0, 0) \
	XX(27, val, input_mode,  INPUT_MODE,  "servo;analog;serial;iBUS;SBUS;CRSF;EXBUS;HoTT", -1,    0,    7,  0, 0, rearm = 1) \
	XX(28, val, input_ch1,   INPUT_CH1,   "",                                               0,    0,   32,  0, 0, rearm = 1) \
	XX(29, val, input_ch2,   INPUT_CH2,   "",                                               0,    0,   32,  0, 0, 0) \
	XX(30, val, telem_mode,  TELEM_MODE,  "KISS;KISS auto;iBUS;S.Port;CRSF;MSB;HoTT",      -1,    0,    6,  0, 0, 0) \
	XX(31, val, telem_phid,  TELEM_PHID,  "",                                               0,    0,   28,  0, 0, 0) \
	XX(32, val, telem_poles, TELEM_POLES, "",                                               0,    2,  100,  0, 0, 0) \
	XX(33, val, telem_volt,  TELEM_VOLT,  "",                                               0,  -80,  160,  0, 0, 0) \
	XX(34, val, telem_curr,  TELEM_CURR,  "",                                               0, -100,  200,  0, 0, 0) \
	XX(35, val, prot_stall,  PROT_STALL,  "ERPM",                                           0,    0, 3500, 10, cfg.brushed, 0) \
	XX(36, val, prot_temp,   PROT_TEMP,   "C",                                              0,    0,  140,  5, 0, 0) \
	XX(37, val, prot_sens,   PROT_SENS,   "ESC;motor;both",                                -1,    0,    2,  0, 0, 0) \
	XX(38, val, prot_volt,   PROT_VOLT,   "V",                                              1,    0,   38,  1, 0, 0) \
	XX(39, val, prot_cells,  PROT_CELLS,  "",                                               0,    0,   24,  0, 0, 0) \
	XX(40, val, prot_curr,   PROT_CURR,   "A",                                              0,    0,  999,  0, 0, 0) \
	XX(41, val, prot_park,   PROT_PARK,   "",                                               0,    0,    4,  0, 0, 0) \
	XX(42, str, music,       MUSIC,,,,,                                                                     0, 0, 0) \
	XX(43, val, volume,      VOLUME,      "%",                                              0,    0,  100,  0, 0, 0) \
	XX(44, val, beacon,      BEACON,      "%",                                              0,    0,  100,  0, 0, 0) \
	XX(45, val, bec,         BEC,         "5.5V;6.5V;7.4V;8.4V;12V",                       -1,    0,    4,  0, 0, 0) \
	XX(46, val, led,         LED,         "",                                               0,    0,   15,  0, 0, 0) \

#define PARAM_CNT 47

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

static int getint(const char *buf, int len) {
	int res = 0;
	while (len--) res <<= 8, res |= *buf++;
	return res;
}

static void appendstr(char **pos, const char *str) {
	*pos = stpcpy(*pos, str);
}

static void appendstr0(char **pos, const char *str) {
	*pos = stpcpy(*pos, str) + 1;
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

static void appendint(char **pos, int val, int len) {
	char *buf = *pos;
	*pos += len;
	while (len--) buf[len] = val, val >>= 8;
}

static void appendbyte(char **pos, int val) {
	*(*pos)++ = val;
}

#define appendpair(type, key) \
	appendstr(&pos, #key); \
	appendstr(&pos, ": "); \
	append##type(&pos, cfg.key); \
	appendstr(&pos, "\n"); \

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

int execcmd(char *str) {
	static const char *const cmds[] = {"help", "info", "show", "get", "set", "save", "reset", "play", "throt", "beep", 0};
	static const char *const keys[] = {
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) #key,
CFG_MAP(XX)
#undef XX
	0};
	char *args[10];
	int narg = split(str, args, 10, " \t\r\n");
	if (!narg) return 0;
	int val;
	char *pos = str;
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
			appendstr(&pos, ".");
			appendval(&pos, cfg.revpatch);
			appendstr(&pos, " [");
			appendstr(&pos, cfg.name);
			appendstr(&pos, "]\nTemp: ");
			appendval(&pos, temp1);
			if (temp2) {
				appendstr(&pos, "C, ext ");
				appendval(&pos, temp2);
			}
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
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) \
			appendpair(type, key);
CFG_MAP(XX)
#undef XX
			break;
		case 3: // 'get <param>'
			if (narg != 2) goto error;
			switch (getidx(args[1], keys)) {
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) \
				case idx: \
					appendpair(type, key); \
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
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) \
				case idx: \
					set##type(args[2], key); \
					checkcfg(); \
					appendpair(type, key); \
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
	return pos - str;
error:
	appendstr(&pos, "ERROR\n");
	return pos - str;
}

#define appendcrsfstr(key, def, opt, dec, min, max, step, exp1, exp2) \
	appendbyte(&pos, exp1 ? 0x8a : 0x0a); \
	appendstr0(&pos, #key); \
	appendstr0(&pos, cfg.key); \
	appendbyte(&pos, sizeof cfg.key - 1); \

#define appendcrsfval(key, def, opt, dec, min, max, step, exp1, exp2) \
	if (dec < 0) { \
		appendbyte(&pos, exp1 ? 0x89 : 0x09); \
		appendstr0(&pos, #key); \
		appendstr0(&pos, opt); \
		appendbyte(&pos, cfg.key); \
		appendbyte(&pos, min); \
		appendbyte(&pos, max); \
		appendbyte(&pos, def); \
		appendbyte(&pos, '\0'); \
	} else if (dec > 0 || step > 0) { \
		appendbyte(&pos, exp1 ? 0x88 : 0x08); \
		appendstr0(&pos, #key); \
		appendint(&pos, cfg.key, 4); \
		appendint(&pos, min, 4); \
		appendint(&pos, max, 4); \
		appendint(&pos, def, 4); \
		appendbyte(&pos, dec); \
		appendint(&pos, step, 4); \
		appendstr0(&pos, opt); \
	} else if (min >= -0x80 && max <= 0x7f) { \
		appendbyte(&pos, exp1 ? 0x81 : 0x01); \
		appendstr0(&pos, #key); \
		appendbyte(&pos, cfg.key); \
		appendbyte(&pos, min); \
		appendbyte(&pos, max); \
		appendbyte(&pos, def); \
		appendstr0(&pos, opt); \
	} else if (min >= -0x8000 && max <= 0x7fff) { \
		appendbyte(&pos, exp1 ? 0x83 : 0x03); \
		appendstr0(&pos, #key); \
		appendint(&pos, cfg.key, 2); \
		appendint(&pos, min, 2); \
		appendint(&pos, max, 2); \
		appendint(&pos, def, 2); \
		appendstr0(&pos, opt); \
	} else { \
		appendbyte(&pos, exp1 ? 0x85 : 0x05); \
		appendstr0(&pos, #key); \
		appendint(&pos, cfg.key, 4); \
		appendint(&pos, min, 4); \
		appendint(&pos, max, 4); \
		appendint(&pos, def, 4); \
		appendstr0(&pos, opt); \
	} \

#define setcrsfstr(key, buf, len) \
	strlcpy(cfg.key, buf, min(len, sizeof cfg.key)); \
	appendstr0(&pos, cfg.key); \

#define setcrsfval(key, buf, len) \
	cfg.key = getint(buf, len); \
	checkcfg(); \
	appendint(&pos, cfg.key, len); \

static void processcrsfcmd(char **pos, const char *name, const char *buf, int len, char *res, int (*func)(void), const char *msg) {
	const char *str = "";
	int val = 0;
	if (res) {
		res[0] = 0x2b;
		res[4] = 0;
		res[5] = 0;
		*pos = res + 6;
		switch (getint(buf, len)) {
			case 1:
				if (msg) {
					str = msg;
					val = 3;
					break;
				} // Fall through
			case 4:
				str = func() ? "Done!" : "Failed!";
				break;
		}
	}
	appendbyte(pos, 0x0d);
	appendstr0(pos, name);
	appendbyte(pos, val);
	appendbyte(pos, 0);
	appendstr0(pos, str);
}

int execcrsfcmd(const char *buf, int len, char *res) {
	if (len < 3) return 0;
	int a = buf[1];
	int b = 0x90 + telphid - 1;
	if ((a && a != b) || !(res[1] = buf[2])) return 0;
	res[2] = b;
	char *pos = res;
	switch (buf[0]) {
		case 0x28: // Ping
			if (len < 3) break;
			res[0] = 0x29;
			pos += 3;
			appendstr(&pos, "ESCape32 ID:");
			appendval(&pos, telphid);
			memset(pos + 1, 0, 12);
			pos[13] = PARAM_CNT + 3;
			pos[14] = 0;
			pos += 15;
			break;
		case 0x2c: // Read
			if (len < 5) break;
			res[0] = 0x2b;
			res[5] = 0;
			pos += 6;
			switch (res[3] = buf[3]) {
				case 0: // Root folder
					appendbyte(&pos, 0x0b);
					appendstr0(&pos, "ROOT");
					for (a = 0; a < PARAM_CNT + 3; appendbyte(&pos, ++a));
					appendbyte(&pos, 0xff);
					break;
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) \
				case idx + 1: \
					appendcrsf##type(key, def, opt, dec, min, max, step, exp1, exp2); \
					break;
CFG_MAP(XX)
#undef XX
				case PARAM_CNT + 1: // Save
					processcrsfcmd(&pos, "Save", 0, 0, 0, 0, 0);
					break;
				case PARAM_CNT + 2: // Reset
					processcrsfcmd(&pos, "Reset", 0, 0, 0, 0, 0);
					break;
				case PARAM_CNT + 3: // Info
					appendbyte(&pos, 0x0c);
					appendstr0(&pos, cfg.name);
					appendstr(&pos, "rev");
					appendval(&pos, cfg.revision);
					appendstr(&pos, ".");
					appendval(&pos, cfg.revpatch);
					++pos;
					break;
				default:
					return 0;
			}
			for (a = pos - res - 5, b = 0; a > 56; a -= 56, ++b);
			if ((a = buf[4]) > b) return 0;
			res[4] = b - a;
			if (!a) break;
			int ofs = a * 56;
			char *dst = res + 5;
			char *src = dst + ofs;
			memmove(dst, src, min(pos - src, 56));
			pos -= ofs;
			break;
		case 0x2d: // Write
			if (len < 5) break;
			res[0] = 0x2d;
			pos += 4;
			switch (res[3] = buf[3]) {
#define XX(idx, type, key, def, opt, dec, min, max, step, exp1, exp2) \
				case idx + 1: \
					setcrsf##type(key, buf + 4, len - 4); \
					(void)(exp2); \
					break;
CFG_MAP(XX)
#undef XX
				case PARAM_CNT + 1: // Save
					processcrsfcmd(&pos, "Save", buf + 4, len - 4, res, savecfg, 0);
					break;
				case PARAM_CNT + 2: // Reset
					processcrsfcmd(&pos, "Reset", buf + 4, len - 4, res, resetcfg, "Reset to defaults?");
					break;
				default:
					return 0;
			}
			break;
	}
	return pos - res;
}
