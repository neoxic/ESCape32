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

#define CMD_PROBE  0
#define CMD_INFO   1
#define CMD_READ   2
#define CMD_WRITE  3
#define CMD_UPDATE 4

#define RES_OK    0
#define RES_ERROR 1

void main(void) {
	init();
	initio();
	if (RCC_CSR & RCC_CSR_SFTRSTF) { // Software reset
		RCC_CSR = RCC_CSR_RMVF; // Clear reset flags
		sendval(RES_OK); // ACK after reboot
	}
	for (;;) {
		switch (recvval()) {
			case CMD_PROBE: // Probe bootloader
				sendval(RES_OK);
				break;
			case CMD_INFO: { // Get info
				char buf[32] = {REVISION};
				senddata(buf, sizeof buf);
				break;
			}
			case CMD_READ: { // Read block
				int num = recvval();
				if (num == -1) break;
				int cnt = recvval();
				if (cnt == -1) break;
				senddata(_rom_end + (num << 10), (cnt + 1) << 2);
				break;
			}
			case CMD_WRITE: { // Write block
				int num = recvval();
				if (num == -1) break;
				char buf[1024];
				int len = recvdata(buf);
				if (len == -1) break;
				sendval(writeflash(_rom_end + (num << 10), buf, len) ? RES_OK : RES_ERROR);
				break;
			}
			case CMD_UPDATE: { // Update bootloader
				char *buf = _ram_end; // Use upper SRAM as buffer
				int pos = 0;
				for (int i = 0, n = (_rom_end - _rom) >> 10; i < n; ++i) {
					int len = recvdata(buf + pos);
					if (len == -1) break;
					sendval(RES_OK);
					pos += len;
					if (len < 1024) break; // Last block
				}
				updateflash(_rom, buf, pos); // Update and reboot
				break;
			}
			default: // Pass control to application
				if (*(uint16_t *)_rom_end != 0x32ea) break;
				__asm__("msr msp, %0" :: "g" (*(uint32_t *)(_rom_end + PAGE_SIZE))); // Initialize stack pointer
				(*(void (**)(void))(_rom_end + PAGE_SIZE + 4))(); // Jump to application
				for (;;); // Never return
		}
	}
}
