_rom = ORIGIN(rom);
_rom_end = ORIGIN(rom) + LENGTH(rom);
_ram_end = ORIGIN(ram) + LENGTH(ram);

INCLUDE cortex-m-generic.ld
