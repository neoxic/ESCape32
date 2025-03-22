SECTIONS {
	.cfg : {
		_cfg_start = .;
		*(.cfg)
		. = ALIGN(8);
		_cfg_end = .;
	} >ram AT >cfg
}

_boot = ORIGIN(boot);
_cfg = ORIGIN(cfg);
_rom = ORIGIN(rom);
_ram = ORIGIN(ram);
_eod = LOADADDR(.data) + SIZEOF(.data);

INCLUDE cortex-m-generic.ld
