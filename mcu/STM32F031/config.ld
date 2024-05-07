MEMORY {
	boot (rx) : ORIGIN = 0x8000000, LENGTH = 4K
	cfg (rx) : ORIGIN = ORIGIN(boot) + LENGTH(boot), LENGTH = 1K
	rom (rx) : ORIGIN = ORIGIN(cfg) + LENGTH(cfg), LENGTH = 32K - LENGTH(boot) - LENGTH(cfg)
	vec (rx) : ORIGIN = 0x20000000, LENGTH = 192
	ram (rwx) : ORIGIN = ORIGIN(vec) + LENGTH(vec), LENGTH = 4K - LENGTH(vec)
}

_vec = ORIGIN(vec);
