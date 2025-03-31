MEMORY {
	boot (rx) : ORIGIN = 0x8000000, LENGTH = 4K
	cfg (rx) : ORIGIN = ORIGIN(boot) + LENGTH(boot), LENGTH = 1K
	rom (rx) : ORIGIN = ORIGIN(cfg) + LENGTH(cfg), LENGTH = 64K - LENGTH(boot) - LENGTH(cfg)
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}
