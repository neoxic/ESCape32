# Tweaks for '.ramtext' sections
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
add_link_options(-Wl,--no-warn-rwx-segments)

add_target(STM32F0  BOOT_F0_PA2  IO_PA2)
add_target(STM32F0  BOOT_F0_PA6  IO_PA6)
add_target(STM32F0  BOOT_F0_PB4)
add_target(STM32G0  BOOT_G0_PA6  IO_PA6)
add_target(STM32G0  BOOT_G0_PB4)
