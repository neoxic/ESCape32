cmake_minimum_required(VERSION 3.15)
project(ESCape32 C)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_FLAGS "-Os -g")
set(CMAKE_C_FLAGS_DEBUG "-Og")
set(OBJCOPY arm-none-eabi-objcopy)
if(UNIX AND NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) # Use 'CMAKE_INSTALL_PREFIX' as alternative root
	set(CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
	include(GNUInstallDirs)
	include_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
	link_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
endif()
find_program(FLASH st-flash)
add_compile_options(-ffreestanding -ffunction-sections -fdata-sections -fsingle-precision-constant
	-Wall -Wextra -Wpedantic -Wundef -Wshadow -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes
	-Wno-variadic-macros -Wno-unused-result -Wno-unused-parameter -Wno-unused-label)
add_link_options(-nostartfiles -Wl,--gc-sections)

function(add_target name mcu)
	include(mcu/${mcu}/config.cmake)
	file(READ src/main.c main_c)
	string(REGEX MATCH "REVISION [0-9]+" rev ${main_c})
	string(REGEX REPLACE "[^0-9]+" "-rev" rev ${rev})
	set(elf ${name}${rev}.elf)
	set(bin ${name}${rev}.bin)
	set(hex ${name}${rev}.hex)
	file(GLOB srcs src/*.c mcu/${mcu}/*.c)
	add_executable(${elf} ${srcs})
	target_include_directories(${elf} PRIVATE src mcu/${mcu})
	target_compile_options(${elf} PRIVATE ${opts})
	target_compile_definitions(${elf} PRIVATE ${mcu} ${name} ${defs} TARGET_NAME="${name}" ${ARGN})
	target_link_options(${elf} PRIVATE -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/${mcu}/config.ld -T${CMAKE_CURRENT_SOURCE_DIR}/mcu/common.ld ${opts})
	target_link_libraries(${elf} c_nano gcc nosys ${libs})
	add_custom_command(
		OUTPUT ${bin} ${hex}
		COMMAND ${OBJCOPY} -O binary ${elf} ${bin}
		COMMAND ${OBJCOPY} -O ihex ${elf} ${hex}
		DEPENDS ${elf}
	)
	add_custom_target(${name} ALL DEPENDS ${hex})
	add_custom_target(flash-${name} COMMAND ${FLASH} --reset --connect-under-reset --format ihex write ${hex} DEPENDS ${hex})
endfunction()

add_subdirectory(boot)

add_target(AIKON1 STM32G071 DEAD_TIME=35 COMP_MAP=213 IO_PA6)
add_target(AIRBOT1 AT32F421 DEAD_TIME=66 COMP_MAP=321 SENS_MAP=0xA3A6 VOLT_MUL=74 CURR_MUL=20 LED_MAP=0xA15B3B4)
add_target(AIRBOT2 STM32F051 DEAD_TIME=26 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=74 IO_PA2)
add_target(EMAX1 STM32F051 DEAD_TIME=26 COMP_MAP=123 IO_PA2)
add_target(ESCAPE1 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA5A4 VOLT_MUL=110 CURR_MUL=40 LED_WS2812 IO_PA2)
add_target(FLYCOLOR1 STM32F051 DEAD_TIME=26 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 LED_MAP=0xB5B4B3 IO_PA2)
add_target(FLYCOLOR2 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=110 LED_MAP=0xB8)
add_target(HAKRC1 STM32F051 DEAD_TIME=26 COMP_MAP=213 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xA15B5B3 LED_INV)
add_target(HAKRC2 AT32F421 DEAD_TIME=66 COMP_MAP=213 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xA15B5B3 LED_INV)
add_target(HGLRC1 STM32F051 DEAD_TIME=26 COMP_MAP=123 SENS_MAP=0xA6 VOLT_MUL=210 IO_PA2)
add_target(IFLIGHT1 STM32F051 DEAD_TIME=26 COMP_MAP=321 SENS_MAP=0xA3A6 VOLT_MUL=110 CURR_MUL=20 LED_MAP=0xB8B5B3 LED_INV)
add_target(IFLIGHT2 STM32G071 DEAD_TIME=35 COMP_MAP=213 SENS_MAP=0xA5A4 VOLT_MUL=110 CURR_MUL=20 LED_WS2812 IO_PA6)
add_target(IFLIGHT3 STM32G071 DEAD_TIME=35 COMP_MAP=132 SENS_MAP=0xA0 VOLT_MUL=110 LED_WS2812)
add_target(NEUTRONRC1 STM32G071 DEAD_TIME=35 COMP_MAP=123 SENS_MAP=0xA6A4 VOLT_MUL=210 CURR_MUL=50 LED_WS2812)
add_target(NEUTRONRC2 AT32F421 DEAD_TIME=66 COMP_MAP=321)
add_target(SKYSTARS1 GD32E230 DEAD_TIME=40 COMP_MAP=321 SENS_MAP=0xA3 VOLT_MUL=110 LED_MAP=0xB5B3A15)
add_target(TMOTOR1 STM32F051 DEAD_TIME=26 COMP_MAP=132 IO_PA2)
add_target(TMOTOR2 STM32F051 DEAD_TIME=26 COMP_MAP=321)
add_target(TMOTOR3 STM32G071 DEAD_TIME=35 COMP_MAP=231 IO_PA6)

# Experimental targets
add_target(_SPIN32 STSPIN32F0 DEAD_TIME=26 ANALOG) # Sensored drive using STEVAL_SPIN3201 board
