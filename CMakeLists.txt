cmake_minimum_required(VERSION 3.15)
project(ESCape32 C)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_FLAGS "-Os -g")
set(CMAKE_C_FLAGS_DEBUG "-O0")
set(OBJCOPY arm-none-eabi-objcopy)
if(NOT CMAKE_BUILD_TYPE MATCHES "^(DEBUG|Debug)$") # Enable LTO
	set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
endif()
if(UNIX AND NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT) # Use 'CMAKE_INSTALL_PREFIX' as alternative root
	set(CMAKE_PREFIX_PATH ${CMAKE_INSTALL_PREFIX})
	include(GNUInstallDirs)
	include_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
	link_directories(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
endif()
find_program(FLASH st-flash)
add_compile_options(-ffreestanding -ffunction-sections -fdata-sections -fsingle-precision-constant
	-Wall -Wextra -Wpedantic -Wundef -Wshadow -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes
	-Wno-variadic-macros -Wno-unused-result -Wno-unused-parameter)
add_link_options(-nostartfiles -Wl,--gc-sections)

function(add_target mcu name)
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

include(targets.cmake)
