cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(test_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(fix
	fix.c
)

# pico_enable_stdio_usb(blink 1)
pico_enable_stdio_uart(fix 1)

pico_add_extra_outputs(fix)

target_link_libraries(fix pico_stdlib)

# load into and run from sram
pico_set_binary_type(fix no_flash)
