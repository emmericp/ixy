cmake_minimum_required(VERSION 2.8)
project(ixy)

set(CMAKE_C_STANDARD 11)

add_compile_options(-g -O2 -march=native -fomit-frame-pointer -std=c11
	-D_XOPEN_SOURCE=700
	-D_DEFAULT_SOURCE
	-Wall
	-Wextra
	-Wformat=2
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(SOURCE_COMMON src/pci.c src/memory.c src/stats.c src/device.c src/ixgbe.c)

add_executable(ixy-fwd src/ixy-fwd.c ${SOURCE_COMMON})
