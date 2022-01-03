cmake_minimum_required(VERSION 2.8)
project(ixy C)

set(CMAKE_C_STANDARD 11)

add_compile_options(-g -O2 -march=native -fomit-frame-pointer -std=c11
	-D_XOPEN_SOURCE=700
	-D_DEFAULT_SOURCE
	-Wall
	-Wextra
	-Wno-unused-parameter
	-Wno-unused-function
	-Wformat=2
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(SOURCE_COMMON src/pci.c src/memory.c src/stats.c src/interrupts.c src/driver/device.c  src/driver/e1000.c src/driver/ixgbe.c src/driver/virtio.c src/libixy-vfio.c)

add_executable(ixy-pktgen src/app/ixy-pktgen.c ${SOURCE_COMMON})
add_executable(ixy-fwd src/app/ixy-fwd.c ${SOURCE_COMMON})
add_executable(ixy-pcap src/app/ixy-pcap.c ${SOURCE_COMMON})
add_library(ixy-vfio SHARED src/libixy-vfio.c)
