cmake_minimum_required(VERSION 2.8)
project(ixy)

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

option(WITH_VFIO "Enable VFIO IOMMU Protection" OFF)

if(WITH_VFIO)
	add_compile_options(-DUSE_VFIO)
endif()


include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(SOURCE_COMMON src/pci.c src/memory.c src/stats.c src/driver/device.c src/driver/ixgbe.c src/driver/virtio.c src/vfio.c)

add_executable(ixy-pktgen src/app/ixy-pktgen.c ${SOURCE_COMMON})
add_executable(ixy-fwd src/app/ixy-fwd.c ${SOURCE_COMMON})
