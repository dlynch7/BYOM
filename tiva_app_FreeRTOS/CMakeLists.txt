cmake_minimum_required(VERSION 2.8.9)
project(BYOM_FreeRTOS)

# Toolchain file
include(tm4c123g.cmake)

# Project header files
set(PROJ_HEADER_PATH "inc/")
include_directories(${PROJ_HEADER_PATH})

# Tivaware files
set(TIVAWARE_PATH "$ENV{HOME}/Embedded/tivaware")
include_directories(${TIVAWARE_PATH})

# FreeRTOS files
#set(FREERTOS_PATH "$ENV{HOME}/Embedded/FreeRTOSv10.1.1/FreeRTOS")
#include_directories(${FREERTOS_PATH})

# Source files
#file(GLOB SOURCES "src/*.c")
file(GLOB_RECURSE SOURCES "src/*.c")
add_executable(${CMAKE_PROJECT_NAME}.axf ${SOURCES})

target_link_libraries(${CMAKE_PROJECT_NAME}.axf
	${TIVAWARE_PATH}/usblib/gcc/libusb.a
	${TIVAWARE_PATH}/driverlib/gcc/libdriver.a
)
