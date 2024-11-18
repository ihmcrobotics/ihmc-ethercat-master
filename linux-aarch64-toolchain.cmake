cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)
set(CMAKE_PROGRAM_PATH /usr/aarch64-linux-gnu/bin)

set(ENV{AARCH64_CROSS} "1")
