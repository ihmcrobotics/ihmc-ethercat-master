cmake_minimum_required(VERSION 2.8.4)

set(CMAKE_POSITION_INDEPENDENT_CODE true)

include_directories("/usr/include/soem")

add_library("ihmcsoemwrapper" STATIC ethercathelper.c)
target_link_libraries("ihmcsoemwrapper" soem)

add_subdirectory(us/ihmc/soem/generated)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/build.gradle.in  ${PROJECT_BINARY_DIR}/build.gradle COPYONLY)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/settings.gradle.in  ${PROJECT_BINARY_DIR}/settings.gradle COPYONLY)
