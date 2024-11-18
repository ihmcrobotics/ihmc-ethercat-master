cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_POSITION_INDEPENDENT_CODE true)

include_directories("/usr/include/soem")

add_library("ihmcsoemwrapper" STATIC ethercathelper.c)
target_link_libraries("ihmcsoemwrapper" soem)

add_subdirectory(us/ihmc/soem/generated)
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/build.gradle.kts.in  ${PROJECT_BINARY_DIR}/build.gradle.kts COPYONLY)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/settings.gradle.kts.in  ${PROJECT_BINARY_DIR}/settings.gradle.kts COPYONLY)

CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/gradle.properties.in  ${PROJECT_BINARY_DIR}/gradle.properties COPYONLY)
