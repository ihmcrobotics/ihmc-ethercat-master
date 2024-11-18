cmake_minimum_required(VERSION 3.5.1)

find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS}) 
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

include_directories("${PROJECT_SOURCE_DIR}/swig/")
include_directories("/usr/include/soem")

set_property(SOURCE soem.i PROPERTY SWIG_FLAGS "-package" "us.ihmc.soem.generated")

if(UNIX AND NOT APPLE)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" AND NOT DEFINED ENV{AARCH64_CROSS})
        message(STATUS "Using target platform x86_64")
        set(LIBRARY_NAME "soemJava-x86_64")
    elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR DEFINED ENV{AARCH64_CROSS})
        message(STATUS "Using target platform arm64")
        set(LIBRARY_NAME "soemJava-arm64")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif()
else()
    message(FATAL_ERROR "This CMake configuration is only for Linux.")
endif()

# Add SWIG module with the architecture-specific library name
swig_add_module(${LIBRARY_NAME} java soem.i)
swig_link_libraries(${LIBRARY_NAME} ihmcsoemwrapper)
