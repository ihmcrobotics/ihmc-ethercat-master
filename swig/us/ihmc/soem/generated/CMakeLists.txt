cmake_minimum_required(VERSION 2.8)

find_package(JNI REQUIRED)
include_directories(${JNI_INCLUDE_DIRS}) 
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/swig/")
INCLUDE_DIRECTORIES("/usr/include/soem")

SET_PROPERTY(SOURCE soem.i PROPERTY SWIG_FLAGS "-package" "us.ihmc.soem.generated")

SWIG_ADD_MODULE(soemJava java soem.i)
swig_link_libraries(soemJava ihmcsoemwrapper)
