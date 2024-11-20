#!/bin/sh
export AARCH64_CROSS=1

# Build for arm64
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=../linux-aarch64-toolchain.cmake ..
make
../gradlew publishToMavenLocal -Ptarget=JAVA
../gradlew publishToMavenLocal -Ptarget=PLATFORM
cd ..
