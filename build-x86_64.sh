#!/bin/sh
# Build for native platform (probably x86_64)
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
../gradlew publishToMavenLocal -Ptarget=JAVA
../gradlew publishToMavenLocal -Ptarget=PLATFORM
cd ..
