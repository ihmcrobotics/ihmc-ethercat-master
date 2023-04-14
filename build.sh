#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

cd build

cmake -DCMAKE_BUILD_TYPE=Release ..
make

pwd
ls ..
../gradlew publishToMavenLocal -Ptarget=JAVA
../gradlew publishToMavenLocal -Ptarget=PLATFORM
