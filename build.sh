#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

cd build

cmake -DCMAKE_BUILD_TYPE=Release ..
make

gradle publishToMavenLocal -Ptarget=JAVA
gradle publishToMavenLocal -Ptarget=PLATFORM
