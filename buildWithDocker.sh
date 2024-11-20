#!/bin/sh
rm -rf buildDocker
mkdir -p buildDocker

echo "Building soemJava x86_64"
sudo docker run \
    --rm \
    --tty \
    --interactive \
    --network host \
    --dns=1.1.1.1 \
    --volume "$(pwd)/buildDocker":/home/robotlab/dev/ihmc-ethercat-master/build \
    --volume "$(pwd)/src":/home/robotlab/dev/ihmc-ethercat-master/src \
    --volume "$(pwd)/swig":/home/robotlab/dev/ihmc-ethercat-master/swig \
    --volume "$(pwd)/build.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/build.gradle.kts \
    --volume "$(pwd)/gradle.properties":/home/robotlab/dev/ihmc-ethercat-master/gradle.properties \
    --volume "$(pwd)/settings.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/settings.gradle.kts \
    --volume "$(pwd)/CMakeLists.txt":/home/robotlab/dev/ihmc-ethercat-master/CMakeLists.txt \
    --volume "$(pwd)/build-x86_64.sh":/home/robotlab/dev/ihmc-ethercat-master/build-x86_64.sh \
    --volume "$(pwd)/gradlew":/home/robotlab/dev/ihmc-ethercat-master/gradlew \
    --volume "$(pwd)/gradle/wrapper":/home/robotlab/dev/ihmc-ethercat-master/gradle/wrapper \
    --volume "$HOME/.m2":/home/robotlab/.m2 \
    ihmcrobotics/soem-compile-x86_64:0.1

sleep 1
rm -rf buildDocker
mkdir -p buildDocker
sleep 1

echo "Building soemJava arm64 (cross compiling)"
sudo docker run \
    --rm \
    --tty \
    --interactive \
    --network host \
    --dns=1.1.1.1 \
    --volume "$(pwd)/buildDocker":/home/robotlab/dev/ihmc-ethercat-master/build \
    --volume "$(pwd)/src":/home/robotlab/dev/ihmc-ethercat-master/src \
    --volume "$(pwd)/swig":/home/robotlab/dev/ihmc-ethercat-master/swig \
    --volume "$(pwd)/build.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/build.gradle.kts \
    --volume "$(pwd)/gradle.properties":/home/robotlab/dev/ihmc-ethercat-master/gradle.properties \
    --volume "$(pwd)/settings.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/settings.gradle.kts \
    --volume "$(pwd)/CMakeLists.txt":/home/robotlab/dev/ihmc-ethercat-master/CMakeLists.txt \
    --volume "$(pwd)/linux-aarch64-toolchain.cmake":/home/robotlab/dev/ihmc-ethercat-master/linux-aarch64-toolchain.cmake \
    --volume "$(pwd)/build-arm64_cross.sh":/home/robotlab/dev/ihmc-ethercat-master/build-arm64_cross.sh \
    --volume "$(pwd)/gradlew":/home/robotlab/dev/ihmc-ethercat-master/gradlew \
    --volume "$(pwd)/gradle/wrapper":/home/robotlab/dev/ihmc-ethercat-master/gradle/wrapper \
    --volume "$HOME/.m2":/home/robotlab/.m2 \
    ihmcrobotics/soem-compile-arm64_cross:0.1
