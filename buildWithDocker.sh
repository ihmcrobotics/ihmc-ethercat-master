#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

mkdir -p buildDocker

if [ ! "$(sudo -u root docker ps -a | grep ethercat-master)" ]; then
    echo "multisense not found. Running new container."
    sudo -u root docker run \
        --tty \
        --interactive \
        --name ethercat-master \
        --network host \
        --dns=1.1.1.1 \
        --volume "$(pwd)/buildDocker":/home/robotlab/dev/ihmc-ethercat-master/build \
        --volume "$(pwd)/src":/home/robotlab/dev/ihmc-ethercat-master/src \
        --volume "$(pwd)/swig":/home/robotlab/dev/ihmc-ethercat-master/swig \
        --volume "$(pwd)/build.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/build.gradle.kts \
        --volume "$(pwd)/gradle.properties":/home/robotlab/dev/ihmc-ethercat-master/gradle.properties \
        --volume "$(pwd)/settings.gradle.kts":/home/robotlab/dev/ihmc-ethercat-master/settings.gradle.kts \
        --volume "$(pwd)/CMakeLists.txt":/home/robotlab/dev/ihmc-ethercat-master/CMakeLists.txt \
        --volume "$(pwd)/build.sh":/home/robotlab/dev/ihmc-ethercat-master/build.sh \
        ihmcrobotics/ethercat-master:0.2
else
    sudo -u root docker start --attach ethercat-master
fi
