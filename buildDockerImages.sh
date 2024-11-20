#!/bin/sh
docker build -f Dockerfile-x86_64 -t ihmcrobotics/soem-compile-x86_64:0.1 .
docker build -f Dockerfile-arm64_cross -t ihmcrobotics/soem-compile-arm64_cross:0.1 .
