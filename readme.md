# IHMC EtherCAT Master

This package provides a lightweight java wrapper around SOEM (https://github.com/OpenEtherCATsociety/SOEM) and provides an Object Orientated interface to the EtherCAT master.

Extra functionality built on top of SOEM includes 

- Automatic configuration of PDO's
- EtherCAT slave state management
- Thread synchronization with DC Master Clock


## Usage

### Supported operating systems

The IHMC EtherCAT master has a native component that is currently compiled for Linux only.

- Tested on Ubuntu 16.04 and Ubuntu 18.04
- Requires OpenJDK JRE 8 or higher (Compatible JRE's should work).
- Native library is compiled statically, should work on most distributions


### Gradle

Add the IHMC EtherCAT Master as dependency
```
repositories {
    maven {
        url  "http://dl.bintray.com/ihmcrobotics/maven-release"
    }
}
	
dependencies {
	compile group: 'us.ihmc', name: 'ihmc-ethercat-master', version: '0.11.2'
}
```

### Examples

Example code is provided in us.ihmc.etherCAT.examples. 


## Supported slaves

All conforming EtherCAT slaves should work with this wrapper. Slave code has been provided for several devices, including

- Beckhoff EK1100
- Beckhoff EL3314
- Beckhoff EL4134
- Elmo Twitter 

General slave code provided 
- DSP402 statemachine

Custom slave code is straightforward to implement based on provided slave examples.


## Slave information
To show the slaves available on the bus the provided class us.ihmc.etherCAT.master.SlaveInfo can be used. 

## License

Copyright 2016 Florida Institute for Human and Machine Cognition (IHMC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.



## Compiling

This library is split up in two parts, the C library with SWIG wrapper and the actual Java library. They need to be compiled independently. SOEM needs to be compiled with position independent code enabled, otherwise a shared JNI library cannot be build.

### Installing SOEM

The Halodi Robotics PPA has a debian package of SOEM that is compatible with this library. Install it using

```
sudo add-apt-repository ppa:halodirobotics/ppa
sudo apt update
sudo apt install soem
```

#### (Optional, not recommended) Compiling SOEM

~Clone SOEM from https://github.com/OpenEtherCATsociety/SOEM~
- ~git clone https://github.com/OpenEtherCATsociety/SOEM.git~

~Optionally, use switch to the same version as used to compile the maven libraries~
- ~cd SOEM~
- ~git checkout 5b2c51b~

A forked version of SOEM is used to enable some new features for Bootloader support. We're working on merging it in upstream.

```
- git clone https://github.com/Halodi/SOEM.git
- cd SOEM
- git checkout feature/config_init_in_state
```

Note that in order to build a shared JNI library, you have to enable position independent code for SOEM. Use the following commands to build and install the SOEM library

```
cd SOEM
mkdir build
cd build
cmake -DHOST_INSTALL="" -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=true -DCMAKE_BUILD_TYPE=Release ..
make
make install
```

### Compiling C library and SWIG wrapper

#### Requirements
- CMake
- OpenJDK 8
- Swig 3.0.8 or higher.

A gradle wrapper is provided, optionally you can use your system gradle by replacing "./gradlew" with gradle.


- cd ihmc-ethercat-master
- mkdir build
- cd build
- cmake -DCMAKE_BUILD_TYPE=Release ..
- make
- ../gradlew publishToMavenLocal -Ptarget=JAVA
- ../gradlew publishToMavenLocal -Ptarget=PLATFORM

Note that if you want to publish multiple platform libraries you only have to run target=JAVA on a single platform

#### Notes for Ubuntu 14.04

The compiled library support Ubuntu 16.04 and higher. If you want to compile for Ubuntu 14.04, these instructions might help.

Ubuntu 14.04 requires some extra packages that do not ship with it by default, including Java 8 and Swig 3.0.8. To install

- sudo add-apt-repository ppa:openjdk-r/ppa
- sudo apt-get update
- sudo apt-get install openjdk-8-jdk
- sudo update-alternatives --config java
- sudo update-alternatives --config javac

Swig 3.0.8 can be installed using the 16.04 package. It can be downloaded from https://packages.ubuntu.com/xenial/amd64/swig3.0/download





### Compiling Java library
- cd ihmc-soem-wrapper
- ./gradlew jar


