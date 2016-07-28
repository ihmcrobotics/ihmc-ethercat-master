# IHMC EtherCAT Master

This package provides a lightweight java wrapper around SOEM (https://github.com/OpenEtherCATsociety/SOEM) and provides an Object Orientated interface to the EtherCAT master.

Extra functionality built on top of SOEM includes 

- Automatic configuration of PDO's
- EtherCAT slave state management
- Thread synchronization with DC Master Clock
- Asynchronous SDO communication

## Supported slaves

All conforming EtherCAT slaves should work with this wrapper. Slave code has been provided for several devices, including

- Beckhoff EK1100
- Beckhoff EL3314
- Beckhoff EL4134
- Elmo Twitter 

General slave code provided 
- DSP402 statemachine

Custom slave code is straightforward to implement based on provided slave examples.

## Examples

Example code is provided in us.ihmc.etherCAT.examples. Feel free to use.

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

### Compiling SOEM

Get SOEM

Note that in order to build a shared library, you have to enable position independent code for SOEM. Use the following commands to build and install the SOEM library

- cd SOEM
- mkdir build
- cd build
- cmake -DHOST_INSTALL="" -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=true -DCMAKE_BUILD_TYPE=Release ..
- make
- make install

### Compiling C library and SWIG wrapper

- cd ihmc-soem-wrapper
- mkdir build
- cd build
- cmake -DCMAKE_BUILD_TYPE=Release ..
- make
- gradle publishToMavenLocal -Ptarget=JAVA
- gradle publishToMavenLocal -Ptarget=PLATFORM

Note that if you want to publish multiple platform libraries you only have to run target=JAVA on a single platform

### Compiling Java library
- cd ihmc-soem-wrapper
- gradle jar
