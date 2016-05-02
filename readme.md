## Compiling SOEM

Get SOEM

Note that in order to build a shared library, you have to enable position indepedent code for SOEM. Use the following commands to build and install the SOEM library

- mkdir build
- cmake -DHOST_INSTALL="" -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=true ..
- make
- make install
