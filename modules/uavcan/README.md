DroneCAN stack in C++
=====================

Portable reference implementation of the [DroneCAN protocol stack](http://dronecan.org) in C++ for embedded systems
and Linux.

DroneCAN is a lightweight protocol designed for reliable communication in aerospace and robotic applications via CAN bus.

## Documentation

* [DroneCAN website](http://dronecan.org)
* [DroneCAN forum](https://dronecan.org/discord)

## Library usage

### Cloning the repository

```bash
git clone https://github.com/DroneCAN/libuavcan
cd libuavcan
git submodule update --init
```

If this repository is used as a git submodule in your project, make sure to use `--recursive` when updating it.

### Using in a Linux application

Libuavcan can be built as a static library and installed on the system globally as shown below.

```bash
mkdir build
cd build
cmake .. # Default build type is RelWithDebInfo, which can be overriden if needed.
make -j8
sudo make install
```

The following components will be installed:

* Libuavcan headers and the static library
* Generated DSDL headers
* Libuavcan DSDL compiler (a Python script named `libuavcan_dsdlc`)
* Libuavcan DSDL compiler's support library (a Python package named `libuavcan_dsdl_compiler`)

Note that Pyuavcan (an implementation of DroneCAN in Python) will not be installed.
You will need to install it separately if you intend to use the Libuavcan's DSDL compiler in your applications.

It is also possible to use the library as a submodule rather than installing it system-wide.
Please refer to the example applications supplied with the Linux platform driver for more information.

### Using with an embedded system

For ARM targets, it is recommended to use [GCC ARM Embedded](https://launchpad.net/gcc-arm-embedded);
however, any other standard-compliant C++ compiler should also work.

## Library development

Despite the fact that the library itself can be used on virtually any platform that has a standard-compliant
C++11 compiler, the library development process assumes that the host OS is Linux.

Prerequisites:

* Google test library for C++ - gtest (dowloaded as part of the build from [github](https://github.com/google/googletest))
* C++11 capable compiler with GCC-like interface (e.g. GCC, Clang)
* CMake 2.8+
* Optional: static analysis tool for C++ - cppcheck (on Debian/Ubuntu use package `cppcheck`)

Building the debug version and running the unit tests:
```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j8
make ARGS=-VV test
```

Test outputs can be found in the build directory under `libuavcan`.

> Note that unit tests suffixed with "_RealTime" must be executed in real time, otherwise they may produce false warnings;
this implies that they will likely fail if ran on a virtual machine or on a highly loaded system.

### Vagrant
Vagrant can be used to setup a compatible Ubuntu virtual image. Follow the instructions on [Vagrantup](https://www.vagrantup.com/) to install virtualbox and vagrant then do:

```bash
vagrant up
vagrant ssh
mkdir build
cd build
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Debug -DCONTINUOUS_INTEGRATION_BUILD=1
```

> Note that -DCONTINUOUS_INTEGRATION_BUILD=1 is required for this build as the realtime unit tests will not work on a virt.

You can build using commands like:

```bash
vagrant ssh -c "cd /vagrant/build && make -j4 && make test"
```

or to run a single test:

```bash
vagrant ssh -c "cd /vagrant/build && make libuavcan_test && ./libuavcan/libuavcan_test --gtest_filter=Node.Basic"
```

### Developing with Eclipse

An Eclipse project can be generated like that:

```bash
cmake ../../libuavcan -G"Eclipse CDT4 - Unix Makefiles" \
                      -DCMAKE_ECLIPSE_VERSION=4.3 \
                      -DCMAKE_BUILD_TYPE=Debug \
                      -DCMAKE_CXX_COMPILER_ARG1=-std=c++11
```

Path `../../libuavcan` in the command above points at the directory where the top-level `CMakeLists.txt` is located;
you may need to adjust this per your environment.
Note that the directory where Eclipse project is generated must not be a descendant of the source directory.

