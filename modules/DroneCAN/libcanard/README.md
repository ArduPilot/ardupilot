# Libcanard

Minimal implementation of the DroneCAN protocol stack in C for resource constrained applications.

Get help on the **[DroneCAN Forum](https://dronecan.org/discord)**.

## Usage

If you're not using Git, you can just copy the entire library into your project tree.
If you're using Git, it is recommended to add Libcanard to your project as a Git submodule,
like this:

```bash
git submodule add https://github.com/DroneCAN/libcanard
```

The entire library is contained in three files:

- `canard.c` - the only translation unit; add it to your build or compile it into a separate static library;
- `canard.h` - the API header; include it in your application;
- `canard_internals.h` - internal definitions of the library;
keep this file in the same directory with `canard.c`.

Add `canard.c` to your application build, add `libcanard` directory to the include paths,
and you're ready to roll.

Also you may want to use one of the available drivers for various CAN backends
that are distributed with Libcanard - check out the `drivers/` directory to find out more.

If you wish to override some of the default options, e.g., assert macros' definition,
define the macro `CANARD_ENABLE_CUSTOM_BUILD_CONFIG` as a non-zero value
(e.g. for GCC or Clang: `-DCANARD_ENABLE_CUSTOM_BUILD_CONFIG=1`)
and provide your implementation in a file named `canard_build_config.h`.

Example for Make:

```make
# Adding the library.
INCLUDE += libcanard
CSRC += libcanard/canard.c

# Adding drivers, unless you want to use your own.
# In this example we're using Linux SocketCAN drivers.
INCLUDE += libcanard/drivers/socketcan
CSRC += libcanard/drivers/socketcan/socketcan.c
```

Example for CMake, first installing dependencies. 

```bash
sudo apt-get update && sudo apt-get install gcc-multilib g++-multilib
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest .
```

There is no dedicated documentation for the library API, because it is simple enough to be self-documenting.
Please check out the explanations provided in the comments in the header file to learn the basics.
Most importantly, check out the [examples](examples) directory for fully worked examples

For generation of de-serialisation and serialisation source code, please refer https://github.com/dronecan/dronecan_dsdlc .

## C++ Interface

The C++ interface is in the canard/ directory. See
[examples/ESCNode_C++](examples/ESCNode_C++) for a fully worked example of the C++ API.

## Library Development

This section is intended only for library developers and contributors.

The library design document can be found in [DESIGN.md](DESIGN.md)

### Building and Running Tests

```bash
mkdir build && cd build
cmake ../libcanard/tests    # Adjust path if necessary
make
./run_tests
```
