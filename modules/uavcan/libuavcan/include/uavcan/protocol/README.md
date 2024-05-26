High-level protocol logic
=========================

Classes defined in this directory implement some high-level functions of UAVCAN.

Since most applications won't use all of them at the same time, their definitions are provided right in the header
files rather than in source files, in order to not pollute the build outputs with unused code (which is also a
requirement for most safety-critical software).

However, some of the high-level functions that are either always used by the library itself or those that are extremely
likely to be used by the application are defined in .cpp files.

When adding a new class here, please think twice before putting its definition into a .cpp file.
