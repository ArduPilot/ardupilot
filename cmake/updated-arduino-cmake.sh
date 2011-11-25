#!/bin/bash
git clone git://github.com/jgoppert/arduino-cmake.git tmp
cp -rf tmp/cmake/modules/* modules
cp -rf tmp/cmake/toolchains/* toolchains
rm -rf tmp
