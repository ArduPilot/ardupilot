#!/bin/bash
git clone git@github.com:jgoppert/arduino-cmake.git tmp
rm -rf modules toolchain
mv tmp/cmake/* .
rm -rf tmp
