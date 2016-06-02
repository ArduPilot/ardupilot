#!/bin/bash

set -ex

. ~/.profile

# CXX and CC are exported by default by travis
c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}
unset CXX CC

export GIT_VERSION="ci_test"
export NUTTX_GIT_VERSION="ci_test"
export PX4_GIT_VERSION="ci_test"

waf=modules/waf/waf-light

echo "Configuring waf for SITL..."
cd master
$waf configure --board sitl --enable-benchmarks --check-c-compiler="$c_compiler" --check-cxx-compiler="$cxx_compiler"
$waf clean