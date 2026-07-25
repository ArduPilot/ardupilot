#! /bin/bash

# make a copy of a waf directory with the same name
#
# the tool 'relocation' (waflib/extras) adds some information
# so that a full rebuild is not performed when the dir changes

rm -rf tmp
mkdir tmp

pushd c
waf configure build
popd
cp -R c tmp/c

cd tmp/c
waf configure build

