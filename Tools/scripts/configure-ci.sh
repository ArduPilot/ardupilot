#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ex

# Disable ccache for the configure phase, it's not worth it
export CCACHE_DISABLE="true"

ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"

RPI_ROOT="master"
RPI_TARBALL="$RPI_ROOT.tar.gz"

CCACHE_ROOT="ccache-3.2.5"
CCACHE_TARBALL="$CCACHE_ROOT.tar.bz2"

mkdir -p $HOME/opt
pushd $HOME

# PX4 toolchain
dir=$ARM_ROOT
if [ ! -d "$HOME/opt/$dir" ]; then
  wget http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL
  tar -xf $ARM_TARBALL -C opt
fi

# RPi/BBB toolchain
dir="tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64"
if [ ! -d "$HOME/opt/$dir" ]; then
  wget http://firmware.ardupilot.org/Tools/Travis/NavIO/$RPI_TARBALL
  tar -xf $RPI_TARBALL -C opt $dir
fi

# CCache
dir=$CCACHE_ROOT
if [ ! -d "$HOME/opt/$dir" ]; then
  wget https://www.samba.org/ftp/ccache/$CCACHE_TARBALL
  tar -xf $CCACHE_TARBALL
  pushd $CCACHE_ROOT
  ./configure --prefix="/tmp" --bindir="$HOME/opt/$dir"
  make
  make install
  popd
fi

popd

mkdir -p $HOME/bin

# symlink to compiler versions
ln -s /usr/bin/gcc-4.9 ~/bin/gcc
ln -s /usr/bin/g++-4.9 ~/bin/g++
ln -s /usr/bin/clang-3.7 ~/bin/clang
ln -s /usr/bin/clang++-3.7 ~/bin/clang++
ln -s /usr/bin/llvm-ar-3.7 ~/bin/llvm-ar

mkdir -p $HOME/ccache

# configure ccache
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/g++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/gcc
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-none-eabi-g++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-none-eabi-gcc
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-linux-gnueabihf-g++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/arm-linux-gnueabihf-gcc
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang++
ln -s ~/opt/$CCACHE_ROOT/ccache ~/ccache/clang

exportline="export PATH=$HOME/ccache"
exportline="${exportline}:$HOME/bin"
exportline="${exportline}:$HOME/opt/gcc-arm-none-eabi-4_9-2015q3/bin"
exportline="${exportline}:$HOME/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin"
exportline="${exportline}:$HOME/opt/$CCACHE_ROOT"
exportline="${exportline}:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then
    echo nothing to do;
else
    echo $exportline >> ~/.profile;
fi

. ~/.profile

pip install --user argparse empy pyserial pexpect mavproxy