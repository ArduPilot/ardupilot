#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ev

PKGS="build-essential gawk ccache genromfs libc6-i386 \
      python-argparse python-empy python-serial zlib1g-dev"

ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"

RPI_ROOT="master"
RPI_TARBALL="$RPI_ROOT.tar.gz"

sudo apt-get -qq -y update
sudo apt-get -qq -y install $PKGS

pushd $HOME

mkdir -p $HOME/opt
pushd $HOME/opt

# PX4 toolchain
compiler=$ARM_ROOT
if [ ! -d "$HOME/opt/$compiler" ]; then
  wget http://firmware.diydrones.com/Tools/PX4-tools/$ARM_TARBALL
  tar -xf $ARM_TARBALL
fi

# RPi/BBB toolchain
compiler="tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64"
if [ ! -d "$HOME/opt/$compiler" ]; then
  wget http://firmware.diydrones.com/Tools/Travis/NavIO/$RPI_TARBALL
  tar -xf $RPI_TARBALL
fi

popd

mkdir -p $HOME/bin

ln -sf /usr/bin/gcc-4.8 $HOME/bin/gcc
ln -sf /usr/bin/g++-4.8 $HOME/bin/g++

exportline="export PATH=$HOME/bin:$HOME/opt/gcc-arm-none-eabi-4_9-2015q3/bin:\
$HOME/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
. ~/.profile

popd

