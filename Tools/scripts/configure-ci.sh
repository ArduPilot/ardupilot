#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ex

PKGS="build-essential gawk ccache arduino-core arduino curl genromfs libc6-i386 \
      python-argparse python-empy python-serial zlib1g-dev gcc-4.9 g++-4.9"

AVR_PKGS="gcc-avr binutils-avr avr-libc"

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
sudo apt-get -qq -y update
sudo apt-get -qq -y install $PKGS $AVR_PKGS
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-4.9

pushd $HOME

mkdir -p $HOME/opt
pushd $HOME/opt

popd

mkdir -p $HOME/bin

# configure ccache
ln -s /usr/bin/ccache ~/bin/g++
ln -s /usr/bin/ccache ~/bin/gcc

exportline="export PATH=$HOME/bin:"
exportline="${exportline}:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then
    echo nothing to do;
else
    echo $exportline >> ~/.profile;
fi

. ~/.profile

popd