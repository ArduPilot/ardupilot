#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ex

PKGS="build-essential gawk ccache arduino-core curl genromfs libc6-i386 \
      python-argparse python-empy python-serial zlib1g-dev gcc-4.9 g++-4.9"

AVR_PKGS="gcc-avr binutils-avr avr-libc"

sudo apt-get -y --force-yes install $PKGS $AVR_PKGS

pushd $HOME

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

pip install --user -U argparse empy pyserial pexpect future lxml

popd
