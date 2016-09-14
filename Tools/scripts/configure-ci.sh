#!/bin/bash
# Install dependencies and configure the environment for CI build testing

set -ex

# Disable ccache for the configure phase, it's not worth it
export CCACHE_DISABLE="true"

PKGS="build-essential gawk ccache curl genromfs libc6-i386 \
      python-argparse python-empy python-serial zlib1g-dev libmpc-dev libtool \
      flex bison libncurses5-dev libgmp-dev libmpfr-dev autoconf texinfo gcc-multilib"

AVR_PKGS="gcc-avr binutils-avr avr-libc"

ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"

RPI_ROOT="master"
RPI_TARBALL="$RPI_ROOT.tar.gz"

#sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
#sudo apt-get -qq -y update
#sudo apt-get -qq -y install $AVR_PKGS
#sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 90 \
#    --slave /usr/bin/g++ g++ /usr/bin/g++-4.9

#wget https://launchpad.net/ubuntu/+archive/primary/+files/gcc-avr_4.9.2+Atmel3.5.0-1_amd64.deb --no-check-certificate -P avrdeb
#wget https://blueprints.launchpad.net/ubuntu/+archive/primary/+files/binutils-avr_2.25+Atmel3.5.0-2_amd64.deb --no-check-certificate -P avrdeb
#wget https://launchpad.net/ubuntu/+archive/primary/+files/avr-libc_1.8.0+Atmel3.5.0-1_all.deb --no-check-certificate -P avrdeb
#sudo dpkg -i --force-all avrdeb/*.deb

wget http://blog.spitzenpfeil.org/arduino/arduino-1.0.6-linux64.tgz --no-check-certificate
tar zxvf arduino-1.0.6-linux64.tgz
sudo mv arduino-1.0.6 /usr/local/share

#export ARDUINO=$(pwd)"/arduino-1.0.5"
wget http://blog.spitzenpfeil.org/arduino/arduino-1.6.11-linux64.tar.xz --no-check-certificat
tar xvf arduino-1.6.11-linux64.tar.xz
AVR_BIN=$(pwd)"/arduino-1.6.11/hardware/tools/avr/bin"

#wget http://www.multiprecision.org/mpc/download/mpc-0.9.tar.gz
#tar zxvf mpc-0.9.tar.gz
#cd mpc-0.9
#./configure --disable-shared --enable-static --prefix=/tmp/gcc --with-gmp=/tmp/gcc --with-mpfr=/tmp/gcc
#make && make check && make install

pushd $HOME

mkdir -p $HOME/opt
pushd $HOME/opt

# PX4 toolchain
#compiler=$ARM_ROOT
#if [ ! -d "$HOME/opt/$compiler" ]; then
#  wget http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL
#  tar -xf $ARM_TARBALL
#fi

# RPi/BBB toolchain
#compiler="tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64"
#if [ ! -d "$HOME/opt/$compiler" ]; then
#  wget http://firmware.ardupilot.org/Tools/Travis/NavIO/$RPI_TARBALL
#  tar -xf $RPI_TARBALL
#fi

popd

mkdir -p $HOME/bin

# configure ccache
#ln -s /usr/bin/ccache ~/bin/g++
#ln -s /usr/bin/ccache ~/bin/gcc
#ln -s /usr/bin/ccache ~/bin/arm-none-eabi-g++
#ln -s /usr/bin/ccache ~/bin/arm-none-eabi-gcc
#ln -s /usr/bin/ccache ~/bin/arm-linux-gnueabihf-g++
#ln -s /usr/bin/ccache ~/bin/arm-linux-gnueabihf-gcc

exportline="export PATH=$HOME/bin"
exportline="${exportline}:$AVR_BIN"
#exportline="${exportline}:$HOME/opt/gcc-arm-none-eabi-4_9-2015q3/bin:"
#exportline="${exportline}:$HOME/opt/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:"
exportline="${exportline}:\$PATH"

if grep -Fxq "$exportline" ~/.profile; then
    echo nothing to do;
else
    echo $exportline >> ~/.profile;
fi

. ~/.profile

popd

avr-g++ -v