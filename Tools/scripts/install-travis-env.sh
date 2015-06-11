#!/bin/bash
# install dependencies for travis build testing

set -e
set -v

CWD=$(pwd)
OPT="$HOME/opt"
echo "PATH=$PATH"

BASE_PKGS="gawk make git arduino-core curl"
SITL_PKGS="g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache python-empy"
AVR_PKGS="gcc-avr binutils-avr avr-libc"
PYTHON_PKGS="pymavlink MAVProxy catkin_pkg"
PX4_PKGS="python-serial python-argparse openocd flex bison libncurses5-dev \
          autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev \
          zip genromfs"
UBUNTU64_PKGS="libc6:i386 libgcc1:i386 gcc-4.6-base:i386 libstdc++5:i386 libstdc++6:i386 gcc-multilib"

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-4_7-2014q2"
ARM_TARBALL="$ARM_ROOT-20140408-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.diydrones.com/Tools/PX4-tools/$ARM_TARBALL"

RPI_ROOT="master"
RPI_TARBALL="$RPI_ROOT.tar.gz"
RPI_TARBALL_URL="http://firmware.diydrones.com/Tools/Travis/NavIO/$RPI_TARBALL"

# Ardupilot Tools
ARDUPILOT_TOOLS="ardupilot/Tools/autotest"

APT_GET="sudo apt-get -qq --assume-yes"

$APT_GET update
$APT_GET install $BASE_PKGS $SITL_PKGS $PX4_PKGS $UBUNTU64_PKGS $AVR_PKGS
sudo pip install --upgrade pip || {
    echo "pip upgrade failed"
}
sudo pip install --upgrade setuptools || {
    echo "setuptools upgrade failed"
}
for pkg in $PYTHON_PKGS; do
    echo "Installing $pkg"
    sudo pip -q install $pkg || echo "FAILED INSTALL OF $pkg"
done

# install some extra packages (for later AVR compiler)
rsync -av firmware.diydrones.com::Tools/Travis/*.deb ExtraPackages
sudo dpkg -i ExtraPackages/*.deb || echo "FAILED INSTALL OF EXTRA DEBS"

# try to upgrade to g++ 4.8. See https://github.com/travis-ci/travis-ci/issues/1379
(sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test &&
sudo apt-get -qq update &&
sudo apt-get -qq install g++-4.8 &&
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 90) || {
    echo "upgrade to gcc 4.8 failed"
}


if [ ! -d PX4Firmware ]; then
    git clone https://github.com/diydrones/PX4Firmware.git
fi

if [ ! -d PX4NuttX ]; then
    git clone https://github.com/diydrones/PX4NuttX.git
fi

if [ ! -d uavcan ]; then
    git clone https://github.com/diydrones/uavcan.git
fi

if [ ! -d VRNuttX ]; then
    git clone https://github.com/virtualrobotix/vrbrain_nuttx.git VRNuttX
fi

mkdir -p $OPT

cd $OPT
wget $ARM_TARBALL_URL
tar xjf ${ARM_TARBALL}
rm -f ${ARM_TARBALL}

cd $OPT
wget $RPI_TARBALL_URL
tar xzf ${RPI_TARBALL}
rm -f ${RPI_TARBALL}

exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
echo $exportline >> ~/.profile

exportline2="export PATH=$CWD/$ARDUPILOT_TOOLS:\$PATH";
echo $exportline2 >> ~/.profile

exportline3="export PATH=$OPT/tools-master/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:\$PATH";
echo $exportline3 >> ~/.profile

. ~/.profile
echo $PATH
ls -l $OPT/$ARM_ROOT/bin
$OPT/$ARM_ROOT/bin/arm-none-eabi-gcc --version

echo "Compiler for NavIO"
arm-linux-gnueabihf-gcc --version
