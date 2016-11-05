#!/bin/bash
# install dependencies for travis build testing

set -e
set -v

CWD=$(pwd)
OPT="$HOME/opt"
echo "PATH=$PATH"

BASE_PKGS="gawk make git arduino-core arduino curl"
SITL_PKGS="g++ python-pip python-matplotlib python-serial python-wxgtk2.8 python-scipy python-opencv python-numpy python-pyparsing ccache python-empy"
AVR_PKGS="gcc-avr binutils-avr avr-libc"
PYTHON_PKGS="pymavlink MAVProxy catkin_pkg"

UBUNTU64_PKGS="libc6-i386"

# Ardupilot Tools
ARDUPILOT_TOOLS="ardupilot/Tools/autotest"

APT_GET="sudo apt-get -qq --assume-yes"

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
$APT_GET update
$APT_GET install $BASE_PKGS $SITL_PKGS $UBUNTU64_PKGS $AVR_PKGS
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

# try to upgrade to g++ 4.8. See https://github.com/travis-ci/travis-ci/issues/1379
(sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test &&
sudo apt-get -qq update &&
sudo apt-get -qq install g++-4.9 &&
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.9 90) || {
    echo "upgrade to gcc 4.9 failed"
}

mkdir -p $OPT

exportline2="export PATH=$CWD/$ARDUPILOT_TOOLS:\$PATH";
echo $exportline2 >> ~/.profile

. ~/.profile
echo $PATH
