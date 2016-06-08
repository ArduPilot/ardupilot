#!/bin/bash
# Install APT packages for CI build testing

set -ex

PKGS="build-essential gawk ccache genromfs libc6-i386 \
      python-dev python-pip zlib1g-dev gcc-4.9 g++-4.9 cmake cmake-data" # clang-3.7 llvm-3.7"

read -r UBUNTU_CODENAME <<<$(lsb_release -c -s)

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y

if [ "$UBUNTU_CODENAME" = "precise" ]; then
    sudo add-apt-repository ppa:george-edison55/precise-backports -y
elif [ "$UBUNTU_CODENAME" = "trusty" ]; then
    sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
fi

#wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -
#sudo add-apt-repository "deb http://llvm.org/apt/${UBUNTU_CODENAME}/ llvm-toolchain-${UBUNTU_CODENAME}-3.7 main" -y
sudo apt-get -qq -y --force-yes update
sudo apt-get -qq -y --force-yes remove clang llvm
sudo apt-get -y --force-yes install $PKGS