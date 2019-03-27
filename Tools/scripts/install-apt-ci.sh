#!/bin/bash
# Install APT packages for CI build testing

set -ex

PKGS=" \
    build-essential \
    gawk \
    genromfs \
    libc6-i386 \
    libxml2-dev \
    libxslt1-dev \
    python-pip \
    python-dev \
    zlib1g-dev \
    gcc-4.9 \
    g++-4.9 \
    cmake3 \
    cmake3-data \
    "

read -r UBUNTU_CODENAME <<<$(lsb_release -c -s)

sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y

#wget -q -O - http://llvm.org/apt/llvm-snapshot.gpg.key | sudo apt-key add -
#sudo add-apt-repository "deb http://llvm.org/apt/${UBUNTU_CODENAME}/ llvm-toolchain-${UBUNTU_CODENAME}-3.7 main" -y
sudo apt-get -qq -y --force-yes update
sudo apt-get -qq -y --force-yes remove clang llvm
sudo apt-get -y --force-yes install $PKGS
