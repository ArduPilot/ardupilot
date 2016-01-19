#!/bin/bash
# default configuration for semaphoreci build testing

CCACHE="/usr/bin/ccache"

export SEMAPHORE_CACHE_DIR="$HOME/opt"

# ccache configuration
mkdir -p $HOME/ccache-bin
sudo ln -s CCACHE $HOME/bin/g++-4.8
sudo ln -s CCACHE $HOME/bin/gcc-4.8
sudo ln -s CCACHE $HOME/bin/gcc-4.8-size
sudo ln -s CCACHE $HOME/bin/gcc-4.8-objcopy
sudo ln -s CCACHE $HOME/bin/arm-none-eabi-g++
sudo ln -s CCACHE $HOME/bin/arm-none-eabi-gcc
sudo ln -s CCACHE $HOME/bin/arm-none-eabi-size
sudo ln -s CCACHE $HOME/bin/arm-none-eabi-objcopy
sudo ln -s CCACHE $HOME/bin/arm-linux-gnueabihf-g++
sudo ln -s CCACHE $HOME/bin/arm-linux-gnueabihf-gcc
sudo ln -s CCACHE $HOME/bin/arm-linux-gnueabihf-size
sudo ln -s CCACHE $HOME/bin/arm-linux-gnueabihf-objcopy
