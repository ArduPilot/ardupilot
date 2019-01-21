#!/bin/bash
set -e
set -x

CWD=$(pwd)
OPT="/opt"

BASE_PKGS="base-devel ccache git gsfonts tk wget"
SITL_PKGS="python2-pip python-pip wxpython opencv python2-numpy python2-scipy"
PX4_PKGS="lib32-glibc zip zlib ncurses cmake"

PYTHON2_PKGS="future lxml pymavlink MAVProxy argparse matplotlib pyparsing"
PYTHON3_PKGS="pyserial empy"

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-6-2017-q2-update"
ARM_TARBALL="$ARM_ROOT-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"

# Ardupilot Tools
ARDUPILOT_TOOLS="ardupilot/Tools/autotest"

function prompt_user() {
      read -p "$1"
      if [[ $REPLY =~ ^[Yy]$ ]]; then
          return 0
      else
          return 1
      fi
}

sudo usermod -a -G uucp $USER

sudo pacman -Sy --noconfirm --needed $BASE_PKGS $SITL_PKGS $PX4_PKGS
sudo pip2 -q install -U $PYTHON2_PKGS
sudo pip3 -q install -U $PYTHON3_PKGS

(
    cd /usr/lib/ccache
    if [ ! -f arm-none-eabi-g++ ]; then
       sudo ln -s /usr/bin/ccache arm-none-eabi-g++
    fi
    if [ ! -f arm-none-eabi-g++ ]; then
        sudo ln -s /usr/bin/ccache arm-none-eabi-gcc
    fi
)

if [ ! -d $OPT/$ARM_ROOT ]; then
    (
        cd $OPT;
        sudo wget $ARM_TARBALL_URL;
        sudo tar xjf ${ARM_TARBALL};
        sudo rm ${ARM_TARBALL};
    )
fi

exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
if ! grep -Fxq "$exportline" ~/.bashrc ; then
    if prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [Y/n]?" ; then
        echo "$exportline" >> ~/.bashrc
        . ~/.bashrc
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
fi

exportline2="export PATH=$CWD/$ARDUPILOT_TOOLS:\$PATH";
if  ! grep -Fxq "$exportline2" ~/.bashrc ; then
    if prompt_user "Add $CWD/$ARDUPILOT_TOOLS to your PATH [Y/n]?" ; then
        echo "$exportline2" >> ~/.bashrc
        . ~/.bashrc
    else
        echo "Skipping adding $CWD/$ARDUPILOT_TOOLS to PATH."
    fi
fi

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
(
    cd $SCRIPT_DIR
    git submodule update --init --recursive
)

echo "Done. Please log out and log in again."
