#!/bin/bash
set -e

command -v yaourt >/dev/null 2>&1 || { echo >&2 "Please install yaourt first. Aborting."; exit 1; }

CWD=$(pwd)
OPT="/opt"

BASE_PKGS="wget curl base-devel git-core tk gsfonts"
SITL_PKGS="python2-pip python-pip wxpython2.8 opencv python2-numpy python2-scipy ccache"
PX4_PKGS="lib32-glibc zip zlib ncurses"

PYTHON2_PKGS="pymavlink MAVProxy droneapi argparse matplotlib pyparsing catkin_pkg"
PYTHON3_PKGS="pyserial empy"
ARCH_AUR_PKGS="genromfs"

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL"

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

sudo pacman -S --noconfirm $BASE_PKGS $SITL_PKGS $PX4_PKGS
sudo pip2 -q install $PYTHON2_PKGS
sudo pip3 -q install $PYTHON3_PKGS
yaourt -S --noconfirm $ARCH_AUR_PKGS

(
 cd /usr/lib/ccache
 sudo ln -s /usr/bin/ccache arm-none-eabi-g++
 sudo ln -s /usr/bin/ccache arm-none-eabi-gcc
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

(
 cd ./ardupilot
 git submodule init
 git submodule update
)

echo "Done. Please log out and log in again."
