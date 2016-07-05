#!/bin/bash
set -e

OPT="/opt"
BASE_PKGS="gawk make git arduino-core curl"
PYTHON_PKGS="pymavlink MAVProxy droneapi catkin_pkg"
PX4_PKGS="python-serial python-argparse openocd flex bison libncurses5-dev \
          autoconf texinfo build-essential libftdi-dev libtool zlib1g-dev \
          zip genromfs python-empy libc6-i386"
BEBOP_PKGS="g++-arm-linux-gnueabihf"
SITL_PKGS="g++ python-pip python-setuptools python-matplotlib python-serial python-scipy python-opencv python-numpy python-pyparsing ccache realpath"
ASSUME_YES=false

read -r UBUNTU_CODENAME <<<$(lsb_release -c -s)

if [ "$UBUNTU_CODENAME" = "xenial" ]; then
    SITL_PKGS+=" python-wxgtk3.0"
else
    SITL_PKGS+=" python-wxgtk2.8"
fi

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-4_9-2015q3"
ARM_TARBALL="$ARM_ROOT-20150921-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.ardupilot.org/Tools/PX4-tools/$ARM_TARBALL"

# Ardupilot Tools
ARDUPILOT_TOOLS="Tools/autotest"

function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    else
        read -p "$1"
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}


OPTIND=1  # Reset in case getopts has been used previously in the shell.
while getopts "y" opt; do
    case "$opt" in
        \?)
            exit 1
            ;;
        y)  ASSUME_YES=true
            ;;
    esac
done

if $ASSUME_YES; then
    APT_GET="sudo apt-get -qq --assume-yes"
else
    APT_GET="sudo apt-get"
fi

sudo usermod -a -G dialout $USER

$APT_GET remove modemmanager
$APT_GET update
$APT_GET install $BASE_PKGS $SITL_PKGS $PX4_PKGS $BEBOP_PKGS
sudo pip2 -q install $PYTHON_PKGS

if [ ! -d $OPT/$ARM_ROOT ]; then
    (
        cd $OPT;
        sudo wget $ARM_TARBALL_URL;
        sudo tar xjf ${ARM_TARBALL};
        sudo rm ${ARM_TARBALL};
    )
fi

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
grep -Fxq "$exportline" ~/.profile 2>/dev/null || {
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [Y/n]?" ; then
        echo $exportline >> ~/.profile
        eval $exportline
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
}

exportline2="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline2" ~/.profile 2>/dev/null || {
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [Y/n]?" ; then
        echo $exportline2 >> ~/.profile
        eval $exportline2
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

apt-cache search arm-none-eabi

(
 cd $ARDUPILOT_ROOT
 git submodule init
 git submodule update
)
