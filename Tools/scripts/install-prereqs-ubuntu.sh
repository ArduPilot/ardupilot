#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x

OPT="/opt"
BASE_PKGS="build-essential ccache g++ gawk git make wget"
PYTHON_PKGS="future lxml pymavlink MAVProxy"
PX4_PKGS="python-argparse openocd flex bison libncurses5-dev \
          autoconf texinfo libftdi-dev zlib1g-dev \
          zip genromfs python-empy cmake cmake-data"
ARM_LINUX_PKGS="g++-arm-linux-gnueabihf pkg-config-arm-linux-gnueabihf"
# python-wxgtk packages are added to SITL_PKGS below
SITL_PKGS="libtool libxml2-dev libxslt1-dev python-dev python-pip python-setuptools python-matplotlib python-serial python-scipy python-opencv python-numpy python-pyparsing xterm"
ASSUME_YES=false
QUIET=false

MACHINE_TYPE=$(uname -m)
if [ ${MACHINE_TYPE} == 'x86_64' ]; then
    PX4_PKGS+=" libc6-i386"
else
  echo "no extra pkgs for i386"
fi

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-6-2017-q2-update"
ARM_TARBALL="$ARM_ROOT-linux.tar.bz2"
ARM_TARBALL_URL="http://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"

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
        q)  QUIET=true
            ;;
    esac
done

APT_GET="sudo apt-get"
if $ASSUME_YES; then
    APT_GET="$APT_GET --assume-yes"
fi
if $QUIET; then
    APT_GET="$APT_GET -qq"
fi

# possibly grab a newer cmake for older ubuntu releases
read -r UBUNTU_CODENAME <<<$(lsb_release -c -s)
if [ "$UBUNTU_CODENAME" = "precise" ]; then
    sudo add-apt-repository ppa:george-edison55/precise-backports -y
elif [ "$UBUNTU_CODENAME" = "trusty" ]; then
    sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
fi

sudo usermod -a -G dialout $USER

if dpkg-query -l "modemmanager"; then
    $APT_GET remove modemmanager
fi
$APT_GET update

if apt-cache search python-wxgtk3.0 | grep wx; then
    SITL_PKGS+=" python-wxgtk3.0 libtool-bin"
else
    # we only support back to trusty:
    SITL_PKGS+=" python-wxgtk2.8"
fi

RP=$(apt-cache search -n '^realpath$')
if [ -n "$RP" ]; then
    BASE_PKGS+=" realpath"
fi

$APT_GET install $BASE_PKGS $SITL_PKGS $PX4_PKGS $ARM_LINUX_PKGS
sudo pip2 -q install -U $PYTHON_PKGS

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
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo $exportline >> ~/.profile
        eval $exportline
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
}

exportline2="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline2" ~/.profile 2>/dev/null || {
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?" ; then
        echo $exportline2 >> ~/.profile
        eval $exportline2
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

apt-cache search arm-none-eabi

(
 cd $ARDUPILOT_ROOT
 git submodule update --init --recursive
)
echo "---------- $0 end ----------"
