#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x

if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

OPT="/opt"
# Ardupilot Tools
ARDUPILOT_TOOLS="Tools/autotest"

ASSUME_YES=false
QUIET=false
sep="##############################################"

OPTIND=1  # Reset in case getopts has been used previously in the shell.
while getopts "yq" opt; do
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

# update apt package list
$APT_GET update

function package_is_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -c "ok installed"
}

# Install lsb-release as it is needed to check Ubuntu version
if package_is_installed "lsb-release" -eq 1; then
    echo "$sep"
    echo "Installing lsb-release"
    echo "$sep"
    $APT_GET install lsb-release
    echo "Done!"
fi

# Checking Ubuntu release to adapt software version to install
RELEASE_CODENAME=$(lsb_release -c -s)
PYTHON_V="python"  # starting from ubuntu 20.04, python isn't symlink to default python interpreter
PIP=pip2

if [ ${RELEASE_CODENAME} == 'xenial' ]; then
    SITLFML_VERSION="2.3v5"
    SITLCFML_VERSION="2.3"
elif [ ${RELEASE_CODENAME} == 'disco' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
elif [ ${RELEASE_CODENAME} == 'eoan' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
elif [ ${RELEASE_CODENAME} == 'focal' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'trusty' ]; then
    SITLFML_VERSION="2"
    SITLCFML_VERSION="2"
else
    SITLFML_VERSION="2.4"
    SITLCFML_VERSION="2.4"
fi

# Lists of packages to install
BASE_PKGS="build-essential ccache g++ gawk git make wget cmake"
PYTHON_PKGS="future lxml pymavlink MAVProxy pexpect"
# add some Python packages required for commonly-used MAVProxy modules and hex file generation:
if [[ $SKIP_AP_EXT_ENV -ne 1 ]]; then
  PYTHON_PKGS="$PYTHON_PKGS pygame intelhex"
fi
ARM_LINUX_PKGS="g++-arm-linux-gnueabihf pkg-config-arm-linux-gnueabihf"
# python-wxgtk packages are added to SITL_PKGS below
SITL_PKGS="libtool libxml2-dev libxslt1-dev ${PYTHON_V}-dev ${PYTHON_V}-pip ${PYTHON_V}-setuptools ${PYTHON_V}-numpy ${PYTHON_V}-pyparsing"
# add some packages required for commonly-used MAVProxy modules:
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
  SITL_PKGS="$SITL_PKGS xterm ${PYTHON_V}-matplotlib ${PYTHON_V}-serial ${PYTHON_V}-scipy ${PYTHON_V}-opencv libcsfml-dev libcsfml-audio${SITLCFML_VERSION} libcsfml-dev libcsfml-graphics${SITLCFML_VERSION} libcsfml-network${SITLCFML_VERSION} libcsfml-system${SITLCFML_VERSION} libcsfml-window${SITLCFML_VERSION} libsfml-audio${SITLFML_VERSION} libsfml-dev libsfml-graphics${SITLFML_VERSION} libsfml-network${SITLFML_VERSION} libsfml-system${SITLFML_VERSION} libsfml-window${SITLFML_VERSION} ${PYTHON_V}-yaml"
fi
if [[ $SKIP_AP_COV_ENV -ne 1 ]]; then
  # Coverage utilities
  COVERAGE_PKGS="lcov gcovr"
fi

# ArduPilot official Toolchain for STM32 boards
function install_arm_none_eabi_toolchain() {
  # GNU Tools for ARM Embedded Processors
  # (see https://launchpad.net/gcc-arm-embedded/)
  ARM_ROOT="gcc-arm-none-eabi-6-2017-q2-update"
  ARM_TARBALL="$ARM_ROOT-linux.tar.bz2"
  ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"
  if [ ! -d $OPT/$ARM_ROOT ]; then
    (
        cd $OPT;
        echo "$sep"
        echo "Installing toolchain for STM32 Boards"
        echo "$sep"
        echo "Downloading from ArduPilot server"
        sudo wget $ARM_TARBALL_URL
        echo "Installing..."
        sudo tar xjf ${ARM_TARBALL}
        echo "... Cleaning"
        sudo rm ${ARM_TARBALL};
    )
  fi
  echo "Registering STM32 Toolchain for ccache"
  sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
  sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
  echo "Done!"
}

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

# possibly grab a newer cmake for older ubuntu releases
if [ ${RELEASE_CODENAME} == "precise" ]; then
    sudo add-apt-repository ppa:george-edison55/precise-backports -y
    $APT_GET update
elif [ ${RELEASE_CODENAME} == "trusty" ]; then
    sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
    $APT_GET update
fi

echo "$sep"
echo "Add user to dialout group to allow managing serial ports"
echo "$sep"
sudo usermod -a -G dialout $USER
echo "Done!"

# Add back python symlink to python interpreter on Ubuntu >= 20.04
if [ ${RELEASE_CODENAME} == 'focal' ]; then
    BASE_PKGS+=" python-is-python3"
    SITL_PKGS+=" libpython3-stdlib" # for argparse
else
  SITL_PKGS+=" python-argparse"
fi

# Check for graphical package for MAVProxy
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
  if [ ${RELEASE_CODENAME} == 'focal' ]; then
    SITL_PKGS+=" python3-wxgtk4.0"
    SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libjpeg8-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif apt-cache search python-wxgtk3.0 | grep wx; then
      SITL_PKGS+=" python-wxgtk3.0"
  else
      # we only support back to trusty:
      SITL_PKGS+=" python-wxgtk2.8"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libjpeg8-dev libpng12-0 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  fi
fi

# Check if we need to manually install realpath
RP=$(apt-cache search -n '^realpath$')
if [ -n "$RP" ]; then
    BASE_PKGS+=" realpath"
fi

# Check if we need to manually install libtool-bin
LBTBIN=$(apt-cache search -n '^libtool-bin')
if [ -n "$LBTBIN" ]; then
    SITL_PKGS+=" libtool-bin"
fi

# Install all packages
$APT_GET install $BASE_PKGS $SITL_PKGS $PX4_PKGS $ARM_LINUX_PKGS $COVERAGE_PKGS
$PIP install --user -U $PYTHON_PKGS

if [[ -z "${DO_AP_STM_ENV}" ]] && maybe_prompt_user "Install ArduPilot STM32 toolchain [N/y]?" ; then
    DO_AP_STM_ENV=1
fi

echo "$sep"
echo "Removing modemmanager package that could conflict with firmware uploading"
echo "$sep"
if package_is_installed "modemmanager" -eq 1; then
    $APT_GET remove modemmanager
fi
echo "Done!"

CCACHE_PATH=$(which ccache)
if [[ $DO_AP_STM_ENV -eq 1 ]]; then
  install_arm_none_eabi_toolchain
fi

echo "$sep"
echo "Check if we are inside docker environment..."
echo "$sep"
IS_DOCKER=false
if [[ -f /.dockerenv ]] || grep -Eq '(lxc|docker)' /proc/1/cgroup ; then
    IS_DOCKER=true
fi
echo "Done!"

SHELL_LOGIN=".profile"
if $IS_DOCKER; then
    echo "Inside docker, we add the tools path into .bashrc directly"
    SHELL_LOGIN=".bashrc"
fi

echo "$sep"
echo "Adding ArduPilot Tools to environment"
echo "$sep"

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")
exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
grep -Fxq "$exportline" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo $exportline >> ~/$SHELL_LOGIN
        eval $exportline
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
}

exportline2="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline2" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?" ; then
        echo $exportline2 >> ~/$SHELL_LOGIN
        eval $exportline2
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

exportline3="source $ARDUPILOT_ROOT/Tools/completion/completion.bash";
grep -Fxq "$exportline3" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]?" ; then
        echo $exportline3 >> ~/.bashrc
        eval $exportline3
    else
        echo "Skipping adding ArduPilot Bash Completion."
    fi
}


exportline4="export PATH=/usr/lib/ccache:\$PATH";
grep -Fxq "$exportline4" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Append CCache to your PATH [N/y]?" ; then
        echo $exportline4 >> ~/$SHELL_LOGIN
        eval $exportline4
    else
        echo "Skipping appending CCache to PATH."
    fi
}
echo "Done!"

if [[ $SKIP_AP_GIT_CHECK -ne 1 ]]; then
  if [ -d ".git" ]; then
    echo "$sep"
    echo "Update git submodules"
    echo "$sep"
    cd $ARDUPILOT_ROOT
    git submodule update --init --recursive
    echo "Done!"
  fi
fi
echo "---------- $0 end ----------"
