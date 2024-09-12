#!/usr/bin/env bash
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

function heading() {
    echo "$sep"
    echo $*
    echo "$sep"
}

# Install lsb-release as it is needed to check Ubuntu version
if ! package_is_installed "lsb-release"; then
    heading "Installing lsb-release"
    $APT_GET install lsb-release
    echo "Done!"
fi

# Checking Ubuntu release to adapt software version to install
RELEASE_CODENAME=$(lsb_release -c -s)

# translate Mint-codenames to Ubuntu-codenames based on https://www.linuxmint.com/download_all.php
case ${RELEASE_CODENAME} in
    wilma)
        RELEASE_CODENAME='noble'
        ;;
    vanessa)
        RELEASE_CODENAME='jammy'
        ;;
    una | uma | ulyssa | ulyana | jolnir)
        RELEASE_CODENAME='focal'
        ;;
    tricia | tina | tessa | tara)
        RELEASE_CODENAME='bionic'
        ;;
    elsie)
        RELEASE_CODENAME='bullseye'
        ;;
esac

PYTHON_V="python3"  # starting from ubuntu 20.04, python isn't symlink to default python interpreter
PIP=pip3

if [ ${RELEASE_CODENAME} == 'bionic' ] ; then
    SITLFML_VERSION="2.4"
    SITLCFML_VERSION="2.4"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'buster' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'focal' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'jammy' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'lunar' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'mantic' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'noble' ]; then
    SITLFML_VERSION="2.6"
    SITLCFML_VERSION="2.6"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'groovy' ] ||
         [ ${RELEASE_CODENAME} == 'bullseye' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
else
    # We assume APT based system, so let's try with apt-cache first.
    SITLCFML_VERSION=$(apt-cache search -n '^libcsfml-audio' | cut -d" " -f1 | head -1 | grep -Eo '[+-]?[0-9]+([.][0-9]+)?')
    SITLFML_VERSION=$(apt-cache search -n '^libsfml-audio' | cut -d" " -f1 | head -1 | grep -Eo '[+-]?[0-9]+([.][0-9]+)?')
    # If we cannot retrieve the number with apt-cache, try a last time with dpkg-query
    re='^[+-]?[0-9]+([.][0-9]+)?$'
    if ! [[ $SITLCFML_VERSION =~ $re ]] || ! [[ $SITLFML_VERSION =~ $re ]] ; then
        # Extract the floating point number that is the version of the libcsfml package.
        SITLCFML_VERSION=$(dpkg-query --search libcsfml-audio | cut -d":" -f1 | grep libcsfml-audio | head -1 | grep -Eo '[+-]?[0-9]+([.][0-9]+)?')
        # And same for libsfml-audio.
        SITLFML_VERSION=$(dpkg-query --search libsfml-audio | cut -d":" -f1 | grep libsfml-audio | head -1 | grep -Eo '[+-]?[0-9]+([.][0-9]+)?')
    fi
fi

# Check whether the specific ARM pkg-config package is available or whether we should emulate the effect of installing it.
# Check if we need to manually install libtool-bin
ARM_PKG_CONFIG_NOT_PRESENT=0
if [ -z "$(apt-cache search -n '^pkg-config-arm-linux-gnueabihf')" ]; then
    ARM_PKG_CONFIG_NOT_PRESENT=$(dpkg-query --search pkg-config-arm-linux-gnueabihf |& grep -c "dpkg-query:")
fi
if [ "$ARM_PKG_CONFIG_NOT_PRESENT" -eq 1 ]; then
    INSTALL_PKG_CONFIG=""
    # No need to install Ubuntu's pkg-config-arm-linux-gnueabihf, instead install the base pkg-config.
    $APT_GET install pkg-config
    if [ -f /usr/share/pkg-config-crosswrapper ]; then
        # We are on non-Ubuntu so simulate effect of installing pkg-config-arm-linux-gnueabihf.
        sudo ln -sf /usr/share/pkg-config-crosswrapper /usr/bin/arm-linux-gnueabihf-pkg-config
    else
        echo "Warning: unable to link to pkg-config-crosswrapper"
    fi
else
    # Package is available so install it later.
    INSTALL_PKG_CONFIG="pkg-config-arm-linux-gnueabihf"
fi

# Lists of packages to install
BASE_PKGS="build-essential ccache g++ gawk git make wget valgrind screen python3-pexpect"
PYTHON_PKGS="future lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan"
PYTHON_PKGS="$PYTHON_PKGS flake8 junitparser"

# add some Python packages required for commonly-used MAVProxy modules and hex file generation:
if [[ $SKIP_AP_EXT_ENV -ne 1 ]]; then
    PYTHON_PKGS="$PYTHON_PKGS pygame intelhex"
fi
ARM_LINUX_PKGS="g++-arm-linux-gnueabihf $INSTALL_PKG_CONFIG"
# python-wxgtk packages are added to SITL_PKGS below

if [ ${RELEASE_CODENAME} == 'bookworm' ] ||
   [ ${RELEASE_CODENAME} == 'lunar' ] ||
   [ ${RELEASE_CODENAME} == 'mantic' ] ||
   [ ${RELEASE_CODENAME} == 'noble' ]; then
    # on Lunar (and presumably later releases), we install in venv, below
    PYTHON_PKGS+=" numpy pyparsing psutil"
    SITL_PKGS="python3-dev"
else
SITL_PKGS="libtool libxml2-dev libxslt1-dev ${PYTHON_V}-dev ${PYTHON_V}-pip ${PYTHON_V}-setuptools ${PYTHON_V}-numpy ${PYTHON_V}-pyparsing ${PYTHON_V}-psutil"
fi

# add some packages required for commonly-used MAVProxy modules:
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    if [ ${RELEASE_CODENAME} == 'bookworm' ] ||
       [ ${RELEASE_CODENAME} == 'lunar' ] ||
       [ ${RELEASE_CODENAME} == 'mantic' ] ||
       [ ${RELEASE_CODENAME} == 'noble' ]; then
        PYTHON_PKGS+=" matplotlib scipy opencv-python pyyaml"
        SITL_PKGS+=" xterm libcsfml-dev libcsfml-audio${SITLCFML_VERSION} libcsfml-dev libcsfml-graphics${SITLCFML_VERSION} libcsfml-network${SITLCFML_VERSION} libcsfml-system${SITLCFML_VERSION} libcsfml-window${SITLCFML_VERSION} libsfml-audio${SITLFML_VERSION} libsfml-dev libsfml-graphics${SITLFML_VERSION} libsfml-network${SITLFML_VERSION} libsfml-system${SITLFML_VERSION} libsfml-window${SITLFML_VERSION}"
  else
  SITL_PKGS="$SITL_PKGS xterm ${PYTHON_V}-matplotlib ${PYTHON_V}-serial ${PYTHON_V}-scipy ${PYTHON_V}-opencv libcsfml-dev libcsfml-audio${SITLCFML_VERSION} libcsfml-dev libcsfml-graphics${SITLCFML_VERSION} libcsfml-network${SITLCFML_VERSION} libcsfml-system${SITLCFML_VERSION} libcsfml-window${SITLCFML_VERSION} libsfml-audio${SITLFML_VERSION} libsfml-dev libsfml-graphics${SITLFML_VERSION} libsfml-network${SITLFML_VERSION} libsfml-system${SITLFML_VERSION} libsfml-window${SITLFML_VERSION} ${PYTHON_V}-yaml"
  fi
fi
if [[ $SKIP_AP_COV_ENV -ne 1 ]]; then
  # Coverage utilities
  COVERAGE_PKGS="lcov gcovr"
fi

# ArduPilot official Toolchain for STM32 boards
function install_arm_none_eabi_toolchain() {
    # GNU Tools for ARM Embedded Processors
    # (see https://launchpad.net/gcc-arm-embedded/)
    ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
    case $(uname -m) in
        x86_64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    echo "Installing toolchain for STM32 Boards"
                    echo "Downloading from ArduPilot server"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    echo "Installing..."
                    sudo chmod -R 777 gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    echo "... Cleaning"
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                )
            fi
            echo "Registering STM32 Toolchain for ccache"
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            echo "Done!";;

        aarch64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    echo "Installing toolchain for STM32 Boards"
                    echo "Downloading from ArduPilot server"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    echo "Installing..."
                    sudo chmod -R 777 gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    echo "... Cleaning"
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                )
            fi
            echo "Registering STM32 Toolchain for ccache"
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            echo "Done!";;
    esac
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

heading "Add user to dialout group to allow managing serial ports"
sudo usermod -a -G dialout $USER
echo "Done!"

# Add back python symlink to python interpreter on Ubuntu >= 20.04
if [ ${RELEASE_CODENAME} == 'focal' ];
then
    BASE_PKGS+=" python-is-python3"
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'groovy' ] ||
         [ ${RELEASE_CODENAME} == 'bullseye' ] ||
         [ ${RELEASE_CODENAME} == 'jammy' ]; then
    BASE_PKGS+=" python-is-python3"
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'lunar' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'buster' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} != 'mantic' ] &&
     [ ${RELEASE_CODENAME} != 'noble' ]; then
  SITL_PKGS+=" python-argparse"
fi

# Check for graphical package for MAVProxy
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
  if [ ${RELEASE_CODENAME} == 'bullseye' ] ||
         [ ${RELEASE_CODENAME} == 'buster' ]; then
    SITL_PKGS+=" libjpeg62-turbo-dev"
  elif [ ${RELEASE_CODENAME} == 'groovy' ] ||
           [ ${RELEASE_CODENAME} == 'focal' ]; then
    SITL_PKGS+=" libjpeg8-dev"
  elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
  elif [ ${RELEASE_CODENAME} == 'lunar' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
  elif [ ${RELEASE_CODENAME} == 'mantic' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  elif [ ${RELEASE_CODENAME} == 'noble' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  elif apt-cache search python-wxgtk3.0 | grep wx; then
      SITL_PKGS+=" python-wxgtk3.0"
  elif apt-cache search python3-wxgtk4.0 | grep wx; then
      # see below
      :
  else
      # we only support back to trusty:
      SITL_PKGS+=" python-wxgtk2.8"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libjpeg8-dev libpng12-0 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  fi

  if [ ${RELEASE_CODENAME} == 'bookworm' ]; then
      PYTHON_PKGS+=" opencv-python"
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif [ ${RELEASE_CODENAME} == 'lunar' ]; then
      PYTHON_PKGS+=" wxpython opencv-python"
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif [ ${RELEASE_CODENAME} == 'mantic' ] ||
       [ ${RELEASE_CODENAME} == 'noble' ]; then
      PYTHON_PKGS+=" wxpython opencv-python"
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif [ ${RELEASE_CODENAME} == 'bullseye' ] ||
         [ ${RELEASE_CODENAME} == 'groovy' ] ||
         [ ${RELEASE_CODENAME} == 'buster' ] ||
         [ ${RELEASE_CODENAME} == 'focal' ] ||
         [ ${RELEASE_CODENAME} == 'jammy' ]; then
    SITL_PKGS+=" python3-wxgtk4.0"
    SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
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

SITL_PKGS+=" ppp"

# Install all packages
$APT_GET install $BASE_PKGS $SITL_PKGS $PX4_PKGS $ARM_LINUX_PKGS $COVERAGE_PKGS

heading "Check if we are inside docker environment..."
IS_DOCKER=false
if [[ ${AP_DOCKER_BUILD:-0} -eq 1 ]] || [[ -f /.dockerenv ]] || grep -Eq '(lxc|docker)' /proc/1/cgroup ; then
    IS_DOCKER=true
fi
echo "Done!"

SHELL_LOGIN=".profile"
if $IS_DOCKER; then
    echo "Inside docker, we add the tools path into .bashrc directly"
    SHELL_LOGIN=".ardupilot_env"
    echo "# ArduPilot env file. Need to be loaded by your Shell." > ~/$SHELL_LOGIN
fi

PIP_USER_ARGUMENT="--user"

# create a Python venv on more recent releases:
PYTHON_VENV_PACKAGE=""
if [ ${RELEASE_CODENAME} == 'bookworm' ] ||
   [ ${RELEASE_CODENAME} == 'lunar' ] ||
   [ ${RELEASE_CODENAME} == 'mantic' ]; then
    PYTHON_VENV_PACKAGE=python3.11-venv
elif [ ${RELEASE_CODENAME} == 'noble' ]; then
    PYTHON_VENV_PACKAGE=python3.12-venv
fi

if [ -n "$PYTHON_VENV_PACKAGE" ]; then
    $APT_GET install $PYTHON_VENV_PACKAGE
    python3 -m venv --system-site-packages $HOME/venv-ardupilot

    # activate it:
    SOURCE_LINE="source $HOME/venv-ardupilot/bin/activate"
    $SOURCE_LINE
    PIP_USER_ARGUMENT=""

    if [[ -z "${DO_PYTHON_VENV_ENV}" ]] && maybe_prompt_user "Make ArduPilot venv default for python [N/y]?" ; then
        DO_PYTHON_VENV_ENV=1
    fi

    if [[ $DO_PYTHON_VENV_ENV -eq 1 ]]; then
        echo $SOURCE_LINE >> ~/$SHELL_LOGIN
    fi
fi

# try update packaging, setuptools and wheel before installing pip package that may need compilation
SETUPTOOLS="setuptools"
if [ ${RELEASE_CODENAME} == 'focal' ]; then
    SETUPTOOLS=setuptools==70.3.0
fi
$PIP install $PIP_USER_ARGUMENT -U pip packaging $SETUPTOOLS wheel

if [ "$GITHUB_ACTIONS" == "true" ]; then
    PIP_USER_ARGUMENT+=" --progress-bar off"
fi

if [ ${RELEASE_CODENAME} == 'bookworm' ] ||
   [ ${RELEASE_CODENAME} == 'lunar' ] ||
   [ ${RELEASE_CODENAME} == 'mantic' ] ||
   [ ${RELEASE_CODENAME} == 'noble' ]; then
    # must do this ahead of wxPython pip3 run :-/
    $PIP install $PIP_USER_ARGUMENT -U attrdict3
fi

# install Python packages one-at-a-time so it is clear which package
# is causing problems:
for PACKAGE in $PYTHON_PKGS; do
    $PIP install $PIP_USER_ARGUMENT -U $PACKAGE
done

if [[ -z "${DO_AP_STM_ENV}" ]] && maybe_prompt_user "Install ArduPilot STM32 toolchain [N/y]?" ; then
    DO_AP_STM_ENV=1
fi

heading "Removing modemmanager and brltty package that could conflict with firmware uploading"
if package_is_installed "modemmanager"; then
    $APT_GET remove modemmanager
fi
if package_is_installed "brltty"; then
    $APT_GET remove brltty
fi
echo "Done!"

CCACHE_PATH=$(which ccache)
if [[ $DO_AP_STM_ENV -eq 1 ]]; then
  install_arm_none_eabi_toolchain
fi

heading "Adding ArduPilot Tools to environment"

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

if [[ $DO_AP_STM_ENV -eq 1 ]]; then
exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
grep -Fxq "$exportline" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo $exportline >> ~/$SHELL_LOGIN
        eval $exportline
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
}
fi

exportline2="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline2" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?" ; then
        echo $exportline2 >> ~/$SHELL_LOGIN
        eval $exportline2
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

if [[ $SKIP_AP_COMPLETION_ENV -ne 1 ]]; then
exportline3="source $ARDUPILOT_ROOT/Tools/completion/completion.bash";
grep -Fxq "$exportline3" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]?" ; then
        echo $exportline3 >> ~/.bashrc
        eval $exportline3
    else
        echo "Skipping adding ArduPilot Bash Completion."
    fi
}
fi

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
    heading "Update git submodules"
    cd $ARDUPILOT_ROOT
    git submodule update --init --recursive
    echo "Done!"
  fi
fi

if $IS_DOCKER; then
    echo "Finalizing ArduPilot env for Docker"
    echo "source ~/.ardupilot_env">> ~/.bashrc
fi

echo "---------- $0 end ----------"
