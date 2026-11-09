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

RELEASE_DISTRIBUTOR=$(lsb_release -i -s | tr '[:upper:]' '[:lower:]')
case ${RELEASE_DISTRIBUTOR} in
    elementary)
        case ${RELEASE_CODENAME} in
            jolnir)
                RELEASE_CODENAME='focal'
                ;;
        esac
        ;;
    linuxmint)
        # translate Mint-codenames to Ubuntu-codenames based on https://www.linuxmint.com/download_all.php
        case ${RELEASE_CODENAME} in
            zara)     ;&
            zena)     ;&
            wilma)    ;&
            xia)      ;&
            _LISTENDA)
                RELEASE_CODENAME='noble'
                ;;
            vanessa)  ;&
            vera)     ;&
            victoria) ;&
            virginia) ;&
            _LISTENDB)
                RELEASE_CODENAME='jammy'
                ;;
            una)      ;&
            uma)      ;&
            ulyssa)   ;&
            ulyana)   ;&
            _LISTENDC)
                RELEASE_CODENAME='focal'
                ;;
            tricia)   ;&
            tina)     ;&
            tessa)    ;&
            tara)     ;&
            _LISTENDD)
                RELEASE_CODENAME='bionic'
                ;;
            elsie)
                RELEASE_CODENAME='bullseye'
                ;;
            *)
                echo "Unable to map ${RELEASE_CODENAME} to an Ubuntu release.  Please patch this script and submit a pull request, or report at https://github.com/ArduPilot/ardupilot/issues"
                exit 1
        esac
        ;;
esac

PYTHON_V="python3"  # starting from ubuntu 20.04, python isn't symlink to default Python interpreter
PIP=pip3

case "${RELEASE_CODENAME}" in
    bionic)   ;&
    buster)   ;&
    focal)    ;&
    groovy)   ;&
    lunar)    ;&
    mantic)   ;&
    bullseye) ;&
    oracular) ;&
    plucky)   ;&
    questing) ;&
    _LISTENDA)
        echo "ArduPilot no longer supports developing on this operating system that has reached end of standard support."
        exit 1
        ;;

    bookworm) ;&
    trixie)   ;&
    forky)    ;&
    jammy)    ;&
    noble)    ;&
    resolute) ;&
    _LISTENDB)
        echo "${RELEASE_CODENAME} is supported"
        ;;

    *)
        echo "This install script does not handle installation for ${RELEASE_CODENAME}"
        exit 1
        ;;
esac

if false; then :
elif [ ${RELEASE_CODENAME} == 'trixie' ]; then
    SITLFML_VERSION="2.6"
    SITLCFML_VERSION="2.6"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'forky' ]; then
    SITLFML_VERSION="3.1"
    SITLCFML_VERSION="3.0"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'jammy' ]; then
    SITLFML_VERSION="2.5"
    SITLCFML_VERSION="2.5"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'noble' ]; then
    SITLFML_VERSION="2.6"
    SITLCFML_VERSION="2.6"
    PYTHON_V="python3"
    PIP=pip3
elif [ ${RELEASE_CODENAME} == 'resolute' ]; then
    SITLFML_VERSION="3.0"
    SITLCFML_VERSION="3.0"
    PYTHON_V="python3"
    PIP="python3 -m pip"
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
BASE_PKGS="build-essential ccache g++ gawk git make wget valgrind screen python3-pexpect astyle"
PYTHON_PKGS="lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan"
PYTHON_PKGS="$PYTHON_PKGS flake8 junitparser wsproto tabulate"

# add some Python packages required for commonly-used MAVProxy modules and hex file generation:
if [[ $SKIP_AP_EXT_ENV -ne 1 ]]; then
    if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
        PYTHON_PKGS="$PYTHON_PKGS pygame"
    fi
    PYTHON_PKGS="$PYTHON_PKGS intelhex"
fi
ARM_LINUX_PKGS="g++-arm-linux-gnueabihf $INSTALL_PKG_CONFIG"
# python-wxgtk packages are added to SITL_PKGS below

if false ||
   [ ${RELEASE_CODENAME} == 'trixie' ] ||
   [ ${RELEASE_CODENAME} == 'bookworm' ] ||
   [ ${RELEASE_CODENAME} == 'forky' ] ||
   [ ${RELEASE_CODENAME} == 'noble' ] ||
   [ ${RELEASE_CODENAME} == 'resolute' ] ||
   false; then
    # on Ubuntu 23.04 Lunar (and presumably later releases), we install in venv, below
    PYTHON_PKGS+=" numpy pyparsing psutil"
    SITL_PKGS="python3-dev"
else
SITL_PKGS="libtool libxml2-dev libxslt1-dev ${PYTHON_V}-dev ${PYTHON_V}-pip ${PYTHON_V}-setuptools ${PYTHON_V}-numpy ${PYTHON_V}-pyparsing ${PYTHON_V}-psutil"
fi

# add some packages required for commonly-used MAVProxy modules:
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    if false ||
       [ ${RELEASE_CODENAME} == 'trixie' ] ||
       [ ${RELEASE_CODENAME} == 'bookworm' ] ||
       [ ${RELEASE_CODENAME} == 'forky' ] ||
       [ ${RELEASE_CODENAME} == 'noble' ] ||
       [ ${RELEASE_CODENAME} == 'resolute' ] ||
       false; then
        PYTHON_PKGS+=" matplotlib scipy opencv-python pyyaml"
        SITL_PKGS+=" xterm xfonts-base libcsfml-dev libcsfml-audio${SITLCFML_VERSION} libcsfml-dev libcsfml-graphics${SITLCFML_VERSION} libcsfml-network${SITLCFML_VERSION} libcsfml-system${SITLCFML_VERSION} libcsfml-window${SITLCFML_VERSION} libsfml-audio${SITLFML_VERSION} libsfml-dev libsfml-graphics${SITLFML_VERSION} libsfml-network${SITLFML_VERSION} libsfml-system${SITLFML_VERSION} libsfml-window${SITLFML_VERSION}"
  else
  SITL_PKGS="$SITL_PKGS xterm xfonts-base ${PYTHON_V}-matplotlib ${PYTHON_V}-serial ${PYTHON_V}-scipy ${PYTHON_V}-opencv libcsfml-dev libcsfml-audio${SITLCFML_VERSION} libcsfml-dev libcsfml-graphics${SITLCFML_VERSION} libcsfml-network${SITLCFML_VERSION} libcsfml-system${SITLCFML_VERSION} libcsfml-window${SITLCFML_VERSION} libsfml-audio${SITLFML_VERSION} libsfml-dev libsfml-graphics${SITLFML_VERSION} libsfml-network${SITLFML_VERSION} libsfml-system${SITLFML_VERSION} libsfml-window${SITLFML_VERSION} ${PYTHON_V}-yaml"
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
    ARM_ARCH=$(uname -m)
    case $ARM_ARCH in
        x86_64)
            ARM_SHA256="21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618"
            ;;
        aarch64)
            ARM_SHA256="343d8c812934fe5a904c73583a91edd812b1ac20636eb52de04135bb0f5cf36a"
            ;;
        *)
            echo "No STM32 toolchain available for $ARM_ARCH, skipping"
            DO_AP_STM_ENV=0
            return
            ;;
    esac
    if [ ! -d $OPT/$ARM_ROOT ]; then
        heading "Installing toolchain for STM32 Boards"
        # extract into a staging dir so an interrupted install is retried on the next run
        ARM_STAGING="$OPT/$ARM_ROOT.partial"
        ARM_TARBALL="$ARM_ROOT-$ARM_ARCH-linux.tar.bz2"
        sudo rm -rf "$ARM_STAGING"
        sudo mkdir "$ARM_STAGING"
        echo "Downloading from ArduPilot server"
        # wget retries network errors itself, retry here on checksum mismatch
        for ARM_ATTEMPT in 1 2 3; do
            sudo wget --progress=dot:giga -O "$ARM_STAGING/$ARM_TARBALL" "https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL" &&
                echo "$ARM_SHA256  $ARM_STAGING/$ARM_TARBALL" | sha256sum -c - &&
                break
            if [ $ARM_ATTEMPT -eq 3 ]; then
                sudo rm -rf "$ARM_STAGING"
                heading "STM32 toolchain download failed or is corrupted (checksum mismatch). Please rerun this script."
                exit 1
            fi
            echo "STM32 toolchain download failed, retrying ($ARM_ATTEMPT/3)"
        done
        echo "Installing..."
        sudo tar xjf "$ARM_STAGING/$ARM_TARBALL" -C "$ARM_STAGING"
        sudo mv "$ARM_STAGING/$ARM_ROOT" "$OPT/$ARM_ROOT"
        sudo rm -rf "$ARM_STAGING"
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

heading "Add user to dialout group to allow managing serial ports"
sudo usermod -a -G dialout $USER
echo "Done!"

# Add back python symlink to python interpreter on Ubuntu >= 20.04
if false; then :
elif [ ${RELEASE_CODENAME} == 'jammy' ]; then
    BASE_PKGS+=" python-is-python3"
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'trixie' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
elif [ ${RELEASE_CODENAME} == 'forky' ]; then
    SITL_PKGS+=" libpython3-stdlib" # for argparse
fi

# Check for graphical package for MAVProxy
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
  if false; then :
  elif [ ${RELEASE_CODENAME} == 'trixie' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
  elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  elif [ ${RELEASE_CODENAME} == 'forky' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  elif [ ${RELEASE_CODENAME} == 'noble' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  elif [ ${RELEASE_CODENAME} == 'resolute' ]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev "
    # see below
  fi

  if false; then :
  elif false ||
       [ ${RELEASE_CODENAME} == 'trixie' ] ||
       [ ${RELEASE_CODENAME} == 'forky' ] ||
       false; then
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif [ ${RELEASE_CODENAME} == 'bookworm' ]; then
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif false ||
       [ ${RELEASE_CODENAME} == 'noble' ] ||
       [ ${RELEASE_CODENAME} == 'resolute' ] ||
       false; then
      PYTHON_PKGS+=" wxpython"
      SITL_PKGS+=" python3-wxgtk4.0"
      SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  elif [ ${RELEASE_CODENAME} == 'jammy' ]; then
    SITL_PKGS+=" python3-wxgtk4.0"
    SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"  # for pygame
  fi
fi

# Check if we need to manually install libtool-bin
LBTBIN=$(apt-cache search -n '^libtool-bin')
if [ -n "$LBTBIN" ]; then
    SITL_PKGS+=" libtool-bin"
fi

SITL_PKGS+=" ppp"

# Install all packages
$APT_GET install $BASE_PKGS $SITL_PKGS $ARM_LINUX_PKGS $COVERAGE_PKGS

if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    # If xfonts-base was just installed, you need to rebuild the font information cache.
    # https://discuss.ardupilot.org/t/using-the-gdb-window-on-a-high-dpi-display/128150/2
    fc-cache
fi

heading "Check if we are inside docker environment..."
IS_DOCKER=false
if [[ ${AP_DOCKER_BUILD:-0} -eq 1 ]] || [[ -f /.dockerenv ]] || grep -Eq '(lxc|docker)' /proc/1/cgroup ; then
    IS_DOCKER=true
fi
echo "Done!"

SHELL_LOGIN=".profile"
if $IS_DOCKER; then
    echo "Inside docker, we add the tools path into ~/.ardupilot_env, sourced from .bashrc"
    SHELL_LOGIN=".ardupilot_env"
    echo "# ArduPilot env file. Need to be loaded by your Shell." > ~/$SHELL_LOGIN
fi

SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

PIP_USER_ARGUMENT="--user"

# create a Python venv on more recent releases:
if false ||
     [ ${RELEASE_CODENAME} == 'bookworm' ] ||
     [ ${RELEASE_CODENAME} == 'trixie' ] ||
     [ ${RELEASE_CODENAME} == 'forky' ] ||
     [ ${RELEASE_CODENAME} == 'noble' ] ||
     [ ${RELEASE_CODENAME} == 'resolute' ] ||
     false; then
    $APT_GET install python3-venv

    # Check if venv already exists in ARDUPILOT_ROOT (check both venv-ardupilot and venv)
    VENV_PATH=""
    if [ -d "$ARDUPILOT_ROOT/venv-ardupilot" ]; then
        VENV_PATH="$ARDUPILOT_ROOT/venv-ardupilot"
        echo "Found existing venv at $VENV_PATH"
    elif [ -d "$ARDUPILOT_ROOT/venv" ]; then
        VENV_PATH="$ARDUPILOT_ROOT/venv"
        echo "Found existing venv at $VENV_PATH"
    elif [ -d "$ARDUPILOT_ROOT/.venv" ]; then
        VENV_PATH="$ARDUPILOT_ROOT/.venv"
        echo "Found existing venv at $VENV_PATH"
    else
        VENV_PATH="$HOME/venv-ardupilot"
        echo "Creating new venv at $VENV_PATH"
        python3 -m venv --system-site-packages "$VENV_PATH"
    fi

    SOURCE_LINE="source $VENV_PATH/bin/activate"

    # activate it:
    $SOURCE_LINE
    PIP_USER_ARGUMENT=""

    grep -Fxq "$SOURCE_LINE" ~/$SHELL_LOGIN 2>/dev/null || {
        if [[ -z "${DO_PYTHON_VENV_ENV}" ]] && maybe_prompt_user "Make ArduPilot venv default for python [N/y]?\nThis means that any terminal will open and load ArduPilot venv" ; then
            DO_PYTHON_VENV_ENV=1
        fi

        if [[ $DO_PYTHON_VENV_ENV -eq 1 ]]; then
            echo "$SOURCE_LINE" >> ~/$SHELL_LOGIN
        else
            echo "Please use \`$SOURCE_LINE\` to activate the ArduPilot venv"
        fi
    }
fi

# try update packaging, setuptools and wheel before installing pip package that may need compilation
$PIP install $PIP_USER_ARGUMENT --upgrade pip packaging setuptools wheel

if [ "$GITHUB_ACTIONS" == "true" ]; then
    PIP_USER_ARGUMENT+=" --progress-bar off"
fi

if false ||
   [ ${RELEASE_CODENAME} == 'trixie' ] ||
   [ ${RELEASE_CODENAME} == 'bookworm' ] ||
   [ ${RELEASE_CODENAME} == 'forky' ] ||
   [ ${RELEASE_CODENAME} == 'noble' ] ||
   [ ${RELEASE_CODENAME} == 'resolute' ] ||
   false; then
    # must do this ahead of wxPython pip3 run :-/
    $PIP install $PIP_USER_ARGUMENT --upgrade attrdict3
fi

# install Python packages one-at-a-time so it is clear which package
# is causing problems:
for PACKAGE in $PYTHON_PKGS; do
    if [ "$PACKAGE" == "wxpython" ]; then
        # PyPI only has the wxPython sources for Linux, try the prebuilt
        # wheel from the latest GitHub release before building it
        case ${RELEASE_CODENAME} in
            noble)    WX_UBUNTU="ubuntu2404" ;;
            resolute) WX_UBUNTU="ubuntu2604" ;;
            *)        WX_UBUNTU="none" ;;
        esac
        WX_TAG=$(python3 -c 'import urllib.request; print(urllib.request.urlopen("https://github.com/wxWidgets/Phoenix/releases/latest").geturl().rsplit("/", 1)[1])' || true)
        PY_TAG=$(python3 -c 'import sys; print("cp%d%d" % sys.version_info[:2])')
        WX_WHEEL="https://github.com/wxWidgets/Phoenix/releases/download/$WX_TAG/wxpython-${WX_TAG#wxPython-}%2B$WX_UBUNTU-$PY_TAG-$PY_TAG-linux_$(uname -m).whl"
        time $PIP install $PIP_USER_ARGUMENT --upgrade "$WX_WHEEL" || {
            echo "##### No prebuilt $PACKAGE wheel, building it takes a *VERY* long time (~30 minutes).  Be patient."
            time $PIP install $PIP_USER_ARGUMENT --upgrade $PACKAGE
        }
    else
        time $PIP install $PIP_USER_ARGUMENT --upgrade $PACKAGE
    fi
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

if [[ $DO_AP_STM_ENV -eq 1 ]]; then
exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
grep -Fxq "$exportline" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo "$exportline" >> ~/$SHELL_LOGIN
        eval "$exportline"
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
}
fi

exportline2="export PATH=\"$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\"\$PATH";
grep -Fxq "$exportline2" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?" ; then
        echo "$exportline2" >> ~/$SHELL_LOGIN
        eval "$exportline2"
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

if [[ $SKIP_AP_COMPLETION_ENV -ne 1 ]]; then
exportline3="source \"$ARDUPILOT_ROOT/Tools/completion/completion.bash\"";
grep -Fxq "$exportline3" ~/.bashrc 2>/dev/null || {
    if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]?" ; then
        echo "$exportline3" >> ~/.bashrc
        eval "$exportline3"
    else
        echo "Skipping adding ArduPilot Bash Completion."
    fi
}
fi

exportline4="export PATH=/usr/lib/ccache:\$PATH";
grep -Fxq "$exportline4" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Append CCache to your PATH [N/y]?" ; then
        echo "$exportline4" >> ~/$SHELL_LOGIN
        eval "$exportline4"
    else
        echo "Skipping appending CCache to PATH."
    fi
}
echo "Done!"

if [[ $SKIP_AP_GIT_CHECK -ne 1 ]]; then
  if [ -e "$ARDUPILOT_ROOT/.git" ]; then
    heading "Update git submodules"
    cd "$ARDUPILOT_ROOT"
    git submodule update --init --recursive
    echo "Done!"
  fi
fi

if $IS_DOCKER; then
    echo "Finalizing ArduPilot env for Docker"
    grep -Fxq "source ~/.ardupilot_env" ~/.bashrc 2>/dev/null || echo "source ~/.ardupilot_env" >> ~/.bashrc
fi

echo "---------- $0 end ----------"
