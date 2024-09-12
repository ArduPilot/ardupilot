#!/usr/bin/env bash
echo "---------- $0 start ----------"
set -e
set -x
set +H

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

ZYPPER="sudo zypper in --no-recommends"
if $ASSUME_YES; then
    ZYPPER="sudo zypper in -y --no-recommends"
fi
PIP3=pip3
if $QUIET; then
    PIP3="pip3 -q"
fi

function package_is_installed() {
    rpm -q $1 &>/dev/null
}

function heading() {
    echo "$sep"
    echo $*
    echo "$sep"
}

#Install python3 1st, as openSUSE TW WSL does not come with preinstalled python
$ZYPPER python3 || echo "Check zypper output for errors"

#As tumbleweed can have many python versions, we need to know which one is currently active
PYPKGVER=python$(python3 --version | cut -d' ' -f2 | awk -F. '{print $1$2}')

BASE_PKGS="patterns-devel-base-devel_basis ccache git axel valgrind screen gcc-c++ xterm free-ttf-fonts sfml2-devel zip glibc-devel-static rsync"
SITL_PKGS="${PYPKGVER}-pip ${PYPKGVER}-devel ${PYPKGVER}-setuptools ${PYPKGVER}-wheel ${PYPKGVER}-lxml ${PYPKGVER}-pyaml ${PYPKGVER}-wxPython ${PYPKGVER}-pyparsing ${PYPKGVER}-opencv ${PYPKGVER}-numpy ${PYPKGVER}-scipy ${PYPKGVER}-matplotlib"

PYTHON_PKGS="future lxml pymavlink MAVProxy pexpect argparse pyparsing geocoder pyserial empy==3.3.4 ptyprocess dronecan"
PYTHON_PKGS+=" flake8 junitparser pygame intelhex psutil pyyaml"
# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
ARM_TARBALL="$ARM_ROOT-x86_64-linux.tar.bz2"
ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"

ARM_LINUX_ROOT=gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf
ARM_LINUX_GCC_URL="https://releases.linaro.org/components/toolchain/binaries/7.5-2019.12/arm-linux-gnueabihf/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf.tar.xz"
ARM_LINUX_TARBALL="gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf.tar.xz"

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

sudo usermod -a -G dialout "$USER"

$ZYPPER $BASE_PKGS $SITL_PKGS || echo "Check zypper output for errors"

python3 -m venv --system-site-packages "$HOME"/venv-ardupilot

SHELL_LOGIN=".profile"
# activate it:
SOURCE_LINE="source $HOME/venv-ardupilot/bin/activate"
$SOURCE_LINE

if ! grep -Fxq "$SOURCE_LINE" ~/.bashrc; then 
    if [[ -z "${DO_PYTHON_VENV_ENV}" ]] && maybe_prompt_user "Make ArduPilot venv default for python [N/y]?" ; then
        DO_PYTHON_VENV_ENV=1
    fi

    if [[ $DO_PYTHON_VENV_ENV -eq 1 ]]; then
        echo $SOURCE_LINE >> ~/.bashrc
    fi
fi

$PIP3 install -U pip packaging setuptools wheel
$PIP3 install -U attrdict3
$PIP3 install -U $PYTHON_PKGS

(
    cd /usr/lib64/ccache
    for C in arm-none-eabi-g++  arm-none-eabi-gcc arm-linux-gnueabihf-g++ arm-linux-gnueabihf-gcc; do
        if [ ! -f "$C" ]; then
            sudo ln -s ../../bin/ccache "$C"
        fi
    done
)

ccache --set-config sloppiness=file_macro,locale,time_macros
ccache --set-config ignore_options="--specs=nano.specs --specs=nosys.specs"

if [ ! -d $OPT/$ARM_ROOT ]; then
    (
        cd $OPT;
        sudo axel -a -c $ARM_TARBALL_URL;
        sudo tar xjf ${ARM_TARBALL};
        sudo rm ${ARM_TARBALL};
    )
fi

if [ ! -d $OPT/$ARM_LINUX_ROOT ]; then
    (
        cd $OPT;
        sudo axel -a -c "${ARM_LINUX_GCC_URL}";
        sudo tar xf ${ARM_LINUX_TARBALL};
        sudo rm ${ARM_LINUX_TARBALL};
    )
fi

heading "Removing modemmanager and brltty package that could conflict with firmware uploading"
if package_is_installed "ModemManager"; then
    sudo zypper rm ModemManager
fi
if package_is_installed "brltty"; then
    sudo zypper rm brltty
fi
echo "Done!"

heading "Adding ArduPilot Tools to environment"

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")


exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH";
if ! grep -Fxq "$exportline" ~/$SHELL_LOGIN ; then
    if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo $exportline >> ~/$SHELL_LOGIN
    else
        echo "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
    fi
fi

exportline1="export PATH=$OPT/$ARM_LINUX_ROOT/bin:\$PATH";
if ! grep -Fxq "$exportline1" ~/$SHELL_LOGIN ; then
    if maybe_prompt_user "Add $OPT/$ARM_LINUX_ROOT/bin to your PATH [N/y]?" ; then
        echo $exportline1 >> ~/$SHELL_LOGIN
    else
        echo "Skipping adding $OPT/$ARM_LINUX_ROOT/bin to PATH."
    fi
fi

exportline2="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
if  ! grep -Fxq "$exportline2" ~/$SHELL_LOGIN ; then
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?" ; then
        echo $exportline2 >> ~/$SHELL_LOGIN
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
fi

exportline3="source $ARDUPILOT_ROOT/Tools/completion/completion.bash";
if  ! grep -Fxq "$exportline3" ~/.bashrc ; then
    if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]?" ; then
        echo $exportline3 >> ~/.bashrc
    else
        echo "Skipping adding ArduPilot Bash Completion."
    fi
fi

exportline4="export PATH=/usr/lib64/ccache:\$PATH";
if ! grep -Fxq "$exportline4" ~/$SHELL_LOGIN ; then
    if maybe_prompt_user "Append CCache to your PATH [N/y]?" ; then
        echo $exportline4 >> ~/$SHELL_LOGIN
    else
        echo "Skipping appending CCache to PATH."
    fi
fi
echo "Done!"

if [[ $SKIP_AP_GIT_CHECK -ne 1 ]]; then
  cd "$ARDUPILOT_ROOT"
  if [ -d ".git" ]; then
    heading "Update git submodules"
    cd $ARDUPILOT_ROOT
    git submodule update --init --recursive
    echo "Done!"
  fi
fi

echo "---------- $0 end ----------"
echo "Done. Please log out and log in again."
