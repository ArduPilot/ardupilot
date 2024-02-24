#!/bin/bash
echo "---------- $0 start ----------"
set -e
set -x

if [ $EUID == 0 ]; then
    echo "Please do not run this script as root; don't sudo it!"
    exit 1
fi

if [[ $SHELL == *"zsh"* ]]; then
    AP_COMPLETION_SCR="completion.zsh"
    SHELL_LOGIN=".zshrc"
elif [[ $SHELL == *"bash"* ]]; then
    AP_COMPLETION_SCR="completion.bash"
    SHELL_LOGIN=".bash_profile"
else
    echo "Unsupported shell"
    exit 1
fi

OPT="/opt"
# Ardupilot Tools
ARDUPILOT_TOOLS="Tools/autotest"

ASSUME_YES=false
sep="##############################################"

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


echo "Checking homebrew..."
$(which -s brew) ||
{
    echo "installing homebrew..."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
} 
echo "Homebrew installed"

#install command line tools
echo "Checking CLI Tools installed..."
{
    ERROR=$(xcode-select --install 2>&1 > /dev/null)
} ||
{
if [[ $ERROR != *"ommand line tools are already installed"* ]]; then
    echo "$ERROR" 1>&2
    exit 1
fi
}

# ArduPilot official Toolchain for STM32 boards
function install_arm_none_eabi_toolchain() {
    # GNU Tools for ARM Embedded Processors
    # (see https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)
    ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
    ARM_TARBALL="$ARM_ROOT-mac.tar.bz2"
    ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"
    if [ ! -d $OPT/$ARM_ROOT ]; then
        (
            cd $OPT;
            echo "Installing toolchain for STM32 Boards"
            echo "Downloading from ArduPilot server"
            sudo wget $ARM_TARBALL_URL
            echo "Installing..."
            sudo tar xjf ${ARM_TARBALL}
            echo "... Cleaning"
            sudo rm ${ARM_TARBALL};
        )
    fi
    echo "Registering STM32 Toolchain for ccache"
    sudo mkdir -p /usr/local/opt/ccache/libexec
    sudo ln -s -f $CCACHE_PATH /usr/local/opt/ccache/libexec/arm-none-eabi-g++
    sudo ln -s -f $CCACHE_PATH /usr/local/opt/ccache/libexec/arm-none-eabi-gcc
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

# delete links installed by github in /usr/local/bin; installing or
# upgrading python via brew fails if these links are in place.  brew
# auto-updates things when you install other packages which depend on
# more recent versions.
# see https://github.com/orgs/Homebrew/discussions/3895
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete

# brew update randomly failing on CI, so ignore errors:
brew update
brew install --force --overwrite gawk curl coreutils wget

PIP=pip
if maybe_prompt_user "Install python using pyenv [N/y]?" ; then
    echo "Checking pyenv..."
    {
        $(which -s pyenv)
    } ||
    {
        echo "Installing pyenv"
        curl https://pyenv.run | bash

        pushd $HOME/.pyenv
        git fetch --tags
        git checkout v2.3.12
        popd
        exportline="export PYENV_ROOT=\$HOME/.pyenv"
        echo $exportline >> ~/$SHELL_LOGIN
        exportline="export PATH=\$PYENV_ROOT/bin:\$PATH"
        echo $exportline >> ~/$SHELL_LOGIN
        evalline="eval \"\$(pyenv init --path)\""
        echo $evalline >> ~/$SHELL_LOGIN
        evalline="eval \"\$(pyenv init -)\""
        echo $evalline >> ~/$SHELL_LOGIN
        source ~/$SHELL_LOGIN
    }
    echo "pyenv installed"
    {
        $(pyenv global 3.10.4)
    } || {
        env PYTHON_CONFIGURE_OPTS="--enable-framework" pyenv install 3.10.4
        pyenv global 3.10.4
    }
fi


if [[ -z "${DO_AP_STM_ENV}" ]] && maybe_prompt_user "Install ArduPilot STM32 toolchain [N/y]?" ; then
    DO_AP_STM_ENV=1
fi

echo "Checking ccache..."
{
    $(which -s ccache)
} ||
{
    brew install ccache
    exportline="export PATH=/usr/local/opt/ccache/libexec:\$PATH";
    eval $exportline
}
CCACHE_PATH=$(which ccache)

if [[ $DO_AP_STM_ENV -eq 1 ]]; then
    install_arm_none_eabi_toolchain
fi

PYTHON_PKGS="future lxml pymavlink MAVProxy pexpect geocoder flake8 junitparser empy==3.3.4 dronecan"
# add some Python packages required for commonly-used MAVProxy modules and hex file generation:
if [[ $SKIP_AP_EXT_ENV -ne 1 ]]; then
    PYTHON_PKGS="$PYTHON_PKGS intelhex gnureadline"
fi
# add some packages required for commonly-used MAVProxy modules:
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    PYTHON_PKGS="$PYTHON_PKGS wxPython billiard"
fi

$PIP install --upgrade pip
$PIP install wheel
$PIP install $PYTHON_PKGS

echo "Adding ArduPilot Tools to environment"

SCRIPT_DIR=$(dirname $(grealpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(grealpath "$SCRIPT_DIR/../../")

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
exportline3="source $ARDUPILOT_ROOT/Tools/completion/$AP_COMPLETION_SCR";
grep -Fxq "$exportline3" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]?" ; then
        echo $exportline3 >> ~/$SHELL_LOGIN
        eval $exportline3
    else
        echo "Skipping adding ArduPilot Bash Completion."
    fi
}
fi

exportline4="export PATH=/usr/local/opt/ccache/libexec:\$PATH";
grep -Fxq "$exportline4" ~/$SHELL_LOGIN 2>/dev/null || {
    if maybe_prompt_user "Append CCache to your PATH [N/y]?" ; then
        echo $exportline4 >> ~/$SHELL_LOGIN
        eval $exportline4
    else
        echo "Skipping appending CCache to PATH."
    fi
}
echo "Done!"

git submodule update --init --recursive

echo "finished"
