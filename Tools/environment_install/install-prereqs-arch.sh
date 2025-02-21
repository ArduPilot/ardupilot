#!/usr/bin/env bash
set -e

ASSUME_YES=false

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

# Automatic selection of the CONFIG FILE according to the SHELL.
if [[ "$SHELL" == "/bin/bash" ]]; then
	CONFIG_FILE="$HOME"/.bashrc
elif [[ "$SHELL" == "/bin/zsh" ]]; then
	CONFIG_FILE="$HOME"/.zshrc
else
	CONFIG_FILE="/dev/null"
fi

# Required Packages
BASE_PKGS="base-devel ccache git gsfonts tk wget gcc"
SITL_PKGS="python-pip python-setuptools python-wheel python-numpy python-scipy opencv python-wxpython"
PX4_PKGS="lib32-glibc zip zlib ncurses"

PYTHON_PKGS="future lxml pymavlink MAVProxy opencv-python pexpect argparse matplotlib pyparsing geocoder pyserial empy==3.3.4 dronecan packaging setuptools wheel"

# GNU Tools for ARM Embedded Processors
# (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
ARM_TARBALL="$ARM_ROOT-x86_64-linux.tar.bz2"
ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"
ARM_TARBALL_CHECKSUM="21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618" 

# Main Directories for Script, Tools and Build Root
SCRIPT_DIR=$(dirname "$(realpath "$0")")
ARDUPILOT_TOOLS_DIR="$(echo "$SCRIPT_DIR" | rev | cut -f2- -d'/' | rev)/autotest"
OPT_DIR="/opt"

function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    else
        read -rp "$1"
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}

sudo usermod -a -G uucp "$USER"

sudo pacman -Syu --noconfirm --needed $BASE_PKGS $SITL_PKGS $PX4_PKGS

python3 -m venv --system-site-packages "$HOME"/venv-ardupilot

# activate it:
SOURCE_VENV="source $HOME/venv-ardupilot/bin/activate"
$SOURCE_VENV

DO_PYTHON_VENV_ENV=false
if maybe_prompt_user "Make ArduPilot venv default for python [N/y]?" ; then
    DO_PYTHON_VENV_ENV=true
fi

if $DO_PYTHON_VENV_ENV; then
    echo "$SOURCE_VENV" >> "$CONFIG_FILE"
else
    echo "Please use << $SOURCE_VENV >> to activate the ArduPilot venv."
fi

python3 -m pip -q install -U $PYTHON_PKGS

(
    cd /usr/lib/ccache
    if [ ! -f arm-none-eabi-g++ ]; then
       sudo ln -s /usr/bin/ccache arm-none-eabi-g++
    fi
    if [ ! -f arm-none-eabi-g++ ]; then
        sudo ln -s /usr/bin/ccache arm-none-eabi-gcc
    fi
)

if [ ! -d $OPT_DIR/$ARM_ROOT ]; then
    (
        cd $OPT_DIR;

        # Check if file exists and verify checksum
        DOWNLOAD_REQUIRED=false
        if [ -e "$ARM_TARBALL" ]; then
            echo "File exists. Verifying checksum..."

            # Calculate the checksum of the existing file
            ACTUAL_CHECKSUM=$(sha256sum "$ARM_TARBALL" | awk '{ print $1 }')

            # Compare the actual checksum with the expected one
            if [ "$ACTUAL_CHECKSUM" == "$ARM_TARBALL_CHECKSUM" ]; then
                echo "Checksum valid. No need to redownload."
            else
                echo "Checksum invalid. Redownloading the file..."
                DOWNLOAD_REQUIRED=true
                sudo rm $ARM_TARBALL
            fi
        else
            echo "File does not exist. Downloading..."
            DOWNLOAD_REQUIRED=true
        fi

        if $DOWNLOAD_REQUIRED; then
            sudo wget -O "$ARM_TARBALL" --progress=dot:giga $ARM_TARBALL_URL
        fi

        sudo tar xjf ${ARM_TARBALL}
    )
fi

PRELOAD_BUILDTOOLS="export PATH=$OPT_DIR/$ARM_ROOT/bin:\$PATH";
if ! grep -Fxq "$PRELOAD_BUILDTOOLS" "$CONFIG_FILE" ; then
    if maybe_prompt_user "Add $OPT_DIR/$ARM_ROOT/bin to your PATH [N/y]?" ; then
        echo "$PRELOAD_BUILDTOOLS" >> "$CONFIG_FILE"
        $PRELOAD_BUILDTOOLS
    else
        echo "Skipping adding $OPT_DIR/$ARM_ROOT/bin to PATH."
    fi
fi

PRELOAD_ARDUTOOLS="export PATH=$ARDUPILOT_TOOLS_DIR:\$PATH";
if  ! grep -Fxq "$PRELOAD_ARDUTOOLS" "$CONFIG_FILE" ; then
    if maybe_prompt_user "Add $ARDUPILOT_TOOLS_DIR to your PATH [N/y]?" ; then
        echo "$PRELOAD_ARDUTOOLS" >> "$CONFIG_FILE"
        $PRELOAD_ARDUTOOLS
    else
        echo "Skipping adding $ARDUPILOT_TOOLS_DIR to PATH."
    fi
fi

(
    cd "$SCRIPT_DIR"
    git submodule update --init --recursive
)

echo "Done. Please log out and log in again."
