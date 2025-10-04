#!/usr/bin/env bash
echo "---------- $0 start ----------"
set -e
#set -x

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Color functions
print_red() {
    echo -e "${RED}$1${NC}"
}

print_green() {
    echo -e "${GREEN}$1${NC}"
}

print_yellow() {
    echo -e "${YELLOW}$1${NC}"
}

print_blue() {
    echo -e "${BLUE}$1${NC}"
}

if [ $EUID == 0 ]; then
    print_red "Please do not run this script as root; don't sudo it!"
    exit 1
fi

OPT="/opt"
# Ardupilot Tools
ARDUPILOT_TOOLS="Tools/autotest"

ASSUME_YES=false
QUIET=false
sep="##############################################"

OPTIND=1 # Reset in case getopts has been used previously in the shell.
while getopts "yq" opt; do
    case "$opt" in
        y)  ASSUME_YES=true
            ;;
        q)  QUIET=true
            ;;
        *)  print_red "Invalid option: -$OPTARG"
            exit 1
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
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -c "ok installed" || true
}

function heading() {
    print_blue "$sep"
    print_blue "$*"
    print_blue "$sep"
}

# Install lsb-release as it is needed to check Ubuntu version
if ! package_is_installed "lsb-release"; then
    heading "Installing lsb-release"
    $APT_GET install lsb-release
    print_green "Done!"
fi

# Checking release
RELEASE_CODENAME=$(lsb_release -c -s)
RELEASE_DISTRIBUTOR=$(lsb_release -i -s | tr '[:upper:]' '[:lower:]')

print_yellow "Detected distribution: $RELEASE_DISTRIBUTOR"
print_yellow "Detected codename: $RELEASE_CODENAME"

# Kali Linux detection
if [ "$RELEASE_DISTRIBUTOR" != "kali" ]; then
    print_red "This script is specifically for Kali Linux. Detected: $RELEASE_DISTRIBUTOR"
    print_red "Please use the appropriate install script for your distribution."
    exit 1
fi

PYTHON_V="python3"
PIP="python3 -m pip"

# For Kali, use generic SFML packages without version numbers
SITLFML_VERSION=""
SITLCFML_VERSION=""

# Lists of packages to install
BASE_PKGS="build-essential ccache g++ gawk git make wget valgrind screen python3-pexpect astyle"
PYTHON_PKGS="future lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan"
PYTHON_PKGS="$PYTHON_PKGS flake8 junitparser wsproto tabulate"

# add some Python packages required for commonly-used MAVProxy modules and hex file generation:
if [[ $SKIP_AP_EXT_ENV -ne 1 ]]; then
    PYTHON_PKGS="$PYTHON_PKGS pygame intelhex"
fi

ARM_LINUX_PKGS="g++-arm-linux-gnueabihf"

# For Kali, install in venv
PYTHON_PKGS+=" numpy pyparsing psutil"
SITL_PKGS="python3-dev"

# add some packages required for commonly-used MAVProxy modules:
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    PYTHON_PKGS+=" matplotlib scipy opencv-python pyyaml"
    # Generic SFML packages for Kali (no version suffix)
    SITL_PKGS+=" xterm xfonts-base libcsfml-dev libsfml-dev"
fi

if [[ $SKIP_AP_COV_ENV -ne 1 ]]; then
    # Coverage utilities
    COVERAGE_PKGS="lcov gcovr"
fi

# Check if we need to install pkg-config
if ! package_is_installed "pkg-config"; then
    print_yellow "Installing pkg-config..."
    $APT_GET install pkg-config
    print_green "pkg-config installed successfully!"
fi

# Kali specific packages
SITL_PKGS+=" libpython3-stdlib"

# Check for graphical package for MAVProxy
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    SITL_PKGS+=" libgtk-3-dev libwxgtk3.2-dev"
    PYTHON_PKGS+=" opencv-python wxpython"
    SITL_PKGS+=" python3-wxgtk4.0"
    SITL_PKGS+=" fonts-freefont-ttf libfreetype6-dev libpng16-16 libportmidi-dev libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"
fi

# Check if we need to manually install realpath
RP=$(apt-cache search -n '^realpath$' || true)
if [ -n "$RP" ]; then
    BASE_PKGS+=" realpath"
fi

# Check if we need to manually install libtool-bin
LBTBIN=$(apt-cache search -n '^libtool-bin' || true)
if [ -n "$LBTBIN" ]; then
    SITL_PKGS+=" libtool-bin"
fi

SITL_PKGS+=" ppp"

# Install all packages
heading "Installing packages"
$APT_GET install $BASE_PKGS $SITL_PKGS $ARM_LINUX_PKGS $COVERAGE_PKGS
print_green "Packages installed successfully!"

if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    # If xfonts-base was just installed, you need to rebuild the font information cache.
    heading "Rebuilding font cache"
    fc-cache
    print_green "Font cache rebuilt successfully!"
fi

heading "Check if we are inside docker environment..."
IS_DOCKER=false
if [[ ${AP_DOCKER_BUILD:-0} -eq 1 ]] || [[ -f /.dockerenv ]] || grep -Eq '(lxc|docker)' /proc/1/cgroup 2>/dev/null; then
    IS_DOCKER=true
fi
print_yellow "Docker environment: $IS_DOCKER"
print_green "Docker check complete!"

SHELL_LOGIN=".profile"
if $IS_DOCKER; then
    print_yellow "Inside docker, we add the tools path into .bashrc directly"
    SHELL_LOGIN=".ardupilot_env"
    echo "# ArduPilot env file. Need to be loaded by your Shell." > ~/$SHELL_LOGIN
fi

SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

PIP_USER_ARGUMENT="--user"

# create a Python venv for Kali
PYTHON_VENV_PACKAGE="python3-venv"

heading "Setting up Python virtual environment"
$APT_GET install $PYTHON_VENV_PACKAGE

# Check if venv already exists in ARDUPILOT_ROOT
VENV_PATH=""
if [ -d "$ARDUPILOT_ROOT/venv-ardupilot" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/venv-ardupilot"
    print_green "Found existing venv at $VENV_PATH"
elif [ -d "$ARDUPILOT_ROOT/venv" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/venv"
    print_green "Found existing venv at $VENV_PATH"
elif [ -d "$ARDUPILOT_ROOT/.venv" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/.venv"
    print_green "Found existing venv at $VENV_PATH"
else
    VENV_PATH="$HOME/venv-ardupilot"
    print_yellow "Creating new venv at $VENV_PATH"
    python3 -m venv --system-site-packages "$VENV_PATH"
    print_green "Virtual environment created successfully!"
fi

SOURCE_LINE="source $VENV_PATH/bin/activate"

# activate it:
$SOURCE_LINE
PIP_USER_ARGUMENT=""

function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    else
        read -p "$1" -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            return 0
        else
            return 1
        fi
    fi
}

if [[ -z "${DO_PYTHON_VENV_ENV}" ]] && maybe_prompt_user "Make ArduPilot venv default for python [N/y]? "; then
    DO_PYTHON_VENV_ENV=1
fi

if [[ $DO_PYTHON_VENV_ENV -eq 1 ]]; then
    echo "$SOURCE_LINE" >> ~/$SHELL_LOGIN
    print_green "ArduPilot venv added to $SHELL_LOGIN"
else
    print_yellow "Please use \`$SOURCE_LINE\` to activate the ArduPilot venv"
fi

print_green "Virtual environment setup complete!"

heading "Upgrading pip and setuptools"
# try update packaging, setuptools and wheel before installing pip package that may need compilation
$PIP install $PIP_USER_ARGUMENT -U pip packaging setuptools wheel
print_green "pip and setuptools upgraded successfully!"

if [ "$GITHUB_ACTIONS" == "true" ]; then
    PIP_USER_ARGUMENT+=" --progress-bar off"
fi

# must do this ahead of wxPython pip3 run
heading "Installing attrdict3"
$PIP install $PIP_USER_ARGUMENT -U attrdict3
print_green "attrdict3 installed successfully!"

# install Python packages one-at-a-time so it is clear which package is causing problems:
heading "Installing Python packages"
for PACKAGE in $PYTHON_PKGS; do
    print_yellow "Installing $PACKAGE..."
    if [ "$PACKAGE" == "wxpython" ]; then
        print_yellow "##### $PACKAGE takes a *VERY* long time to install (~30 minutes). Be patient."
        time $PIP install $PIP_USER_ARGUMENT -U $PACKAGE || {
            print_red "Warning: Failed to install $PACKAGE, continuing anyway..."
        }
    else
        time $PIP install $PIP_USER_ARGUMENT -U $PACKAGE || {
            print_red "Warning: Failed to install $PACKAGE, continuing anyway..."
        }
    fi
    
    # Check if installation was successful
    if $PIP show $PACKAGE &>/dev/null; then
        print_green "✓ $PACKAGE installed successfully!"
    fi
done
print_green "All Python packages processed!"

# Force reinstall Pillow for Kali
heading "Reinstalling Pillow"
$PIP install --force-reinstall pillow
print_green "Pillow reinstalled successfully!"

heading "Add user to dialout group to allow managing serial ports"
sudo usermod -a -G dialout $USER
print_green "User added to dialout group successfully!"

if [[ -z "${DO_AP_STM_ENV}" ]] && maybe_prompt_user "Install ArduPilot STM32 toolchain [N/y]? "; then
    DO_AP_STM_ENV=1
fi

heading "Removing modemmanager and brltty package that could conflict with firmware uploading"
if package_is_installed "modemmanager"; then
    $APT_GET remove modemmanager
    print_green "modemmanager removed successfully!"
else
    print_green "modemmanager is not installed (already clean)"
fi
if package_is_installed "brltty"; then
    $APT_GET remove brltty
    print_green "brltty removed successfully!"
else
    print_green "brltty is not installed (already clean)"
fi
print_green "Conflicting packages check complete!"

# ArduPilot official Toolchain for STM32 boards
function install_arm_none_eabi_toolchain() {
    ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
    case $(uname -m) in
        x86_64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    print_yellow "Downloading from ArduPilot server"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    print_yellow "Installing..."
                    sudo chmod -R 755 gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    print_yellow "Cleaning..."
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    print_green "STM32 toolchain installed successfully!"
                )
            else
                print_green "STM32 toolchain already installed!"
            fi
            CCACHE_PATH=$(which ccache)
            print_yellow "Registering STM32 Toolchain for ccache"
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            print_green "STM32 Toolchain registered for ccache!"
            ;;

        aarch64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    print_yellow "Downloading from ArduPilot server"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    print_yellow "Installing..."
                    sudo chmod -R 755 gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    print_yellow "Cleaning..."
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    print_green "STM32 toolchain installed successfully!"
                )
            else
                print_green "STM32 toolchain already installed!"
            fi
            CCACHE_PATH=$(which ccache)
            print_yellow "Registering STM32 Toolchain for ccache"
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -s -f $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            print_green "STM32 Toolchain registered for ccache!"
            ;;
    esac
}

CCACHE_PATH=$(which ccache)
if [[ $DO_AP_STM_ENV -eq 1 ]]; then
    install_arm_none_eabi_toolchain
fi

heading "Adding ArduPilot Tools to environment"

if [[ $DO_AP_STM_ENV -eq 1 ]]; then
    exportline="export PATH=$OPT/$ARM_ROOT/bin:\$PATH"
    if ! grep -Fxq "$exportline" ~/$SHELL_LOGIN 2>/dev/null; then
        if maybe_prompt_user "Add $OPT/$ARM_ROOT/bin to your PATH [N/y]? "; then
            echo "$exportline" >> ~/$SHELL_LOGIN
            eval "$exportline"
            print_green "STM32 toolchain added to PATH!"
        else
            print_yellow "Skipping adding $OPT/$ARM_ROOT/bin to PATH."
        fi
    else
        print_green "STM32 toolchain already in PATH!"
    fi
fi

exportline2="export PATH=\"$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\"\$PATH"
if ! grep -Fxq "$exportline2" ~/$SHELL_LOGIN 2>/dev/null; then
    if maybe_prompt_user "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]? "; then
        echo "$exportline2" >> ~/$SHELL_LOGIN
        eval "$exportline2"
        print_green "ArduPilot Tools added to PATH!"
    else
        print_yellow "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
else
    print_green "ArduPilot Tools already in PATH!"
fi

if [[ $SKIP_AP_COMPLETION_ENV -ne 1 ]]; then
    exportline3="source \"$ARDUPILOT_ROOT/Tools/completion/completion.bash\""
    if ! grep -Fxq "$exportline3" ~/$SHELL_LOGIN 2>/dev/null; then
        if maybe_prompt_user "Add ArduPilot Bash Completion to your bash shell [N/y]? "; then
            echo "$exportline3" >> ~/.bashrc
            eval "$exportline3"
            print_green "ArduPilot Bash Completion added!"
        else
            print_yellow "Skipping adding ArduPilot Bash Completion."
        fi
    else
        print_green "ArduPilot Bash Completion already configured!"
    fi
fi

exportline4="export PATH=/usr/lib/ccache:\$PATH"
if ! grep -Fxq "$exportline4" ~/$SHELL_LOGIN 2>/dev/null; then
    if maybe_prompt_user "Append CCache to your PATH [N/y]? "; then
        echo "$exportline4" >> ~/$SHELL_LOGIN
        eval "$exportline4"
        print_green "CCache added to PATH!"
    else
        print_yellow "Skipping appending CCache to PATH."
    fi
else
    print_green "CCache already in PATH!"
fi
print_green "Environment configuration complete!"

if [[ $SKIP_AP_GIT_CHECK -ne 1 ]]; then
    if [ -d ".git" ]; then
        heading "Update git submodules"
        cd "$ARDUPILOT_ROOT"
        git submodule update --init --recursive
        print_green "Git submodules updated successfully!"
    fi
fi

if $IS_DOCKER; then
    print_yellow "Finalizing ArduPilot env for Docker"
    echo "source ~/.ardupilot_env" >> ~/.bashrc
    print_green "Docker environment finalized!"
fi

heading "Installation complete!"
print_green "✓ All installations completed successfully!"
print_yellow "Please log out and log back in for group changes to take effect."
print_yellow "Then activate the Python virtual environment with:"
print_blue "  $SOURCE_LINE"
echo "---------- $0 end ----------"
