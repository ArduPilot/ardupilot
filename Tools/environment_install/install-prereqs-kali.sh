#!/usr/bin/env bash
echo "---------- $0 start ----------"
set -e

# ===== COLORI =====
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_red()   { echo -e "${RED}$1${NC}"; }
print_green() { echo -e "${GREEN}$1${NC}"; }
print_yellow(){ echo -e "${YELLOW}$1${NC}"; }
print_blue()  { echo -e "${BLUE}$1${NC}"; }

# ===== PRELIMINARI =====
if [ $EUID == 0 ]; then
    print_red "Please do not run this script as root; don't sudo it!"
    exit 1
fi

OPT="/opt"
ARDUPILOT_TOOLS="Tools/autotest"
ASSUME_YES=false
QUIET=false
sep="##############################################"

OPTIND=1
while getopts "yq" opt; do
    case "$opt" in
        y)  ASSUME_YES=true ;;
        q)  QUIET=true ;;
        *)  print_red "Invalid option: -$OPTARG"; exit 1 ;;
    esac
done

APT_GET="sudo apt-get"
$ASSUME_YES && APT_GET="$APT_GET --assume-yes"
$QUIET && APT_GET="$APT_GET -qq"

$APT_GET update

# ===== FUNZIONI =====
function package_is_installed() {
    dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -c "ok installed"
}

function heading() {
    print_blue "$sep"
    print_blue "$*"
    print_blue "$sep"
}

# ===== DETECTION DISTRIBUZIONE =====
if ! package_is_installed "lsb-release"; then
    heading "Installing lsb-release"
    $APT_GET install lsb-release
    print_green "Done!"
fi

RELEASE_CODENAME=$(lsb_release -c -s)
RELEASE_DISTRIBUTOR=$(lsb_release -i -s | tr '[:upper:]' '[:lower:]')

print_yellow "Detected distribution: $RELEASE_DISTRIBUTOR"
print_yellow "Detected codename: $RELEASE_CODENAME"

if [ "$RELEASE_DISTRIBUTOR" != "kali" ]; then
    print_red "This script is for Kali Linux only. Detected: $RELEASE_DISTRIBUTOR"
    exit 1
fi

PYTHON_V="python3"
PIP="python3 -m pip"

# ===== PACCHETTI BASE =====
BASE_PKGS="build-essential ccache g++ gawk git make wget valgrind screen python3-pexpect astyle"
PYTHON_PKGS="future lxml pymavlink pyserial MAVProxy geocoder empy==3.3.4 ptyprocess dronecan flake8 junitparser wsproto tabulate"
[[ $SKIP_AP_EXT_ENV -ne 1 ]] && PYTHON_PKGS="$PYTHON_PKGS pygame intelhex"

ARM_LINUX_PKGS="g++-arm-linux-gnueabihf"
PYTHON_PKGS+=" numpy pyparsing psutil"
SITL_PKGS="python3-dev"

# ===== GRAFICA / MAVPROXY (VERSIONE CORRETTA PER KALI) =====
if [[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]]; then
    PYTHON_PKGS+=" matplotlib scipy opencv-python pyyaml wxpython"
    
    # Pacchetti base sempre disponibili
    SITL_PKGS+=" xterm xfonts-base libgtk-3-dev python3-wxgtk4.0 \
fonts-freefont-ttf libfreetype6-dev libportmidi-dev"
    
    # SFML/CSFML - usa solo i pacchetti -dev senza versioni specifiche
    SITL_PKGS+=" libcsfml-dev libsfml-dev"
    
    # Prova a trovare libpng (potrebbe essere libpng-dev o libpng16-16)
    if apt-cache show libpng16-16 >/dev/null 2>&1; then
        SITL_PKGS+=" libpng16-16"
    elif apt-cache show libpng-dev >/dev/null 2>&1; then
        SITL_PKGS+=" libpng-dev"
    fi
    
    # SDL - controlla disponibilità
    if apt-cache show libsdl1.2-dev >/dev/null 2>&1; then
        SITL_PKGS+=" libsdl-image1.2-dev libsdl-mixer1.2-dev libsdl-ttf2.0-dev libsdl1.2-dev"
    else
        print_yellow "SDL 1.2 packages not found, trying SDL2..."
        SITL_PKGS+=" libsdl2-dev libsdl2-image-dev libsdl2-mixer-dev libsdl2-ttf-dev"
    fi
    
    # wxWidgets - controlla versione disponibile
    if apt-cache show libwxgtk3.2-dev >/dev/null 2>&1; then
        SITL_PKGS+=" libwxgtk3.2-dev"
    elif apt-cache show libwxgtk3.0-gtk3-dev >/dev/null 2>&1; then
        SITL_PKGS+=" libwxgtk3.0-gtk3-dev"
    fi
fi

[[ $SKIP_AP_COV_ENV -ne 1 ]] && COVERAGE_PKGS="lcov gcovr"

# ===== CONTROLLI REALI SU REALPATH E LIBTOOL =====
if ! command -v realpath >/dev/null 2>&1; then
    print_yellow "Installing realpath..."
    $APT_GET install realpath
    print_green "realpath installed successfully!"
fi

if apt-cache search -n '^libtool-bin' | grep -q libtool-bin; then
    SITL_PKGS+=" libtool-bin"
fi

if ! package_is_installed "pkg-config"; then
    print_yellow "Installing pkg-config..."
    $APT_GET install pkg-config
    print_green "pkg-config installed successfully!"
fi

SITL_PKGS+=" libpython3-stdlib ppp"

# ===== AGGIUNTA AL GRUPPO DIALOUT =====
if ! groups $USER | grep -q '\bdialout\b'; then
    heading "Adding user to dialout group"
    sudo usermod -a -G dialout $USER
    print_green "User added to dialout group (relog required)."
else
    print_green "User already in dialout group."
fi

# ===== INSTALLAZIONE PACCHETTI =====
heading "Installing packages"
print_yellow "Installing base packages..."
$APT_GET install $BASE_PKGS $ARM_LINUX_PKGS $COVERAGE_PKGS

# Installa SITL_PKGS uno alla volta per evitare che un pacchetto mancante blocchi tutto
print_yellow "Installing SITL packages (one by one to handle missing packages)..."
for pkg in $SITL_PKGS; do
    if $APT_GET install $pkg 2>/dev/null; then
        print_green "✓ Installed $pkg"
    else
        print_yellow "⚠ Skipped $pkg (not available)"
    fi
done

print_green "Packages installed successfully!"

[[ $SKIP_AP_GRAPHIC_ENV -ne 1 ]] && { heading "Rebuilding font cache"; fc-cache; print_green "Font cache rebuilt!"; }

# ===== DOCKER CHECK =====
heading "Check if we are inside docker environment..."
IS_DOCKER=false
if [[ ${AP_DOCKER_BUILD:-0} -eq 1 ]] || [[ -f /.dockerenv ]] || grep -Eq '(lxc|docker)' /proc/1/cgroup 2>/dev/null; then
    IS_DOCKER=true
fi
print_yellow "Docker environment: $IS_DOCKER"

SHELL_LOGIN=".profile"
if $IS_DOCKER; then
    print_yellow "Inside docker, adjusting environment file"
    SHELL_LOGIN=".ardupilot_env"
    echo "# ArduPilot env file. Need to be loaded by your Shell." > ~/$SHELL_LOGIN
fi

SCRIPT_DIR="$(dirname "$(realpath "${BASH_SOURCE[0]}")")"
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")

# ===== VENV SETUP =====
PIP_USER_ARGUMENT="--user"
PYTHON_VENV_PACKAGE="python3-venv"

heading "Setting up Python virtual environment"
$APT_GET install $PYTHON_VENV_PACKAGE

VENV_PATH=""
if [ -d "$ARDUPILOT_ROOT/venv-ardupilot" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/venv-ardupilot"
elif [ -d "$ARDUPILOT_ROOT/venv" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/venv"
elif [ -d "$ARDUPILOT_ROOT/.venv" ]; then
    VENV_PATH="$ARDUPILOT_ROOT/.venv"
else
    VENV_PATH="$HOME/venv-ardupilot"
    print_yellow "Creating new venv at $VENV_PATH"
    python3 -m venv --system-site-packages "$VENV_PATH"
fi

SOURCE_LINE="source $VENV_PATH/bin/activate"
$SOURCE_LINE
PIP_USER_ARGUMENT=""

print_green "Virtual environment setup complete!"

# ===== AGGIORNAMENTO PIP E PACCHETTI PYTHON =====
heading "Upgrading pip and setuptools"
$PIP install $PIP_USER_ARGUMENT -U pip packaging setuptools wheel
$PIP install $PIP_USER_ARGUMENT -U attrdict3

heading "Installing Python packages"
for PACKAGE in $PYTHON_PKGS; do
    print_yellow "Installing $PACKAGE..."
    if ! time $PIP install $PIP_USER_ARGUMENT -U $PACKAGE; then
        print_red "Warning: Failed to install $PACKAGE, continuing..."
    fi
done

# ===== REINSTALL PILLOW =====
heading "Reinstalling Pillow"
$PIP install --force-reinstall pillow

# ===== RIMOZIONE PACCHETTI CHE CREANO CONFLITTI =====
heading "Removing conflicting packages"
if package_is_installed "modemmanager"; then
    $APT_GET remove modemmanager
fi
if package_is_installed "brltty"; then
    $APT_GET remove brltty
fi
print_green "Conflicting packages removed if present!"

# ===== TOOLCHAIN ARM =====
function install_arm_none_eabi_toolchain() {
    ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
    case $(uname -m) in
        x86_64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    sudo chmod -R 755 gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2
                )
            fi
            CCACHE_PATH=$(which ccache)
            sudo ln -sf $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -sf $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            ;;
        aarch64)
            if [ ! -d $OPT/$ARM_ROOT ]; then
                (
                    cd $OPT
                    heading "Installing toolchain for STM32 Boards"
                    sudo wget --progress=dot:giga https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    sudo chmod -R 755 gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    sudo tar xjf gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                    sudo rm gcc-arm-none-eabi-10-2020-q4-major-aarch64-linux.tar.bz2
                )
            fi
            CCACHE_PATH=$(which ccache)
            sudo ln -sf $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-g++
            sudo ln -sf $CCACHE_PATH /usr/lib/ccache/arm-none-eabi-gcc
            ;;
    esac
}

if [[ -z "${DO_AP_STM_ENV}" ]] && read -p "Install ArduPilot STM32 toolchain [N/y]? " -n 1 -r; then
    echo
    [[ $REPLY =~ ^[Yy]$ ]] && install_arm_none_eabi_toolchain
fi

# ===== CONFIGURAZIONE AMBIENTE =====
heading "Adding ArduPilot Tools to environment"
exportline="export PATH=\"$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH\""
grep -Fxq "$exportline" ~/$SHELL_LOGIN 2>/dev/null || echo "$exportline" >> ~/$SHELL_LOGIN

exportline2="export PATH=/usr/lib/ccache:\$PATH"
grep -Fxq "$exportline2" ~/$SHELL_LOGIN 2>/dev/null || echo "$exportline2" >> ~/$SHELL_LOGIN

print_green "Environment configuration complete!"

if [[ $SKIP_AP_GIT_CHECK -ne 1 ]] && [ -d ".git" ]; then
    heading "Updating git submodules"
    cd "$ARDUPILOT_ROOT"
    git submodule update --init --recursive
    print_green "Git submodules updated!"
fi

if $IS_DOCKER; then
    echo "source ~/.ardupilot_env" >> ~/.bashrc
    print_yellow "Docker environment finalized!"
fi

heading "Installation complete!"
print_green "✓ All installations completed successfully!"
print_yellow "Please log out and log back in for group changes to take effect."
print_yellow "Then activate the Python virtual environment with:"
print_blue "  $SOURCE_LINE"
echo "---------- $0 end ----------"