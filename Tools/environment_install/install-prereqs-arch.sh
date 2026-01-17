#!/usr/bin/env bash
set -euo pipefail

# Configuration flags
ASSUME_YES=false
QUIET=false

# Parse command-line options
OPTIND=1
while getopts "yq" opt; do
    case "$opt" in
        y)  ASSUME_YES=true ;;
        q)  QUIET=true ;;
        *)  exit 1 ;;
    esac
done

# Get configuration file with shell detection
case "$(basename "$SHELL")" in
    bash) CONFIG_FILE="$HOME/.bashrc" ;;
    zsh)  CONFIG_FILE="$HOME/.zshrc"  ;;
    *)    CONFIG_FILE="/dev/null"     ;;
esac

# Development Packages
BASE_PKGS=(base-devel gcc ccache git wget gsfonts tk)
SITL_PKGS=(python-pip python-setuptools python-wheel python-numpy python-scipy opencv python-wxpython)
PX4_PKGS=(lib32-glibc zip zlib ncurses)
PYTHON_PKGS=(future lxml pymavlink MAVProxy opencv-python pexpect argparse matplotlib pyparsing geocoder pyserial empy==3.3.4 dronecan packaging setuptools wheel)

# GNU Toolchain for ARM Embedded Processors (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
ARM_TARBALL="$ARM_ROOT-x86_64-linux.tar.bz2"
ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"
ARM_TARBALL_CHECKSUM="21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618"

# Main directories
SCRIPT_DIR=$(dirname "$(realpath "$0")")
ARDUPILOT_TOOLS_DIR="${SCRIPT_DIR%/environment_install}/autotest"
VENV_DIR="$HOME/venv-ardupilot"
ARM_TOOLCHAIN_DIR="/opt/$ARM_ROOT"
CCACHE_DIR="/usr/lib/ccache"

# Helper functions
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

add_config_entry() {
    local line="$1" prompt="$2"
    if ! grep -Fxq "$line" "$CONFIG_FILE"; then
        if maybe_prompt_user "$prompt [N/y]?"; then
            echo "$line" >> "$CONFIG_FILE"
            echo "Added to $CONFIG_FILE"
            return 0
        fi
        echo "Skipped: $prompt"
        return 0
    fi
    echo "Already configured: $prompt"
    return 0
}

# Main execution
echo "=== ArduPilot Development Environment Setup ==="

# User group setup
if ! groups "$USER" | grep -q '\buucp\b'; then
    echo "Adding user to uucp group..."
    sudo usermod -aG uucp "$USER"
fi

# System packages installation
echo "Installing system packages..."
PACMAN_OPTS=(--needed)
if "$QUIET"; then
    PACMAN_OPTS+=(--color=auto --quiet --noconfirm)
else
    PACMAN_OPTS+=(--color=always)
fi
sudo pacman -Syu "${PACMAN_OPTS[@]}" "${BASE_PKGS[@]}" "${SITL_PKGS[@]}" "${PX4_PKGS[@]}"

# Python virtual environment setup
if [[ ! -d "$VENV_DIR" ]]; then
    echo "Creating Python virtual environment..."
    if ! python3 -m venv --system-site-packages "$VENV_DIR"; then
        echo "FATAL: Failed to create virtual environment"
        exit 1
    fi
fi

# Install Python packages within the virtual environment
echo "Installing Python packages..."
PIP_OPTS=(-U --no-warn-script-location)
"$QUIET" && PIP_OPTS+=(-q)
"$VENV_DIR"/bin/python3 -m pip install "${PIP_OPTS[@]}" --upgrade pip
"$VENV_DIR"/bin/python3 -m pip install "${PIP_OPTS[@]}" "${PYTHON_PKGS[@]}"

# CCache setup
echo "Configuring C Cache symlinks for build optimizations..."
for compiler in arm-none-eabi-g++ arm-none-eabi-gcc; do
    if [[ ! -f "$CCACHE_DIR/$compiler" ]]; then
        sudo ln -sf /usr/bin/ccache "$CCACHE_DIR/$compiler"
    fi
done

# ARM toolchain installation
echo "Setting up ARM toolchain..."

# Download with verification
if [[ ! -f "/opt/$ARM_TARBALL" ]] || \
    [[ $(sudo sha256sum "/opt/$ARM_TARBALL" | awk '{print $1}') != "$ARM_TARBALL_CHECKSUM" ]]; then
    echo "Downloading ARM toolchain..."
    WGET_OPTS=(-O "/opt/$ARM_TARBALL")
    if "$QUIET"; then
        WGET_OPTS+=(--quiet)
    else
        WGET_OPTS+=(--progress=dot:giga)
    fi
    if ! sudo wget "${WGET_OPTS[@]}" "$ARM_TARBALL_URL"; then
        echo "FATAL: Unable to download the toolchain tarball!"
        sudo rm -f "/opt/$ARM_TARBALL"
        exit 1
    fi
fi

# Post-download verification
ACTUAL_CHECKSUM=$(sudo sha256sum "/opt/$ARM_TARBALL" | awk '{print $1}')
if [[ "$ACTUAL_CHECKSUM" != "$ARM_TARBALL_CHECKSUM" ]]; then
    echo "FATAL: Checksum mismatch after download!"
    sudo rm -f "/opt/$ARM_TARBALL"
    exit 1
fi

# Secure extraction
if [[ ! -d "$ARM_TOOLCHAIN_DIR" ]]; then
    echo "Extracting toolchain..."
    TAR_OPTS=(--extract --file="/opt/$ARM_TARBALL" --directory="/opt")
    "$QUIET" || TAR_OPTS+=(--checkpoint="$(("$(stat -c %s "/opt/$ARM_TARBALL")"/1048576))" --checkpoint-action=.)
    sudo tar "${TAR_OPTS[@]}"
fi

# Extraction validation
if [[ ! -d "$ARM_TOOLCHAIN_DIR/bin" ]]; then
    echo "FATAL: Extraction failed - invalid tarball structure"
    exit 1
fi

# Validate ARM toolchain functionality
sudo chmod -R 755 "$ARM_TOOLCHAIN_DIR"
if ! "$ARM_TOOLCHAIN_DIR/bin/arm-none-eabi-gcc" --version &>/dev/null; then
    echo "FATAL: ARM toolchain failed"
    exit 1
else
    echo "ARM toolchain installed!"
    sudo rm "/opt/$ARM_TARBALL"
fi

# Environment configuration
CONFIG_ENTRIES=(
    "export PATH=$ARM_TOOLCHAIN_DIR/bin:\$PATH|Add ARM toolchain to PATH"
    "export PATH=$ARDUPILOT_TOOLS_DIR:\$PATH|Add ArduPilot test tools to PATH"
    "source $VENV_DIR/bin/activate|Auto-activate Python virtual environment"
)

for entry in "${CONFIG_ENTRIES[@]}"; do
    IFS='|' read -r line prompt <<< "$entry"
    add_config_entry "$line" "$prompt"
done

# Repository submodules initialization
echo "Initializing repository submodules..."
GIT_OPTS=(--init --recursive)
"$QUIET" && GIT_OPTS+=(--quiet)
git -C "${SCRIPT_DIR%/Tools/environment_install}" submodule update "${GIT_OPTS[@]}"

# Final instructions
echo -e "\n=== Setup complete ==="
echo "Recommended actions:"
echo "1. Reload your shell config: source ${CONFIG_FILE}"
echo "2. Verify ARM toolchain in environment: which arm-none-eabi-gcc"
echo "3. Activate Python virtual environment: source ${VENV_DIR}/bin/activate"
echo "4. Log out and back in for group changes to take effect"
