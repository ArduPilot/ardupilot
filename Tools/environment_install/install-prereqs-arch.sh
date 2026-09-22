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

# Get configuration file with shell detection.
# An empty CONFIG_FILE means "unsupported shell": print the lines for the user
# instead of appending them somewhere they will never be read.
case "$(basename "${SHELL:-}")" in
    bash) CONFIG_FILE="$HOME/.bashrc" ;;
    zsh)  CONFIG_FILE="$HOME/.zshrc"  ;;
    *)    CONFIG_FILE=""              ;;
esac

# Development Packages
BASE_PKGS=(base-devel gcc ccache git wget gsfonts tk grep tar)
SITL_PKGS=(python-pip python-setuptools python-wheel python-numpy python-scipy opencv python-wxpython)
PX4_PKGS=(lib32-glibc zip zlib ncurses)
PYTHON_PKGS=(lxml pymavlink MAVProxy opencv-python pexpect argparse matplotlib pyparsing geocoder pyserial empy==3.3.4 dronecan packaging setuptools wheel)

# GNU Toolchain for ARM Embedded Processors (see https://launchpad.net/gcc-arm-embedded/)
ARM_ROOT="gcc-arm-none-eabi-10-2020-q4-major"
ARM_TARBALL="$ARM_ROOT-x86_64-linux.tar.bz2"
ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"
ARM_TARBALL_CHECKSUM="21134caa478bbf5352e239fbc6e2da3038f8d2207e089efc96c3b55f1edcd618"

# Main directories
SCRIPT_DIR=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
ARDUPILOT_ROOT="${SCRIPT_DIR%/Tools/environment_install}"
ARDUPILOT_TOOLS_DIR="${SCRIPT_DIR%/environment_install}/autotest"
VENV_DIR="$ARDUPILOT_ROOT/.venv"
ARM_TOOLCHAIN_DIR="/opt/$ARM_ROOT"
ARM_TARBALL_PATH="/opt/$ARM_TARBALL"
CCACHE_DIR="/usr/lib/ccache"

# Helper functions
function log() {
    $QUIET || echo "$@"
}

function maybe_prompt_user() {
    if $ASSUME_YES; then
        return 0
    fi
    local reply=""
    # `read` returns non-zero on EOF (piped stdin, CI). Treat that as "no"
    # rather than letting it bubble up as a failure.
    read -rp "$1" reply || true
    [[ $reply =~ ^[Yy]$ ]]
}

function add_config_entry() {
    local line="$1" prompt="$2"

    if [[ -z "$CONFIG_FILE" ]]; then
        echo "Unsupported shell. Please add this line to your shell config yourself:"
        echo "    $line"
        return 0
    fi

    if grep -Fxq "$line" "$CONFIG_FILE" 2>/dev/null; then
        log "Already configured: $prompt"
        return 0
    fi

    if maybe_prompt_user "$prompt [N/y]?"; then
        echo "$line" >> "$CONFIG_FILE"
        log "Added to $CONFIG_FILE: $prompt"
    else
        log "Skipped: $prompt"
    fi
}

# Privilege escalation.
# Nothing is hardcoded to `sudo`: run as root and no escalation is used at all,
# otherwise take whichever tool the system actually has.
if [[ $EUID -eq 0 && -n "${SUDO_USER:-}" ]]; then
    echo "FATAL: do not run this script with sudo." >&2
    echo "Run it as your normal user. It elevates only the steps that need root," >&2
    echo "so the venv, the git checkout and your shell config stay yours." >&2
    exit 1
fi

if [[ $EUID -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
elif command -v doas >/dev/null 2>&1; then
    SUDO=(doas)
elif command -v run0 >/dev/null 2>&1; then
    SUDO=(run0)
else
    echo "FATAL: root privileges are required, but no sudo, doas or run0 was found." >&2
    exit 1
fi

# Ask for the password once, up front, instead of at a random point mid-install.
if [[ "${SUDO[0]-}" == "sudo" ]]; then
    sudo -v
fi

# Main execution
log "=== ArduPilot Development Environment Setup ==="

# User group setup.
# $USER is not exported in every context (cron, CI, `env -i`), which would be
# fatal under `set -u`. `id -nG` avoids a pipeline that `pipefail` can trip on.
CURRENT_USER="${USER:-$(id -un)}"
if [[ " $(id -nG "$CURRENT_USER") " != *" uucp "* ]]; then
    log "Adding user to uucp group..."
    "${SUDO[@]}" usermod -aG uucp "$CURRENT_USER"
fi

# System packages installation
log "Installing system packages..."
PACMAN_OPTS=(--needed)
if $QUIET; then
    PACMAN_OPTS+=(--color=auto --quiet)
else
    PACMAN_OPTS+=(--color=always)
fi
# -y and -q both mean "do not stop to ask me anything".
if $ASSUME_YES || $QUIET; then
    PACMAN_OPTS+=(--noconfirm)
fi
"${SUDO[@]}" pacman -Syu "${PACMAN_OPTS[@]}" "${BASE_PKGS[@]}" "${SITL_PKGS[@]}" "${PX4_PKGS[@]}"

# Python virtual environment setup
if [[ ! -d "$VENV_DIR" ]]; then
    log "Creating Python virtual environment..."
    if ! python3 -m venv --system-site-packages "$VENV_DIR"; then
        echo "FATAL: Failed to create virtual environment"
        exit 1
    fi
fi

# Keep the venv out of `git status` without touching the tracked .gitignore.
if [[ -d "$ARDUPILOT_ROOT/.git" ]]; then
    GIT_EXCLUDE="$ARDUPILOT_ROOT/.git/info/exclude"
    mkdir -p "$(dirname "$GIT_EXCLUDE")"
    if ! grep -Fxq '.venv/' "$GIT_EXCLUDE" 2>/dev/null; then
        echo '.venv/' >> "$GIT_EXCLUDE"
    fi
fi

# Install Python packages within the virtual environment
log "Installing Python packages..."
PIP_OPTS=(-U --no-warn-script-location)
if $QUIET; then
    PIP_OPTS+=(-q)
fi
"$VENV_DIR"/bin/python3 -m pip install "${PIP_OPTS[@]}" --upgrade pip
"$VENV_DIR"/bin/python3 -m pip install "${PIP_OPTS[@]}" "${PYTHON_PKGS[@]}"

# CCache setup
log "Configuring ccache symlinks for build optimizations..."
for compiler in arm-none-eabi-g++ arm-none-eabi-gcc; do
    if [[ ! -e "$CCACHE_DIR/$compiler" ]]; then
        "${SUDO[@]}" ln -sf /usr/bin/ccache "$CCACHE_DIR/$compiler"
    fi
done

# ARM toolchain installation.
# The whole block is skipped when the toolchain is already in place, so a
# re-run does not re-download ~130 MB.
if [[ ! -d "$ARM_TOOLCHAIN_DIR/bin" ]]; then
    log "Setting up ARM toolchain..."

    # Download with verification
    if [[ ! -f "$ARM_TARBALL_PATH" ]] || \
       [[ $(sha256sum "$ARM_TARBALL_PATH" | awk '{print $1}') != "$ARM_TARBALL_CHECKSUM" ]]; then
        log "Downloading ARM toolchain..."
        WGET_OPTS=(-O "$ARM_TARBALL_PATH")
        if $QUIET; then
            WGET_OPTS+=(--quiet)
        else
            WGET_OPTS+=(--progress=dot:giga)
        fi
        if ! "${SUDO[@]}" wget "${WGET_OPTS[@]}" "$ARM_TARBALL_URL"; then
            echo "FATAL: Unable to download the toolchain tarball!"
            "${SUDO[@]}" rm -f "$ARM_TARBALL_PATH"
            exit 1
        fi
    fi

    # Post-download verification
    ACTUAL_CHECKSUM=$(sha256sum "$ARM_TARBALL_PATH" | awk '{print $1}')
    if [[ "$ACTUAL_CHECKSUM" != "$ARM_TARBALL_CHECKSUM" ]]; then
        echo "FATAL: Checksum mismatch after download!"
        "${SUDO[@]}" rm -f "$ARM_TARBALL_PATH"
        exit 1
    fi

    # Extraction. --checkpoint counts 512-byte records, so 2048 is one dot
    # per MiB extracted.
    log "Extracting toolchain..."
    TAR_OPTS=(--extract --file="$ARM_TARBALL_PATH" --directory=/opt)
    if ! $QUIET; then
        TAR_OPTS+=(--checkpoint=2048 --checkpoint-action=dot)
    fi
    "${SUDO[@]}" tar "${TAR_OPTS[@]}"
    $QUIET || echo

    # Extraction validation
    if [[ ! -d "$ARM_TOOLCHAIN_DIR/bin" ]]; then
        echo "FATAL: Extraction failed - invalid tarball structure"
        exit 1
    fi

    # Validate ARM toolchain functionality.
    # a+rX sets execute on directories and on files that are already
    # executable, unlike a blanket 755.
    "${SUDO[@]}" chmod -R a+rX "$ARM_TOOLCHAIN_DIR"
    if ! "$ARM_TOOLCHAIN_DIR/bin/arm-none-eabi-gcc" --version &>/dev/null; then
        echo "FATAL: ARM toolchain failed to run"
        exit 1
    fi
    log "ARM toolchain installed!"
    "${SUDO[@]}" rm -f "$ARM_TARBALL_PATH"
else
    log "ARM toolchain already present at $ARM_TOOLCHAIN_DIR"
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
log "Initializing repository submodules..."
GIT_OPTS=(--init --recursive)
if $QUIET; then
    GIT_OPTS+=(--quiet)
fi
git -C "$ARDUPILOT_ROOT" submodule update "${GIT_OPTS[@]}"

# Final instructions
log ""
log "=== Setup complete ==="
log "Recommended actions:"
if [[ -n "$CONFIG_FILE" ]]; then
    log "1. Reload your shell config: source $CONFIG_FILE"
else
    log "1. Add the lines printed above to your shell config"
fi
log "2. Verify ARM toolchain in environment: which arm-none-eabi-gcc"
log "3. Activate Python virtual environment: source $VENV_DIR/bin/activate"
log "4. Log out and back in for group changes to take effect"
