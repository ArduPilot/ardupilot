#!/bin/bash

# Define ANSI colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BRIGHT_RED='\033[0;91m'
BRIGHT_GREEN='\033[0;92m'
BRIGHT_YELLOW='\033[0;93m'
BRIGHT_BLUE='\033[0;94m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Check operation system
HOST_OS=$(uname)
SHELL_PROFILE=~/.bashrc # Ubuntu by default
PICO_DIR=~/pico
PICOTOOL_REPO_PATH=""
SDK_PATH=""
SDK_BRANCH_NAME=master
SDK_COMMIT_SHA=""
SDK_GIT_TAG=2.2.0
FREERTOS_PATH=""
FREERTOS_BRANCH_NAME=main
FREERTOS_COMMIT_SHA="fed39c5ea7483dde7bf7c92950d49e6b75539f5a"
FREERTOS_GIT_TAG=""

# Display error messages in red and exit
log_error_and_exit() {
    echo -e "${BRIGHT_RED}[ERROR]:${RED} $1"${NC} >&2
    exit 1
}

# Display success messages in green
log_success() {
    echo -e "${BRIGHT_GREEN}[SUCCESS]:${GREEN} $1${NC}"
}

# Display informational messages in blue
log_info() {
    echo -e "${BRIGHT_BLUE}[INFO]:${BLUE} $1${NC}"
}

# Display warnings in yellow
log_warning() {
    echo -e "${BRIGHT_YELLOW}[WARNING]:${YELLOW} $1${NC}"
}

log_debug() {
    echo -e "${GREEN}[DEBUG]: $1${NC}"
}

begin() {
    log_info "--- Running the Raspberry Pi Pico SDK installation script ---"
}

end() {
    log_success "The installation of the Raspberry Pi Pico C/C++ SDK has been completed successfully!"
}

check_root_dir() {
    if [ ! -d modules ]; then
        log_error_and_exit "This script needs to be run from the root of your repo, sorry, giving up."
    fi
    echo `ls modules`
    cd modules

    if [ ! -d pico ]; then
        log_warning "Did not find modules/pico folder, making it." 
        mkdir -p -v pico
    else
        log_info "Found modules/pico folder"
    fi
    cd pico
    PICO_DIR=`pwd`

    # Expand the path to be absolute (e.g. ~ in /home/user)
    PICO_DIR=$(eval echo "$PICO_DIR")
    SDK_PATH="$PICO_DIR/pico-sdk"
    FREERTOS_PATH="$PICO_DIR/FreeRTOS-Kernel"
    PICOTOOL_REPO_PATH="$PICO_DIR/picotool"
    LITTLEFS_PATH="$PICO_DIR/littlefs-lib"

    log_debug "PICO_DIR: $PICO_DIR"
    log_debug "SDK_PATH: $SDK_PATH"
    log_debug "FREERTOS_PATH: $FREERTOS_PATH"
    log_debug "PICOTOOL_REPO_PATH: $PICOTOOL_REPO_PATH"
    log_debug "LITTLEFS_PATH: $LITTLEFS_PATH"
}

# Install dependencies and toolchain
install_toolchain() {
    log_info "*** (STAGE 1): Installing system dependencies and toolchain..."

    if [ "$HOST_OS" == "Linux" ]; then
        # Check if apt is present (for Ubuntu)
        if command -v apt-get &> /dev/null; then
            sudo apt-get update || log_error_and_exit "Failed to update apt package list."
            sudo apt-get install -y make cmake gcc g++ libnewlib-arm-none-eabi build-essential libstdc++-arm-none-eabi-newlib gcc-arm-none-eabi libusb-1.0-0-dev pkg-config || log_error_and_exit "Could not install apt dependencies."
        else
            log_error_and_exit "Linux system does not use apt. Please install dependencies manually using your package manager."
        fi
    elif [ "$HOST_OS" == "Darwin" ]; then
        # MacOS
        # Install Command Line Tools for Xcode
        xcode-select --install &> /dev/null
        # Check and install Homebrew
        if ! command -v brew &> /dev/null; then
            log_info "Installing Homebrew..."
            /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)" || log_error_and_exit "Unable to install Homebrew."
        fi
        brew install cmake || log_error_and_exit "Unable to install cmake via Homebrew."
        brew tap ArmMbed/homebrew-formulae || log_error_and_exit "Unable to add tap ArmMbed/homebrew-formulae."
        brew install arm-none-eabi-gcc || log_error_and_exit "Unable to install arm-none-eabi-gcc via Homebrew. Try installing it manually from the official website."
    else
        log_error_and_exit "Unsupported operating system: $HOST_OS. The script only works on Linux (Ubuntu) and MacOS."
    fi
    log_success "System dependencies and toolchain installed."
}

# Clone PICO-SDK and initialize submodules
clone_sdk() {
    mkdir -p "$PICO_DIR" || log_error_and_exit "Unable to create directory $PICO_DIR."
    cd "$PICO_DIR" || log_error_and_exit "Unable to go to directory $PICO_DIR."
    log_success "The working directory $PICO_DIR has been created."

    log_info "*** (STAGE 2): Cloning PICO-SDK and initializing submodules..."

    if [ -d "$SDK_PATH" ]; then
        log_warning "SDK directory already exists. Skipping cloning. Updating submodules..."
    else
        git clone https://github.com/raspberrypi/pico-sdk.git --branch ${SDK_BRANCH_NAME} || log_error_and_exit "Unable to clone PICO-SDK."
    fi
    cd pico-sdk || log_error_and_exit "Could not change to existing pico-sdk directory."
    if [ -n "${SDK_GIT_TAG}" ]; then
        git checkout ${SDK_GIT_TAG} || log_error_and_exit "Unable to switch to ${SDK_GIT_TAG} tag."
    fi
    if [ -n "${SDK_COMMIT_SHA}" ]; then
        git checkout ${SDK_COMMIT_SHA} || log_error_and_exit "Unable to switch to ${SDK_COMMIT_SHA} commit."
    fi
    git submodule update --init || log_error_and_exit "Could not update submodules of existing PICO-SDK."
    log_success "PICO-SDK has been successfully downloaded and initialized."
}

# Clone FreeRTOS-Kernel (using the Raspberry Pi fork for better compatibility)
clone_freertos_kernel() {
    cd "$PICO_DIR" || log_error_and_exit "Unable to go to directory $PICO_DIR."
    log_info "Cloning FreeRTOS-Kernel..."
    if [ -d "$FREERTOS_PATH" ]; then
        log_warning "FreeRTOS-Kernel directory already exists. Skipping clone. Updating submodules..."
    else
        # Using the official FreeRTOS repo as recommended by guides
        git clone https://github.com/FreeRTOS/FreeRTOS-Kernel.git --branch ${FREERTOS_BRANCH_NAME} || log_error_and_exit "Unable to clone FreeRTOS-Kernel."
    fi
    cd FreeRTOS-Kernel || log_error_and_exit "Unable to change to FreeRTOS-Kernel directory."
    if [ -n "${FREERTOS_GIT_TAG}" ]; then
        git checkout ${FREERTOS_GIT_TAG} || log_error_and_exit "Unable to switch to ${FREERTOS_GIT_TAG} tag."
    fi
    if [ -n "${FREERTOS_COMMIT_SHA}" ]; then
        git checkout ${FREERTOS_COMMIT_SHA} || log_error_and_exit "Unable to switch to ${FREERTOS_COMMIT_SHA} commit."
    fi 
    git submodule update --init || log_error_and_exit "Unable to update FreeRTOS-Kernel submodules."
    log_success "FreeRTOS-Kernel successfully downloaded/updated."
}

update_env_var() {
    local name=$1
    local path=$2
    if ! grep -q "export $name=" "$SHELL_PROFILE"; then
        echo "export $name=$path" >> "$SHELL_PROFILE"
        log_success "$name added to $SHELL_PROFILE."
    else
        # Use sed to replace the existing line (works differently on Linux/macOS)
        if [ "$HOST_OS" == "Darwin" ]; then
            sed -i "" "s|export $name=.*|export $name=$path|" "$SHELL_PROFILE"
        else
            sed -i "s|export $name=.*|export $name=$path|" "$SHELL_PROFILE"
        fi
        log_success "$name updated in $SHELL_PROFILE."
    fi
    # Export for the current session
    export $name=$path
}

# Set the PICO_SDK_PATH, FREERTOS_KERNEL_PATH, PICO_FILESYSTEM_PATH environment variables
set_paths() {
    log_info "*** (STAGE 3): Setting the PICO_SDK_PATH environment variable..."
    if [ "$HOST_OS" == "Darwin" ]; then
        # MacOS uses zsh by default
        SHELL_PROFILE=~/.zshrc
        [ ! -f "$SHELL_PROFILE" ] && SHELL_PROFILE=~/.bash_profile
    fi
    if [ -n "$SHELL_PROFILE" ]; then
        update_env_var PICO_SDK_PATH "$SDK_PATH"
        update_env_var FREERTOS_KERNEL_PATH "$FREERTOS_PATH"

        log_info "Variables exported to the current session."
        log_warning "To use variables in new terminals, restart your terminal or run 'source $SHELL_PROFILE'."
    else
        log_error_and_exit "Could not find shell profile file to add PICO_SDK_PATH and FREERTOS_KERNEL_PATH. Set them manually."
    fi
}

install_picotool() {
    log_info "*** (STAGE 4): Installing Picotool..."
    # Check if picotool is already installed (e.g. via brew on macOS)
    if command -v picotool &> /dev/null && [ "$OS" == "Darwin" ]; then
        log_success "Picotool already found in system PATH (likely via Homebrew). Skipping build from source."
    else
        # Build picotool from source
        log_info "Building picotool from source..."
        if [ ! -d "$PICOTOOL_REPO_PATH" ]; then
          git clone https://github.com/raspberrypi/picotool.git "$PICOTOOL_REPO_PATH" || log_error_and_exit "Unable to clone picotool repository."
        fi

        cd "$PICOTOOL_REPO_PATH" || log_error_and_exit "Unable to change directory to picotool repo."
        mkdir -p build || log_error_and_exit "Unable to create picotool build directory."
        cd build || log_error_and_exit "Unable to change to picotool build directory."

        # Use CMAKE_INSTALL_PREFIX=/usr/local so SDK can find it later
        cmake -DCMAKE_INSTALL_PREFIX=/usr/local .. || log_error_and_exit "Unable to configure picotool build with CMake."
        make -j$(nproc) || make || log_error_and_exit "Unable to compile picotool."
        sudo make install || log_error_and_exit "Unable to install picotool to /usr/local/bin. Sudo permission required."
        log_success "Picotool built and installed to /usr/local/bin."
    fi
    # Add udev rules for Linux so picotool can run without sudo
    if [ "$OS" == "Linux" ]; then
        log_info "Installing udev rules for picotool (requires sudo)..."
        sudo cp "$PICOTOOL_REPO_PATH/udev/60-picotool.rules" /etc/udev/rules.d/ || log_error_and_exit "Unable to copy udev rules."
        sudo udevadm control --reload-rules || log_error_and_exit "Unable to reload udev rules."
        sudo udevadm trigger || log_error_and_exit "Unable to trigger udevadm."
        log_success "udev rules installed. Reconnect your Pico to use picotool without sudo."
    fi
}

while getopts ":d:-directory:" opt; do
  case $opt in
    d)
      PICO_DIR=$OPTARG
      ;;
    directory)
      PICO_DIR=$OPTARG
      ;;
    \?)
      log_error_and_exit "Invalid parameter: -$OPTARG. Use -d <path> or --directory <path>."
      ;;
    :)
      log_error_and_exit "The -$OPTARG parameter requires an argument (directory path)."
      ;;
  esac
done

check_root_dir
begin
install_toolchain
clone_sdk
clone_freertos_kernel
set_paths
install_picotool
end
