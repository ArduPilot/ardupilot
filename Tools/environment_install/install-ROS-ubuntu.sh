#!/usr/bin/env bash
set -euo pipefail

# ROS 2 + Gazebo setup for ArduPilot SITL on Ubuntu. Companion to
# install-prereqs-ubuntu.sh and install-prereqs-arch.sh: same options
# (-y -q -s), same promise (no hardcoded sudo, no unprompted shell-config
# edits, refuses to run under sudo), same logging.
#
# Ubuntu-only: ROS 2 publishes apt packages for Ubuntu, not Debian. The
# distro is picked from the Ubuntu codename (jammy → humble, noble → jazzy,
# plucky → kilted, questing/resolute → rolling); use -r DISTRO to override.
#
# ROS 1 (Noetic on focal) is deliberately not supported: focal is EOL, and
# ArduPilot's ardupilot_ros is ROS 2 only. Debian codenames (bookworm,
# trixie, forky) are rejected for the same reason apt has no ROS 2 packages
# for them.
#
# Left out on purpose (each checked against ardupilot_ros / mavros source,
# not assumed): the ROS 1 rosdep/rosinstall/catkin tools, ROS_MASTER_URI
# and ROS_HOSTNAME (ROS 1 TCP coordination; ROS 2 uses DDS), the
# python-is-python3 shim, xterm (only used by the twins' SITL launch, not
# here), and the Gz Sim ISO build from source (apt's gz-garden is what the
# current ardupilot_gazebo CMakeLists expects).

# Configuration flags
ASSUME_YES=false
QUIET=false
SKIP_SYSTEM=false
ROS_DISTRO_OVERRIDE=""

# Parse command-line options
#   -r DISTRO  use this ROS 2 distro instead of the codename default
#   -s         skip the system part (apt, repo setup, ROS/Gazebo packages)
OPTIND=1
while getopts "yqsr:" opt; do
    case "$opt" in
        y)  ASSUME_YES=true ;;
        q)  QUIET=true ;;
        s)  SKIP_SYSTEM=true ;;
        r)  ROS_DISTRO_OVERRIDE="$OPTARG" ;;
        *)  exit 1 ;;
    esac
done

# Colors, but only when stdout is a terminal (CI/cron/pipes have no tty).
if [[ -t 1 ]] && command -v tput >/dev/null 2>&1; then
    red=$(tput setaf 1)
    green=$(tput setaf 2)
    reset=$(tput sgr0)
else
    red=""; green=""; reset=""
fi
sep="##############################################"

# Shell config file with shell detection. Empty CONFIG_FILE means "unsupported
# shell": print the lines for the user instead of appending them elsewhere.
case "$(basename "${SHELL:-}")" in
    bash) CONFIG_FILE="$HOME/.bashrc" ;;
    zsh)  CONFIG_FILE="$HOME/.zshrc"  ;;
    *)    CONFIG_FILE=""              ;;
esac

# Workspace roots. Both are under $HOME by default; override to taste.
WS_ROOT="${WS_ROOT:-$HOME/ardupilot-ws}"
GZ_WS="${GZ_WS:-$HOME/ardupilot_gazebo_ws}"

# Helper functions
function log() { $QUIET || echo "$@"; }
function warn() { echo "${red}$*${reset}" >&2; }
function heading() { log "$sep"; log "$*"; log "$sep"; }

function maybe_prompt_user() {
    if $ASSUME_YES; then return 0; fi
    local reply=""
    read -rp "$1" reply || true
    [[ $reply =~ ^[Yy]$ ]]
}

function add_config_entry() {
    local line="$1" prompt="$2"
    if [[ -z "$CONFIG_FILE" ]]; then
        log "Unsupported shell. Add this line to your shell config yourself:"
        log "    $line"
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

# The Ubuntu codename this system is based on. Empty when unknown.
function release_codename() {
    local UBUNTU_CODENAME="" VERSION_CODENAME=""
    if [[ -r "${OS_RELEASE:-/etc/os-release}" ]]; then
        # shellcheck source=/dev/null
        source "${OS_RELEASE:-/etc/os-release}"
    fi
    echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}"
}

# Default ROS 2 distro for a codename, empty if unsupported.
function default_ros_distro() {
    case "$1" in
        jammy)             echo "humble"  ;;
        noble)             echo "jazzy"   ;;
        plucky)            echo "kilted"  ;;
        questing|resolute) echo "rolling" ;;
        *)                 echo ""        ;;
    esac
}

# Privilege escalation (same shape as the twins).
if [[ $EUID -eq 0 && -n "${SUDO_USER:-}" ]]; then
    warn "FATAL: do not run this script with sudo."
    warn "Run as your normal user; it elevates only the steps that need root."
    exit 1
fi

if $SKIP_SYSTEM; then
    SUDO=()
elif [[ $EUID -eq 0 ]]; then
    SUDO=()
elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
elif command -v doas >/dev/null 2>&1; then
    SUDO=(doas)
elif command -v run0 >/dev/null 2>&1; then
    SUDO=(run0)
else
    warn "FATAL: root required, but no sudo, doas or run0 was found."
    exit 1
fi

if [[ "${SUDO[0]-}" == "sudo" ]]; then
    sudo -v
fi

# Main execution
log "=== ArduPilot ROS 2 + Gazebo setup ==="

# Resolve ROS 2 distro (always, so -s also validates the environment).
CODENAME=$(release_codename)
if [[ -z "$ROS_DISTRO_OVERRIDE" ]]; then
    ROS_DISTRO=$(default_ros_distro "$CODENAME")
else
    ROS_DISTRO=$(echo "$ROS_DISTRO_OVERRIDE" | tr '[:upper:]' '[:lower:]')
fi
if [[ -z "$ROS_DISTRO" ]]; then
    warn "FATAL: no ROS 2 distro for '${CODENAME:-unknown}'."
    warn "Supported codenames: jammy, noble, plucky, questing, resolute."
    warn "Pass -r DISTRO to force one (e.g. -r humble)."
    exit 1
fi
log "Release: ${CODENAME:-unknown} → ROS 2 $ROS_DISTRO"

# System packages (skipped by -s)
if ! $SKIP_SYSTEM; then
    heading "Installing base tools"
    APT_OPTS=()
    APT_ENV=()
    if $ASSUME_YES || $QUIET; then
        APT_OPTS+=(--assume-yes)
        APT_ENV=(env DEBIAN_FRONTEND=noninteractive)
    fi
    if $QUIET; then APT_OPTS+=(-qq); fi
    "${SUDO[@]}" "${APT_ENV[@]}" apt-get update "${APT_OPTS[@]}"
    "${SUDO[@]}" "${APT_ENV[@]}" apt-get install "${APT_OPTS[@]}" \
        software-properties-common curl gnupg lsb-release \
        build-essential cmake git wget

    "${SUDO[@]}" add-apt-repository -y universe

    heading "Adding ROS 2 apt repository"
    # Modern keyring layout. apt-key is removed in Ubuntu 24.04+.
    if [[ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]]; then
        curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
            | "${SUDO[@]}" tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
    fi
    ROS_APT_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $CODENAME main"
    echo "$ROS_APT_LINE" | "${SUDO[@]}" tee /etc/apt/sources.list.d/ros2-latest.list >/dev/null

    "${SUDO[@]}" "${APT_ENV[@]}" apt-get update "${APT_OPTS[@]}"

    heading "Installing ROS 2 $ROS_DISTRO"
    "${SUDO[@]}" "${APT_ENV[@]}" apt-get install "${APT_OPTS[@]}" \
        "ros-$ROS_DISTRO-desktop" \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        "ros-$ROS_DISTRO-mavros"

    # rosdep: only init when not already initialized; || true hid real failures.
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        "${SUDO[@]}" rosdep init
    fi

    heading "Installing GeographicLib datasets (needed by mavros)"
    GEO_SCRIPT=$(mktemp)
    # -f so a failed download doesn't leave an empty file that `bash` rejects
    # with a confusing "unexpected EOF" message.
    curl -sSLf -o "$GEO_SCRIPT" \
        https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    "${SUDO[@]}" bash "$GEO_SCRIPT"
    rm -f "$GEO_SCRIPT"

    heading "Installing Gazebo dependencies"
    "${SUDO[@]}" "${APT_ENV[@]}" apt-get install "${APT_OPTS[@]}" \
        gz-garden rapidjson-dev
fi

# rosdep update: no root needed, runs in -s mode too so the workspace build
# below can resolve its dependencies.
rosdep update

# Source ROS 2. -s mode requires it to be already installed.
if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    warn "FATAL: /opt/ros/$ROS_DISTRO/setup.bash not found."
    warn "Run without -s to install ROS 2 first."
    exit 1
fi
# shellcheck source=/dev/null
source "/opt/ros/$ROS_DISTRO/setup.bash"

heading "Setting up ROS 2 workspace at $WS_ROOT"
if [[ ! -d "$WS_ROOT" ]]; then
    mkdir -p "$WS_ROOT/src"
    pushd "$WS_ROOT" >/dev/null
    pushd src >/dev/null
    git clone https://github.com/ArduPilot/ardupilot_ros.git
    popd >/dev/null
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
    popd >/dev/null
else
    log "Workspace already exists, skipping."
fi

heading "Setting up ardupilot_gazebo at $GZ_WS"
if [[ ! -d "$GZ_WS" ]]; then
    # The Gz apt packages were installed above with the other system packages.
    mkdir -p "$GZ_WS/src"
    pushd "$GZ_WS/src" >/dev/null
    git clone https://github.com/ArduPilot/ardupilot_gazebo
    pushd ardupilot_gazebo >/dev/null
    mkdir -p build
    pushd build >/dev/null
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    make -j4
    popd >/dev/null; popd >/dev/null; popd >/dev/null
else
    log "ardupilot_gazebo already exists, skipping."
fi

heading "Shell configuration"
add_config_entry "source /opt/ros/$ROS_DISTRO/setup.bash" \
    "Auto-source ROS 2 $ROS_DISTRO"
add_config_entry "source $WS_ROOT/install/setup.bash" \
    "Auto-source ArduPilot ROS workspace"
add_config_entry "export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_WS/src/ardupilot_gazebo/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH}" \
    "Add ardupilot_gazebo plugins to GZ_SIM_SYSTEM_PLUGIN_PATH"
add_config_entry "export GZ_SIM_RESOURCE_PATH=$GZ_WS/src/ardupilot_gazebo/models:$GZ_WS/src/ardupilot_gazebo/worlds:\${GZ_SIM_RESOURCE_PATH}" \
    "Add ardupilot_gazebo models/worlds to GZ_SIM_RESOURCE_PATH"

heading "Setup complete"
log ""
log "Recommended actions:"
if [[ -n "$CONFIG_FILE" ]]; then
    log "1. Reload your shell config: source $CONFIG_FILE"
else
    log "1. Add the lines printed above to your shell config"
fi
log "2. Verify ROS 2: ros2 --version"
log "3. Verify the workspace: source $WS_ROOT/install/setup.bash && ros2 pkg list | grep ardupilot"
