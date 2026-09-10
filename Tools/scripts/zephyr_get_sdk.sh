#!/usr/bin/env bash
#
# Install the Zephyr SDK toolchain, pinned and checksum-verified.
#
# Deliberately does NOT use `west sdk install`. This repository does not use
# west - see "No west" in libraries/AP_HAL_Zephyr/README.md and "Dependencies
# without west" in ARCHITECTURAL.md - and `west sdk` is not even a built-in: it
# is an extension command that only resolves inside a west workspace, so with
# Zephyr vendored as a git submodule it fails with
#
#     west: unknown command "sdk"; do you need to run this inside a workspace?
#
# regardless of the working directory, -z, or ZEPHYR_BASE.
#
# Same shape as Tools/renode/tests/fetch_renode.sh: a pinned version, a pinned
# SHA-256 per artifact, and a download that refuses to proceed if either fails
# to match. Only the ARM toolchain is fetched; nothing here targets the other
# architectures the SDK ships.

set -euo pipefail

version="${ZEPHYR_SDK_VERSION:-1.0.1}"
destination="${ZEPHYR_SDK_INSTALL_DIR:-$HOME/zephyr-sdk-$version}"
base_url="${ZEPHYR_SDK_BASE_URL:-https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v$version}"
toolchain="arm-zephyr-eabi"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Only Linux Zephyr SDK packages are supported by this helper" >&2
    exit 1
fi

case "$(uname -m)" in
    x86_64|amd64)
        host="linux-x86_64"
        sdk_sha256="ca9bc0ff66fafca1dac9d592a36d953cf16d096a9d09b1c0357f021cf9f6a7eb"
        toolchain_sha256="21b85981cb5a1818d9bc53d82af80f208946ec038b982ff1907287572ed3a634"
        ;;
    aarch64|arm64)
        host="linux-aarch64"
        sdk_sha256="d79c5bfc68e679488659bea289a4026e52a64f03338875c8c9c850fff13cee30"
        toolchain_sha256="b9805b691f2f0a8926c92694cae378d05ba07b76abca745e216fcc52753cc4d6"
        ;;
    *)
        echo "No published Zephyr SDK package for architecture $(uname -m)" >&2
        exit 1
        ;;
esac

# The version is part of the path, so an existing directory is the right
# version by construction. Re-running is a no-op, which matters in CI where
# this sits behind a cache.
if [[ -d "$destination" ]]; then
    echo "Zephyr SDK $version already present at $destination"
    echo "ZEPHYR_SDK_INSTALL_DIR=$destination"
    exit 0
fi

sdk_archive="zephyr-sdk-${version}_${host}_minimal.tar.xz"
toolchain_archive="toolchain_gnu_${host}_${toolchain}.tar.xz"

workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

fetch() {
    local name="$1" want="$2"
    echo "Fetching $name"
    if ! curl -fsSL --retry 3 -o "$workdir/$name" "$base_url/$name"; then
        echo "download failed: $base_url/$name" >&2
        exit 1
    fi
    # Verified before anything is unpacked: a truncated or substituted archive
    # must never reach the extract step.
    if ! echo "$want  $workdir/$name" | sha256sum -c --status; then
        echo "SHA-256 mismatch for $name" >&2
        echo "  expected $want" >&2
        echo "  got      $(sha256sum "$workdir/$name" | cut -d' ' -f1)" >&2
        exit 1
    fi
}

fetch "$sdk_archive" "$sdk_sha256"
fetch "$toolchain_archive" "$toolchain_sha256"

parent="$(dirname "$destination")"
mkdir -p "$parent"

echo "Unpacking the SDK into $destination"
tar -x -f "$workdir/$sdk_archive" -C "$parent"

unpacked="$parent/zephyr-sdk-$version"
if [[ "$unpacked" != "$destination" ]]; then
    mv "$unpacked" "$destination"
fi

# The toolchain unpacks INSIDE the SDK directory, which is where setup.sh -t
# would have put it. Doing it here keeps the download pinned rather than
# letting setup.sh fetch an unverified copy of its own.
echo "Unpacking the $toolchain toolchain"
tar -x -f "$workdir/$toolchain_archive" -C "$destination"

# Registers the SDK's CMake package so a build can find it without
# ZEPHYR_SDK_INSTALL_DIR being exported. -c only; -t would re-download the
# toolchain that was just verified and installed above.
if [[ -x "$destination/setup.sh" ]]; then
    echo "Registering the SDK CMake package"
    ( cd "$destination" && ./setup.sh -c )
fi

echo "Zephyr SDK $version installed at $destination"
echo "ZEPHYR_SDK_INSTALL_DIR=$destination"
