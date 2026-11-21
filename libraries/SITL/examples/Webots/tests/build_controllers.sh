#!/usr/bin/env bash
# Compile-check the Webots controllers without Webots installed.
#
# Builds each controller against tests/webots_stub, a header-and-link-only
# stand-in for the Webots C API.  This catches the class of breakage that made
# these controllers unbuildable on modern toolchains (tentative definitions in
# headers failing under -fno-common, implicit declarations, etc).
#
# It does NOT run a simulation.  For that, build with the real Webots
# Makefiles: set WEBOTS_HOME and run `make` in each controller directory.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$here/.."
stub="$here/webots_stub"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

CC="${CC:-gcc}"
CFLAGS="${CFLAGS:--std=c11 -Wall -Wextra -Wno-unused-parameter -Werror}"

common="$root/controllers/common"

status=0
for dir in "$root"/controllers/ardupilot_SITL_*/; do
    name="$(basename "$dir")"
    printf '==> %s\n' "$name"
    if ! $CC $CFLAGS -I"$stub" -I"$dir" -I"$common" \
         "$dir"/*.c "$common"/*.c "$stub"/webots_stub.c -lm -o "$out/$name"; then
        status=1
        printf '    FAILED\n'
    else
        printf '    ok\n'
    fi
done

exit $status
