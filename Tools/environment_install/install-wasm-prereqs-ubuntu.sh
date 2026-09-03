#!/bin/bash

set -e
set -x

EMSCRIPTEN_VERSION=${EMSCRIPTEN_VERSION:-6.0.8}
EMSDK_ROOT=${EMSDK_ROOT:-$HOME/emsdk}
SHELL_LOGIN=${SHELL_LOGIN:-.profile}

if ! python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
    echo "Emscripten SDK requires Python 3.10 or newer."
    exit 1
fi

if [ ! -d "$EMSDK_ROOT/.git" ]; then
    git clone https://github.com/emscripten-core/emsdk.git "$EMSDK_ROOT"
else
    if ! git -C "$EMSDK_ROOT" pull --ff-only; then
        echo "Warning: could not update emsdk; continuing with the existing checkout." >&2
    fi
fi

"$EMSDK_ROOT/emsdk" install "$EMSCRIPTEN_VERSION"
"$EMSDK_ROOT/emsdk" activate "$EMSCRIPTEN_VERSION"

exportline_wasm=". \"$EMSDK_ROOT/emsdk_env.sh\" >/dev/null"
grep -Fxq "$exportline_wasm" "$HOME/$SHELL_LOGIN" 2>/dev/null || echo "$exportline_wasm" >> "$HOME/$SHELL_LOGIN"
eval "$exportline_wasm"
