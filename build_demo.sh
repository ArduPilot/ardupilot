#!/bin/bash
set -e

APPLET=libraries/AP_Module/applets/aerobatics
SINE=libraries/AP_Module/examples/sine
TARGET=thumbv7em-none-eabihf

# Use the builders' PATH defaults unless the environment selects a toolchain.
if [[ -n ${ARM_FC:-} ]]; then
    export FC="$ARM_FC"
fi
if [[ -n ${ADA_ARM_INCLUDE_PATH:-} ]]; then
    export ADA_INCLUDE_PATH="$ADA_ARM_INCLUDE_PATH"
fi

build_pids=()
build_names=()

start_build()
{
    local name=$1
    shift
    "$@" &
    build_pids+=("$!")
    build_names+=("$name")
}

start_build "aerobatics rust" python3 Tools/modules/build.py "$APPLET/main.rs" aerobatics_rust_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug --opt-level 1 \
    --stack-size 16384 --heap-size 26000 "$@"

start_build "aerobatics ada" python3 "$APPLET/ada/build_airshow.py" aerobatics_ada_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 16384 --heap-size 4096 "$@"

start_build "aerobatics f90" python3 "$APPLET/f90/build.py" aerobatics_f90_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 16384 --heap-size 32768 "$@"

start_build "lua" python3 Tools/modules/lua/build.py lua-h7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 24576 --heap-size 131072 "$@"

for source in rust/sine.rs f90/sine.f90 ada/sine.adb; do
    language=${source%%/*}
    start_build "sine $language" python3 Tools/modules/build.py "$SINE/$source" "sine_${language}_H7.apmod" \
        --target "$TARGET" --cpu cortex-m7 --debug \
        --stack-size 8192 --heap-size 4096 "$@"
done

build_status=0
for index in "${!build_pids[@]}"; do
    if ! wait "${build_pids[$index]}"; then
        printf 'Build failed: %s\n' "${build_names[$index]}" >&2
        build_status=1
    fi
done
if (( build_status != 0 )); then
    exit "$build_status"
fi

ls -ltr -- *.apmod
