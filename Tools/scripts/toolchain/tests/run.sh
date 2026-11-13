#!/usr/bin/env bash
set -euo pipefail
export CCACHE_DISABLE=1

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)
BUILD_SCRIPT=$ROOT_DIR/build-toolchain.py
SYMBOL_VERIFIER=$ROOT_DIR/scripts/verify-newlib-symbols.sh

fail()
{
    echo "FAIL: $*" >&2
    exit 1
}

for script in "$ROOT_DIR"/scripts/*.sh "$ROOT_DIR"/tests/*.sh; do
    bash -n "$script"
done
python3 -m py_compile "$BUILD_SCRIPT"

profiles=$(python3 "$BUILD_SCRIPT" --list-profiles)
grep -qx gcc-16.1 <<<"$profiles" || fail "gcc-16.1 profile is not listed"
grep -qx ardupilot-10 <<<"$profiles" || fail "ardupilot-10 profile is not listed"
python3 "$BUILD_SCRIPT" --show-profile gcc-16.1 | grep -q '^GCC_VERSION=16.1.0$' || \
    fail "gcc-16.1 GCC version is wrong"
python3 "$BUILD_SCRIPT" --help | grep -q -- '--download-only' || \
    fail "help output is incomplete"
python3 "$BUILD_SCRIPT" --help | grep -q -- '--newlib-only' || \
    fail "library-only rebuild option is missing"
python3 "$BUILD_SCRIPT" --help | grep -q -- '--package-only' || \
    fail "package-only option is missing"
for target in all-binutils all-gas all-ld; do
    grep -q "\"$target\"" "$BUILD_SCRIPT" || \
        fail "binutils target is missing: $target"
done
grep -q -- '--disable-newlib-reent-check-verify' "$BUILD_SCRIPT" || \
    fail "newlib internal allocation checks must not pull in abort"
if grep -q -- '--enable-newlib-reent-check-verify' "$BUILD_SCRIPT"; then
    fail "newlib internal allocation checks unexpectedly enabled"
fi
grep -q '"CFLAGS_FOR_TARGET": "-O2 -g -DNDEBUG' "$BUILD_SCRIPT" || \
    fail "full newlib must compile out internal assert calls"
grep -q '"CFLAGS_FOR_TARGET": "-Os -g -DNDEBUG' "$BUILD_SCRIPT" || \
    fail "nano newlib must compile out internal assert calls"
grep -q 'patches.*newlib-' "$BUILD_SCRIPT" || \
    fail "version-specific newlib patch directory is not selected"
grep -q 'apply_newlib_patches' "$BUILD_SCRIPT" || \
    fail "version-specific newlib patches are not applied"
grep -q 'verify_newlib_symbols' "$BUILD_SCRIPT" || \
    fail "newlib archive symbol verification is not part of the build"
grep -q 'self.env.pop("BASH_ENV", None)' "$BUILD_SCRIPT" || \
    fail "non-interactive bash startup files must not contaminate configure"
if grep -q 'sanitize_newlib\|sanitize-newlib' "$BUILD_SCRIPT"; then
    fail "newlib must be built without forbidden objects, not rewritten after installation"
fi
for version in 3.3.0 4.6.0.20260123; do
    patch_dir=$ROOT_DIR/patches/newlib-$version
    [[ -f $patch_dir/0001-disable-internal-asserts.patch ]] || \
        fail "newlib $version internal assert patch is missing"
    [[ -f $patch_dir/0002-omit-forbidden-interfaces.patch ]] || \
        fail "newlib $version forbidden-interface patch is missing"
done
if grep -R -q 'OBJECTS :=' "$ROOT_DIR"/patches/newlib-*; then
    fail "newlib object filters must be deferred until OBJEXT is defined"
fi
install_count=$(grep -c 'self.make(build, "install", \*overrides, build_env=build_env, jobs=1)' \
    "$BUILD_SCRIPT")
[[ $install_count == 2 ]] || fail "full and nano newlib installs must be serial"
if sed -n '/^    def build_gcc_nano/,/^    def build_gdb/p' "$BUILD_SCRIPT" | \
    grep -q 'self.make(build, "install-gcc"'
then
    fail "nano staging prefix must not install a non-standalone GCC driver"
fi
grep -q 'x86_64-linux' "$BUILD_SCRIPT" || fail "Linux package host tag is missing"

required_blacklist=(
    abort exit _Exit quick_exit system
    _sbrk _sbrk_r _malloc_r _calloc_r _free_r ftell realloc
    fopen fflush fwrite fread fputs fgets clearerr fseek ferror fclose tmpfile
    getc ungetc feof freopen remove vfprintf vfprintf_r fscanf
    _gettimeofday _times _times_r _gettimeofday_r time clock setjmp gmtime
)
for symbol in "${required_blacklist[@]}"; do
    grep -qxF "$symbol" "$ROOT_DIR/config/forbidden-newlib-symbols.txt" || \
        fail "ArduPilot ChibiOS blacklist symbol is not forbidden: $symbol"
done

test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
prefix=$test_dir/prefix
mkdir -p "$prefix/bin" "$prefix/arm-none-eabi/lib/thumb/test"
ln -s "$(command -v nm)" "$prefix/bin/arm-none-eabi-nm"

printf '%s\n' 'void abort(void) { for (;;) {} }' >"$test_dir/abort.c"
printf '%s\n' 'void _sbrk(void) { for (;;) {} }' >"$test_dir/sbrk.c"
printf '%s\n' 'int safe_function(void) { return 42; }' >"$test_dir/safe.c"
cc -c "$test_dir/abort.c" -o "$test_dir/abort.o"
cc -c "$test_dir/sbrk.c" -o "$test_dir/sbrk.o"
cc -c "$test_dir/safe.c" -o "$test_dir/safe.o"
ar cr "$prefix/arm-none-eabi/lib/libc.a" "$test_dir/safe.o"
cp "$prefix/arm-none-eabi/lib/libc.a" \
    "$prefix/arm-none-eabi/lib/thumb/test/libc_nano.a"

printf '%s\n' abort _sbrk >"$test_dir/policy"
$SYMBOL_VERIFIER "$prefix" "$test_dir/policy"

ar cr "$prefix/arm-none-eabi/lib/libforbidden-test.a" \
    "$test_dir/abort.o" "$test_dir/sbrk.o" "$test_dir/safe.o"
if $SYMBOL_VERIFIER "$prefix" "$test_dir/policy" >"$test_dir/verifier.out" 2>&1; then
    fail "forbidden newlib symbols were not rejected"
fi
grep -q 'forbidden definition: abort' "$test_dir/verifier.out" || \
    fail "abort violation was not reported"
grep -q 'forbidden definition: _sbrk' "$test_dir/verifier.out" || \
    fail "_sbrk violation was not reported"
nm -g --defined-only "$prefix/arm-none-eabi/lib/libforbidden-test.a" \
    >"$test_dir/bad-archive-symbols"
grep -qw abort "$test_dir/bad-archive-symbols" || \
    fail "symbol verification unexpectedly rewrote the archive"
rm -f "$prefix/arm-none-eabi/lib/libforbidden-test.a"
$SYMBOL_VERIFIER "$prefix" "$test_dir/policy"

for archive in \
    "$prefix/arm-none-eabi/lib/libc.a" \
    "$prefix/arm-none-eabi/lib/thumb/test/libc_nano.a"
do
    nm -g --defined-only "$archive" >"$test_dir/safe-archive-symbols"
    grep -qw safe_function "$test_dir/safe-archive-symbols" || \
        fail "safe archive member was removed"
done

echo "All script tests passed"
