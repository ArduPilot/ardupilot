#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=common.sh
. "$SCRIPT_DIR/common.sh"

if (($# < 1 || $# > 2)); then
    echo "Usage: verify-toolchain.sh PREFIX [FORBIDDEN-SYMBOL-POLICY]" >&2
    exit 2
fi

prefix=$(absolute_path "$1")
policy=${2:-$SCRIPT_DIR/../config/forbidden-newlib-symbols.txt}
target=arm-none-eabi
bindir=$prefix/bin

for tool in \
    addr2line ar as c++ c++filt cpp elfedit g++ gcc gcc-ar gcc-nm gcc-ranlib \
    gdb gprof ld nm objcopy objdump ranlib readelf size strings strip
do
    [[ -x $bindir/$target-$tool ]] || die "missing tool: $target-$tool"
done

gcc=$bindir/$target-gcc
gxx=$bindir/$target-g++
nm=$bindir/$target-nm
gdb=$bindir/$target-gdb

note "$($gcc --version | sed -n '1p')"
note "$("$bindir/$target-ld" --version | sed -n '1p')"
note "$($gdb --version | sed -n '1p')"

python_output=$($gdb -q -nx -batch -ex \
    'python import sys; print("GDB_PYTHON=" + sys.version.split()[0])' 2>&1) || {
    echo "$python_output" >&2
    die "GDB Python command failed"
}
grep -q '^GDB_PYTHON=' <<<"$python_output" || {
    echo "$python_output" >&2
    die "GDB was built without working Python support"
}
note "$(grep '^GDB_PYTHON=' <<<"$python_output")"

multilib_output=$($gcc -print-multi-lib)
for expected_dir in thumb/v6-m/nofp thumb/v7-m/nofp thumb/v7e-m+fp/hard; do
    grep -q "^${expected_dir};" <<<"$multilib_output" || \
        die "required multilib is missing: $expected_dir"
done

while IFS= read -r multilib; do
    multidir=${multilib%%;*}
    libdir=$prefix/$target/lib/$multidir
    for artifact in libc.a libc_nano.a libg_nano.a libstdc++_nano.a \
        libsupc++_nano.a nano.specs nosys.specs
    do
        [[ -f $libdir/$artifact ]] || die "missing $libdir/$artifact"
    done
done <<<"$multilib_output"

test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
printf '%s\n' \
    '#include <stdint.h>' \
    'volatile uint32_t value;' \
    'int main(void) { value = 42U; return (int)value - 42; }' >"$test_dir/smoke.c"
printf '%s\n' \
    '#include <cstdio>' \
    '#include <cstdint>' \
    '#include <cstdlib>' \
    'template<typename T> constexpr T twice(T v) { return v + v; }' \
    'volatile std::uint32_t value;' \
    'char output[32];' \
    'int main() { value = twice(21U) + static_cast<unsigned>(std::rand()); return std::snprintf(output, sizeof(output), "%f", static_cast<double>(value)); }' >"$test_dir/smoke.cc"
printf '%s\n' \
    '#include <stddef.h>' \
    'static unsigned char heap[4096];' \
    'void *__wrap__malloc_r(void *reent, size_t size) { (void)reent; (void)size; return heap; }' \
    'void *__wrap__calloc_r(void *reent, size_t count, size_t size) { (void)reent; (void)count; (void)size; return heap; }' \
    'void __wrap__free_r(void *reent, void *ptr) { (void)reent; (void)ptr; }' \
    >"$test_dir/allocator-wrappers.c"

compile_variants=(
    "-mcpu=cortex-m0 -mthumb -mfloat-abi=soft"
    "-mcpu=cortex-m3 -mthumb -mfloat-abi=soft"
    "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
    "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard"
)

variant_number=0
for flags in "${compile_variants[@]}"; do
    read -r -a flag_array <<<"$flags"
    "$gcc" "${flag_array[@]}" --specs=nano.specs -Os -ffunction-sections \
        -fdata-sections -c "$test_dir/smoke.c" -o "$test_dir/c-$variant_number.o"
    "$gxx" "${flag_array[@]}" --specs=nano.specs -std=gnu++11 -Os \
        -fno-exceptions -fno-rtti -c "$test_dir/smoke.cc" \
        -o "$test_dir/cxx-$variant_number.o"
    ((variant_number += 1))
done

read -r -a link_flags <<<"${compile_variants[3]}"
"$gcc" "${link_flags[@]}" -Os -c "$test_dir/allocator-wrappers.c" \
    -o "$test_dir/allocator-wrappers.o"
"$gxx" "${link_flags[@]}" --specs=nano.specs --specs=nosys.specs \
    -nostartfiles -Wl,-e,main -Wl,--gc-sections -Wl,-u,_printf_float \
    -Wl,--wrap,_malloc_r -Wl,--wrap,_calloc_r -Wl,--wrap,_free_r \
    "$test_dir/cxx-3.o" "$test_dir/allocator-wrappers.o" \
    -o "$test_dir/smoke.elf"
"$bindir/$target-readelf" -h "$test_dir/smoke.elf" >/dev/null

declare -A is_forbidden=()
while IFS= read -r symbol; do
    is_forbidden["$symbol"]=1
done < <(policy_symbols "$policy")

# Inspect each archive once regardless of policy size. The POSIX nm format
# emits archive-member headings as well as records; only an exact first-field
# match can be a forbidden definition.
while IFS= read -r -d '' archive; do
    if ! "$nm" -g --defined-only --format=posix "$archive" \
        >"$test_dir/archive-symbols" 2>"$test_dir/archive-nm-errors"
    then
        cat "$test_dir/archive-nm-errors" >&2
        die "could not inspect target archive: $archive"
    fi
    while read -r candidate _; do
        if [[ -n ${is_forbidden[$candidate]:-} ]]; then
            die "forbidden symbol $candidate remains in $archive"
        fi
    done <"$test_dir/archive-symbols"
done < <(find "$prefix" -type f -name '*.a' -print0)

# Public assert() is intentionally unusable because its abort dependency is
# forbidden, but no implementation detail inside newlib may pull it in.
while IFS= read -r -d '' archive; do
    if "$nm" -A -g --undefined-only "$archive" 2>/dev/null | \
        awk '$0 !~ /:lib_a-assert[.]o:/ && / U __assert_func$/ { found=1 }
             END { exit !found }'
    then
        die "newlib-internal assertion remains in $archive"
    fi
done < <(find "$prefix/$target/lib" -type f \( \
    -name libc.a -o -name libg.a -o \
    -name libc_nano.a -o -name libg_nano.a \
\) -print0)

note "All toolchain smoke tests passed"
