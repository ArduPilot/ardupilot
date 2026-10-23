#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=common.sh
. "$SCRIPT_DIR/common.sh"

if (($# < 1 || $# > 2)); then
    echo "Usage: verify-newlib-symbols.sh PREFIX [FORBIDDEN-SYMBOL-POLICY]" >&2
    exit 2
fi

prefix=$(absolute_path "$1")
policy=${2:-$SCRIPT_DIR/../config/forbidden-newlib-symbols.txt}
target=arm-none-eabi
nm=$prefix/bin/$target-nm

[[ -x $nm ]] || die "missing tool: $nm"
[[ -f $policy ]] || die "forbidden-symbol policy not found: $policy"

declare -A is_forbidden=()
while IFS= read -r symbol; do
    is_forbidden["$symbol"]=1
done < <(policy_symbols "$policy")
((${#is_forbidden[@]} > 0)) || die "the forbidden-symbol policy is empty"

test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
archive_count=0
violation_count=0

# Scan every installed target archive.  This deliberately runs before the
# complete toolchain verifier so a newlib-only build cannot install a forbidden
# definition and then hide it behind a later packaging or post-processing step.
while IFS= read -r -d '' archive; do
    ((archive_count += 1))
    if ! "$nm" -g --defined-only --format=posix "$archive" \
        >"$test_dir/archive-symbols" 2>"$test_dir/archive-nm-errors"
    then
        cat "$test_dir/archive-nm-errors" >&2
        die "could not inspect target archive: $archive"
    fi
    while read -r candidate _; do
        if [[ -n ${is_forbidden[$candidate]:-} ]]; then
            printf '%s: forbidden definition: %s\n' "$archive" "$candidate" >&2
            ((violation_count += 1))
        fi
    done <"$test_dir/archive-symbols"
done < <(find "$prefix/$target" "$prefix/lib/gcc/$target" \
    -type f -name '*.a' -print0 2>/dev/null)

((archive_count > 0)) || die "no target archives found below $prefix"
((violation_count == 0)) || die "forbidden symbols remain in target archives"

note "No forbidden newlib symbols found in $archive_count target archives"
