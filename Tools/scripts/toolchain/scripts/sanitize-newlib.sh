#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=common.sh
. "$SCRIPT_DIR/common.sh"

usage()
{
    cat <<'EOF'
Usage: sanitize-newlib.sh --prefix DIR --policy FILE [options]

Options:
  --ar PROGRAM       archive tool (default: DIR/bin/arm-none-eabi-ar)
  --nm PROGRAM       symbol tool (default: DIR/bin/arm-none-eabi-nm)
  --ranlib PROGRAM   ranlib tool (default: DIR/bin/arm-none-eabi-ranlib)
  --report FILE      write the removal report to FILE
EOF
}

prefix=
policy=
ar_program=
nm_program=
ranlib_program=
report=

while (($#)); do
    case $1 in
        --prefix) prefix=$2; shift 2 ;;
        --policy) policy=$2; shift 2 ;;
        --ar) ar_program=$2; shift 2 ;;
        --nm) nm_program=$2; shift 2 ;;
        --ranlib) ranlib_program=$2; shift 2 ;;
        --report) report=$2; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; die "unknown option: $1" ;;
    esac
done

[[ -n $prefix && -d $prefix ]] || die "--prefix must name an installed toolchain"
[[ -n $policy && -f $policy ]] || die "--policy must name a readable policy file"

ar_program=${ar_program:-$prefix/bin/arm-none-eabi-ar}
nm_program=${nm_program:-$prefix/bin/arm-none-eabi-nm}
ranlib_program=${ranlib_program:-$prefix/bin/arm-none-eabi-ranlib}
report=${report:-$prefix/share/ardupilot-toolchain/removed-newlib-members.txt}

[[ -x $ar_program ]] || die "archive tool not executable: $ar_program"
[[ -x $nm_program ]] || die "nm tool not executable: $nm_program"
[[ -x $ranlib_program ]] || die "ranlib tool not executable: $ranlib_program"

mapfile -t forbidden < <(policy_symbols "$policy")
((${#forbidden[@]} > 0)) || die "the forbidden-symbol policy is empty"

declare -A is_forbidden=()
declare -A found_symbol=()
for symbol in "${forbidden[@]}"; do
    is_forbidden["$symbol"]=1
done

mkdir -p "$(dirname "$report")"
: >"$report"

scratch_dir=$(mktemp -d)
trap 'rm -rf "$scratch_dir"' EXIT
symbols_file=$scratch_dir/symbols
removed_count=0

# libc contains most public interfaces, while libgloss archives provide the
# low-level syscall implementations. Sanitize both so a forbidden definition
# cannot be recovered by changing specs or selecting a semihosting library.
mapfile -d '' libraries < <(
    find "$prefix/arm-none-eabi/lib" -type f \( \
        -name libc.a -o -name libg.a -o -name libm.a -o \
        -name libc_nano.a -o -name libg_nano.a -o \
        -name libnosys.a -o -name libgloss-linux.a -o \
        -name librdimon.a -o -name librdimon_nano.a -o \
        -name librdimon-v2m.a -o -name librdpmon.a \
    \) -print0 | sort -z
)
((${#libraries[@]} > 0)) || \
    die "no installed newlib or libgloss archives found below $prefix"

for library in "${libraries[@]}"; do
    declare -A remove_member=()
    if ! "$nm_program" -A -g --defined-only --format=posix \
        "$library" >"$symbols_file"
    then
        die "could not inspect newlib archive: $library"
    fi
    while IFS= read -r record; do
        location=${record%% *}
        symbol_record=${record#* }
        symbol=${symbol_record%% *}
        [[ -n ${is_forbidden[$symbol]:-} ]] || continue

        member=${location##*[}
        member=${member%]:}
        [[ $member != "$location" && -n $member ]] || \
            die "cannot parse archive member from: $record"
        remove_member["$member"]=1
        found_symbol["$symbol"]=1
    done <"$symbols_file"

    if ((${#remove_member[@]})); then
        mapfile -t members < <(printf '%s\n' "${!remove_member[@]}" | sort)
        for member in "${members[@]}"; do
            printf '%s: %s\n' "${library#"$prefix"/}" "$member" >>"$report"
            ((removed_count += 1))
        done
        "$ar_program" d "$library" "${members[@]}"
        "$ranlib_program" "$library"
    fi
    unset remove_member
done

for symbol in "${forbidden[@]}"; do
    if [[ -z ${found_symbol[$symbol]:-} ]]; then
        note "Policy symbol was not supplied by this newlib version: $symbol"
    fi
done

# A forbidden definition in any other target archive is just as linkable. Do
# not modify an unexpected archive automatically: report it and stop instead.
violations=$scratch_dir/violations
: >"$violations"
while IFS= read -r -d '' archive; do
    if ! "$nm_program" -A -g --defined-only --format=posix \
        "$archive" >"$symbols_file"
    then
        die "could not inspect target archive: $archive"
    fi
    while IFS= read -r record; do
        symbol_record=${record#* }
        symbol=${symbol_record%% *}
        [[ -z ${is_forbidden[$symbol]:-} ]] || printf '%s\n' "$record" >>"$violations"
    done <"$symbols_file"
done < <(find "$prefix" -type f -name '*.a' -print0)

if [[ -s $violations ]]; then
    cat "$violations" >&2
    die "forbidden symbols remain in installed target archives"
fi

note "Removed $removed_count members from ${#libraries[@]} newlib archives"
note "Removal report: $report"
