#!/usr/bin/env bash
set -euo pipefail
# sort order sets the SRAM layout, so it must not follow the build host
export LC_ALL=C

# Generate linker section picks for RP2350 RAM/scratch function placement.
# Reads three registry files and produces three .ld output files:
#   rp2350_ramfunc2_sections.ld  -- main SRAM (ram0, 512 KB striped)
#   rp2350_scratchx_sections.ld  -- Scratch X (SRAM8, 4 KB, core0 dedicated)
#   rp2350_scratchy_sections.ld  -- Scratch Y (SRAM9, 4 KB, core1 dedicated)

# With --strict, a registry entry that matches no symbol fails the build.
# Without it misses are ignored: the registries are tuned for one vehicle, and
# the bootloader and other vehicles legitimately lack most of their entries.
# --toolchain names the cross toolchain prefix, so the symbols are read and
# demangled by the cross binutils the build uses, not whatever the host has.
usage() {
    echo "usage: $0 <buildroot> [--strict] [--toolchain <prefix>]" >&2
    exit 2
}
strict=0
toolchain=arm-none-eabi
buildroot=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --strict) strict=1 ;;
        --toolchain) [[ $# -ge 2 ]] || usage; toolchain="$2"; shift ;;
        *) [[ -z "$buildroot" ]] || usage; buildroot="$1" ;;
    esac
    shift
done
[[ -n "$buildroot" ]] || usage
script_dir="$(cd "$(dirname "$0")" && pwd)"

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

raw_txt="$tmpdir/raw_symbols.txt"
map_txt="$tmpdir/mangled_map.txt"
misses_txt="$tmpdir/misses.txt"
: > "$misses_txt"

# Collect all defined global symbols from build artifacts (done once, shared).
# nm cannot read GIMPLE-only LTO objects (empty/guarded-out translation units)
# and returns non-zero for them; tolerate that under pipefail so those objects
# are skipped without aborting. Symbols from readable objects in the same batch
# are still emitted.
find "$buildroot" -type f \( -name '*.a' -o -name '*.o' \) -print0 \
    | { xargs -0 -r "${toolchain}-nm" --defined-only 2>/dev/null || true; } \
  | awk '
      {
          n=NF
          if (n >= 3) {
              typ=$(n-1)
              sym=$n
              if (typ ~ /^[TtWw]$/ && sym !~ /^\./) print sym
          }
      }
  ' | sort -u > "$raw_txt"

# Build normalized demangled->mangled map (done once, shared across all registries).
paste "$raw_txt" <("${toolchain}-c++filt" < "$raw_txt") \
  | awk '
      {
          raw=$1
          $1=""
          dem=$0
          sub(/^[[:space:]]+/, "", dem)
          # A decltype return type (MAX, MIN) holds parentheses of its own:
          # drop it, so the name is matched as MAX<float, float>.
          if (substr(dem, 1, 8) == "decltype") {
              depth = 0
              for (i = 9; i <= length(dem); i++) {
                  c = substr(dem, i, 1)
                  if (c == "(") {
                      depth++
                  } else if (c == ")" && --depth == 0) {
                      dem = substr(dem, i + 1)
                      sub(/^[[:space:]]+/, "", dem)
                      break
                  }
              }
          }
          # Cut the argument list at the first "(" outside template brackets,
          # so Bitmask<(unsigned short)256>::get keeps its template argument.
          # A name that never returns to depth 0 (operator<) is cut at its
          # first "(" as before.
          depth = 0
          cut = 0
          for (i = 1; i <= length(dem); i++) {
              c = substr(dem, i, 1)
              if (c == "<") {
                  depth++
              } else if (c == ">" && depth > 0) {
                  depth--
              } else if (c == "(" && depth == 0) {
                  cut = i
                  break
              }
          }
          if (cut > 0) {
              dem = substr(dem, 1, cut - 1)
          } else {
              sub(/\(.*/, "", dem)
          }
          gsub(/[[:space:]]+/, "", dem)
          if (dem != "") print dem "|" raw
      }
  ' > "$map_txt"

# Members of the toolchain libraries that "(archive)" entries pick from.
# Member names do not vary between multilibs, so the default ones serve.
members_txt="$tmpdir/archive_members.txt"
for lib in libm.a libc_nano.a libc.a; do
    path="$("${toolchain}-gcc" -print-file-name="$lib")"
    if [[ -f "$path" ]]; then
        "${toolchain}-ar" t "$path"
    fi
done | sort -u > "$members_txt"

# The member an "(archive)" entry names, as this toolchain spells it: newlib
# 3.x names libm's objects lib_a-<name>.o, and newer releases, which build
# libm on its own, libm_a-<name>.o. Prints nothing if neither exists.
resolve_member() {
    local base="${1#lib_a-}"
    base="${base#libm_a-}"
    local name
    for name in "lib_a-$base" "libm_a-$base"; do
        if grep -qxF "$name" "$members_txt"; then
            echo "$name"
            return
        fi
    done
}

# Is a hwdef define enabled for this build? Used by the "[needs X]" marker so an
# entry whose symbol only exists under some define does not raise a spurious
# "no symbol match". Absent hwdef.h means we cannot tell, so count it as a miss.
# The #define line that takes effect for a name in hwdef.h. A board's own
# define comes before the generic "#ifndef NAME / #define NAME default"
# fallback, so an unguarded line wins and the guarded default is used only
# when there is no other.
define_line() {
    awk -v name="$1" '
        $0 ~ "^#ifndef[[:space:]]+" name "([[:space:]]|$)" { guarded = 1; next }
        $0 ~ "^#define[[:space:]]+" name "([[:space:]]|$)" {
            if (guarded) { if (fallback == "") fallback = $0 } else { line = $0 }
            guarded = 0; next
        }
        { guarded = 0 }
        END { print (line != "" ? line : fallback) }' "$2"
}

# A define's value reads as off: FALSE, or zero with any integer suffix, since
# the generator writes counts such as HAL_HAVE_PIO_UARTS as 0U.
value_is_off() {
    [[ "$1" == "FALSE" || "$1" =~ ^\(?0+[uUlL]*\)?$ ]]
}

define_enabled() {
    local name="$1" hdr="$buildroot/hwdef.h" line val
    [[ -f "$hdr" ]] || return 0
    line="$(define_line "$name" "$hdr")"
    [[ -n "$line" ]] || return 1
    val="$(printf '%s' "$line" | sed -E "s/^#define[[:space:]]+${name}[[:space:]]*//" | tr -d '[:space:]')"
    value_is_off "$val" && return 1
    return 0
}

# Does hwdef.h explicitly switch a define off? Used by the "[unless X]" marker
# for options that default on outside hwdef.h, where absence means enabled.
define_disabled() {
    local name="$1" hdr="$buildroot/hwdef.h" line val
    [[ -f "$hdr" ]] || return 1
    line="$(define_line "$name" "$hdr")"
    [[ -n "$line" ]] || return 1
    val="$(printf '%s' "$line" | sed -E "s/^#define[[:space:]]+${name}[[:space:]]*//" | tr -d '[:space:]')"
    value_is_off "$val"
}

# generate_ld_from_registry <registry_file> <out_ld> <header_comment>
generate_ld_from_registry() {
    local registry="$1"
    local out_ld="$2"
    local header="$3"

    local symbols_txt="$tmpdir/syms_$(basename "$registry").txt"
    local archives_txt="$tmpdir/arch_$(basename "$registry").txt"

    awk -F'|' '
        /^[[:space:]]*#/ { next }
        /^[[:space:]]*$/ { next }
        NF >= 2 {
            p=$1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", p)
            if (p == "(archive)") { next }
            cond=""
            if (match($0, /\[needs [A-Za-z_][A-Za-z0-9_]*\]/)) {
                cond = "needs:" substr($0, RSTART + 7, RLENGTH - 8)
            } else if (match($0, /\[unless [A-Za-z_][A-Za-z0-9_]*\]/)) {
                cond = "unless:" substr($0, RSTART + 8, RLENGTH - 9)
            } else if (match($0, /\[without [A-Za-z_][A-Za-z0-9_]*\]/)) {
                cond = "without:" substr($0, RSTART + 9, RLENGTH - 10)
            } else if (match($0, /\[inlinable\]/)) {
                cond = "inlinable"
            }
            s=$2
            gsub(/#.*/, "", s)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", s)
            if (s != "") print s "\t" cond
        }
    ' "$registry" | sort -u > "$symbols_txt"

    # Archive-member entries pick a whole object out of a prebuilt library.
    # Toolchain libraries such as newlib are not compiled with
    # -ffunction-sections, so they have one .text per object and there is no
    # per-function section to select by name.
    awk -F'|' '
        /^[[:space:]]*#/ { next }
        /^[[:space:]]*$/ { next }
        NF >= 2 {
            p=$1
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", p)
            if (p != "(archive)") { next }
            s=$2
            gsub(/#.*/, "", s)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", s)
            if (s != "") print s
        }
    ' "$registry" | sort -u > "$archives_txt"

    {
        echo "/* auto-generated from $(basename "$registry"); do not edit */"
        while IFS=$'\t' read -r wanted cond; do
            norm_wanted="$(printf '%s' "$wanted" | sed 's/[[:space:]]//g')"
            picks="$(awk -F'|' -v want="$norm_wanted" '
                $1 == want {
                    raw=$2
                    print "        *(.text." raw ")"
                    print "        *(.text.hot." raw ")"
                    print "        *(.text.startup." raw ")"
                    print "        *(.text.unlikely." raw ")"
                    print "        *(.gnu.linkonce.t." raw ")"
                }
            ' "$map_txt")"
            if [[ -z "$picks" ]]; then
                # An entry tagged [needs X] is expected to be absent when X is
                # off, so say nothing. Everything else is a real miss, and
                # silent misses are a trap: the build succeeds, the binary is
                # unchanged, and the entry looks like it took effect.
                if [[ "$cond" == needs:* ]] && ! define_enabled "${cond#needs:}"; then
                    continue
                fi
                if [[ "$cond" == unless:* ]] && define_disabled "${cond#unless:}"; then
                    continue
                fi
                if [[ "$cond" == without:* ]] && define_enabled "${cond#without:}"; then
                    continue
                fi
                if [[ "$cond" == "inlinable" ]]; then
                    continue
                fi
                if [[ $strict -eq 1 ]]; then
                    echo "rp2350_ramfunc2_sections: $(basename "$registry"): no symbol match for '$wanted'" >&2
                    echo "$wanted" >> "$misses_txt"
                fi
                continue
            fi
            printf '%s\n' "$picks"
        done < "$symbols_txt"
        while IFS= read -r member; do
            [[ -n "$member" ]] || continue
            found="$(resolve_member "$member")"
            if [[ -z "$found" ]]; then
                if [[ $strict -eq 1 ]]; then
                    echo "rp2350_ramfunc2_sections: $(basename "$registry"): no archive member '$member'" >&2
                    echo "$member" >> "$misses_txt"
                fi
                continue
            fi
            printf '        *%s(.text .text.*)\n' "$found"
        done < "$archives_txt"
    } | awk '!seen[$0]++' > "$out_ld"
}

generate_ld_from_registry \
    "$script_dir/rp2350_ramfunc2_registry.txt" \
    "$buildroot/rp2350_ramfunc2_sections.ld" \
    "rp2350_ramfunc2_registry.txt"

generate_ld_from_registry \
    "$script_dir/rp2350_scratchx_registry.txt" \
    "$buildroot/rp2350_scratchx_sections.ld" \
    "rp2350_scratchx_registry.txt"

generate_ld_from_registry \
    "$script_dir/rp2350_scratchy_registry.txt" \
    "$buildroot/rp2350_scratchy_sections.ld" \
    "rp2350_scratchy_registry.txt"

if [[ -s "$misses_txt" ]]; then
    echo "rp2350_ramfunc2_sections: $(wc -l < "$misses_txt") registry entries matched no symbol" >&2
    exit 1
fi

exit 0
