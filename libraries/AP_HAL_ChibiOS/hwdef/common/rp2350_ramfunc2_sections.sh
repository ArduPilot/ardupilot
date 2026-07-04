#!/usr/bin/env bash
set -euo pipefail

# Generate linker section picks for RP2350 RAM/scratch function placement.
# Reads three registry files and produces three .ld output files:
#   rp2350_ramfunc2_sections.ld  — main SRAM (ram0, 512 KB striped)
#   rp2350_scratchx_sections.ld  — Scratch X (SRAM8, 4 KB, core0 dedicated)
#   rp2350_scratchy_sections.ld  — Scratch Y (SRAM9, 4 KB, core1 dedicated)

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <buildroot>" >&2
    exit 2
fi

buildroot="$1"
script_dir="$(cd "$(dirname "$0")" && pwd)"

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

raw_txt="$tmpdir/raw_symbols.txt"
map_txt="$tmpdir/mangled_map.txt"

# Collect all defined global symbols from build artifacts (done once, shared).
# nm cannot read GIMPLE-only LTO objects (empty/guarded-out translation units)
# and returns non-zero for them; tolerate that under pipefail so those objects
# are skipped without aborting. Symbols from readable objects in the same batch
# are still emitted.
find "$buildroot" -type f \( -name '*.a' -o -name '*.o' \) -print0 \
    | { xargs -0 -r nm --defined-only 2>/dev/null || true; } \
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
paste "$raw_txt" <(c++filt < "$raw_txt") \
  | awk '
      {
          raw=$1
          $1=""
          dem=$0
          sub(/^[[:space:]]+/, "", dem)
          sub(/\(.*/, "", dem)
          gsub(/[[:space:]]+/, "", dem)
          if (dem != "") print dem "|" raw
      }
  ' > "$map_txt"

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
            s=$2
            gsub(/#.*/, "", s)
            gsub(/^[[:space:]]+|[[:space:]]+$/, "", s)
            if (s != "") print s
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
        while IFS= read -r wanted; do
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
                # Silent misses are a trap: the build succeeds, the binary is
                # unchanged, and the entry looks like it took effect.
                echo "rp2350_ramfunc2_sections: $(basename "$registry"): no symbol match for '$wanted'" >&2
                continue
            fi
            printf '%s\n' "$picks"
        done < "$symbols_txt"
        while IFS= read -r member; do
            [[ -n "$member" ]] || continue
            printf '        *%s(.text .text.*)\n' "$member"
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

exit 0
