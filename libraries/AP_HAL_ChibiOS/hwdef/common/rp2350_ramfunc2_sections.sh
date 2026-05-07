#!/usr/bin/env bash
set -euo pipefail

# Generate linker section picks for RP2350 RAM function placement from
# centralized registry file.

if [[ $# -ne 1 ]]; then
    echo "usage: $0 <buildroot>" >&2
    exit 2
fi

buildroot="$1"
script_dir="$(cd "$(dirname "$0")" && pwd)"
registry="$script_dir/rp2350_ramfunc2_registry.txt"
out_ld="$buildroot/rp2350_ramfunc2_sections.ld"

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

symbols_txt="$tmpdir/registry_symbols.txt"
raw_txt="$tmpdir/raw_symbols.txt"
map_txt="$tmpdir/mangled_map.txt"

# Extract requested symbol names from registry (the text after '|').
awk -F'|' '
    /^[[:space:]]*#/ { next }
    /^[[:space:]]*$/ { next }
    NF >= 2 {
        s=$2
        gsub(/^[[:space:]]+|[[:space:]]+$/, "", s)
        if (s != "") print s
    }
' "$registry" | sort -u > "$symbols_txt"

# Collect all defined global symbols from build artifacts.
find "$buildroot" -type f \( -name '*.a' -o -name '*.o' \) -print0 \
    | xargs -0 -r nm --defined-only 2>/dev/null \
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

# Build normalized demangled->mangled map.
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

{
    echo "/* auto-generated from rp2350_ramfunc2_registry.txt; do not edit */"
    while IFS= read -r wanted; do
        norm_wanted="$(printf '%s' "$wanted" | sed 's/[[:space:]]//g')"
        awk -F'|' -v want="$norm_wanted" '
            $1 == want {
                raw=$2
                print "        *(.text." raw ")"
                print "        *(.text.hot." raw ")"
                print "        *(.text.startup." raw ")"
                print "        *(.text.unlikely." raw ")"
                print "        *(.gnu.linkonce.t." raw ")"
            }
        ' "$map_txt"
    done < "$symbols_txt"
} | awk '!seen[$0]++' > "$out_ld"

exit 0
