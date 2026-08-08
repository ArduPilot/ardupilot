#!/usr/bin/env bash

die()
{
    echo "error: $*" >&2
    exit 1
}

note()
{
    echo "==> $*"
}

absolute_path()
{
    realpath -m -- "$1"
}

require_command()
{
    command -v "$1" >/dev/null 2>&1 || die "required command not found: $1"
}

verify_sha512()
{
    local file=$1
    local expected=$2
    local actual

    actual=$(sha512sum "$file")
    actual=${actual%% *}
    [[ $actual == "$expected" ]] || die "SHA-512 mismatch for $file (got $actual)"
}

download_archive()
{
    local url=$1
    local output=$2
    local expected=$3
    local partial=${output}.part

    if [[ -f $output ]]; then
        verify_sha512 "$output" "$expected"
        note "Using cached $(basename "$output")"
        return
    fi

    mkdir -p "$(dirname "$output")"
    note "Downloading $url"
    curl --fail --location --retry 3 --continue-at - --output "$partial" "$url"
    verify_sha512 "$partial" "$expected"
    mv -f "$partial" "$output"
}

extract_stripped()
{
    local archive=$1
    local destination=$2

    mkdir -p "$destination"
    tar -xf "$archive" -C "$destination" --strip-components=1
}

policy_symbols()
{
    sed -e 's/[[:space:]]*#.*$//' \
        -e 's/^[[:space:]]*//' \
        -e 's/[[:space:]]*$//' \
        -e '/^$/d' "$1"
}
