#!/bin/bash

usage() {
    cat >&$1 <<EOF
Usage: $0 [OPTIONS]

Read a list of files relative to ardupilot's root directory and output the
non-libraries subsystems they belong to.

Options:
    --show-paths, -p    Print also file paths after the library name.
    --help, -h          Show this help message.
EOF
}

show_paths=false

while [[ -n $1 ]]; do
    case "$1" in
    --show-paths|-p)
        show_paths=true
        ;;
    --help|-h)
        usage 1
        exit 0
        ;;
    *)
        usage 2
        exit 1
        ;;
    esac

    shift
done

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ROOT=$(dirname $(git -C $SCRIPT_DIR rev-parse --git-dir))

if $show_paths; then
    sedcmd="s,\([^/]\+\).*,\1\t\0,"
else
    sedcmd="s,\([^/]\+\).*,\1,"
fi

grep -v "^libraries" | \
    sed $sedcmd | \
    sort | \
    uniq | \
    if $show_paths; then
        while read d f; do
            [[ -d "$ROOT/$d" ]] && printf "%s\t%s\n" "$d" "$f"
        done
    else
        while read d; do
            [[ -d "$ROOT/$d" ]] && echo "$d"
        done
    fi
