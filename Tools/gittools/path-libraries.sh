#!/bin/bash

usage() {
    cat >&$1 <<EOF
Usage: $0 [OPTIONS]

Read a list of files relative to ardupilot's root directory and output the
libraries they belong to.

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

if $show_paths; then
    sedcmd="s,libraries/\([^/]\+\).*,\1\t\0,"
else
    sedcmd="s,libraries/\([^/]\+\).*,\1,"
fi

grep "^libraries/[^\/]\+" | \
    sed $sedcmd | \
    sort | \
    uniq
