#!/bin/bash

set -e

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

MODULES=('waflib/extras/clang_compilation_database.py')
TMP_WAF_DIR=

usage() {
    echo "
Usage: $(basename $0) <version>

Update waf to <version>, also downloading the modules
" 1>&$1
}

die() {
    echo "ERROR: $1" >&2
    if [ $# -gt 1 -a $2 ]; then
        usage 2
    fi
    exit 1
}

# Parse options
args=0
while getopts "h" o; do
    case "${o}" in
        h) usage 1; exit 0;;
        \?) usage 2; exit 1;;
    esac
done
shift $args

if [ -z "$1" ]; then
    echo "Missing version. See usage below:" >&2
    usage 2
    exit 1
fi

VERSION=$1

# Top level directory
if [ ! -d libraries -o "$(git rev-parse --git-dir)" != ".git" ]; then
    die "Need to run on top level directory with a git checkout"
fi

trap '
    ret=$?
    if [ -e "$TMP_WAF_DIR" ]; then
        rm -rf "$TMP_WAF_DIR"
    fi
    exit $ret
    ' EXIT

# clean up after ourselves no matter how we die
trap 'exit 1;' SIGINT

# Download new waf
wget https://waf.io/waf-$VERSION -O waf

TMP_WAF_DIR=$(mktemp -d --tmpdir waf-update.XXXXXX)
git clone --depth 1 https://github.com/waf-project/waf -b waf-$VERSION "$TMP_WAF_DIR"

for m in $MODULES; do
    cp "$TMP_WAF_DIR/$m" Tools/ardupilotwaf/
done

DESCRIBE=$(git -C "$TMP_WAF_DIR" describe)
echo "Done! Suggested commit message:"
echo "
-----8<------
waf: update to $VERSION

Update waf build system. Tools are updated to tag ${DESCRIBE}."
