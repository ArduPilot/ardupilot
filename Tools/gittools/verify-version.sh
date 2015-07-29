#!/bin/bash
SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ROOT=$(dirname $(git -C $SCRIPT_DIR rev-parse --git-dir))

install_version=$(git -C $ROOT config gittools.commithash)
current_version=$(git -C $ROOT log -n 1 --format=%H $SCRIPT_DIR)

[[ $install_version == $current_version ]] && exit 0

cat >&2 <<EOF
note: it seems like the installation version of gittools differs from the
current tree, you can update ardupilot's gittools installation by running
$SCRIPT_DIR/install.sh
EOF
echo

exit 1
