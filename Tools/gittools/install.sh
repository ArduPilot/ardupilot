#!/bin/bash

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ROOT=$(dirname $(git -C $SCRIPT_DIR rev-parse --git-dir))

errors=0

# link hooks
echo "Linking git hooks..."
for f in $SCRIPT_DIR/hooks/*; do
    ( cd $ROOT/.git/hooks && ln -fs $f ) && continue
    echo "Error on linking git hook $f" >&2
    ((errors++))
done

if ((errors > 0)); then
    echo "gittools install script has finished with $errors error(s)"
    exit 1
else
    echo "gittools install script has finished successfully!"
    echo "Updating gitools version information..."
    git -C $ROOT config gittools.commithash \
        $(git -C $ROOT log -n 1 --format=%H $SCRIPT_DIR)
    echo "Install finished"
fi
