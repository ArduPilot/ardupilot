#!/bin/bash
# Install dependencies and configure the environment for CI build testing

BASEDIR=$(dirname "$0")
if [ `uname -s` == "Darwin" ]; then
    bash $BASEDIR/configure-ci-osx.sh
else
    bash $BASEDIR/configure-ci-linux.sh && . ~/.profile
fi
