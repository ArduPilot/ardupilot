#!/usr/bin/env bash

if [ -z "$1" ]; then
    printf "\nArguments missing, Usage: '$0 <files to style>'\n\n"
    exit 1
fi

if [ $(uname) = "Darwin" ]; then
    DIR=$(dirname $(greadlink -f $0))
else
    DIR=$(dirname $(readlink -f $0))
fi

astyle --options="${DIR}"/astylerc $*
