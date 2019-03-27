#!/bin/sh

if [ $(uname) = "Darwin" ]; then
    DIR=$(dirname $(greadlink -f $0))
else
    DIR=$(dirname $(readlink -f $0))
fi

astyle --options="${DIR}"/astylerc $*

