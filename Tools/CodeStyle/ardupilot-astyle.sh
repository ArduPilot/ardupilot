#!/bin/sh
if [[ "$OSTYPE" == "darwin"* ]]; then
    DIR=$(dirname $(greadlink -f $0))
else
    DIR=$(dirname $(readlink -f $0))
fi

astyle --options="${DIR}"/astylerc $*

