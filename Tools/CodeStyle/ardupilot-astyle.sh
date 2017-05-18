#!/bin/sh

DIR=$(dirname $(readlink -f $0))

astyle --options="${DIR}"/astylerc $*

