#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $DIR/..

. docs/setup.sh

if [ ! -f $DOCS_OUTPUT_BASE/tags/libraries ]; 
then
	echo "Must build libraries first"
	exit 0
fi

doxygen docs/config/arducopter

