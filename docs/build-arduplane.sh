#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

cd $DIR/..

if [ ! -f docs/tags/libraries ]; 
then
	echo "Must build libraries first"
	exit 0
fi

doxygen docs/config/arduplane

