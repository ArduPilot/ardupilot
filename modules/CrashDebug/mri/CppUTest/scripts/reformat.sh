#!/bin/bash
set -e
ASTYLE_OPTIONS="--convert-tabs --indent=spaces=4 --indent-classes --indent-switches --indent-preprocessor --style=ansi"

find $1 -name "*.h" -o -name "*.c" -o -name "*.cpp" |
while read filename; do 
    tmpfile=${filename}.astyle.cpp
    astyle ${ASTYLE_OPTIONS} <"${filename}" > "${tmpfile}"
    mv "${tmpfile}" "${filename}"
done
