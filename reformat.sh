#!/bin/bash
DIR=$1

function format_cpp {
  echo 'format_cpp' $1
  uncrustify --no-backup -c uncrustify_cpp.cfg $1
}

function format_header {
  echo 'format_header' $1
  uncrustify --no-backup -c uncrustify_headers.cfg  $1
}

PDEFILES=`find $DIR -name '*.pde' -print`
CPPFILES=`find $DIR -name '*.cpp' -print`
CFILES=`find $DIR -name '*.c' -print`
HFILES=`find $DIR -name '*.h' -print`

for f in $PDEFILES $CPPFILES $CFILES; do
  format_cpp $f
done


for f in $HFILES; do
  format_header $f
done
