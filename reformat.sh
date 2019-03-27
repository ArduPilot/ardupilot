#!/bin/bash
DIR=$1
COMMIT=$2

function do_commit {
  if [ $COMMIT == 1 ] ;then
      git commit $1 --author="uncrustify <pat@moreproductive.org>" \
          --message="uncrustify $1";
  fi
}

function format_cpp {
  echo 'format_cpp' $1
  uncrustify --no-backup -c uncrustify_cpp.cfg $1
  do_commit $1
}

function format_header {
  echo 'format_header' $1
  uncrustify --no-backup -c uncrustify_headers.cfg  $1
  do_commit $1
}

CPPFILES=`find $DIR -name '*.cpp' -print`
CFILES=`find $DIR -name '*.c' -print`
HFILES=`find $DIR -name '*.h' -print`

for f in $CPPFILES $CFILES; do
  format_cpp $f
done


for f in $HFILES; do
  format_header $f
done
