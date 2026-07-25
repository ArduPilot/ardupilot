#!/bin/bash
if [ $# -eq 0 ]
then
  find . -name board.fmpp -exec bash genboards.sh '{}' \;
elif [ $# -eq 1 ]
then
  path=$(readlink -f $(dirname $1))
  echo "Processing: $1"
  cd $path
  if ! fmpp -q -C board.fmpp
  then
    echo
    echo "aborted"
    exit 1
  fi
else
  echo "illegal number of arguments"
fi
