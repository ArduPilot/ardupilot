#!/bin/bash
if [ $# -eq 1 ]
then
  echo "Processing: $1"
  if ! fmpp -q -C $1/cfg/board.fmpp
  then
    echo
    echo "aborted"
    exit 1
  else
    echo
    echo "done"
    exit 0
  fi
else
  echo "Usage: genboard <board_directory>"
fi
