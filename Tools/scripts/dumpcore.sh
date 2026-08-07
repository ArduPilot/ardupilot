#!/bin/sh

# attaches GDB to a process to dump a core

# we want everything on stderr, so the program is not disturbed
exec 1>&2

PID=$1
TMPFILE=/tmp/gdb.$$
COREFILE="ap-$$.core"
cat << EOF  > $TMPFILE
set height 0
generate-core-file $COREFILE
quit
EOF
gdb -n -batch -x $TMPFILE --pid $PID < /dev/null 2>&1
rm -f $TMPFILE
