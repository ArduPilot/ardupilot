#!/bin/bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "Starting $name : $*"
# default to xterm as it has the most consistent options and can start minimised
if [ -x /usr/bin/xterm ]; then
  /usr/bin/xterm -iconic -n "$name" -name "$name" -T "$name" -hold -e $* &
elif [ -x /usr/bin/konsole ]; then
  /usr/bin/konsole --hold -e $*
elif [ -x /usr/bin/gnome-terminal ]; then
  /usr/bin/gnome-terminal -e "$*"
else
  echo "ERROR: Please install xterm"
  exit 1
fi
exit 0
