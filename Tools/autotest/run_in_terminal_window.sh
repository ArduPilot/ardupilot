#!/bin/bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "Starting $name : $*"
# default to xterm as it has the most consistent options and can start minimised
if [ -n "$DISPLAY" -a -x /usr/bin/xterm ]; then
  /usr/bin/xterm -iconic -xrm 'XTerm*selectToClipboard: true' -xrm 'XTerm*initialFont: 6' -n "$name" -name "$name" -T "$name" -hold -e $* &
elif [ -n "$DISPLAY" -a -x /usr/bin/konsole ]; then
  /usr/bin/konsole --hold -e $*
elif [ -n "$DISPLAY" -a -x /usr/bin/gnome-terminal ]; then
  /usr/bin/gnome-terminal -e "$*"
elif [ -n "$STY" ]; then
  # We are running inside of screen, try to start it there
  /usr/bin/screen -X screen -t $name $*
else
  filename="/tmp/$name.log"
  echo "Window access not found, logging to $filename"
  cmd="$1"
  shift
# the following "true" is to avoid bash optimising the following call
# to avoid creating a subshell.  We need that subshell, or
# _fdm_input_step sees ArduPilot has no parent and kills ArduPilot!
  ( : ; $cmd $* &>$filename < /dev/null ) &
fi
exit 0
