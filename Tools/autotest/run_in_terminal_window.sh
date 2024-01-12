#!/bin/bash

# Try to run a command in an appropriate type of terminal window
# depending on whats available
# Sigh: theres no common way of handling command line args :-(
name="$1"
shift
echo "RiTW: Starting $name : $*"

if [ -z "$SITL_RITW_MINIMIZE" ]; then
    SITL_RITW_MINIMIZE=1
fi

if [ -n "$SITL_RITW_TERMINAL" ]; then
  # create a small shell script containing the command to run; this
  # avoids problems where "screen" expects arguments in
  # argv[1],argv[2],argv[3] where gnome-terminal expects the command
  # to run be in argv[n+1] where argv[n] is "-e"
  # this should work with:
  # export SITL_RITW_TERMINAL="screen -D -m"
  # export SITL_RITW_TERMINAL="gnome-terminal -e"
  # export SITL_RITW_TERMINAL="konsole -e"

  test -z "$TMPDIR" && TMPDIR="/tmp/"
  FILENAME="ritw-`date '+%Y%m%d%H%M%S'`"
  FILEPATH="$TMPDIR/$FILENAME"
  echo "#!/bin/sh" >"$FILEPATH"
  printf "%q " "$@" >>"$FILEPATH"
  chmod +x "$FILEPATH"
  $SITL_RITW_TERMINAL "$FILEPATH" &
elif [ "$TERM" = "screen" ] && [ -n "$TMUX" ]; then
  tmux new-window -dn "$name" "$*"
elif [ -n "$DISPLAY" -a -n "$(which osascript)" ]; then
  osascript -e 'tell application "Terminal" to do script "'"cd $(pwd) && clear && $* "'"'
elif [ -n "$DISPLAY" -a -n "$(which xterm)" ]; then
  if [ $SITL_RITW_MINIMIZE -eq 1 ]; then
      ICONIC=-iconic
  fi
  xterm $ICONIC -xrm 'XTerm*selectToClipboard: true' -xrm 'XTerm*initialFont: 6' -n "$name" -name "$name" -T "$name" -hold -e $* &
elif [ -n "$DISPLAY" -a -n "$(which konsole)" ]; then
  konsole --hold -e $*
elif [ -n "$DISPLAY" -a -n "$(which gnome-terminal)" ]; then
  gnome-terminal -e "$*"
elif [ -n "$STY" ]; then
  # We are running inside of screen, try to start it there
  screen -X screen -t "$name" bash -c "cd $PWD; $*"
else
  filename="/tmp/$name.log"
  echo "RiTW: Window access not found, logging to $filename"
  cmd="$1"
  shift
# the following "true" is to avoid bash optimising the following call
# to avoid creating a subshell.  We need that subshell, or
# _fdm_input_step sees ArduPilot has no parent and kills ArduPilot!
  ( : ; "$cmd" $* &>"$filename" < /dev/null ) &
fi
exit 0
