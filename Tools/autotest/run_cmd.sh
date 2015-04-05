#!/bin/bash

# This is a helper script for run_in_terminal_window.sh.  I'm not sure why Ardupilot.elf exits if not run from inside of a script
echo running $*
( $* )
echo cmd exited
