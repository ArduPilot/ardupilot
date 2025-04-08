#!/usr/bin/env zsh

_AP_COMPLETION_DIR=$(builtin cd "`dirname "$0"`" > /dev/null && pwd)
fpath+=$_AP_COMPLETION_DIR/zsh/
