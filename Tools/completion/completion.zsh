#!/usr/bin/env zsh

_AP_COMPLETION_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
fpath+=$_AP_COMPLETION_DIR/zsh/

