#!/usr/bin/env bash

_AP_COMPLETION_DIR=$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)
source "$_AP_COMPLETION_DIR/bash/_waf"
source "$_AP_COMPLETION_DIR/bash/_sim_vehicle"
source "$_AP_COMPLETION_DIR/bash/_ap_bin"
source "$_AP_COMPLETION_DIR/bash/_ap_autotest"
