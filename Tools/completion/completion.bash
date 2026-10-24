#!/usr/bin/env bash

# shellcheck source-path=SCRIPTDIR
_AP_COMPLETION_DIR=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" > /dev/null && pwd)
# shellcheck source=bash/_waf
source "$_AP_COMPLETION_DIR/bash/_waf"
# shellcheck source=bash/_sim_vehicle
source "$_AP_COMPLETION_DIR/bash/_sim_vehicle"
# shellcheck source=bash/_ap_bin
source "$_AP_COMPLETION_DIR/bash/_ap_bin"
# shellcheck source=bash/_ap_autotest
source "$_AP_COMPLETION_DIR/bash/_ap_autotest"
