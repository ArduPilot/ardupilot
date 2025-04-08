#!/usr/bin/env bash
# Run lua check for all lua files passing AP specific config
# Can also pass any number of arguments that are passed onto luacheck
# for example the path of a single script could be given

cd "$(dirname "$0")"
cd ../..

CHECK_PATH="$*"
if test -z "$CHECK_PATH"
then
    CHECK_PATH="*/"
fi

luacheck ${CHECK_PATH} --config libraries/AP_Scripting/tests/luacheck.lua
