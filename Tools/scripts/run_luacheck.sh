#!/bin/bash
# Run lua check for all lua files passing AP specific config

cd "$(dirname "$0")"
cd ../..

luacheck */ --config libraries/AP_Scripting/tests/luacheck.lua
