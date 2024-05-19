#!/usr/bin/env bash
# Generate md file from lua docs with lua-language-sever

cd "$(dirname "$0")"
cd ../..

if [ -n "$(ls -A lualogs)" ]; then
    echo "lualogs Not Empty"
    exit 1
fi

if [ -e "lua-language-server/bin/lua-language-server" ]; then
    LLS_PATH=$(realpath lua-language-server/bin/lua-language-server)
else
    LLS_PATH=lua-language-server
fi

# Need to use abs paths due to lua-language-sever bug
CONFIG_PATH=$(realpath libraries/AP_Scripting/tests/docs.json)
DOC_PATH=$(realpath libraries/AP_Scripting/docs/)

${LLS_PATH} --configpath ${CONFIG_PATH}  --logpath lualogs --doc ${DOC_PATH}

mv lualogs/doc.md ScriptingDocs.md
rm -r lualogs
