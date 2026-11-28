#!/usr/bin/env bash
# if you have modules/esp_idf setup as a submodule, then leave it as a submodule and switch branches

COMMIT=""
BRANCH="release/v6.1"

if [ ! -d modules ]; then
echo "this script needs to be run from the root of your repo, sorry, giving up."
exit 1
fi
echo `ls modules`
cd modules

if [ ! -d esp_idf ]; then
    echo 'did not find modules/esp_idf folder, making it.' ; 
    mkdir -p -v esp_idf
else
    echo 'found modules/esp_idf folder' ; 
fi

echo "looking for submodule or repo..."
if [ `git submodule | grep esp_idf | wc | cut -c1-7` == '1'  ]; then 
    echo "found real submodule, syncing"
    ../Tools/gittools/submodule-sync.sh >/dev/null
else
    echo "esp_idf is NOT a submodule"

    if  [ ! `ls  esp_idf/install.sh 2>/dev/null` ]; then
        echo "found empty IDF, cloning"
        # add esp_idf as almost submodule, depths  uses less space
        git clone -b "$BRANCH"  https://github.com/espressif/esp-idf.git esp_idf
        if [ -n "$COMMIT" ]; then
            git -C esp_idf checkout "$COMMIT"
        fi
    fi
fi

echo "inspecting possible IDF... "
cd esp_idf
echo `git rev-parse HEAD`
# these are a selection of possible specific commit/s that represent v6.1 branch of the esp_idf 
if [ -n "$COMMIT" ] && [ `git rev-parse HEAD` == "$COMMIT" ]; then 
    echo "IDF version '$BRANCH' found OK, great."; 
else
    echo "looks like an idf, but not $BRANCH branch, or wrong commit , trying to switch branch and reflect upstream";
    ../../Tools/gittools/submodule-sync.sh >/dev/null
    git fetch ; git checkout -f "$BRANCH" 
    if [ -n "$COMMIT" ]; then
        git checkout "$COMMIT"
    fi

    # retry same as above
    echo `git rev-parse HEAD`
    if [ -n "$COMMIT" ] && [ `git rev-parse HEAD` == "$COMMIT" ]; then 
        echo "IDF version '$BRANCH' found OK, great."; 
        git checkout "$COMMIT"
    elif [ -z "$COMMIT" ]; then
        echo "IDF branch '$BRANCH' checked out.";
    fi
fi
cd ../..

cd modules/esp_idf
git submodule update --init --recursive

cd ../..

IDF_VER="${BRANCH#release/v}"
IDF_VENV_PY=$(find -L "$HOME/.espressif/python_env" -maxdepth 5 -path "*idf${IDF_VER}*" -name "python*" -type f 2>/dev/null | head -n 1)

echo
echo "after changing IDF versions [ such as between 4.4 and 6.1 ] you should re-run these in your console:"
echo "./modules/esp_idf/install.sh"
if [ -n "$IDF_VENV_PY" ]; then
    echo "\"$IDF_VENV_PY\" -m pip install empy==3.3.4 pexpect"
else
    echo "IDF_VENV_PY=\$(find -L \"\$HOME/.espressif/python_env\" -maxdepth 5 -path \"*idf${IDF_VER}*\" -name \"python*\" -type f 2>/dev/null | head -n 1) && \"\$IDF_VENV_PY\" -m pip install empy==3.3.4 pexpect"
fi
echo "source ./modules/esp_idf/export.sh"
