#!/bin/bash
# if you have modules/esp_idf setup as a submodule, then leave it as a submodule and switch branches
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
        #git clone -b v4.4 --single-branch --depth 10 https://github.com/espressif/esp-idf.git esp_idf
        git clone -b 'release/v4.4'  https://github.com/espressif/esp-idf.git esp_idf
        # check if we've got v4.4 checked out, only this version of esp_idf is tested and works?
        
    fi
fi

echo "inspecting possible IDF... "
cd esp_idf
echo `git rev-parse HEAD`
# these are a selection of possible specific commit/s that represent v4.4 branch of the esp_idf 
if [ `git rev-parse HEAD` == 'f98ec313f2a9bc50151349c404a8f2f10ed99649' ]; then 
    echo "IDF version 'release/4.4' found OK, great."; 
else
    echo "looks like an idf, but not v4.4 branch, trying to switch branch and reflect upstream";
    ../../Tools/gittools/submodule-sync.sh >/dev/null
    git fetch ; git checkout -f release/v4.4 

    # retry same as above
    echo `git rev-parse HEAD`
    if [ `git rev-parse HEAD` == 'f98ec313f2a9bc50151349c404a8f2f10ed99649' ]; then 
        echo "IDF version 'release/4.4' found OK, great."; 
    fi
fi
cd ../..

cd modules/esp_idf 
git submodule update --init --recursive
cd ../..
echo "after changing IDF versions [ such as between 4.2 and 4.4 ] you should re-run these in your console:"
echo "./modules/esp_idf/install.sh"
echo "source ./modules/esp_idf/export.sh"