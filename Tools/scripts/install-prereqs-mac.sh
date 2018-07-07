#!/bin/bash
echo "Checking homebrew..."
$(which -s brew)
if [[ $? != 0 ]] ; then
    echo "installing homebrew..."
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
else
    echo "Homebrew installed"
fi

#install xconde dependencies
xcode-select --install

brew tap ardupilot/homebrew-px4
brew update
brew install genromfs
brew install gcc-arm-none-eabi
brew install gawk

echo "Checking pip..."
$(which -s pip)
if [[ $? != 0 ]] ; then
    echo "installing pip..."
    sudo easy_install pip
else
    echo "pip installed"
fi

sudo pip2 install pyserial future empy

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")
ARDUPILOT_TOOLS="Tools/autotest"

exportline="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";
grep -Fxq "$exportline" ~/.profile 2>/dev/null || {
    read -p "Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [Y/n]?" -n 1 -r
    if [[ $REPLY =~ ^[Yy]$ ]] ; then
        echo $exportline >> ~/.profile
        eval $exportline
    else
        echo "Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH."
    fi
}

git submodule update --init --recursive

echo "finished"
