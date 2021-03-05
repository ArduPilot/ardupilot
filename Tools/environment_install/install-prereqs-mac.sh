#!/bin/bash

echo "`tput setaf 2`Checking homebrew...`tput sgr0`"
$(which -s brew)
if [[ $? != 0 ]] ; then
    echo "`tput setaf 2`installing homebrew... `tput sgr0`"
    ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
else
    echo "`tput setaf 2`Homebrew installed`tput sgr0`"
fi

echo "`tput setaf 2`Installing Xcode Command Line Tools`tput sgr0`"
#Install Xcode Dependencies
xcode-select --install

echo "`tput setaf 2`Installing Homebrew Packages`tput sgr0`"
brew tap ardupilot/homebrew-ardupilot
brew update
brew install pyenv
brew install coreutils
brew install gcc-arm-none-eabi
brew install gawk

echo "`tput setaf 2`Setting up pyenv...`tput sgr0`"
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bash_profile
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bash_profile
echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.bash_profile

echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.zshrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.zshrc
echo -e 'if command -v pyenv 1>/dev/null 2>&1; then\n  eval "$(pyenv init -)"\nfi' >> ~/.zshrc

source $HOME/.zshrc

pyenv init

echo "`tput setaf 2`Installing Python 3.9.1 and setting it as pyenv global`tput sgr0`"
env PYTHON_CONFIGURE_OPTS="--enable-framework" pyenv install 3.9.1
pyenv global 3.9.1

# May need to reload shell or call pyenv init 
pyenv versions
python --version

echo "`tput setaf 2`Checking pip...`tput sgr0`"
$(which -s pip)
if [[ $? != 0 ]] ; then
    echo "`tput setaf 2`Installing pip...`tput sgr0`"
    # Easy install does not support python 2.x anymore
    # sudo easy_install pip
    curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
    sudo -H python get-pip.py
else
    echo "`tput setaf 2`pip already installed checking for updates...`tput sgr0`"
    sudo -H pip install --upgrade pip
    sudo -H pip install --upgrade setuptools
fi

echo "`tput setaf 2`pip installed, version:`tput sgr0`"
pip --version

echo "`tput setaf 2`Installing brew version of wxpython`tput sgr0`"
brew install wxpython

echo "`tput setaf 2`Installing python packages...`tput sgr0`"
pip install wheel
pip install --user pyserial future empy mavproxy pexpect

pythonpath="export PATH=$HOME/Library/Python/3.9/bin:\$PATH";
echo $pythonpath >> ~/.bash_profile
echo $pythonpath >> ~/.zshrc
eval $pythonpath

SCRIPT_DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))
ARDUPILOT_ROOT=$(realpath "$SCRIPT_DIR/../../")
ARDUPILOT_TOOLS="Tools/autotest"

exportline="export PATH=$ARDUPILOT_ROOT/$ARDUPILOT_TOOLS:\$PATH";

grep -Fxq "$exportline" ~/.bash_profile 2>/dev/null || {
   read -p "`tput setaf 11`Add $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to your PATH [N/y]?`tput sgr0`" -n 1 -r
   if [[ $REPLY =~ ^[Yy]$ ]] ; then
       echo $exportline >> ~/.bash_profile
       echo $exportline >> ~/.zshrc
       eval $exportline
   else
       echo "`tput setaf 2`Skipping adding $ARDUPILOT_ROOT/$ARDUPILOT_TOOLS to PATH.`tput sgr0`"
   fi
}

git submodule update --init --recursive

echo "`tput setaf 11`Finished!, please restart your terminal for changes to take effect.`tput sgr0`"
