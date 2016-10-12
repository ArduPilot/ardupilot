#!/bin/bash

# OS X dependencies for Tracis CI

brew update
brew install pyenv
pyenv install -s 2.7.12
pyenv global 2.7.12
pyenv version
which pip
