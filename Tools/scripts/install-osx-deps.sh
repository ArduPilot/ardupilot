#!/bin/bash

# OS X dependencies for Tracis CI

brew update
brew outdated pyenv || brew upgrade pyenv
pyenv install -s 2.7.12
pyenv global 2.7.12
pyenv version
which pip
