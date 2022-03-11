#!/bin/sh

# this copes with moving origin remote to a new git organisation
# we run it 3 times due to the poor handling of recursion
git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive

git submodule update --recursive --force --init
git submodule sync --recursive
