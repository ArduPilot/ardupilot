#!/bin/sh

# this copes with moving origin remote to a new git organisation
git submodule update --recursive --force --init
git submodule sync --recursive
git submodule update --recursive --force --init
