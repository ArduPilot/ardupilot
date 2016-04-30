#!/bin/bash
# Install dependencies and configure the environment for CI build testing on OSX

brew install ccache
brew install bash

sudo pip install pexpect
sudo pip install mavproxy

sudo bash -c 'echo /usr/local/bin/bash >> /etc/shells'
