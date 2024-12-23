#!/usr/bin/env sh
echo "---------- $0 start ----------"
set -e
set -x

echo "==================================================================="
echo "Warning: This script is not fully tested. Please report any issues."
echo "==================================================================="


apk update && apk add --no-cache \
        linux-headers \
        g++ \
        python3 \
        py-future \
        py-pip \
        libxml2-dev \
        libxslt-dev \
        git \
    &&  rm -rf /var/cache/apk/*

python3 -m pip install --user --no-deps --no-cache-dir empy==3.3.4 pexpect ptyprocess --break-system-packages
