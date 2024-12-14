#/usr/bin/env bash
set -euxo pipefail

PYTHON=$(which python)
echo "Installing all sub-libraries with $PYTHON"

git submodule update --init --recursive
$PYTHON -m ensurepip
$PYTHON -m pip install --upgrade pip setuptools wheel
$PYTHON -m pip install --upgrade empy==3.3.4 pexpect
find . | egrep -v '/build/' | grep setup.py | xargs readlink -e | xargs dirname | xargs -I{} bash -c "pushd {} ; $PYTHON -m pip install . ; popd"
