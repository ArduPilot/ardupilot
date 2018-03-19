#!/bin/sh
set -e
set -x

pushd minimosd-extra/
git checkout -f master
git fetch origin
git reset --hard origin/master
popd
# git diff # this should show a change to git hash of OSD
git add minimosd-extra
git commit -m 'HAL_F4Light: updated OSD submodule'
