#!/bin/sh
rm -rf _tmp
git clone git://github.com/pixhawk/mavlink.git _tmp
rm -rf _tmp/.git
rsync -av _tmp/* .
rm -rf _tmp
