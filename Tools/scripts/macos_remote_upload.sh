#!/usr/bin/env bash
# allows uploading firmware via ssh from remote computer to 
# device connected to macos system
# place export AP_OVERRIDE_UPLOAD_CMD=". /path/to/macos_remote_upload.sh" to bashrc
# to use this
USER_HOST=user@hostname # please edit this with macos ssh 
tmpdir=$(ssh $USER_HOST mktemp -d)
filename=$(basename $@)
uploader=$(realpath $(dirname $@))/../../../Tools/scripts/uploader.py
scp $@ $USER_HOST:$tmpdir/
scp $uploader $USER_HOST:$tmpdir/
#  source "\$HOME/.bash_profile" && $tmpdir/uploader.py $tmpdir/$filename
ssh $USER_HOST /bin/bash << ENDSSH
source ~/.bash_profile
$tmpdir/uploader.py $tmpdir/$filename
rm -r $tmpdir
ENDSSH