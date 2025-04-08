#!/usr/bin/env bash

# unpack latest MissionPlanner*.zip on firmware.ardupilot.org
# to facilitate upgrade of existing MissionPlanner installs

export PATH=$PATH:/bin:/usr/bin

directory="$1"
[ "$#" -eq 1 ] || {
    echo "Usage: unpack_mp.sh <DIRECTORY>"
    exit 1
}
cd $directory || exit 1

mplatest=$(/bin/ls -tr MissionPlanner*zip | tail -1)
lastsum="$(cat .latest 2> /dev/null)"
[ "$lastsum" = "$(md5sum $mplatest)" ] && {
    exit 0
}

echo "$(date) unpacking $mplatest"
mkdir -p upgrade.new upgrade
cd upgrade.new || exit 1
unzip -q "../$mplatest"
cd ..
md5sum $mplatest > .latest
mv upgrade upgrade.old
mv upgrade.new upgrade
rm -rf upgrade.old
find upgrade -type f  \( ! -iname "files.html" ! -iname ".makehtml" \) -print0 | xargs -i -0 md5sum '{}' > checksums.txt
