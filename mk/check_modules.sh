#!/bin/sh

echo "Checking modules"

MODULE_LIST="PX4Firmware PX4NuttX uavcan"

NEED_INIT=0

cd $(dirname "$0")/.. || exit 1

for m in $MODULE_LIST; do
    [ -d modules/$m ] || {
        echo "module/$m missing - need module init"
        NEED_INIT=1
        break
    }
    [ -f modules/$m/.git ] || {
        echo "module/$m/.git missing - need module init"
        NEED_INIT=1
        break
    }
done

[ $NEED_INIT = 1 ] && {
    set -x
    git submodule init || {
        echo "git submodule init failed"
        exit 1
    }
    git submodule update || {
        echo "git submodule update failed"        
        exit 1
    }
cat <<EOF
==============================
git submodules are initialised

Please see http://dev.ardupilot.com/wiki/git-submodules/

Please restart the build
==============================
EOF
exit 1
}

for m in $MODULE_LIST; do
	RET=$(git submodule summary modules/$m | egrep "^..<")
	[ -z "$RET" ] || {
            echo "Module modules/$m out of date"
            git submodule summary modules/$m
cat <<EOF

You need to run 'git submodule update'

Please see http://dev.ardupilot.com/wiki/git-submodules/
EOF
            exit 1
        }
done

exit 0
