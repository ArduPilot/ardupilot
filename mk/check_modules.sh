#!/bin/sh

echo "Checking modules"

MODULE_LIST="PX4Firmware PX4NuttX mavlink uavcan uavcan/dsdl uavcan/libuavcan/dsdl_compiler/pyuavcan"

NEED_INIT=0

cd $(dirname "$0")/.. || exit 1

for m in $MODULE_LIST; do
    [ -d modules/$m ] || {
        echo "modules/$m missing - need module init"
        NEED_INIT=1
        break
    }
    [ -f modules/$m/.git ] || {
        echo "modules/$m/.git missing - need module init"
        NEED_INIT=1
        break
    }
done

[ $NEED_INIT = 1 ] && {
    set -x
    git submodule init || {
        echo "git submodule init failed"
        git submodule status --recursive
        exit 1
    }
    (cd modules/uavcan && git submodule init) || {
        echo "init of uavcan failed"
        git submodule status --recursive
        exit 1
    }
    git submodule update --recursive || {
        echo "git submodule update failed"        
        git submodule status --recursive
        exit 1
    }
    for m in $MODULE_LIST; do
        [ -d modules/$m ] || {
            echo "modules/$m missing - failed module init"
            exit 1
        }
        [ -f modules/$m/.git ] || {
            echo "modules/$m/.git missing - failed module init"
            exit 1
        }
    done
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
