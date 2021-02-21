#!/bin/sh

echo "Checking modules"

MODULE_LIST="gbenchmark gtest mavlink mavlink/pymavlink PX4Firmware PX4Firmware/src/lib/matrix PX4Firmware/Tools/gencpp PX4Firmware/Tools/genmsg PX4NuttX uavcan uavcan/dsdl uavcan/libuavcan/dsdl_compiler/pyuavcan waf"

NEED_INIT=0

export GIT_PAGER=cat

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
    for m in $MODULE_LIST; do
        [ -f modules/$m/.gitmodules ] && {
            (cd modules/$m && git submodule init) || {
                echo "init of $m failed"
                git submodule status --recursive
                exit 1
            }
        }
    done
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

Please see https://dev.ardupilot.org/wiki/git-submodules/
EOF
            exit 1
        }
done

exit 0
