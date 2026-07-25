#!/bin/bash

set -e
set -x

test -z "$MDEF" && MDEF="../message_definitions"

# MAVLINK_DIALECT=ardupilotmega python setup.py clean build install --user

tools/mavgen.py --lang C $MDEF/v1.0/all.xml -o generator/C/include_v1.0 --wire-protocol=1.0
tools/mavgen.py --lang C $MDEF/v1.0/all.xml -o generator/C/include_v2.0 --wire-protocol=2.0
tools/mavgen.py --lang C++11 $MDEF/v1.0/all.xml -o generator/CPP11/include_v2.0 --wire-protocol=2.0

pushd generator/C/test/posix
make clean testmav1.0_ardupilotmega testmav2.0_ardupilotmega test_issues

# these test tools emit the test packet as hexadecimal and human-readable,
# other tools consume it as a cross-reference, we ignore the hex here.
./testmav1.0_ardupilotmega | egrep -v '(^fe|^fd)'
./testmav2.0_ardupilotmega | egrep -v '(^fe|^fd)'
./test_issues
popd

pushd generator/CPP11/test/posix
make clean all
popd
