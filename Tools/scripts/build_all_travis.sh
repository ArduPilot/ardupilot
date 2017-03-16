#!/bin/bash
# useful script to test all the different build types that we support.
# This helps when doing large merges
# Andrew Tridgell, November 2011

. config.mk

set -e
set -x

. ~/.profile

# CXX and CC are exported by default by travis
c_compiler=${CC:-gcc}
cxx_compiler=${CXX:-g++}
unset CXX CC

# Override CI_BUILD_TARGET
CI_BUILD_TARGET="sitl apm1 apm2"

declare -A build_platforms
declare -A build_concurrency
declare -A waf_supported_boards

build_platforms=(  ["plane"]="sitl apm1 apm2"
                   ["copter"]="sitl"
                   ["rover"]="sitl apm1 apm2"
                   ["antennatracker"]="sitl apm1 apm2")

build_concurrency=(["sitl"]="-j2"
                   ["apm2"]="-j2"
                   ["apm1"]="-j2")

waf=modules/waf/waf-light

# get list of boards supported by the waf build
for board in $($waf list_boards | head -n1); do waf_supported_boards[$board]=1; done

echo "Targets: $CI_BUILD_TARGET"

$waf distclean
for t in $CI_BUILD_TARGET; do

    echo "Starting waf build for board ${t}..."
	$waf configure --board $t
	$waf clean

    for v in ${!build_platforms[@]}; do
        if [[ ${build_platforms[$v]} != *$t* ]]; then
            continue
        fi

        echo "Building $v for ${t}..."
		$waf ${build_concurrency[$t]} $v

    done

	if [ $t != "sitl" ]; then
		$waf ${build_concurrency[$t]} examples
	fi

done

echo build OK
exit 0
