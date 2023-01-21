#!/bin/sh

./waf configure --board=linux --debug
./waf build --target=examples/AP_Marvelmind_test -j6
repo_root="$(git rev-parse --show-toplevel)/"
"$repo_root/build/linux/examples/AP_Marvelmind_test" -A tcp:127.0.0.1:5111 &
sleep 1
cat ${repo_root}/libraries/AP_Beacon/examples/AP_Marvelmind_test/sample.dump| socat - tcp:127.0.0.1:5111
wait
