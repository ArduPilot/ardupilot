# /bin/bash

D=$PWD
pushd ../..
./waf-light configure build --tools=$D/cbdlib.py --prelude=$'\tfrom waflib.extras import cbdlib\n\tcbdlib.start(cwd, VERSION, wafdir)\n\tsys.exit(0)'
popd
cp ../../waf cbd

