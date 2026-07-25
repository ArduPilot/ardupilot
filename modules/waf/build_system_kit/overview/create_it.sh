# /bin/bash

D=$PWD
pushd ../..
./waf-light configure build --tools=$D/bbdlib.py --prelude=$'\tfrom waflib.extras import bbdlib\n\tbbdlib.start(cwd, VERSION, wafdir)\n\tsys.exit(0)'
popd
cp ../../waf bbd

