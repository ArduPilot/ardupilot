# /bin/bash

D=$PWD
pushd ../..
./waf-light configure build --tools=$D/dbdlib.py --prelude=$'\tfrom waflib.extras import dbdlib\n\tdbdlib.start(cwd, VERSION, wafdir)\n\tsys.exit(0)'
popd
cp ../../waf dbd

