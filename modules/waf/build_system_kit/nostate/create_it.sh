# /bin/bash

D=$PWD
pushd ../..
./waf-light configure build --tools=$D/ebdlib.py --prelude=$'\tfrom waflib.extras import ebdlib\n\tebdlib.start(cwd, VERSION, wafdir)\n\tsys.exit(0)'
popd
cp ../../waf ebd

