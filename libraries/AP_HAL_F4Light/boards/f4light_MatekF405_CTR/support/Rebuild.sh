#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../../../..; pwd`

export PATH=/usr/local/bin:$PATH

echo $ROOT


( # MatekF405_CTR board
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light VERBOSE=1 BOARD=f4light_MatekF405_CTR
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=f4light_MatekF405_CTR
)

