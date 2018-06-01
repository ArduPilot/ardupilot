#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../../../..; pwd`

export PATH=/usr/local/bin:$PATH

echo $ROOT


( # AirBotF4 board
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light BOARD=f4light_cl_racing
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light BOARD=f4light_cl_racing
)

