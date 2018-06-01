#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../../../..; pwd`

export PATH=/usr/local/bin:$PATH

echo $ROOT


(  # AirBotF4 board
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light BOARD=f4light_Airbot &&
 cp $ROOT/ArduPlane/f4light_Airbot.bin $ROOT/Release/Plane &&
 cp $ROOT/ArduPlane/f4light_Airbot.hex $ROOT/Release/Plane &&
 cp $ROOT/ArduPlane/f4light_Airbot.dfu $ROOT/Release/Plane 
) && (
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light BOARD=f4light_Airbot &&
 
 cp $ROOT/ArduCopter/f4light_Airbot.bin $ROOT/Release/Copter &&
 cp $ROOT/ArduCopter/f4light_Airbot.hex $ROOT/Release/Copter &&
 cp $ROOT/ArduCopter/f4light_Airbot.dfu $ROOT/Release/Copter 
) 

