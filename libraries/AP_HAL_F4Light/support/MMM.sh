#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../..; pwd`

export PATH=/usr/local/bin:$PATH

echo $ROOT

mkdir -p $ROOT/Release/Copter
mkdir -p $ROOT/Release/Plane


( # RevoMini board
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_Revolution && (

 cp $ROOT/ArduCopter/revomini_Revolution.bin $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_Revolution.hex $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_Revolution.dfu $ROOT/Release/Copter
 )
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_Revolution && (

 cp $ROOT/ArduPlane/revomini_Revolution.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_Revolution.hex $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_Revolution.dfu $ROOT/Release/Plane
 )
) && ( # AirBotF4 board
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_Airbot  && (

 cp $ROOT/ArduCopter/revomini_Airbot.bin $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_Airbot.hex $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_Airbot.dfu $ROOT/Release/Copter

 make f4light-clean

 )
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_Airbot && (

 cp $ROOT/ArduPlane/revomini_Airbot.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_Airbot.hex $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_Airbot.dfu $ROOT/Release/Plane

 make f4light-clean

 )
) && ( # Cl_Racing F4 board
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revo_cl_racing  && (

 cp $ROOT/ArduCopter/revo_cl_racing.bin $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revo_cl_racing.hex $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revo_cl_racing.dfu $ROOT/Release/Copter

 make f4light-clean

 )
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_Airbot && (

 cp $ROOT/ArduPlane/revo_cl_racing.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revo_cl_racing.hex $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revo_cl_racing.dfu $ROOT/Release/Plane

 make f4light-clean

 )
) && ( # AirBotF4 board
 cd $ROOT/ArduCopter
# make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_AirbotV2  && (

 cp $ROOT/ArduCopter/revomini_AirbotV2.bin $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_AirbotV2.hex $ROOT/Release/Copter
 cp $ROOT/ArduCopter/revomini_AirbotV2.dfu $ROOT/Release/Copter


 )
) && (
 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=revomini_AirbotV2 && (

 cp $ROOT/ArduPlane/revomini_AirbotV2.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_AirbotV2.hex $ROOT/Release/Plane
 cp $ROOT/ArduPlane/revomini_AirbotV2.dfu $ROOT/Release/Plane
 )

) && (
 cd $ROOT

 zip -r latest.zip Release
 git add . -A
)











