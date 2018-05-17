#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../..; pwd`

export PATH=/usr/local/bin:$PATH

echo $ROOT

mkdir -p $ROOT/Release/Copter
mkdir -p $ROOT/Release/Plane


make_copter(){
 local BOARD=$1
 
 cd $ROOT/ArduCopter
 make f4light-clean
 make f4light BOARD=$BOARD && (

  cp $ROOT/ArduCopter/$BOARD.bin $ROOT/Release/Copter
  cp $ROOT/ArduCopter/$BOARD.hex $ROOT/Release/Copter
  cp $ROOT/ArduCopter/$BOARD.dfu $ROOT/Release/Copter
  cp $ROOT/ArduCopter/${BOARD}_bl.bin $ROOT/Release/Copter
  cp $ROOT/ArduCopter/${BOARD}_bl.dfu $ROOT/Release/Copter
 )
}

make_plane(){
 local BOARD=$1

 cd $ROOT/ArduPlane
 make f4light-clean
 make f4light VERBOSE=1 BOARD=$BOARD && (

 cp $ROOT/ArduPlane/$BOARD.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/$BOARD.hex $ROOT/Release/Plane
 cp $ROOT/ArduPlane/$BOARD.dfu $ROOT/Release/Plane
 cp $ROOT/ArduPlane/${BOARD}_bl.bin $ROOT/Release/Plane
 cp $ROOT/ArduPlane/${BOARD}_bl.dfu $ROOT/Release/Plane
 )

}

( # RevoMini board
 make_copter "f4light_Revolution" && \
 make_plane  "f4light_Revolution"
) && ( # AirBotF4 board
 make_copter "f4light_Airbot" && \
 make_plane  "f4light_Airbot"
) && ( # Cl_Racing F4 board
 make_copter "f4light_cl_racing" && \
 make_plane  "f4light_cl_racing"
) && ( # AirBotV2 board
 make_copter "f4light_AirbotV2" && \
 make_plane  "f4light_AirbotV2"
) && ( # OmnibusV3 board
 make_copter "f4light_OmnibusV3" && \
 make_plane  "f4light_OmnibusV3"
) && ( # RevoMini board with SD card
 make_copter "f4light_Revolution_SD" && \
 make_plane  "f4light_Revolution_SD"
) && ( # MatekF405_CTR board
 make_copter "f4light_MatekF405_CTR" && \
 make_plane  "f4light_MatekF405_CTR"
) && (
 cd $ROOT

 zip -r latest.zip Release
# git add latest.zip
)











