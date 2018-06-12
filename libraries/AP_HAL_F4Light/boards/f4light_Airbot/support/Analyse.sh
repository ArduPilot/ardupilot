#git submodule init && git submodule update
export TOOLCHAIN

ROOT=`cd ../../../../..; pwd`

export PATH=/usr/local/bin:$PATH:/usr/local/cov-analysis-linux/bin

#cov-configure --comptype gcc --compiler /usr/local/bin/arm-none-eabi-gcc
#cov-configure --comptype g++ --compiler /usr/local/bin/arm-none-eabi-g++
#it required itself:
# ./cov-configure -co /usr/local/bin/arm-none-eabi-g++ -- -fno-exceptions -std=gnu++11 -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -mcpu=cortex-m4 -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard




 cd $ROOT/ArduCopter
 rm -rf cov-int
 
 cov-build --dir cov-int make f4light BOARD=revomini_Airbot 
 tail cov-int/build-log.txt
 
 rm -f ardurevo-analyse.tgz
 tar czf ardurevo-analyse.tgz cov-int
