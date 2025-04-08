# How to profile ardupilot binary on ESP32 and use result to place critical functions to the IRAM
## Prepare source
1. Configure project with `--enable-profile` option: `./waf configure --board=esp32diy --enable-profile`
2. Rebuild binaries
## Run program with profiling enabled
1. Flash and run binary
2. Try to use mode silimiar to real flight. I.E. compass/gps connected, armed state, and so no
3. It will write profile stat to the `/APM/APM/PROF000.TXT` files on the sdcard every one minute
## Use profile info to optimize program
1. Copy `PROF*.TXT` file from sd card
2. Copy file `arducopter.map` from build directory
2. Use them to produce `functions.list` by the script `LinkerScriptGenerator.kt` (modify paths and limits inside)
3. Place file `functions.list` to the `target/copter/main` folder


# how to build LinkerScriptGenerator.kt

$ sudo snap install --classic kotlin

# build
kotlinc LinkerScriptGenerator.kt -d generator.jar

# collect up profiling file into this folder... as per above.

# files that the kotlin tool expects to find in this folder:
        "ardusub.map",
        "PROF000.TXT",

# file/s it will create in this folder:
        "functions.list"


cp ../../../build/esp32buzz/idf-plane/arduplane.map ardusub.map
touch PROF000.TXT

#run:
java -jar generator.jar


