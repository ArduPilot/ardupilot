Getting the source
==================

We'll assume you are putting the source in `/home/name/ardupilot`.

You can either download the source using the "ZIP" button at
https://github.com/diydrones/ardupilot, or you can grab it from github
using git:

```
git clone git://github.com/diydrones/ardupilot.git
```

Building using Arduino
======================

First install the libraries:

 1. Copy the `libraries` directory to your
    `/path/to/arduino/hardware/libraries/` or
    `/path/to/arduino/libraries` directory.

 2. Restart the Arduino IDE.

Each library comes with a simple example. You can find the examples in
the menu File→Examples

Building using make 
===================

 1. Go to the directory of the sketch and type `make`.

 2. Type `make upload` to upload according to the parameters in
    `config.mk`.

For example:

```
cd ArduPlane  # or ArduCopter etc.
make
make upload
```

Building using cmake
====================
```
cd ArduPlane  # or ArduCopter etc.
mkdir build
cd build

# If you have Arduino installed in a non-standard location you by
# specify it by using -DARDUINO_SDK_PATH=/path/to/arduino
cmake .. -DAPM_BOARD=mega -DAPM_PORT=/dev/ttyUSB0  # Or -DAPM_BOARD=mega2560

make  # Will build the sketch.
make ArduPlane-upload  # Will upload the sketch.
```

If you have a sync error during upload, reset the board or power cycle
the board before the upload starts.

Building using Eclipse
======================

Generating the Eclipse project for your system
----------------------------------------------

```    
mkdir /home/name/apm-build 
cd /home/name/apm-build
cmake -G"Eclipse CDT4 - Unix Makefiles" \
  -D CMAKE_BUILD_TYPE=Debug \
  -D BOARD=mega \
  -D PORT=/dev/ttyUSB0 \
  ../ardupilot/ArduCopter
```

Change the `../ardupilot/ArduCopter` above to be whatever sketch you
want to build.

_Note: Unix can be substituted for MinGW/MSYS/NMake (for Windows).
See http://www.vtk.org/Wiki/Eclipse_CDT4_Generator)._

### Define options

 * `CMAKE_BUILD_TYPE` choose from `DEBUG`, `RELEASE` etc.

 * `PORT` is the port for uploading to the board, `COM0` etc. on
   Windows, `/dev/ttyUSB0` etc. on Linux.

 * `BOARD` is your board type, `mega` for the 1280 or `mega2560` for
   the 2560 boards.

 * `ARDUINO_SDK_PATH` to specify the path to your Arduino installation
   if it isn't in the default path.
        
Importing the Eclipse build project
-----------------------------------

 1. Import project using Menu File→Import.

 2. Select General→Existing projects into workspace.

 3. Browse to where your build tree is and select the root build tree
    directory.

 4. Keep "Copy projects into workspace" unchecked.

You should now have a fully functional eclipse project.

Importing the Eclipse source project
------------------------------------
    
You can also import the source repository (`/home/name/ardupilot`) if you
want to modify the source/commit using git.

Configuring Eclipse to recognize PDE files
------------------------------------------

 * File association: Go to Window→Preferences→General→Content
   Types. This tree associates a filename or filename pattern with its
   content type so that tools can treat it properly. Source and header
   files for most languages are under the Text tree. Add "*.pde" as a
   C++ Source.

 * Autocompletion: Right click on source project→Properties→Project
   References→apm-build Project
    
 * Regenerating the Eclipse source project file: `cmake -G"Eclipse
   CDT4 - Unix Makefiles" -DECLIPSE_CDT4_GENERATE_SOURCE_PROJECT=TRUE
   /home/name/ardupilot`


Build a package using cpack
===========================
```
cd build
cmake ..
make package
make package_source
```
