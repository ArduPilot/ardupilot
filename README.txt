Building using arduino
--------------------------
To install the libraries:
 - copy Library Directories to your \arduino\hardware\libraries\ or arduino\libraries directory
 - Restart arduino IDE

 * Each library comes with a simple example. You can find the examples in menu File->Examples

Building using make 
-----------------------------------------------
 - go to directory of sketch and type make.

Building using cmake
-----------------------------------------------
 - mkdir build
 - cd build
 - cmake .. -DBOARD=mega -DPORT=/dev/ttyUSB0
    You can select from mega/mega2560.
    If you have arduino installed in a non-standard location you by specify it by using:
        -DARDUINO_SDK_PATH=/path/to/arduino ..
 - make (will build every sketch)
 - make ArduPlane (will build just ArduPlane etc.)
 - make ArduPloat-upload (will upload the sketch)

    If you have a sync error during upload reset the board/power cycle the board
    before the upload starts.

 
Building using eclipse
-----------------------------------------------

    Getting the Source:

        assuming source located here: /home/name/apm-src
        You can either download it or grab it from git:
        git clone https://code.google.com/p/ardupilot-mega/ /home/name/apm-src

    Generating the Eclipse Project for Your System:
    
        mkdir /home/name/apm-build 
        cd /home/name/apm-build
        cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../apm-src -D BOARD=mega -D PORT=/dev/ttyUSB0

        Note: Unix can be substituted for MinGW/ MSYS/ NMake (for windows)
            (see http://www.vtk.org/Wiki/Eclipse_CDT4_Generator)

        input options:

            CMAKE_BUILD_TYPE choose from DEBUG, RELEASE etc.
            PORT is the port for uploading to the board, COM0 etc on windows. /dev/ttyUSB0 etc. on linux
            BOARD is your board type, mega for the 1280 or mega2560 for the 2560 boards.
            ARDUINO_SDK_PATH if it is not in default path can specify as /path/to/arduino
        
    Importing the Eclipse Build Project:

        Import project using Menu File->Import
        Select General->Existing projects into workspace:
        Browse where your build tree is and select the root build tree directory. 
        Keep "Copy projects into workspace" unchecked.
        You get a fully functional eclipse project

    Importing the Eclipse Source Project:
    
        You can also import the source repository (/home/name/apm-src) if you want to modify the source/ commit using git.

    Settings up Eclipse to Recognize PDE files:

         Window > Preferences > General > Content Types. This tree associates a
            filename or filename pattern with its content type so that tools can treat it
            properly. Source and header files for most languages are under the Text tree. 
            Add "*.pde" as a C++ Source.

	Autocompletion:
	
		Right click on source project -> Properties -> Project References -> apm-build Project
    
    Advanced:
    
        * Regenerating the eclipse source project file:
            cmake -G"Eclipse CDT4 - Unix Makefiles" -DECLIPSE_CDT4_GENERATE_SOURCE_PROJECT=TRUE /home/name/apm-src

Build a package using cpack
-----------------------------------------------
 - cd build
 - cmake ..
 - make package
 - make package_source


vim:ts=4:sw=4:expandtab
