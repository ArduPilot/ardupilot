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
 - cmake ..
 - make (will build every sketch)
 - make ArduPlane (will build just ArduPlane etc.)
 
Building using eclipse
-----------------------------------------------

	assuming source located here: /home/name/apm-src
	You can either download it or grab it from git:
	git clone https://code.google.com/p/ardupilot-mega/ /home/name/apm-src
	
 	mkdir /home/name/apm-build 
 	cd /home/name/apm-build
 	cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../apm-src -D BOARD=mega -D PORT=/dev/ttyUSB0
 	
 	Note: unix can be substitude for MinGW/ MSYS/ NMake (for windows)
 	(see http://www.vtk.org/Wiki/Eclipse_CDT4_Generator)

    Import project using Menu File->Import
    Select General->Existing projects into workspace:
    Browse where your build tree is and select the root build tree directory. Keep "Copy projects into workspace" unchecked.
    You get a fully functional eclipse project
    
    You can also import the source repository (/home/name/apm-src) if you want to modify the source/ commit using git.
    
  
Build a package using cpack
-----------------------------------------------
 - cd build
 - cmake ..
 - make package
 - make package_source
