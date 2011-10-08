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

Build a package using cpack
-----------------------------------------------
 - cd build
 - cmake ..
 - make package
 - make package_source
