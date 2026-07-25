The New*.sh scripts are helpful for creating the initial files for a new class...
	NewClass.sh - for TTDing a new C++ class
	NewInterface.sh - for TDDing a new interface along with its Mock
	NewCModule.sh - for TDDing a C module
	NewCmiModule.sh - for TDDing a C module where there will be multiple 
			instances of the module's data structure

Run InstallScripts.sh to
	1) Copy the scripts to /usr/local/bin
	2) Define symbolic links for each of the scripts

Like this:
	./InstallScripts.sh
	
You might have to add the execute privilege to the shell scripts.
Like this:
	chmod *.sh

Using NewClass for example:
	cd to the directory where you want the files located
	NewClass SomeClass
	
The script gets you ready for TDD and saves a lot of tedious typing
	Creates SomeClass.h SomeClass.cpp SomeClassTest.cpp
	with the class and test essentials in place
	(If the file already exists, no file is generated)


These scripts are written in bash.

