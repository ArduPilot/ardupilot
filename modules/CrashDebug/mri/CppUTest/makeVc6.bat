rem ****
rem * Command line build - For CppUTest - Run from dsw directory
rem * 
rem * A single parameter is supported and it is the last parameter of msdev
rem * for example:
rem * 	/clean
rem * make sure to use the slash
rem * this needs to be in your path 
rem ****PATH=C:\Program Files\Microsoft Visual Studio\VC98\Bin;%PATH%

msdev CppUTest.dsp /MAKE "CppUTest - Debug" %1
msdev tests\AllTests.dsp /MAKE "AllTests - Debug" %1
if "%1" EQU "/clean" goto end
if "%1" EQU "/CLEAN" goto end
tests\Debug\AllTests
:end
