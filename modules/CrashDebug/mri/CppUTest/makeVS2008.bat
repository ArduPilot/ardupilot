rem ****
rem * Command line build - For CppUTest - Run from CppUTest directory
rem * 
rem * this path works on my machine
rem ****PATH=C:\Windows\Microsoft.NET\Framework\v3.5;c:\windows;c:\windows\system32

msbuild /t:rebuild /verbosity:quiet CppUTest_VS2008.sln
set test_exe=tests\Debug\AllTests.exe
if exist %test_exe% %test_exe% -v
