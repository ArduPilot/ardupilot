rem File to update the APM source
rem Assumes a Cgywin install at C:\cygwin
@echo off

rem get current dir for Cygwin
set pth=%CD:~2,99%
set pth=%pth:\=/%
set drv=%CD:~0,1%
set "fullpath=/cygdrive/%drv%%pth%"

rem update the source
C:\cygwin\bin\bash.exe --login -i -c "cd ""%fullpath%"" && cd ../../../ && git pull && git submodule update --init --recursive"

rem re-configure build config
C:\cygwin\bin\bash.exe --login -i -c "cd ""%fullpath%"" && cd ../../../ && ./modules/waf/waf-light configure --board=sitl"

pause
