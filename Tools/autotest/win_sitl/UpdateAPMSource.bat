rem File to update the APM source
rem Assumes a Cgywin install at C:\cygwin
@echo off

rem Assumes a Cgywin install at C:\cygwin64
if "%CYGWIN_LOCATION%" == "" (set "CYGWIN_LOCATION=C:\cygwin64")

rem get current dir for Cygwin
set pth=%CD:~2,99%
set pth=%pth:\=/%
set drv=%CD:~0,1%
set "fullpath=/cygdrive/%drv%%pth%"

rem update the source
%CYGWIN_LOCATION%\bin\bash.exe --login -i -c "cd ""%fullpath%"" && cd ../../../ && git pull && git submodule update --init --recursive"

rem re-configure build config
%CYGWIN_LOCATION%\bin\bash.exe --login -i -c "cd ""%fullpath%"" && cd ../../../ && ./modules/waf/waf-light configure --board=sitl"

pause
