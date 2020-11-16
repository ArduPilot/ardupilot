rem File run APM:Rover SITL
SETLOCAL enableextensions
@echo off

rem Assumes a Cgywin install at C:\cygwin
if "%CYGWIN_LOCATION%" == "" (set "CYGWIN_LOCATION=C:\cygwin")

rem get current dir for Cygwin
set pth=%CD:~2,99%
set pth=%pth:\=/%
set drv=%CD:~0,1%
set "fullpath=/cygdrive/%drv%%pth%"

%CYGWIN_LOCATION%\bin\bash.exe --login -i -c "cd ""%fullpath%"" && cd ../../../Rover && ../Tools/autotest/sim_vehicle.py"
