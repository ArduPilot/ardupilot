@echo off
setlocal
set tempFilename=%1.temp
set tempSortedFilename=%1.sort
set outputFilename=%2
del %tempFilename% >nul 2>nul
del %tempSortedFilename% >nul 2>nul
del %outputFilename% >nul 2>nul

for /f "tokens=1-3 delims=: " %%i in (%1) do call :ProcessLine %%i %%j %%k

sort %tempFilename% /O %tempSortedFilename%

set lastLine=""
for /f "tokens=* delims=:" %%i in (%tempSortedFilename%) do call :Unique "%%i"
echo.>>%outputFilename%

del %tempFilename% >nul 2>nul
del %tempSortedFilename% >nul 2>nul
endlocal
goto :EOF



:ProcessLine
set filename=%2
set filename=%filename:~1,-6%
if "%1"=="Lines" set lastCoverage=%3
if "%1"=="Creating" call :OutputCovLine %lastCoverage%%%  %filename%
goto :EOF

:OutputCovLine
set percent=%1
set /a wholePercent=%percent:~0,-3%
set fractPercent=%percent:~-3%
if %wholePercent% LSS 10  echo   %wholePercent%%fractPercent%  %2>>%tempFilename%
if %wholePercent% LSS 100 echo  %wholePercent%%fractPercent%  %2>>%tempFilename%
if %wholePercent% GEQ 100 echo %wholePercent%%fractPercent%  %2>>%tempFilename%
goto :EOF

:Unique
set currentLine=%1
if not %lastLine%==%currentLine% echo %currentLine:~1,-1%>>%outputFilename%
set lastLine=%currentLine%
goto :EOF