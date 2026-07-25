@echo off

rem from issue #964

Setlocal EnableDelayedExpansion

rem Check Windows Version
set TOKEN=tokens=2*
ver | findstr /i "5\.0\." > nul
if %ERRORLEVEL% EQU 0 SET TOKEN=tokens=3*
ver | findstr /i "5\.1\." > nul
if %ERRORLEVEL% EQU 0 SET TOKEN=tokens=3*
ver | findstr /i "5\.2\." > nul
if %ERRORLEVEL% EQU 0 SET TOKEN=tokens=3*

rem Start calculating PYTHON and PYTHON_DIR
set PYTHON=
set PYTHON_DIR=

Setlocal EnableDelayedExpansion

set PYTHON_DIR_OK=FALSE
set REGPATH=

for %%i in (3.12 3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4 3.3 3.2 3.1 3.0 2.7 2.6 2.5) do (
for %%j in (HKCU HKLM) do (
for %%k in (SOFTWARE\Wow6432Node SOFTWARE) do (
for %%l in (Python\PythonCore IronPython) do (
set REG_PYTHON_EXE=python.exe
if "%%l"=="IronPython" (
set REG_PYTHON_EXE=ipy.exe
)

@echo on

set REGPATH=%%j\%%k\%%l\%%i\InstallPath
rem @echo Regpath !REGPATH!
REG QUERY "!REGPATH!" /ve 1>nul 2>nul
if !ERRORLEVEL! equ 0 (
  for /F "%TOKEN% delims=	 " %%A IN ('REG QUERY "!REGPATH!" /ve') do @set REG_PYTHON_DIR=%%B
  if exist !REG_PYTHON_DIR!  (
    IF NOT "!REG_PYTHON_DIR:~-1!"=="\" SET REG_PYTHON_DIR=!REG_PYTHON_DIR!\
    set REG_PYTHON=!REG_PYTHON_DIR!!REG_PYTHON_EXE!
    rem set PYTHON_DIR_OK=TRUE
    if "!PYTHON_DIR_OK!"=="FALSE" (
      set PYTHON_DIR=!REG_PYTHON_DIR!
      set PYTHON=!REG_PYTHON!
      set PYTHON_DIR_OK=TRUE
    )

    rem set PYTHON_DIR_OK=FALSE
    rem @echo Find !REG_PYTHON!
    rem goto finished
  )
)

echo off

)
rem for l
)
rem for k
)
rem for j
)
rem for i



:finished

Endlocal & SET PYTHON_DIR=%PYTHON_DIR% & SET PYTHON=%PYTHON%

if "%PYTHON_DIR%" == "" (
rem @echo No Python dir
set PYTHON=py
goto running
)

rem @echo %PYTHON_DIR%

if "%PYTHON%" == "" (
rem @echo No Python
set PYTHON=py
goto running
)

:running

@echo Using %PYTHON%

"%PYTHON%" -x "%~dp0waf" %*
Endlocal
exit /b %ERRORLEVEL%
