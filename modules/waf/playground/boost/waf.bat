@echo off

rem path are automatically detected on Linux
set BOOST_PATH=e:\project\libs\boost_1_55_0
set BOOST_LIB=%BOOST_PATH%\lib32-msvc-12.0

set LIB=%BOOST_LIB%
set INCLUDE=%BOOST_PATH%

python waf --boost-linkage_autodetect --boost-python=27 %*
