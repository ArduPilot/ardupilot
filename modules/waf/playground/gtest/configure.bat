@echo off

rem edit this path
set LIB_PATH=e:/project/libs

set INCLUDE=%CD%
pushd "%LIB_PATH%"
set LIB_PATH=%CD%
popd

rem edit those paths
set GTEST_PATH=%LIB_PATH%\gtest-1.7.0
set INCLUDE=%INCLUDE%;%GTEST_PATH%\include
set LIB=%LIB%;%GTEST_PATH%\msvc\gtest-msvc2013\Debug
set LIB=%LIB%;%GTEST_PATH%\msvc\gtest-msvc2013\Release

python waf configure
