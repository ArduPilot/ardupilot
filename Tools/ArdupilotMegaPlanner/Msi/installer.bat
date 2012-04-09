@echo off

wix.exe ..\bin\release\

del installer.wixobj

"%wix%\bin\candle" installer.wxs -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension


"%wix%\bin\light" installer.wixobj "%wix%\bin\difxapp_x86.wixlib" -o APMPlanner32.msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension


"%wix%\bin\light" installer.wixobj "%wix%\bin\difxapp_x64.wixlib" -o APMPlanner64.msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension


pause
