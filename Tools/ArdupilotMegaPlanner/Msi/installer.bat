@echo off

wix.exe ..\bin\release\

pause

del installer.wixobj

"%wix%\bin\candle" installer.wxs -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension


"%wix%\bin\light" installer.wixobj "%wix%\bin\difxapp_x86.wixlib" -o MissionPlanner32.msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension


"%wix%\bin\light" installer.wixobj "%wix%\bin\difxapp_x64.wixlib" -o MissionPlanner64.msi -ext WiXNetFxExtension -ext WixDifxAppExtension -ext WixUIExtension.dll -ext WixUtilExtension

"C:\Program Files\7-Zip\7z.exe" a -tzip "Mission Planner.zip" ..\bin\release\*


pause


rem googlecode_upload.py -s "Mission Planner installer (32-bit)" -p ardupilot-mega -u meee146 MissionPlanner32.msi
rem googlecode_upload.py -s "Mission Planner installer (64-bit)" -p ardupilot-mega -u meee146 MissionPlanner64.msi