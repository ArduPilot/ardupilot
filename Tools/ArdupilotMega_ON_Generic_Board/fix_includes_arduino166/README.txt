With this tool you can change all include headers files to work with arduino 1.6.6 in windows.

To use this tool you need first compile with VS2013 or with C# Develop.
Then you use as follow:

fix_includes_arduino166 -P <PATH_TO_FOLDER>

Example: Supose that you cloned Ardupilot in c:\ardupilot then you will have c:\ardupilot\libraries and you want make AntennaTracker c:\ardupilot\AntennaTracker then,

fix_includes_arduino166 -P c:\ardupilot\libraries
fix_includes_arduino166 -P c:\ardupilot\AntennaTracker

WARNING!
This tool make changes to the files in all librarie path and this tool doesn't make a backup of your files. You must be sure.