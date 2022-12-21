# Script Control LUA Script

This script allows the user to have groups of scripts in three subdirectories of the main scripting directory and copy them into the main directory, removing any files in that directory with the .lua extension (except itself) and issue a scripting restart based on the position of a switch with the auxiliary function of "302". This provides  means to have differing scripting depending on the mission or conditions, and use whichever set is desired without having them consume memory all the time and without requiring a GCS to manage the content of the scripts directory. 

# Setup and Operation

The user must setup an RCx_OPTION to function 302 to determine which of the three scripts subdirectories will be used: LOW:scripts/1, MIDDLE:scripts/2, or HIGH:scripts/3. If no RC has been established during ground start, it will behave as if subdirectory 1 is selected. Changing this switch position either prior to ground start or after, will remove ALL the *.lua files in the scripts directory (except itself), and copy any *.lua files (only) from the desginated subdirectory, and then issue a scripting restart. 

# CAUTIONS

If parameters are being created and used be sure that different scripts do not use the same PARAMETER KEY to create their param tables, unless you desire to use the same parameter set in two different scripts. Otherwise, param values will be corrupt when you change scripts.
Be sure to have unique parameter names for different parameter sets. Names are only visible and used by Ground Control Stations. While indentically named params in different scripts will operate fine, you will not be able to distinguish them in the GCS displays!

Any .lua script in the scripts directory will be ERASED! when this script is run. In order to keep another .lua script running when using this script, place it in every subdirectory so that it will be copied over when changing directories.


