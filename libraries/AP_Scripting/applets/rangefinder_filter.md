# RangeFinder Filter Lua Script

This script provides an easy framework to implement custom filters with RangeFinders. Two simple examples are implemented.  With this script if two or more rangefinders are present, you can:
A. Get the minimum of the two rangefinders
B. Get the average of the two rangefinders

Setup:
This script requires RNGFND1_TYPE = 36. This is compulsory since ArduPilot polls the RangeFinder in asscending order, and will use the first "healthy" rangefinder. I.e, if both RNGFND1 and RNGFND2 are healthy, RNGFND1 will always be used.

Set RNGFND1_MAX/MIN/ORIENT to the same as all the other rangefinders that are configured.

# Parameters
The script adds the following parameters:

## RF_FILT_TYPE
 RangeFinder Filter Type. Set 0 to use minimum distances from all the other RangeFinders in the same direction. Set 1 to use average of all RangeFinders.
