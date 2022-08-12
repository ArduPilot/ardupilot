# arming-checks.lua

NOTE: probably requires increasing SCR_HEAP_SIZE to at least 204800

This script executes pre-arm checks on a flight controller. 
Checks can be flagged as SEVERITY_ERROR which will prevent arming or
SEVERITY_WARNING which will display a warning message on the GCS

Multiple checks can be added or removed.

The code has been written to avoid spamming the GCS, so in most cases, the user will be
notified only if the error/warning condition trips (failes) and again if the issue is resolved.

To add new checks, copy one of the existing methods to reuse this pattern to avoid spam messages.

The default checks are:
GeoFence enabled when there is no fence set up on the flight controler
GeoFence autoenable set when there is no fence - but only triggers in AUTO or TAKEOFF mode
Tridge's RTL_AUTOLAND check, which can now be disabled (or reverted to a warning)
