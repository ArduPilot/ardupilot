# Quadplane Package Place Support

This script implements support for package place in quadplanes.

To use this script you need to install it in the APM/scripts folder on
your microSD card (or build it into the firmware in ROMFS). Then
enable scripting with SCR_ENABLE=1 and reboot.

# Parameters

The script adds the following parameters:

## PKG_ENABLE

You need to set PKG_ENABLE=1 to enable this script

## PKG_RELEASE_FUNC

This needs to be set to the SERVOn_FUNCTION of the release servo. It
is recommended that you leave it at the default of 94 and set
SERVOn_FUNCTION to 94 for the servo you want to use for package
release.

## PKG_RELEASE_HGT

The parameter PKG_RELEASE_HGT controls the rangefinder height at which
the package will be released. This can be zero if you want to release
the package after you land.

## PKG_RELEASE_HOLD

This controls the time that the vehicle will stop the descent before
it releases the package. This defaults to 1 second and is used to let
the vehicle get into a steady hover.

After package release the vehicle will hold for another
PKG_RELEASE_HOLD seconds to let the package cleanly release before the
vehicle climbs.

# Operation

To setup a mission for package place you should setup your vehicle for
rangefinder landings. Setup a good lidar or radar and test that it
works. Then set RNGFND_LANDING=1 to enable use of rangefinder for VTOL
landing.

In the mission you should add a PAYLOAD_PLACE waypoint at the desired
location of the payload placement. The altitude of the PAYLOAD_PLACE
command should be set to zero as a relative height.

The PAYLOAD_PLACE command has a parametere "max descent" (parameter 1
of the command). If this is non-zero then this is the maximum amount
of height that the aircraft will descend from the start of the
descent. If the aircraft tries to descend more than this height then
the payload place will abort and the aircraft will climb back up to
the initial descent height then continue the mission.

# Landing Then Release

You can also do payload place where you wait till the vehicle fully
lands before doing the release. To do that set PGK_RELEASE_HGT to 0
and the aircraft will land fully and shutdown the motors for
PKG_RELEASE_HOLD seconds before releasing the servo. It will then hold
for another PKG_RELEASE_HOLD seconds, then will climb back up to the
original descent start height before continuing with the mission.
