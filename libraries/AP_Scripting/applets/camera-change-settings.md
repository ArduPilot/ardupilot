# Camera Change Settings

Allows changing some camera settings that are not normallly used by the autopilot

# Parameters

## CAM1_THERM_PAL

Set the camera's thermal palette

Supported values are
-1: leave unchanged
0: WhiteHot
2: Sepia
3: IronBow
4: Rainbow
5: Night
6: Aurora
7: RedHot
8: Jungle
9: Medical
10: BlackHot
11: GloryHot

## CAM1_THERM_GAIN

Set the camera's thermal gain

Supported values are
-1: leave unchanged
0: LowGain (50C to 550C)
1: HighGain (-20C to 150C)

## CAM1_THERM_RAW

Enable/Disable the saving of raw thermal images.  Enabling raw iamges slightly slows the live video feed

Supported values are
-1: leave unchanged
0: Disabled (30fps)
1: Enabled (25 fps)

# Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controllers microSD card. Review the above parameter descriptions and
decide on the right parameter values for your vehicle and operations.
