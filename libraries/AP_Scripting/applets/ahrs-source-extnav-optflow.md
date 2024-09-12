# ExternalNav / Optical flow source switching

Switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
This script is intended to help vehicles automatically switch between ExternalNav and optical flow

## Parmeter Descriptions

-- ESRC_EXTN_THRESH : ExternalNav innovation threshold
-- ESRC_EXTN_QUAL : ExternalNav quality threshold
-- ESRC_FLOW_THRESH : OpticalFlow innovation threshold
-- ESRC_FLOW_QUAL : OpticalFlow quality threshold
-- ESRC_RNGFND_MAX : Rangefinder altitude threshold (in meters)

## How to use

Configure a downward facing lidar with a range of at least 5m
Set RCx_OPTION = 90 (EKF Pos Source) to select the source (low=ExternalNav, middle=opticalflow, high=Not Used)
Set RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
Set SRC_ENABLE = 1 (enable scripting)
Set EK3_SRCn_ parameters so that ExternalNav is the primary source, opticalflow is secondary

  - EK3_SRC1_POSXY = 6 (ExternalNav)
  - EK3_SRC1_VELXY = 6 (ExternalNav)
  - EK3_SRC1_VELZ  = 6 (ExternalNav)
  - EK3_SRC1_POSZ  = 6 (ExternalNav) or 1 (Baro)
  - EK3_SRC1_YAW   = 6 (ExternalNav) or 1 (Compass)
  - EK3_SRC2_POSXY = 0 (None)
  - EK3_SRC2_VELXY = 5 (OpticalFlow)
  - EK3_SRC2_VELZ  = 0 (None)
  - EK3_SRC2_POSZ  = 1 (Baro)
  - EK3_SRC2_YAW   = 1 (Compass)
  - EK3_SRC_OPTIONS = 0 (Do not fuse all velocities)

When the 2nd auxiliary switch (300/Scripting1) is pulled high automatic source selection is used based on the following criteria:

  - ESRC_EXTN_THRESH holds the threshold for ExternalNav innovation threshold (around 0.3 is a good choice)
  - ESRC_EXTN_QUAL holds the ExternalNav quality threshold (about 10 is a good choice)
  - ESRC_FLOW_QUAL holds the optical flow quality threshold (about 50 is a good choice)
  - ESRC_FLOW_THRESH holds the threshold for optical flow innovations (about 0.15 is a good choice)
  - ESRC_RNGFND_MAX holds the threshold (in meters) for rangefinder altitude

  - If ExternalNav's quality is above ESRC_EXTN_QUAL and innovations are below ESRC_EXTN_THRESH, ExternalNav is used
  - Optical flow is used if the above is not true and:

    - Quality is above ESRC_FLOW_QUAL
    - Innovations are below ESRC_FLOW_THRESH
    - Rangefinder distance is below ESRC_RNGFND_MAX
