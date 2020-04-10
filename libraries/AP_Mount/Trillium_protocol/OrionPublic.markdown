<style>
    body {
        text-align:justify;
        max-width: 25cm;
        margin-left: auto;
        margin-right: auto;
        font-family: Georgia;
        counter-reset: h1counter h2counter  h3counter toc1counter toc2counter toc3counter;
     }

    table {
       border: 1px solid #e0e0e0;
       border-collapse: collapse;
       margin-bottom: 25px;
    }

    th, td {
        border: 1px solid #e0e0e0;
        font-family: Courier, monospace;
        font-size: 90%;
        padding: 2px;
    }

    /*
     * Alternate colors for the table, including the heading row
     */
    th {
    background-color: #e0e0e0   
    }
    tr:nth-child(even){background-color: #e0e0e0}

    h1, h2, h3, h4, h5 { font-family: Arial; }
    h1 { font-size:120%; margin-bottom: 25px; }
    h2 { font-size:110%; margin-bottom: 15px; }
    h3 { font-size:100%; margin-bottom: 10px;}
    h4, li { font-size:100%; }

    caption{ font-family:Arial; font-size:85%;}

    code, pre, .codelike {
        font-family: Courier, monospace;
        font-size: 100%;
        color: darkblue;
    }

    /*
     * Counters for the main headings
     */

    h1:before {
        counter-increment: h1counter;
        content: counter(h1counter) "\00a0 ";
    }
    h1 {
        counter-reset: h2counter;
    }
    
    h2:before {
        counter-increment: h2counter;
        content: counter(h1counter) "." counter(h2counter) "\00a0 ";
    }
    h2 {
        counter-reset: h3counter;
    }
    
    h3:before {
      counter-increment: h3counter;
      content: counter(h1counter) "." counter(h2counter) "." counter(h3counter) "\00a0 ";
    }

    toctitle {font-family: Arial; font-size:120%; font-weight: bold; margin-bottom:25px; display:block;}
    toc1, toc2, toc3 {font-family: Arial; font-size:100%; margin-bottom:2px; display:block;}
    toc1 {text-indent: 0px;}
    toc2 {text-indent: 15px;}
    toc3 {text-indent: 30px;}
    
    toc1:before {
        content: counter(toc1counter) "\00a0 ";
        counter-increment: toc1counter;
    }
    toc1 {
        counter-reset: toc2counter;
    }
    
    toc2:before {
        content: counter(toc1counter) "." counter(toc2counter) "\00a0 ";
        counter-increment: toc2counter;
    }
    toc2 {
        counter-reset: toc3counter;
    }

    toc3:before {
      content: counter(toc1counter) "." counter(toc2counter) "." counter(toc3counter) "\00a0 ";
      counter-increment: toc3counter;
    }

    /* How it looks on a screen, notice the fancy hr blocks and lack of page breaks */
    @media screen {
      body {
        background-color: #f0f0f0;
      }
      .page-break { display: none; }
      hr { 
        height: 25px; 
        border-style: solid; 
        border-color: gray; 
        border-width: 1px 0 0 0; 
        border-radius: 10px; 
      } 
      hr:before { 
        display: block; 
        content: ""; 
        height: 25px; 
        margin-top: -26px; 
        border-style: solid; 
        border-color: gray; 
        border-width: 0 0 1px 0; 
        border-radius: 10px; 
      }
    }

    /* How it looks when printed, hr turned off, in favor of page breaks*/
    @media print {
      hr {display: none;}
      body {background-color: white;}
      .page-break{page-break-before: always;}
    }
}
</style>



# OrionPublic Protocol

The *OrionPublic* protocol is the packet description for the public interface
of the Orion gimbal from [Trillium Engineering](http://w3.trilliumeng.com/).
The Orion gimbal communicates to users with packets of data transported over
asyncrhonous serial, or transported by TCP/IP over ethernet.

 Protocol version is 1.3.0.a.

## Packet format

The general packet format is given in OrionPublicPacketShim.h and TrilliumPacket.h (in the Orion SDK), and has this form:

[Orion public packet general format]
| Byte         | Name    | Value                                                  |
|--------------|---------|--------------------------------------------------------|
| 0            | Sync0   | 0xD0                                                   |
| 1            | Sync1   | 0x0D                                                   |
| 2            | ID      | Packet identifier from 0 to 255                        |
| 3            | Length  | Number of Data bytes, from 0 to 140                    |
| 4...Length+3 | Data    | Data bytes of the packet, in the layout described here |
| Length+4     | Fletch0 | Most significant byte of the 16-bit checksum           |
| Length+5     | Fletch1 | Least significant byte of the 16-bit checksum          |

The checksum of the packet is a modified 16-bit Fletcher's checksum, using an initial value of 1 and modulo 251 instead of modulo 255. The checksum starts on byte 0 and goes through all data bytes. See TrilliumPacket.c (in the Orion SDK) for details.

    static void InitChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
    {
        // For the first iteration, both checksum bytes should be equal
        *pA = *pB = (((UInt16)Byte & 0xFF) + 1) % 251;

    }// InitChecksum

    static void UpdateChecksum(UInt8 Byte, UInt16 *pA, UInt16 *pB)
    {
        // Fletcher's checksum mod 251 (prime number) instead of 255
        *pA = (*pA + ((UInt16)Byte & 0xFF)) % 251;
        *pB = (*pB + *pA) % 251;

    }// UpdateChecksum 


<div class="page-break"></div>


---


# Enumerations

Packets in the protocol refer to these global enumerations.


## OrionPktType_t enumeration

Identifiers of Orion public packets. The purpose of an Orion packet is determined by its ID field to this list.

[<a name="OrionPktType_t"></a>OrionPktType_t enumeration]
| Name                                                                | Value | Description                                                                 |
| ------------------------------------------------------------------- | :---: | --------------------------------------------------------------------------- |
| `ORION_PKT_INITIALIZE`                                              | 0     |                                                                             |
| [`ORION_PKT_CMD`](#ORION_PKT_CMD)                                   | 1     | Basic commands for control of gimbal pan and tilt.                          |
| [`ORION_PKT_UART_CONFIG`](#ORION_PKT_UART_CONFIG)                   | 2     | Baud rate and protocol configuration for crown serial ports                 |
| [`ORION_PKT_LASER_CMD`](#ORION_PKT_LASER_CMD)                       | 3     | Command to fire the laser                                                   |
| `ORION_PKT_RESET`                                                   | 4     | Send this packet to cause the gimbal to reset                               |
| [`ORION_PKT_LASER_STATES`](#ORION_PKT_LASER_STATES)                 | 6     | Laser state information                                                     |
| [`ORION_PKT_STARTUP_CMD`](#ORION_PKT_STARTUP_CMD)                   | 7     | Command to run on startup at the end of motor initialization                |
| [`ORION_PKT_LIMITS`](#ORION_PKT_LIMITS)                             | 34    | Pan and tilt limits for current, acceleration, velocity, and position       |
| [`ORION_PKT_CLEVIS_VERSION`](#ORION_PKT_CLEVIS_VERSION)             | 37    | Software version for the clevis board                                       |
| [`ORION_PKT_RESET_SOURCE`](#ORION_PKT_RESET_SOURCE)                 | 38    | Reset information for a processor                                           |
| [`ORION_PKT_BOARD`](#ORION_PKT_BOARD)                               | 39    | Information for a specific board                                            |
| [`ORION_PKT_CROWN_VERSION`](#ORION_PKT_CROWN_VERSION)               | 40    | Software version for the crown board                                        |
| [`ORION_PKT_PAYLOAD_VERSION`](#ORION_PKT_PAYLOAD_VERSION)           | 41    | Software version for the payload board                                      |
| [`ORION_PKT_TRACKER_VERSION`](#ORION_PKT_TRACKER_VERSION)           | 44    | Software version for the tracker board (if present)                         |
| [`ORION_PKT_DIAGNOSTICS`](#ORION_PKT_DIAGNOSTICS)                   | 65    | Electrical diagnostic data                                                  |
| `ORION_PKT_FAULTS`                                                  | 66    |                                                                             |
| [`ORION_PKT_PERFORMANCE`](#ORION_PKT_PERFORMANCE)                   | 67    | Measurements related to stabilization performance                           |
| [`ORION_PKT_SOFTWARE_DIAGNOSTICS`](#ORION_PKT_SOFTWARE_DIAGNOSTICS) | 68    | Diagnostic information related to software performance for a processor      |
| [`ORION_PKT_VIBRATION`](#ORION_PKT_VIBRATION)                       | 69    | Platform vibration statistics                                               |
| [`ORION_PKT_NETWORK_DIAGNOSTICS`](#ORION_PKT_NETWORK_DIAGNOSTICS)   | 70    | Network diagnostics                                                         |
| [`ORION_PKT_CAMERA_SWITCH`](#ORION_PKT_CAMERA_SWITCH)               | 96    | Command the gimbal to swtich cameras                                        |
| [`ORION_PKT_CAMERA_STATE`](#ORION_PKT_CAMERA_STATE)                 | 97    | Command a camera in the gimbal                                              |
| [`ORION_PKT_NETWORK_VIDEO`](#ORION_PKT_NETWORK_VIDEO)               | 98    | Command the IP settings for video delivery                                  |
| [`ORION_PKT_CAMERAS`](#ORION_PKT_CAMERAS)                           | 99    | Camera settings information                                                 |
| [`ORION_PKT_FLIR_SETTINGS`](#ORION_PKT_FLIR_SETTINGS)               | 103   | FLIR-specific camera settings                                               |
| [`ORION_PKT_APTINA_SETTINGS`](#ORION_PKT_APTINA_SETTINGS)           | 104   | Aptina-specific camera settings                                             |
| [`ORION_PKT_ZAFIRO_SETTINGS`](#ORION_PKT_ZAFIRO_SETTINGS)           | 105   | DRS Zafiro-specific camera settings                                         |
| [`ORION_PKT_HITACHI_SETTINGS`](#ORION_PKT_HITACHI_SETTINGS)         | 106   | Hitachi-specific camera settings                                            |
| `ORION_PKT_BAE_SETTINGS`                                            | 107   | BAE-specific camera settings                                                |
| [`ORION_PKT_SONY_SETTINGS`](#ORION_PKT_SONY_SETTINGS)               | 108   | Sony-specific camera settings                                               |
| [`ORION_PKT_KTNC_SETTINGS`](#ORION_PKT_KTNC_SETTINGS)               | 109   | KTnC-specific camera settings                                               |
| [`ORION_PKT_AUTOPILOT_DATA`](#ORION_PKT_AUTOPILOT_DATA)             | 128   | Autopilot state data, if available                                          |
| [`ORION_PKT_RETRACT_CMD`](#ORION_PKT_RETRACT_CMD)                   | 160   | Retract or deploy command for a gimbal retraction mechanism                 |
| [`ORION_PKT_RETRACT_STATUS`](#ORION_PKT_RETRACT_STATUS)             | 161   | Current gimbal retraction mechanism status                                  |
| [`ORION_PKT_DEBUG_STRING`](#ORION_PKT_DEBUG_STRING)                 | 177   | A human readable text string which is typically used for debugging purposes |
| [`ORION_PKT_USER_DATA`](#ORION_PKT_USER_DATA)                       | 178   | Packet for sending user data to a serial port in the gimbal                 |
| [`ORION_PKT_KLV_USER_DATA`](#ORION_PKT_KLV_USER_DATA)               | 179   | KLV user-specified field settings                                           |
| [`ORION_PKT_GPS_DATA`](#ORION_PKT_GPS_DATA)                         | 209   | Raw GPS data input or output                                                |
| [`ORION_PKT_EXT_HEADING_DATA`](#ORION_PKT_EXT_HEADING_DATA)         | 210   | Externally supplied heading information                                     |
| [`ORION_PKT_INS_QUALITY`](#ORION_PKT_INS_QUALITY)                   | 211   | Quality of the inertial navigation system                                   |
| [`ORION_PKT_GEOLOCATE_TELEMETRY`](#ORION_PKT_GEOLOCATE_TELEMETRY)   | 212   | Information needed to locate the gimbal and its imagery                     |
| [`ORION_PKT_GEOPOINT_CMD`](#ORION_PKT_GEOPOINT_CMD)                 | 213   | Command the gimbal to go to Geopoint mode                                   |
| [`ORION_PKT_RANGE_DATA`](#ORION_PKT_RANGE_DATA)                     | 214   | Range information from the gimbal to its target                             |
| [`ORION_PKT_PATH`](#ORION_PKT_PATH)                                 | 215   | Command the gimbal to path track mode                                       |
| [`ORION_PKT_INS_OPTIONS`](#ORION_PKT_INS_OPTIONS)                   | 216   | Configuration options for the Inertial Navigation System                    |
| [`ORION_PKT_STARE_START`](#ORION_PKT_STARE_START)                   | 217   | Stare portion of step stare started                                         |
| [`ORION_PKT_STARE_ACK`](#ORION_PKT_STARE_ACK)                       | 218   | Stare acknowledged, clear to move                                           |
| [`ORION_PKT_NETWORK_SETTINGS`](#ORION_PKT_NETWORK_SETTINGS)         | 228   | Change the IP network settings in the gimbal                                |
| [`ORION_PKT_RETRACT_VERSION`](#ORION_PKT_RETRACT_VERSION)           | 248   | Software version for the retract board (if present)                         |
| [`ORION_PKT_LENSCTL_VERSION`](#ORION_PKT_LENSCTL_VERSION)           | 249   | Software version for the lens control board (if present)                    |



## GimbalAxis_t enumeration

Enumeration of gimbal pan and tilt axes.

[<a name="GimbalAxis_t"></a>GimbalAxis_t enumeration]
| Name               | Value | Description                        |
| ------------------ | :---: | ---------------------------------- |
| `GIMBAL_AXIS_PAN`  | 0     | The pan axis                       |
| `GIMBAL_AXIS_TILT` | 1     | The tilt axis                      |
| `NUM_GIMBAL_AXES`  | 2     | Total number of axes in the gimbal |



## Axis_t enumeration

3D attitude axis enumeration.

[<a name="Axis_t"></a>Axis_t enumeration]
| Name         | Value | Description                   |
| ------------ | :---: | ----------------------------- |
| `AXIS_ROLL`  | 0     | Roll axis (rotation about x)  |
| `AXIS_PITCH` | 1     | Pitch axis (rotation about y) |
| `AXIS_YAW`   | 2     | Yaw axis (rotation about z)   |
| `NUM_AXES`   | 3     | Number of attitude axes       |



## BodyAxis_t enumeration

Enumeration for body axis X, Y, Z arrays.

[<a name="BodyAxis_t"></a>BodyAxis_t enumeration]
| Name      | Value | Description                             |
| --------- | :---: | --------------------------------------- |
| `AXIS_X`  | 0     | X points forward (away from connector)  |
| `AXIS_Y`  | 1     | Y points to the right (if X is forward) |
| `AXIS_Z`  | 2     | Z points down                           |
| `NUM_XYZ` | 3     | Number of body axes                     |



## Stator_t enumeration

Enumeration for motor stator axes.

[<a name="Stator_t"></a>Stator_t enumeration]
| Name          | Value | Description                       |
| ------------- | :---: | --------------------------------- |
| `STATOR_U`    | 0     | First field of three phase motor  |
| `STATOR_V`    | 1     | Second field of three phase motor |
| `STATOR_W`    | 2     | Third field of three phase motor  |
| `NUM_STATORS` | 3     | Number of motor phases            |



## OrionBoardEnumeration_t enumeration

Enumeration identifying different boards in the Orion Gimbal.

[<a name="OrionBoardEnumeration_t"></a>OrionBoardEnumeration_t enumeration]
| Name             | Value | Description                              |
| ---------------- | :---: | ---------------------------------------- |
| `BOARD_NONE`     | 0     | Unknown board                            |
| `BOARD_CLEVIS`   | 1     | The clevis board (motor control)         |
| `BOARD_CROWN`    | 2     | The crown board (ins and communications) |
| `BOARD_PAYLOAD`  | 3     | The paylaod board (camera)               |
| `BOARD_LENSCTRL` | 4     | The lens control board                   |
| `BOARD_MISSCOMP` | 5     | The mission computer                     |



## OrionMode_t enumeration

The mode of operation of the gimbal.

[<a name="OrionMode_t"></a>OrionMode_t enumeration]
| Name                            | Value | Description                                                                                                                                                                                                                                                            |
| ------------------------------- | :---: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ORION_MODE_DISABLED`           | 0     | disable motor output                                                                                                                                                                                                                                                   |
| `ORION_MODE_FAULT`              | 1     | Gimbal has experienced a fault and motors have been disabled                                                                                                                                                                                                           |
| `ORION_MODE_RATE`               | 16    | Commands give the line of sight angular rate of the camera                                                                                                                                                                                                             |
| `ORION_MODE_GEO_RATE`           | 17    | Inertial rate mode, with bulk motion compensation when valid slant range is available                                                                                                                                                                                  |
| `ORION_MODE_FFC_AUTO`           | 32    | Commands the gimbal to perform a flat field correction, pan and tilt will drive the IR camera to look at the black body                                                                                                                                                |
| `ORION_MODE_FFC`                | 32    | Legacy FFC mode, alias of ORION_MODE_FFC_AUTO                                                                                                                                                                                                                          |
| `ORION_MODE_FFC_MANUAL`         | 33    | Commands the gimbal to perform a flat field correction, pan and tilt will drive the IR camera to the commanded positions. NOTE: The pan/tilt targets specify the payload position, not the boresight position, so the reported angles may not match the command target |
| `ORION_MODE_SCENE`              | 48    | Commands give the angular rate of the image produced by the camera                                                                                                                                                                                                     |
| `ORION_MODE_TRACK`              | 49    | Commands give the angular rate of the track box in the camera image                                                                                                                                                                                                    |
| `ORION_MODE_CALIBRATION`        | 64    | Drives the pan and tilt axes to calibrate the stabilizer gyros                                                                                                                                                                                                         |
| `ORION_MODE_POSITION`           | 80    | Commands give the position of the pan and tilt axes with respect to the gimbal mount                                                                                                                                                                                   |
| `ORION_MODE_POSITION_NO_LIMITS` | 81    | Commands give the position of the pan and tilt axes with respect to the gimbal mount. NOTE: This mode ignores the pan/tilt limits and can result in serious damage to the gimbal if used incorrectly                                                                   |
| `ORION_MODE_GEOPOINT`           | 96    | Gimbal is pointing toward a fixed spatial location                                                                                                                                                                                                                     |
| `ORION_MODE_PATH`               | 112   | Gimbal is pointing toward a fixed path in space, and may be doing step-stare                                                                                                                                                                                           |
| `ORION_MODE_DOWN`               | 113   | Gimbal is pointing straight down, and may be doing step-stare                                                                                                                                                                                                          |
| `ORION_MODE_UNKNOWN`            | 255   | Unknown gimbal mode. This is unused in normal operation and is strictly to maintain compatibility with v1.2                                                                                                                                                            |



## OrionMaxLasers_t enumeration

The maximum number of lasers in an Orion gimbal.

[<a name="OrionMaxLasers_t"></a>OrionMaxLasers_t enumeration]
| Name         | Value | Description                                  |
| ------------ | :---: | -------------------------------------------- |
| `MAX_LASERS` | 2     | Maximum number of lasers in an Orion gimbal. |



## OrionLaserType_t enumeration

Laser type identifiers.

[<a name="OrionLaserType_t"></a>OrionLaserType_t enumeration]
| Name                       | Value | Description        |
| -------------------------- | :---: | ------------------ |
| `LASER_TYPE_NONE`          | 0     | No laser installed |
| `LASER_TYPE_POINTER`       | 1     | Laser pointer      |
| `LASER_TYPE_10MJ_MARKER`   | 2     | Laser marker       |
| `LASER_TYPE_LIGHTWARE`     | 3     | LightWare LRF      |
| `LASER_TYPE_JENOPTIK_DLEM` | 4     | Jenoptik DLEM LRF  |
| `LASER_TYPE_VECTRONIX`     | 5     | Vectronix LRF      |



## RangeDataSrc_t enumeration

Sources of ranging information.

[<a name="RangeDataSrc_t"></a>RangeDataSrc_t enumeration]
| Name                      | Value | Description                                                                                                                                                     |
| ------------------------- | :---: | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `RANGE_SRC_NONE`          | 0     | No ranging information                                                                                                                                          |
| `RANGE_SRC_SKYLINK`       | 1     | Range information from skylink terrrain lookup                                                                                                                  |
| `RANGE_SRC_LASER`         | 2     | Range information from laser rangefinder                                                                                                                        |
| `RANGE_SRC_OTHER`         | 3     | Other source of range data                                                                                                                                      |
| `RANGE_SRC_INTERNAL`      | 4     | Range information is being estimated internally by the gimbal. Note that estimates are not guaranteed be accurate and should only be used as a fallback option. |
| `RANGE_SRC_INTERNAL_DTED` | 5     | Range information estimated by the gimbal based on a DTED model                                                                                                 |
| `NUM_RANGE_SRCS`          | 6     |                                                                                                                                                                 |


<div class="page-break"></div>


---


# Packets sent regularly by the gimbal

These packets are sent by the gimbal automatically, depending on the operational mode of the gimbal.


## <a name="ORION_PKT_GEOLOCATE_TELEMETRY"></a>GeolocateTelemetryCore packet

This packet is transmitted at 10 Hz by the gimbal to communicate the information needed to locate the gimbal and its imagery, to determine the gimbal mode of operation, and to determine how the video output stream is being shifted by the gimbal. This is the primary packet that users should monitor to determine the gimbal status and populate user interfaces.

- packet identifier: `ORION_PKT_GEOLOCATE_TELEMETRY` : 212
- minimum data length: 46
- maximum data length: 89

Enumeration INS rotation options

[<a name="InsRotationOptions"></a>InsRotationOptions enumeration]
| Name                | Value | Description                                                                                |
| ------------------- | :---: | ------------------------------------------------------------------------------------------ |
| `insInGimbalNative` | 0     | The INS is running in the gimbal native axis                                               |
| `insInPlatform`     | 1     | The INS is running in the platform axis, see the INS options for specific roataion details |
| `insInPayloadBall`  | 2     | The INS is running in the payload ball.                                                    |





[GeolocateTelemetryCore packet bytes]
| Bytes   | Name                                        | [Enc](#Enc) | Repeat                           | Description                                                                                                                                                                                                                                                                                                                                         |
| ------- | ------------------------------------------- | :---------: | :------------------------------: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3   | 1)systemTime                                | U32         | 1                                | Milliseconds since system bootup.                                                                                                                                                                                                                                                                                                                   |
| 4...7   | 2)gpsITOW                                   | U32         | 1                                | GPS time of week in milliseconds.                                                                                                                                                                                                                                                                                                                   |
| 8...9   | 3)gpsWeek                                   | U16         | 1                                | GPS week number since Jan 6 1980.                                                                                                                                                                                                                                                                                                                   |
| 10...11 | 4)geoidUndulation                           | I16         | 1                                | Height of the geoid with respect to the WGS84 ellipsoid, in meters.<br>Scaled by 32767/(120) from -120.0 to 120.0.                                                                                                                                                                                                                                  |
| 12...15 | 5)posLat                                    | I32         | 1                                | Geodetic latitude of the gimbal in radians, positive North.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                                                                                       |
| 16...19 | 6)posLon                                    | I32         | 1                                | Longitude of the gimbal in radians, positive East.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                                                                                                |
| 20...23 | 7)posAlt                                    | I32         | 1                                | Altitude of the gimbal in meters above the WGS-84 ellipsoid.<br>Scaled by 10000 from -214748.3647 to 214748.3647.                                                                                                                                                                                                                                   |
| 24...29 | 8)velNED                                    | I16         | 3                                | Velocity of the gimbal in North, East, Down meters per second.<br>Scaled by 100 from -327.67 to 327.67.                                                                                                                                                                                                                                             |
| 30...37 | 9)gimbalQuat                                | I16         | 4                                | Gimbal quaternion describing the rotation from the gimbal mount frame to the North, East, Down navigation frame.<br>Scaled by 32767/(1) from -1.0 to 1.0.                                                                                                                                                                                           |
| 38...39 | 10)pan                                      | I16         | 1                                | Gimbal pan angle in radians from -pi to pi. This includes the camera alignment offsets.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.                                                                                                                                                                                                                |
| 40...41 | 11)tilt                                     | I16         | 1                                | Gimbal tilt angle in radians from -pi to pi This includes the camera alignment offsets.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.                                                                                                                                                                                                                |
| 42...43 | 12)hfov                                     | U16         | 1                                | Horizontal field of view of the camera in radians from 0 to 2pi.<br>Scaled by 65535/(2&times;&pi) from 0.0 to 2&pi;.                                                                                                                                                                                                                                |
| 44...45 | 13)vfov                                     | U16         | 1                                | Vertical field of view of the camera in radians from 0 to 2pi.<br>Scaled by 65535/(2&times;&pi) from 0.0 to 2&pi;.                                                                                                                                                                                                                                  |
| 46...51 | 14)losECEF                                  | I16         | 3                                | Vector from the gimbal to the image location in ECEF meters. The image location is only computed if ranging data are provided to the gimbal. Otherwise the image location data are 0,0,0.<br>Scaled by 1 from -32767.0 to 32767.0.<br>This field is optional. If it is not included then the value is assumed to be 0.                              |
| 52...53 | 15)pixelWidth                               | U16         | 1                                | Active focal plane width in pixels.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                                                                                                             |
| 54...55 | 16)pixelHeight                              | U16         | 1                                | Active focal plane height in pixels.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                                                                                                            |
| 56      | 17)[mode](#OrionMode_t)                     | U8          | 1                                | Operational mode of the gimbal.<br>This field is optional. If it is not included then the value is assumed to be ORION_MODE_UNKNOWN.                                                                                                                                                                                                                |
| 57      | 18)pathProgress                             | U8          | 1                                | Distance along the path from 0 (the starting point) to 1 (the ending point). This only applies if the mode is ORION_MODE_PATH.<br>Scaled by 255/(1) from 0.0 to 1.0.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                            |
| 58      | 19)stareTime                                | U8          | 1                                | Seconds remaining in the stare portion of a step stare point, 0 if stare is not in progress. This only applies if the mode is ORION_MODE_PATH.<br>Scaled by 255/(2.55) from 0.0 to 2.55.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                        |
| 59      | 20)pathFrom                                 | U8          | 1                                | Index of the path point gimbal is traveling from. This only applies if the mode is ORION_MODE_PATH.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                                             |
| 60      | 21)pathTo                                   | U8          | 1                                | Index of the path point gimbal is traveling to. This only applies if the mode is ORION_MODE_PATH.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                                               |
| 61...68 | 22)imageShifts                              | I32         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | X (horizontal) and Y (vertical) instantaneous image shifts in radians.<br>Scaled by 1000000 from -2147.483647 to 2147.483647.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                   |
| 69...70 | 23)imageShiftDeltaTime                      | U16         | 1                                | The amount of time in seconds over which these shifts apply. If no new shift data has arrived since this packet was last transmitted then this will be 0.<br>Scaled by 1000 from 0.0 to 65.535.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                 |
| 71      | 24)imageShiftConfidence                     | U8          | 1                                | Registration solution confidence, from 0 (no confidence) to 1 (full confidence).<br>Scaled by 255/(1) from 0.0 to 1.0.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                                          |
| 72...75 | 25)outputShifts                             | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | X (horizontal) and Y (vertical) image shifts in radians, as displayed in the video stream.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                         |
| 76      | 26)[rangeSource](#RangeDataSrc_t)           | U8          | 1                                | Range data source used in line of sight calculation.<br>This field is optional. If it is not included then the value is assumed to be RANGE_SRC_SKYLINK.                                                                                                                                                                                            |
| 77      | 27)leapSeconds                              | U8          | 1                                | Leap seconds to subtract from GPS time to compute UTC time.<br>This field is optional. If it is not included then the value is assumed to be 18.                                                                                                                                                                                                    |
| 78      | 28)panAlignment                             | I8          | 1                                | Pan camera alignment offsets in radian. The raw pan angle is given as pan + panAlignment.<br>Scaled by 1000 from -0.127 to 0.127.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                               |
| 79      | 29)tiltAlignment                            | I8          | 1                                | Tilt camera alignment offsets in radian. The raw tilt angle is given as tilt + tiltAlignment.<br>Scaled by 1000 from -0.127 to 0.127.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                           |
| 80      | 30)[insRotationOption](#InsRotationOptions) | U8          | 1                                | This flag indicates special INS options. If insSpecialOption is 1 the INS is running in the gimbal crown, but is rotated from the gimbal native axis. If insSpecialOption is 2 the INS is running in the ball of the gimbal.<br>This field is optional. If it is not included then the value is assumed to be insInGimbalNative.                    |
| 81...88 | 31)insQuat                                  | I16         | 4                                | The rotation of the INS from its native frame to the North, East, Down navigation frame. If insSpecialOption is 0 then this rotation is the same as the gimbalQuat.<br>Scaled by 32767/(1) from -1.0 to 1.0.<br>Only included if insRotationOption is non-zero.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_PERFORMANCE"></a>OrionPerformance packet

Measurements related to stabilization performance. This packet is sent by the gimbal 4 times per second.

- packet identifier: `ORION_PKT_PERFORMANCE` : 67
- data length: 20


[OrionPerformance packet bytes]
| Bytes   | Name      | [Enc](#Enc) | Repeat                           | Description                                                 |
| ------- | --------- | :---------: | :------------------------------: | ----------------------------------------------------------- |
| 0...3   | 1)RmsQuad | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Quad rms errors.<br>Scaled by 1000000 from 0.0 to 0.065535. |
| 4...7   | 2)RmsDir  | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Dir rms errors.<br>Scaled by 1000000 from 0.0 to 0.065535.  |
| 8...11  | 3)RmsVel  | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Vel rms errors.<br>Scaled by 1000 from 0.0 to 65.535.       |
| 12...15 | 4)RmsPos  | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Pos rms errors.<br>Scaled by 1000000 from 0.0 to 0.065535.  |
| 16...19 | 5)Iout    | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Output current.<br>Scaled by 1000 from 0.0 to 65.535.       |


## <a name="ORION_PKT_DIAGNOSTICS"></a>OrionDiagnostics packet

Electrical diagnostic data for the Orion gimbal. This packet is sent by the gimbal every 3 seconds.

- packet identifier: `ORION_PKT_DIAGNOSTICS` : 65
- data length: 28


[OrionDiagnostics packet bytes]
| Bytes   | Name             | [Enc](#Enc) | Repeat | Description                                                                                                  |
| ------- | ---------------- | :---------: | :----: | ------------------------------------------------------------------------------------------------------------ |
| 0...1   | 1)Voltage24      | U16         | 1      | Voltage of the 24-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                           |
| 2...3   | 2)Voltage12      | U16         | 1      | Voltage of the 12-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                           |
| 4...5   | 3)Voltage3v3     | U16         | 1      | Voltage of the 3.3-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                          |
| 6...7   | 4)Current24      | U16         | 1      | Current in Amps of the 24-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                   |
| 8...9   | 5)Current12      | U16         | 1      | Current in Amps of the 12-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                   |
| 10...11 | 6)Current3v3     | U16         | 1      | Current in Amps of the 3.3-volt rail.<br>Scaled by 1000 from 0.0 to 65.535.                                  |
| 12      | 7)CrownTemp      | I8          | 1      | Temperature of the crown board in degrees Celsius.<br>Scaled by 1.0 from -127.0 to 127.0.                    |
| 13      | 8)SlaTemp        | I8          | 1      | Temperature of the SLA-1500 board (if applicable) in degrees Celsius.<br>Scaled by 1.0 from -127.0 to 127.0. |
| 14      | 9)GyroTemp       | I8          | 1      | Stabilization gyro ADC temperature in degrees Celsius.<br>Scaled by 1.0 from -127.0 to 127.0.                |
| 15      | 10)reserved      | U8          | 1      | Reserved space for future expansion.<br>Data are given constant value on encode 0.                           |
| 16...17 | 11)Voltage24Var  | U16         | 1      | Voltage variance of the 24-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                               |
| 18...19 | 12)Voltage12Var  | U16         | 1      | Voltage variance of the 12-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                               |
| 20...21 | 13)Voltage3v3Var | U16         | 1      | Voltage variance of the 3.3-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                              |
| 22...23 | 14)Current24Var  | U16         | 1      | Current variance in Amps of the 24-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                       |
| 24...25 | 15)Current12Var  | U16         | 1      | Current variance in Amps of the 12-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                       |
| 26...27 | 16)Current3v3Var | U16         | 1      | Current variance in Amps of the 3.3-volt rail.<br>Scaled by 100000 from 0.0 to 0.65535.                      |


## <a name="ORION_PKT_SOFTWARE_DIAGNOSTICS"></a>OrionSoftwareDiagnostics packet

Diagnostic information related to software performance for a specific board in the Orion Gimbal. This packet is sent by the gimbal once per second.

- packet identifier: `ORION_PKT_SOFTWARE_DIAGNOSTICS` : 68
- minimum data length: 4
- maximum data length: 134


[OrionSoftwareDiagnostics packet bytes]
| Bytes   | Name                                      | [Enc](#Enc) | Repeat               | Description                                                                                                                                                   |
| ------- | ----------------------------------------- | :---------: | :------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0       | 1)[sourceBoard](#OrionBoardEnumeration_t) | U8          | 1                    | Enumeration defining which board in the gimbal generated this message.                                                                                        |
| 1       | 2)numCores                                | U8          | 1                    | Number of CPU cores that this packet reports for.                                                                                                             |
| 2...3   | 3)reserved                                | U16         | 1                    | Reserved space for future expansion.                                                                                                                          |
| 4...133 | 4)CoreLoading                                          || numCores, up to 2    | Per-Core CPU load information                                                                                                                                 |
| 4       | 4.1)cpuLoad                               | U8          | 1                    | CPU utilization as a percentage.<br>Scaled by 255/(100) from 0.0 to 100.0.                                                                                    |
| 5       | 4.2)heapLoad                              | U8          | 1                    | Heap memory utilization as a percentage, 0 if no heap is present.<br>Scaled by 255/(100) from 0.0 to 100.0.                                                   |
| 6       | 4.3)stackLoad                             | U8          | 1                    | System stack memory utilization as a percentage, 0 if no system stack is present.<br>Scaled by 255/(100) from 0.0 to 100.0.                                   |
| 7       | 4.4)reserved                              | U8          | 1                    | Reserved space for future expansion.                                                                                                                          |
| 8       | 4.5)numThreads                            | U8          | 1                    | Number of threads in the system, must be at least 1.                                                                                                          |
| 9...68  | 4.6)ThreadLoading                                      || numThreads, up to 10 | Per-thread information, in order of priority (highest to lowest)                                                                                              |
| 9       | 4.6.1)cpuLoad                             | U8          | 1                    | CPU utilization of this thread as a percentage.<br>Scaled by 255/(100) from 0.0 to 100.0.                                                                     |
| 10      | 4.6.2)heapLoad                            | U8          | 1                    | Heap memory utilization of this thread as a percentage, 0 if no heap is present.<br>Scaled by 255/(100) from 0.0 to 100.0.                                    |
| 11      | 4.6.3)stackLoad                           | U8          | 1                    | Stack memory utilization of this thread as a percentage, 100 if stack overflow occurs.<br>Scaled by 255/(100) from 0.0 to 100.0.                              |
| 12      | 4.6.4)watchdogLeft                        | U8          | 1                    | Worst case amount of watchdog time remaining in this reporting period for this thread, 0 if watchdog has timed out.<br>Scaled by 255/(100) from 0.0 to 100.0. |
| 13      | 4.6.5)numIterations                       | U8          | 1                    | Number of times this thread iterated during the reporting period.<br>Scaled by 0.1 from 0.0 to 2550.0.                                                        |
| 14      | 4.6.6)worstcase                           | U8          | 1                    | Worst case thread run time as a ratio with the average thread run time during the reporting period.<br>Scaled by 255/(25.5) from 0.0 to 25.5.                 |


## <a name="ORION_PKT_VIBRATION"></a>OrionVibration packet

Platform vibration amplitude and frequency information. This packet is sent by the gimbal once every five seconds.

- packet identifier: `ORION_PKT_VIBRATION` : 69
- data length: 140


[OrionVibration packet bytes]
| Bytes    | Name          | [Enc](#Enc) | Repeat              | Description                                                                                                 |
| -------- | ------------- | :---------: | :-----------------: | ----------------------------------------------------------------------------------------------------------- |
| 0...5    | 1)MaxAccel    | I16         | [NUM_AXES](#Axis_t) | Maximum acceleration values, in m/s/s.<br>Scaled by 32767/(20) from -20.0 to 20.0.                          |
| 6...11   | 2)MaxGyro     | I16         | [NUM_AXES](#Axis_t) | Maximum angular rate values, in rad/s.<br>Scaled by 32767/(0.1) from -0.1 to 0.1.                           |
| 12...139 | 3)FftData                  || 16                  | A series of points containing FFT-derived frequency and magnitude data                                      |
| 12...13  | 3.1)Frequency | U16         | 1                   | Frequency, in Hertz, of the data point defined by this element.<br>Scaled by 65535/(200) from 0.0 to 200.0. |
| 14...16  | 3.2)Accel     | U8          | [NUM_AXES](#Axis_t) | Acceleration magnitude, expressed as a percentage of MaxAccel.<br>Scaled by 255/(1) from 0.0 to 1.0.        |
| 17...19  | 3.3)Gyro      | U8          | [NUM_AXES](#Axis_t) | Angular rate magnitude, expressed as a percentage of MaxGyro.<br>Scaled by 255/(1) from 0.0 to 1.0.         |


## <a name="ORION_PKT_NETWORK_DIAGNOSTICS"></a>NetworkDiagnostics packet

Ethernet network diagnostic information

- packet identifier: `ORION_PKT_NETWORK_DIAGNOSTICS` : 70
- data length: 36


[NetworkDiagnostics packet bytes]
| Bytes   | Name             | [Enc](#Enc) | Repeat | Description                                                |
| ------- | ---------------- | :---------: | :----: | ---------------------------------------------------------- |
| 0...1   | 1)Flags          | U16         | 1      | Ethernet interface configuration flags.                    |
| 2...5   | 2)RxBytes        | U32         | 1      | Ethernet bytes received.                                   |
| 6...9   | 3)TxBytes        | U32         | 1      | Ethernet bytes transmitted.                                |
| 10...13 | 4)RxPackets      | U32         | 1      | Ethernet packets received.                                 |
| 14...17 | 5)TxPackets      | U32         | 1      | Ethernet packets transmitted.                              |
| 18...19 | 6)RxErrors       | U16         | 1      | Number of receive errors.                                  |
| 20...21 | 7)TxErrors       | U16         | 1      | Number of transmit errors.                                 |
| 22...23 | 8)RxDrops        | U16         | 1      | Number of receive packets dropped by the Ethernet driver.  |
| 24...25 | 9)TxDrops        | U16         | 1      | Number of transmit packets dropped by the Ethernet driver. |
| 26...27 | 10)RxFifoErrors  | U16         | 1      | Number of receive FIFO errors.                             |
| 28...29 | 11)TxFifoErrors  | U16         | 1      | Number of transmit FIFO errors.                            |
| 30...31 | 12)FrameErrors   | U16         | 1      | Framing error count.                                       |
| 32...33 | 13)Collisions    | U16         | 1      | Hardware interface collision count.                        |
| 34...35 | 14)CarrierErrors | U16         | 1      | Number of carrier loss events.                             |


## <a name="ORION_PKT_RETRACT_STATUS"></a>OrionRetractStatus packet

Current gimbal retraction mechanism status. This is sent by the gimbal regularly only if a retract system is installed.

- packet identifier: `ORION_PKT_RETRACT_STATUS` : 161
- data length: 6

The retraction mechanism state

[<a name="OrionRetractState_t"></a>OrionRetractState_t enumeration]
| Name                       | Value | Description                                 |
| -------------------------- | :---: | ------------------------------------------- |
| `RETRACT_STATE_DISABLED`   | 0     | Retract is disabled                         |
| `RETRACT_STATE_RETRACTED`  | 1     | Retract is fully retracted                  |
| `RETRACT_STATE_RETRACTING` | 2     | Retract is moving to the retracted position |
| `RETRACT_STATE_DEPLOYING`  | 3     | Retract is moving to the deployed position  |
| `RETRACT_STATE_DEPLOYED`   | 4     | Retract is fully deployed                   |
| `RETRACT_STATE_FAULT`      | 5     | Retract is experiencing a fault condition   |





[OrionRetractStatus packet bytes]
| Bytes | Name                            | [Enc](#Enc) | Repeat | Description                                                                    |
| ----- | ------------------------------- | :---------: | :----: | ------------------------------------------------------------------------------ |
| 0     | 1)[Cmd](#OrionRetractCmd_t)     | U8          | 1      | The current command being tracked by the retract mechanism.                    |
| 1     | 2)[State](#OrionRetractState_t) | U8          | 1      | The retraction mechanism state.                                                |
| 2...3 | 3)Pos                           | I16         | 1      | Current retract position in radians.<br>Scaled by 1000 from -32.767 to 32.767. |
| 4...5 | 4)Flags                         | U16         | 1      | GPIO status information flags.                                                 |


## <a name="ORION_PKT_STARE_START"></a>StareStart packet

This packet is transmitted by the gimbal during step-stare operations to indicate that the gimbal is staring and a step-stare image should be acquired. The position data mark the *desired* location that the gimbal is trying to stare at. The *actual* position can be computed by using the gimbal position, quaternion, and pan and tilt angles to project the camera line of sight until to the terrain model. Once the image has been acquired send a ORION_PKT_STARE_ACK packet to tell the gimbal it can move to the next stare point.

- packet identifier: `ORION_PKT_STARE_START` : 217
- minimum data length: 20
- maximum data length: 52


[StareStart packet bytes]
| Bytes   | Name                              | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                                                                                                  |
| ------- | --------------------------------- | :---------: | :----: | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3   | 1)systemTime                      | U32         | 1      | Milliseconds since system bootup. This value should be echoed in the ORION_PKT_STARE_ACK respone.                                                                                                                                                                                            |
| 4       | 2)maxStareTime                    | U8          | 1      | Seconds remaining before the stare is finished. Send ORION_PKT_STARE_ACK to terminate the stare early.<br>Scaled by 255/(2.55) from 0.0 to 2.55.                                                                                                                                             |
| 5:7     | 3)alongTrackStare                 | B1          | 1      | If set then this is the start of an along track stare, not a cross track track stare.                                                                                                                                                                                                        |
| 5:6     | 4)prevStareTerminatedByTime       | B1          | 1      | If set then the previous stare operation was terminated by timeout rather than an ORION_PKT_STARE_ACK packet.                                                                                                                                                                                |
| 5:5     | 5)stareStartedByTime              | B1          | 1      | If set then the stare operation was initiated by timeout rather than by achieving the pan and tilt angles necessary to look at the desired location. In this instance it is possible that the actual stare location is significantly different from the desired stare location.              |
| 5:4     | 6)gimbalDataValid                 | B1          | 1      | If set then the gimbal position and attitude data are valid. If not set then only the desired stare position is valid.                                                                                                                                                                       |
| 5:3     | 7)stareReset                      | B1          | 1      | If set then the stare position had to be reset due to falling behind.                                                                                                                                                                                                                        |
| 6       | 8)prevStareTime                   | U8          | 1      | Seconds of time elapsed between the previous stare start and stare end. If this number approaches stareTime the image-acquisition system is in danger of not keeping up.<br>Scaled by 255/(2.55) from 0.0 to 2.55.                                                                           |
| 7       | 9)stareLoad                       | U8          | 1      | Amount of time spent stepping and staring, as a ratio to the time between stare starts. As the frequency of stare events goes up this number will go towards 1.0, indicating the system has reached maximum utilization.<br>Scaled by 255/(1) from 0.0 to 1.0.                               |
| 8...11  | 10)posLat                         | I32         | 1      | Geodetic latitude of the desired stare point in radians, positive North.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                   |
| 12...15 | 11)posLon                         | I32         | 1      | Longitude of the desired stare point in radians, positive East.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                            |
| 16...19 | 12)posAlt                         | I32         | 1      | Altitude of the desired stare point in meters above the WGS-84 ellipsoid.<br>Scaled by 10000 from -214748.3647 to 214748.3647.                                                                                                                                                               |
| 20...23 | 13)gimbalPosLat                   | I32         | 1      | Geodetic latitude of the gimbal in radians, positive North.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                            |
| 24...27 | 14)gimbalPosLon                   | I32         | 1      | Longitude of the gimbal in radians, positive East.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                     |
| 28...31 | 15)gimbalPosAlt                   | I32         | 1      | Altitude of the gimbal in meters above the WGS-84 ellipsoid.<br>Scaled by 10000 from -214748.3647 to 214748.3647.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                        |
| 32...39 | 16)gimbalQuat                     | I16         | 4      | Gimbal quaternion describing the rotation from the gimbal mount frame to the North, East, Down navigation frame.<br>Scaled by 32767/(1) from -1.0 to 1.0.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                |
| 40...41 | 17)pan                            | I16         | 1      | Gimbal pan angle in radians from -pi to pi.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                 |
| 42...43 | 18)tilt                           | I16         | 1      | Gimbal tilt angle in radians from -pi to pi.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                                |
| 44      | 19)[rangeSource](#RangeDataSrc_t) | U8          | 1      | Target range data source.<br>This field is optional. If it is not included then the value is assumed to be RANGE_SRC_NONE.                                                                                                                                                                   |
| 45...47 | 20)slantRange                     | U24         | 1      | Slant range to target, if available (see rangeSource).<br>Scaled by 100 from 0.0 to 167772.15.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                                           |
| 48...49 | 21)alongStareAngle                | I16         | 1      | Angle in radians of the view of the path with respect to the perpendicular ideal view. Positive angles indicate looking ahead, negative are looking behind.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 50...51 | 22)crossStareAngle                | I16         | 1      | Cross track angle in radians of the stare location, which will be zero if cross stepping is not being used. Negative if staring left of the path.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.           |


## <a name="ORION_PKT_RESET_SOURCE"></a>OrionResetSource packet

Reset information for a processor in the Orion Gimbal. This packet is transmitted by each processor in the gimbal following its bootup

- packet identifier: `ORION_PKT_RESET_SOURCE` : 38
- minimum data length: 8
- maximum data length: 9


[OrionResetSource packet bytes]
| Bytes | Name                                      | [Enc](#Enc) | Repeat | Description                                                                                                                                                           |
| ----- | ----------------------------------------- | :---------: | :----: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3 | 1)Vector                                  | U32         | 1      | Interrupt vector for the last reset.                                                                                                                                  |
| 4...7 | 2)Address                                 | U32         | 1      | Program counter prior to the reset (not yet used).                                                                                                                    |
| 8     | 3)[sourceBoard](#OrionBoardEnumeration_t) | U8          | 1      | Enumeration defining which board in the gimbal generated this message.<br>This field is optional. If it is not included then the value is assumed to be BOARD_CLEVIS. |


## <a name="ORION_PKT_DEBUG_STRING"></a>DebugString packet

This packet sends a human readable text string which is typically used for debugging purposes.

- packet identifier: `ORION_PKT_DEBUG_STRING` : 177
- minimum data length: 5
- maximum data length: 132

Priority level of a debug message

[<a name="debugPriority"></a>debugPriority enumeration]
| Name              | Value | Description                                        |
| ----------------- | :---: | -------------------------------------------------- |
| `debugLevelLog`   | 0     | Logging information only, lowest priority          |
| `debugLevelInfo`  | 1     | Informational message                              |
| `debugLevelWarn`  | 2     | Warning message                                    |
| `debugLevelError` | 3     | Describes a recoverable error                      |
| `debugLevelFatal` | 4     | Describes an unrecoverable error, highest priority |





[DebugString packet bytes]
| Bytes   | Name                                 | [Enc](#Enc)                            | Repeat | Description                                                |
| ------- | ------------------------------------ | :------------------------------------: | :----: | ---------------------------------------------------------- |
| 0       | 1)[source](#OrionBoardEnumeration_t) | U8                                     | 1      | Identifier to determine which board sent the debug packet. |
| 1       | 2)[priority](#debugPriority)         | U8                                     | 1      | The importance of the debug message.                       |
| 2       | 3)reserved1                          | U8                                     | 1      |                                                            |
| 3       | 4)reserved2                          | U8                                     | 1      |                                                            |
| 4...131 | 5)description                        | Zero-terminated string up to 128 bytes         || human readable description of the debug problem.           |


## <a name="ORION_PKT_KLV_USER_DATA"></a>KlvUserData packet

This packet is used to set and query the values of user-configurable KLV tags

- packet identifier: `ORION_PKT_KLV_USER_DATA` : 179
- minimum data length: 3
- maximum data length: 130


[KlvUserData packet bytes]
| Bytes   | Name     | [Enc](#Enc) | Repeat            | Description                                                     |
| ------- | -------- | :---------: | :---------------: | --------------------------------------------------------------- |
| 0       | 1)Key    | U8          | 1                 | UAS Local Set key.                                              |
| 1       | 2)SubKey | U8          | 1                 | UAS Local Set sub-key, if applicable (e.g., security metadata). |
| 2       | 3)Length | U8          | 1                 | Data length, in bytes.                                          |
| 3...129 | 4)Value  | U8          | Length, up to 127 | Variable-length value string.                                   |


## <a name="ORION_PKT_GPS_DATA"></a>GpsData packet

Raw GPS data. This packet is normally sent by the gimbal once per second, unless the gimbal is in INS debug mode, in which case this packet is sent for every GPS report. This packet can also be sent to the gimbal in order to provide GPS information. Sending this packet to the gimbal will override any internal GPS. The ideal rate to send this packet to the gimbal is 4Hz.

Note that the position information of this packet is *not* the definitive position of the gimbal. That information comes from the INS in the gimbal and is visible in the GeolocateTelemetryCore packet.

- packet identifier: `ORION_PKT_GPS_DATA` : 209
- minimum data length: 54
- maximum data length: 69

Enumeration for source of GPS data.

[<a name="gpsSource_t"></a>gpsSource_t enumeration]
| Name              | Value | Description                          |
| ----------------- | :---: | ------------------------------------ |
| `externalSource`  | 0     | External GPS, user supplied data     |
| `ubloxSource`     | 1     | uBlox internal GPS                   |
| `mavlinkSource`   | 2     | Mavlink supplied data                |
| `nmeaSource`      | 3     | External GPS, supplied by NMEA input |
| `novatelSource`   | 4     | Novatel dual frequency GPS           |
| `autopilotSource` | 5     | Autopilot host GPS data              |
| `piccoloSource`   | 5     | Preserved for backward compatibility |
| `piksiSource`     | 6     | Piksi external GPS                   |
| `numGpsSources`   | 7     | Number of possible GPS sources       |




Enumeration for types of GPS solutions, derived from uBlox specification.

[<a name="ubloxFixType_t"></a>ubloxFixType_t enumeration]
| Name                | Value | Description                        |
| ------------------- | :---: | ---------------------------------- |
| `noFix`             | 0     | No solution                        |
| `deadReckoningOnly` | 1     | Dead reckoning only                |
| `twoDFix`           | 2     | Two dimensional solution           |
| `threeDFix`         | 3     | Three dimensional solution         |
| `gnssDeadReckoning` | 4     | GNSS and dead reckoning combined   |
| `timeOnly`          | 5     | Time only, no position or velocity |





[GpsData packet bytes]
| Bytes       | Name                      | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                                                                   |
| ----------- | ------------------------- | :---------: | :----: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0:7         | 1)multiAntHeadingValid    | B1          | 1      | Set if the GPS has a multi antenna heading solution.                                                                                                                                                                                                          |
| 0:6...0:4   | 2)reserved                | B3          | 1      | Reserved bits in the packet.                                                                                                                                                                                                                                  |
| 0:3...0:0   | 3)FixType                 | B4          | 1      | uBlox fix type.                                                                                                                                                                                                                                               |
| 1           | 4)FixState                | U8          | 1      | uBlox fix state.                                                                                                                                                                                                                                              |
| 2           | 5)TrackedSats             | U8          | 1      | The number of satellites used in the GPS solution.                                                                                                                                                                                                            |
| 3           | 6)PDOP                    | U8          | 1      | Position dilution of precision.<br>Scaled by 10 from 0.0 to 25.5.                                                                                                                                                                                             |
| 4...7       | 7)Latitude                | I32         | 1      | Geodetic latitude in radians, positive North.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                               |
| 8...11      | 8)Longitude               | I32         | 1      | Longitude in radians, positive East.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                        |
| 12...15     | 9)Altitude                | I32         | 1      | Altitude in meters above the WGS-84 ellipsoid.<br>Scaled by 10000 from -214748.3647 to 214748.3647.                                                                                                                                                           |
| 16...27     | 10)VelNED                 | I32         | 3      | Velocity in North, East, Down meters per second.<br>Scaled by 1000 from -2147483.647 to 2147483.647.                                                                                                                                                          |
| 28...31     | 11)Hacc                   | I32         | 1      | Horizontal accuracy in meters.<br>Scaled by 1000 from -2147483.647 to 2147483.647.                                                                                                                                                                            |
| 32...35     | 12)Vacc                   | I32         | 1      | Vertical accuracy in meters.<br>Scaled by 1000 from -2147483.647 to 2147483.647.                                                                                                                                                                              |
| 36...39     | 13)SpeedAcc               | I32         | 1      | Speed accuracy in meters per second.<br>Scaled by 1000 from -2147483.647 to 2147483.647.                                                                                                                                                                      |
| 40...43     | 14)HeadingAcc             | I32         | 1      | Course accuracy in radians.<br>Scaled by 5729578 from -374.8065995436313 to 374.8065995436313.                                                                                                                                                                |
| 44...47     | 15)ITOW                   | U32         | 1      | GPS time of week in milliseconds.                                                                                                                                                                                                                             |
| 48...49     | 16)Week                   | U16         | 1      | GPS week number since Jan 6 1980.                                                                                                                                                                                                                             |
| 50...51     | 17)GeoidUndulation        | I16         | 1      | Height of the mean seal level geoid with respect to the WGS-84 ellipsoid, in meters.<br>Scaled by 100 from -327.67 to 327.67.                                                                                                                                 |
| 52          | 18)[source](#gpsSource_t) | U8          | 1      | The source of the GPS data.                                                                                                                                                                                                                                   |
| 53:7...53:2 | 19)reserved               | B6          | 1      | Reserved bits in the packet.                                                                                                                                                                                                                                  |
| 53:1        | 20)detailedAccuracyValid  | B1          | 1      | Set if detailed accuracy information (posAccuracy and velAccuracy) are included in this packet.                                                                                                                                                               |
| 53:0        | 21)verticalVelocityValid  | B1          | 1      | 0 if vertical velocity data are not valid.                                                                                                                                                                                                                    |
| 54          | 22)leapSeconds            | U8          | 1      | Leap seconds to subtract from GPS time to compute UTC time.<br>This field is optional. If it is not included then the value is assumed to be 17.                                                                                                              |
| 55...56     | 23)multiAntHeading        | I16         | 1      | The heading in radians from the multi antenna solution from -pi to pi.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>Only included if multiAntHeadingValid is non-zero.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 57...62     | 24)posAccuracy            | U16         | 3      | North, East, Down position accuracy estimate in meters.<br>Scaled by 1000 from 0.0 to 65.535.<br>Only included if detailedAccuracyValid is non-zero.<br>This field is optional. If it is not included then the value is assumed to be 0.                      |
| 63...68     | 25)velAccuracy            | U16         | 3      | North, East, Down velocity accuracy estimate in meters per second.<br>Scaled by 1000 from 0.0 to 65.535.<br>Only included if detailedAccuracyValid is non-zero.<br>This field is optional. If it is not included then the value is assumed to be 0.           |


## <a name="ORION_PKT_INS_QUALITY"></a>InsQuality packet

This packet is transmitted once per second by the gimbal to communicate the quality of the inertial navigation system (INS) data. The INS is responsible for fusing the crown board IMU and GPS to determine the position, velocity, attitude, and IMU errors. The quality of the INS data directly impacts the quality of the data in the GeolocateTelemetryCore packet. This data are only sent once per second as the quality data changes slowly.

- packet identifier: `ORION_PKT_INS_QUALITY` : 211
- minimum data length: 34
- maximum data length: 79

Enumeration for the inertial navigation operational mode

[<a name="insMode_t"></a>insMode_t enumeration]
| Name              | Value | Description                                               |
| ----------------- | :---: | --------------------------------------------------------- |
| `insModeInit1`    | 0     | Awaiting first IMU measurement                            |
| `insModeInit2`    | 1     | Awaiting IMU stabilization                                |
| `insModeAHRS`     | 2     | Running without GPS inputs                                |
| `insModeRunHard`  | 3     | Running with forced GPS inputs                            |
| `insModeRun`      | 4     | Normal mode of operation                                  |
| `insModeRunTight` | 5     | Normal mode of operation with tightly coupled corrections |




Enumeration for IMU type

[<a name="imuType_t"></a>imuType_t enumeration]
| Name              | Value | Description               |
| ----------------- | :---: | ------------------------- |
| `imuTypeInternal` | 0     | Internal Trillium IMU     |
| `imuTypeSensonor` | 1     | Sensonor IMU              |
| `imuTypeDmu11`    | 2     | DMU-11 IMU                |
| `imuTypeExternal` | 3     | Other external IMU        |
| `imuTypeEpson`    | 4     | Epson IMU                 |
| `numImuTypes`     | 5     | Number of known IMU types |





[InsQuality packet bytes]
| Bytes     | Name                        | [Enc](#Enc) | Repeat                           | Description                                                                                                                                                                 |
| --------- | --------------------------- | :---------: | :------------------------------: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3     | 1)systemTime                | U32         | 1                                | Milliseconds since system bootup.                                                                                                                                           |
| 4         | 2)[gpsSource](#gpsSource_t) | U8          | 1                                | The source of GPS data.                                                                                                                                                     |
| 5:7...5:3 | 3)reserved                  | B5          | 1                                | Reserved packet space.                                                                                                                                                      |
| 5:2...5:0 | 4)[imuType](#imuType_t)     | B3          | 1                                | IMU type.                                                                                                                                                                   |
| 6         | 5)[insMode](#insMode_t)     | U8          | 1                                | The operational mode of the INS.                                                                                                                                            |
| 7:7       | 6)hasGyroBias               | B1          | 1                                | 1 if the INS is computing and reporting gyro bias states.                                                                                                                   |
| 7:6       | 7)hasGravityBias            | B1          | 1                                | 1 if the INS is computing and reporting a gravity bias state.                                                                                                               |
| 7:5       | 8)hasAccelBias              | B1          | 1                                | 1 if the INS is computing and reporting accelerometer bias states.                                                                                                          |
| 7:4       | 9)hasClockBias              | B1          | 1                                | 1 if the INS is computing and reporting clock bias and drift states.                                                                                                        |
| 7:3       | 10)hasPanTiltBias           | B1          | 1                                | 1 if the INS is computing and reporting pan and tilt bias states.                                                                                                           |
| 7:2       | 11)posRejected              | B1          | 1                                | 1 if the INS rejected the last position update because the chi-square value was too large.                                                                                  |
| 7:1       | 12)velRejected              | B1          | 1                                | 1 if the INS rejected the last velocity update because the chi-square value was too large.                                                                                  |
| 7:0       | 13)hdgRejected              | B1          | 1                                | 1 if the INS rejected the last heading update because the chi-square value was too large.                                                                                   |
| 8         | 14)gpsPeriod                | U8          | 1                                | Time in seconds between GPS measurements, 0 if no measurement.<br>Scaled by 100 from 0.0 to 2.55.                                                                           |
| 9         | 15)hdgPeriod                | U8          | 1                                | Time in seconds between heading measurements, 0 if no measurement.<br>Scaled by 100 from 0.0 to 2.55.                                                                       |
| 10...11   | 16)posChiSquare             | F16:9       | 1                                | Chi-square statistic of the last position observation.                                                                                                                      |
| 12...13   | 17)velChiSquare             | F16:9       | 1                                | Chi-square statistic of the last velocity observation.                                                                                                                      |
| 14...15   | 18)hdgChiSquare             | F16:9       | 1                                | Chi-square statistic of the last heading observation.                                                                                                                       |
| 16...21   | 19)attConfidence            | U16         | 3                                | Attitude confidence in radians as a body Roll, Pitch, Yaw vector.<br>Scaled by 10000 from 0.0 to 6.5535.                                                                    |
| 22...27   | 20)velConfidence            | U16         | 3                                | Velocity confidence in meters per second as a North, East, Down vector.<br>Scaled by 100 from 0.0 to 655.35.                                                                |
| 28...33   | 21)posConfidence            | U16         | 3                                | Position confidence in meters as a North, East, Down vector.<br>Scaled by 100 from 0.0 to 655.35.                                                                           |
| 34...39   | 22)gyroConfidence           | U16         | 3                                | Rate gyro bias confidence in radians per second as a body Roll, Pitch, Yaw vector.<br>Scaled by 100000 from 0.0 to 0.65535.<br>Only included if hasGyroBias is non-zero.    |
| 40...45   | 23)accelConfidence          | U16         | 3                                | Accelerometer bias confidence in m/s/s as a body X, Y, Z vector.<br>Scaled by 30000 from 0.0 to 2.1845.<br>Only included if hasAccelBias is non-zero.                       |
| 46...47   | 24)gravityConfidence        | U16         | 1                                | Gravity bias confidence in m/s/s.<br>Scaled by 30000 from 0.0 to 2.1845.<br>Only included if hasGravityBias is non-zero.                                                    |
| 48...49   | 25)clockBiasConfidence      | U16         | 1                                | Clock bias confidence (for tightly coupled) in meters.<br>Scaled by 10000 from 0.0 to 6.5535.<br>Only included if hasClockBias is non-zero.                                 |
| 50...51   | 26)clockDriftConfidence     | U16         | 1                                | Clock drift confience (for tightly coupled) in meters per second.<br>Scaled by 10000 from 0.0 to 6.5535.<br>Only included if hasClockBias is non-zero.                      |
| 52...57   | 27)gyroBias                 | I16         | 3                                | Rate gyro bias estimate in radians per second as a body Roll, Pitch, Yaw vector.<br>Scaled by 100000 from -0.32767 to 0.32767.<br>Only included if hasGyroBias is non-zero. |
| 58...63   | 28)accelBias                | U16         | 3                                | Accelerometer bias estimate in m/s/s as a body X, Y, Z vector.<br>Scaled by 30000 from 0.0 to 2.1845.<br>Only included if hasAccelBias is non-zero.                         |
| 64...65   | 29)gravityBias              | U16         | 1                                | Gravity bias estimate in m/s/s.<br>Scaled by 30000 from 0.0 to 2.1845.<br>Only included if hasGravityBias is non-zero.                                                      |
| 66...67   | 30)clockBias                | I16         | 1                                | Clock bias estimate (for tightly coupled) in meters.<br>Scaled by 100 from -327.67 to 327.67.<br>Only included if hasClockBias is non-zero.                                 |
| 68...69   | 31)clockDrift               | I16         | 1                                | Clock drift estimate (for tightly coupled) in meters per second.<br>Scaled by 1000 from -32.767 to 32.767.<br>Only included if hasClockBias is non-zero.                    |
| 70        | 32)numTightSatPosUpdates    | U8          | 1                                | Number of satellites that contributed to tightly coupled position updates.<br>Only included if hasClockBias is non-zero.                                                    |
| 71        | 33)numTightSatVelUpdates    | U8          | 1                                | Number of satellites that contributed to tightly coupled velocity updates.<br>Only included if hasClockBias is non-zero.                                                    |
| 72        | 34)numTightPosUpdates       | U8          | 1                                | Number of signals that contributed to tightly coupled position updates.<br>Only included if hasClockBias is non-zero.                                                       |
| 73        | 35)numTightVelUpdates       | U8          | 1                                | Number of signals that contributed to tightly coupled velocity updates.<br>Only included if hasClockBias is non-zero.                                                       |
| 74...77   | 36)panTiltBias              | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Pan and tilt boresight biases.<br>Scaled by 32767/(10&times;&pi/180) from -0.1745329251994329 to 0.1745329251994329.<br>Only included if hasPanTiltBias is non-zero.        |
| 78        | 37)panTiltChiSquare         | U8          | 1                                | Pan and tilt chi-square statistic.<br>Scaled by 255/(10) from 0.0 to 10.0.<br>Only included if hasPanTiltBias is non-zero.                                                  |

<div class="page-break"></div>


---


# Packets for gimbal control

These packets are sent by the user to control the gimbal.


## <a name="ORION_PKT_CMD"></a>OrionCmd packet

Basic commands for control of gimbal pan and tilt. Send this packet to the gimbal to effect basic pan and tilt control. The gimbal will echo the packet as confirmation of receiving it. Note that this packet cannot be used to command the Geopoint or Path modes; those modes need to be commanded through their respective packets. If the gimbal is in Geopoint or Path mode receiving this packet will cancel those modes.

- packet identifier: `ORION_PKT_CMD` : 1
- data length: 7


[OrionCmd packet bytes]
| Bytes | Name                     | [Enc](#Enc) | Repeat                           | Description                                                                                                                                                          |
| ----- | ------------------------ | :---------: | :------------------------------: | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...6 | 1)Cmd                                 || 1                                | Pan/tilt command                                                                                                                                                     |
| 0...3 | 1.1)Target               | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Pan and tilt commands in radians or radians per second. Interpretation depends on Mode.<br>Scaled by 1000.0 from -32.767 to 32.767.                                  |
| 4     | 1.2)[Mode](#OrionMode_t) | U8          | 1                                | Operational mode of the gimbal. Note that not all modes can be directly commanded.                                                                                   |
| 5     | 1.3)Stabilized           | U8          | 1                                | Non-zero to enable inertial stabilization. If zero then Target is with respect to the gimbal mount, otherwise Target is with respect to the inertial or image space. |
| 6     | 1.4)ImpulseTime          | U8          | 1                                | Amount of time in seconds for changing the gimbal position.<br>Scaled by 10.0 from 0.0 to 25.5.                                                                      |


## <a name="ORION_PKT_STARTUP_CMD"></a>OrionStartupCmd packet

Command to run upon motor initialization

- packet identifier: `ORION_PKT_STARTUP_CMD` : 7
- data length: 7


[OrionStartupCmd packet bytes]
| Bytes | Name                     | [Enc](#Enc) | Repeat                           | Description                                                                                                                                                          |
| ----- | ------------------------ | :---------: | :------------------------------: | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...6 | 1)Cmd                                 || 1                                | Pan/tilt command. Defaults to position mode, pan/tilt of 0                                                                                                           |
| 0...3 | 1.1)Target               | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Pan and tilt commands in radians or radians per second. Interpretation depends on Mode.<br>Scaled by 1000.0 from -32.767 to 32.767.                                  |
| 4     | 1.2)[Mode](#OrionMode_t) | U8          | 1                                | Operational mode of the gimbal. Note that not all modes can be directly commanded.                                                                                   |
| 5     | 1.3)Stabilized           | U8          | 1                                | Non-zero to enable inertial stabilization. If zero then Target is with respect to the gimbal mount, otherwise Target is with respect to the inertial or image space. |
| 6     | 1.4)ImpulseTime          | U8          | 1                                | Amount of time in seconds for changing the gimbal position.<br>Scaled by 10.0 from 0.0 to 25.5.                                                                      |


## <a name="ORION_PKT_GEOPOINT_CMD"></a>GeopointCmd packet

Send this packet to command the gimbal to go to Geopoint mode. In geopoint the gimbal looks at a target as given by a latitude, longitude, altitude position. The geopoint command can include a target velocity, which will cause the gimbal to smoothly move its stare point as it propagates the target position. Geopoint mode is canceled by sending a normal OrionCmd packet.

- packet identifier: `ORION_PKT_GEOPOINT_CMD` : 213
- minimum data length: 18
- maximum data length: 21

Options controlling how the geopoint is interpreted

[<a name="geopointOptions"></a>geopointOptions enumeration]
| Name                   | Value | Description                                                                                                                                                                                                       |
| ---------------------- | :---: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `geopointNone`         | 0     | No geopoint options set.                                                                                                                                                                                          |
| `geopointStare`        | 1     | If set this geopoint will issue a stare packet when it is pointing at the target location. This option is ignored if joystickRange is non-zero.                                                                   |
| `geopointClosure`      | 2     | If set this geopoint command will cause the gimbal to enter closure mode, in which it will drive at maximum performance to achieve the desired pointing. This option is ignored if the joystickRange is non-zero. |
| `geopointStareClosure` | 3     | Logical OR of geopointStare and geopointClosure.                                                                                                                                                                  |





[GeopointCmd packet bytes]
| Bytes   | Name                          | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| ------- | ----------------------------- | :---------: | :----: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 0...3   | 1)targetLat                   | I32         | 1      | Geodetic latitude of the target in radians, positive North.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| 4...7   | 2)targetLon                   | I32         | 1      | Longitude of the target in radians, positive East.<br>Scaled by 180&times;10000000/&pi from -3.748066027288564 to 3.748066027288564.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| 8...11  | 3)targetAlt                   | I32         | 1      | Altitude of the target in meters above the WGS-84 ellipsoid.<br>Scaled by 10000 from -214748.3647 to 214748.3647.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                |
| 12...17 | 4)targetVelNED                | I16         | 3      | Velocity of the target in North, East, Down meters per second.<br>Scaled by 100 from -327.67 to 327.67.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |
| 18...19 | 5)joystickRange               | U16         | 1      | Estimated range to target in meters. If non-zero *and* if geopoint is already running the position information in this packet is ignored and only the velocity and range information is used to propagate the geopoint command position. This is used for cases where a joystick is being used to drive the geopoint. If the gimbal has its own range estimate then a non-zero joystickRange will be replaced with the gimbal's internal range estimate. If you want the gimbal use the joystick mode but you cannot compute the range, use a range less than 10 meters and the gimbal will not do the range adjustment.<br>Scaled by 1 from 0.0 to 65535.0.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 20      | 6)[options](#geopointOptions) | U8          | 1      | Options for the geopoint packet.<br>This field is optional. If it is not included then the value is assumed to be geopointNone.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |


## <a name="ORION_PKT_PATH"></a>OrionPath packet

The path packet contains a series of waypoints that define a path in 3D space. Sending this packet enables the path track logic, which will cause the gimbal to look at the nearest point on the path; or look directly downward. Path mode is canceled by sending a normal OrionCmd packet.

- packet identifier: `ORION_PKT_PATH` : 215
- minimum data length: 2
- maximum data length: 140

Maximum number of points in a path.

[<a name="maxPathPoints"></a>maxPathPoints enumeration]
| Name              | Value | Description                        |
| ----------------- | :---: | ---------------------------------- |
| `MAX_PATH_POINTS` | 15    | Maximum number of points in a path |





[OrionPath packet bytes]
| Bytes     | Name                  | [Enc](#Enc) | Repeat                                             | Description                                                                                                                                                                                                                                                                                            |
| --------- | --------------------- | :---------: | :------------------------------------------------: | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| 0         | 1)numPoints           | U8          | 1                                                  | Number of points in the path.                                                                                                                                                                                                                                                                          |
| 1:7       | 2)pointDown           | B1          | 1                                                  | If numPoints is zero, and this is set, then the gimbal will point downwards.                                                                                                                                                                                                                           |
| 1:6...1:4 | 3)reserved            | B3          | 1                                                  | Reserved for future expansion.                                                                                                                                                                                                                                                                         |
| 1:3...1:0 | 4)numCrossTrackSteps  | B4          | 1                                                  | Number of steps for across track stepping. Must be more than 1 to enable cross track stepping.                                                                                                                                                                                                         |
| 2...136   | 5)Point                            || numPoints, up to [MAX_PATH_POINTS](#maxPathPoints) | List of path points                                                                                                                                                                                                                                                                                    |
| 2...10    | 5.1)posEcef           | I24         | 3                                                  | ECEF position in meters.<br>Scaled by 1 from -8388607.0 to 8388607.0.                                                                                                                                                                                                                                  |
| 137...138 | 6)alongTrackStepAngle | U16         | 1                                                  | Angle in radians for step-stare functionality along track. Set to zero for no step-stare. If doing downward pointing the gimbal must be receiving ORION_PKT_RANGE_DATA.<br>Scaled by 65535/(&pi) from 0.0 to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 139       | 7)crossTrackStepRatio | U8          | 1                                                  | Ratio of cross track step angle to along track step angle. Set to zero for no cross track stepping.<br>Scaled by 255/(2.55) from 0.0 to 2.55.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                      |


## <a name="ORION_PKT_STARE_ACK"></a>StareAck packet

This packet tells the gimbal that the step-stare image has been acquired and the gimbal can move to the next step stare point.

- packet identifier: `ORION_PKT_STARE_ACK` : 218
- data length: 4


[StareAck packet bytes]
| Bytes | Name             | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                   |
| ----- | ---------------- | :---------: | :----: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3 | 1)echoSystemTime | U32         | 1      | Milliseconds since system bootup received from ORION_PKT_STARE_START. If this value matches the value sent in ORION_PKT_STARE_START then the gimbal will terminate the stare operation early. |


## <a name="ORION_PKT_CAMERA_SWITCH"></a>OrionCameraSwitch packet

Command the gimbal to swtich cameras.

- packet identifier: `ORION_PKT_CAMERA_SWITCH` : 96
- data length: 1


[OrionCameraSwitch packet bytes]
| Bytes | Name    | [Enc](#Enc) | Repeat | Description                                            |
| ----- | ------- | :---------: | :----: | ------------------------------------------------------ |
| 0     | 1)Index | U8          | 1      | The index of the camera to switch to, counting from 0. |


## <a name="ORION_PKT_CAMERA_STATE"></a>OrionCameraState packet

Command a camera in the gimbal. Note this packet will eventually be deprecated.

- packet identifier: `ORION_PKT_CAMERA_STATE` : 97
- minimum data length: 2
- maximum data length: 5


[OrionCameraState packet bytes]
| Bytes | Name    | [Enc](#Enc) | Repeat | Description                                                                                                                                             |
| ----- | ------- | :---------: | :----: | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...1 | 1)Zoom  | I16         | 1      | Digital zoom ratio.<br>Scaled by 100 from -327.67 to 327.67.                                                                                            |
| 2...3 | 2)Focus | I16         | 1      | Focus command/position.<br>Scaled by 10000 from -3.2767 to 3.2767.<br>This field is optional. If it is not included then the value is assumed to be -1. |
| 4     | 3)Index | U8          | 1      | The index of the camera to command.<br>This field is optional. If it is not included then the value is assumed to be 0.                                 |


## <a name="ORION_PKT_FLIR_SETTINGS"></a>OrionFlirSettings packet

FLIR-specific camera settings.

- packet identifier: `ORION_PKT_FLIR_SETTINGS` : 103
- minimum data length: 5
- maximum data length: 7

FLIR NUC type enumeration

[<a name="FlirNucType_t"></a>FlirNucType_t enumeration]
| Name                     | Value | Description                                     |
| ------------------------ | :---: | ----------------------------------------------- |
| `NUC_TYPE_MANUAL`        | 0     | Typical operation mode, manual FFC only         |
| `NUC_TYPE_SSN_NO_MOTION` | 1     | Silent shutterless NUC (SSN), no motion setting |
| `NUC_TYPE_SSN_LOW`       | 2     | Silent shutterless NUC (SSN), low setting       |
| `NUC_TYPE_SSN_MEDIUM`    | 3     | Silent shutterless NUC (SSN), medium setting    |
| `NUC_TYPE_SSN_HIGH`      | 4     | Silent shutterless NUC (SSN), high setting      |





[OrionFlirSettings packet bytes]
| Bytes     | Name                        | [Enc](#Enc) | Repeat | Description                                                                                                                                                    |
| --------- | --------------------------- | :---------: | :----: | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0         | 1)Index                     | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected FLIR camera.                                                       |
| 1:7       | 2)Reserved                  | B1          | 1      | Reserved bits in the packet.<br>Data are given constant value on encode 0.                                                                                     |
| 1:6...1:4 | 3)Palette                   | B3          | 1      |                                                                                                                                                                |
| 1:3...1:1 | 4)[NucType](#FlirNucType_t) | B3          | 1      | NUC type setting.                                                                                                                                              |
| 1:0       | 5)BlackHot                  | B1          | 1      | Image polarity setting - set to 0 for white hot or 1 for black hot. This field is ignored if Palette is non-zero.                                              |
| 2         | 6)MaxAgcGain                | U8          | 1      | Maximum AGC output gain.                                                                                                                                       |
| 3         | 7)AceLevel                  | I8          | 1      | Active Contrast Enhancement (ACE) level.                                                                                                                       |
| 4         | 8)DdeThreshold              | U8          | 1      | DDE spatial threshold.                                                                                                                                         |
| 5         | 9)AgcMidpoint               | U8          | 1      | AGC output midpoint, from 0 to 255.<br>This field is optional. If it is not included then the value is assumed to be 128.                                      |
| 6         | 10)IntegrationTime          | U8          | 1      | Integration time, in milliseconds.<br>Scaled by 255/29.0 from 1.0 to 30.0.<br>This field is optional. If it is not included then the value is assumed to be 8. |


## <a name="ORION_PKT_APTINA_SETTINGS"></a>OrionAptinaSettings packet

Aptina-specific camera settings.

- packet identifier: `ORION_PKT_APTINA_SETTINGS` : 104
- minimum data length: 13
- maximum data length: 15


[OrionAptinaSettings packet bytes]
| Bytes | Name           | [Enc](#Enc) | Repeat | Description                                                                                                                       |
| ----- | -------------- | :---------: | :----: | --------------------------------------------------------------------------------------------------------------------------------- |
| 0     | 1)Index        | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected Aptina camera.                        |
| 1...2 | 2)MinExposure  | U16         | 1      | Minimum exposure time in microseconds.<br>Scaled by 65535/32.95 from 0.05 to 33.0.                                                |
| 3...4 | 3)MaxExposure  | U16         | 1      | Maximum exposure time in milliseconds.<br>Scaled by 65535/32.95 from 0.05 to 33.0.                                                |
| 5...6 | 4)MinGain      | U16         | 1      | Minimum sensor luminance gain.<br>Scaled by 65535/31.0 from 1.0 to 32.0.                                                          |
| 7...8 | 5)MaxGain      | U16         | 1      | Maximum sensor luminance gain.<br>Scaled by 65535/31.0 from 1.0 to 32.0.                                                          |
| 9     | 6)Brightness   | I8          | 1      | Brightness adjustment.                                                                                                            |
| 10    | 7)Contrast     | I8          | 1      | Contrast adjustment.<br>Scaled by 127/(1) from -1.0 to 1.0.                                                                       |
| 11    | 8)Saturation   | I8          | 1      | Saturation adjustment.<br>Scaled by 127/(1) from -1.0 to 1.0.                                                                     |
| 12    | 9)Sharpness    | U8          | 1      | Sharpening filter strength.<br>Scaled by 255/(5) from 0.0 to 5.0.                                                                 |
| 13:7  | 10)DebugEnable | B1          | 1      | Set to 1 to enable AGC debugging information.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 14    | 11)Hue         | I8          | 1      | Hue adjustment.<br>This field is optional. If it is not included then the value is assumed to be 0.                               |


## <a name="ORION_PKT_ZAFIRO_SETTINGS"></a>OrionZafiroSettings packet

Zafiro-specific camera settings.

- packet identifier: `ORION_PKT_ZAFIRO_SETTINGS` : 105
- minimum data length: 6
- maximum data length: 7


[OrionZafiroSettings packet bytes]
| Bytes | Name                 | [Enc](#Enc) | Repeat | Description                                                                                                                          |
| ----- | -------------------- | :---------: | :----: | ------------------------------------------------------------------------------------------------------------------------------------ |
| 0     | 1)Index              | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected Zafiro camera.                           |
| 1     | 2)BlackHot           | U8          | 1      | Image polarity setting - set to 0 for white hot or 1 for black hot.                                                                  |
| 2     | 3)IntegrationTime    | U8          | 1      | Integration time, in milliseconds.<br>Scaled by 255/29.0 from 1.0 to 30.0.                                                           |
| 3     | 4)LapEnabled         | U8          | 1      | Local Area Processing (LAP) enable/disable flag. Set to zero to disable, or non-zero to enable.                                      |
| 4     | 5)LapGain            | U8          | 1      | LAP Sharpening gain.<br>Scaled by 255/3.5 from 0.5 to 4.0.                                                                           |
| 5     | 6)AgcSaturation      | U8          | 1      | AGC histogram saturation.<br>Scaled by 255/(0.1) from 0.0 to 0.1.                                                                    |
| 6     | 7)UseNucTablePresets | U8          | 1      | Set to 1 to use NUC table presets, default is 0.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_HITACHI_SETTINGS"></a>OrionHitachiSettings packet

Hitachi-specific camera settings.

- packet identifier: `ORION_PKT_HITACHI_SETTINGS` : 106
- minimum data length: 5
- maximum data length: 6


[OrionHitachiSettings packet bytes]
| Bytes | Name              | [Enc](#Enc) | Repeat | Description                                                                                                                                           |
| ----- | ----------------- | :---------: | :----: | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | 1)Index           | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected Hitachi camera.                                           |
| 1...2 | 2)IntegrationTime | I16         | 1      | Integration time in microseconds, or negative to disable shutter-priority mode.                                                                       |
| 3...4 | 3)Aperture        | I16         | 1      | Aperture f-number, or negative to disable aperture-priority mode.<br>Scaled by 10 from -3276.7 to 3276.7.                                             |
| 5     | 4)Sharpness       | U8          | 1      | Sharpening filter strength.<br>Scaled by 255/(5) from 0.0 to 5.0.<br>This field is optional. If it is not included then the value is assumed to be 1. |


## <a name="ORION_PKT_SONY_SETTINGS"></a>OrionSonySettings packet

Sony-specific camera settings.

- packet identifier: `ORION_PKT_SONY_SETTINGS` : 108
- data length: 12


[OrionSonySettings packet bytes]
| Bytes | Name          | [Enc](#Enc) | Repeat | Description                                                                                              |
| ----- | ------------- | :---------: | :----: | -------------------------------------------------------------------------------------------------------- |
| 0     | 1)Index       | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected Sony camera. |
| 1...2 | 2)MinExposure | U16         | 1      | Minimum exposure time in milliseconds.<br>Scaled by 65535/32.95 from 0.05 to 33.0.                       |
| 3...4 | 3)MaxExposure | U16         | 1      | Maximum exposure time in milliseconds.<br>Scaled by 65535/32.95 from 0.05 to 33.0.                       |
| 5     | 4)MinGain     | U8          | 1      | Minimum sensor luminance gain, in dB.                                                                    |
| 6     | 5)MaxGain     | U8          | 1      | Maximum sensor luminance gain, in dB.                                                                    |
| 7     | 6)Brightness  | U8          | 1      | Brightness target, from 0 to 255.                                                                        |
| 8     | 7)Contrast    | I8          | 1      | Contrast adjustment.<br>Scaled by 127/(1) from -1.0 to 1.0.                                              |
| 9     | 8)Saturation  | I8          | 1      | Saturation adjustment.<br>Scaled by 127/(1) from -1.0 to 1.0.                                            |
| 10    | 9)Sharpness   | U8          | 1      | Sharpening filter strength.<br>Scaled by 255/(5) from 0.0 to 5.0.                                        |
| 11    | 10)FrameRate  | U8          | 1      | Camera frame rate, in Hertz.<br>Scaled by 255/(30) from 0.0 to 30.0.                                     |


## <a name="ORION_PKT_KTNC_SETTINGS"></a>OrionKtncSettings packet

KTnC-specific camera settings.

- packet identifier: `ORION_PKT_KTNC_SETTINGS` : 109
- minimum data length: 5
- maximum data length: 11


[OrionKtncSettings packet bytes]
| Bytes | Name               | [Enc](#Enc) | Repeat | Description                                                                                                                                                    |
| ----- | ------------------ | :---------: | :----: | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | 1)Index            | U8          | 1      | Index of the camera that these settings should apply to. Use 0xFF to apply to any connected KTnC camera.                                                       |
| 1...2 | 2)IntegrationTime  | I16         | 1      | Integration time in microseconds, or negative to disable shutter-priority mode.                                                                                |
| 3...4 | 3)Aperture         | I16         | 1      | Aperture f-number, or negative to disable aperture-priority mode.<br>Scaled by 10 from -3276.7 to 3276.7.                                                      |
| 5     | 4)Sharpness        | U8          | 1      | Sharpening adjustment.<br>Scaled by 255/(16) from 0.0 to 16.0.<br>This field is optional. If it is not included then the value is assumed to be 8.             |
| 6     | 5)VerticalFlip     | U8          | 1      | Set to non-zero to vertically flip the video.<br>This field is optional. If it is not included then the value is assumed to be 0.                              |
| 7     | 6)ExposureComp     | U8          | 1      | Exposure compenstation adjustment.<br>Scaled by 255/(14) from 0.0 to 14.0.<br>This field is optional. If it is not included then the value is assumed to be 7. |
| 8     | 7)Contrast         | U8          | 1      | Contrast adjustment.<br>Scaled by 255/(32) from 0.0 to 32.0.<br>This field is optional. If it is not included then the value is assumed to be 8.               |
| 9     | 8)Saturation       | U8          | 1      | Saturation adjustment.<br>Scaled by 255/(32) from 0.0 to 32.0.<br>This field is optional. If it is not included then the value is assumed to be 14.            |
| 10:7  | 9)NightMode        | B1          | 1      | Night mode enable.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                         |
| 10:6  | 10)ResetToDefaults | B1          | 1      | Reset to defaults.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                         |


## OrionRetractCmd_t enumeration

The retraction mechanism commands.

[<a name="OrionRetractCmd_t"></a>OrionRetractCmd_t enumeration]
| Name                  | Value | Description                             |
| --------------------- | :---: | --------------------------------------- |
| `RETRACT_CMD_DISABLE` | 0     | Disable the retract motor               |
| `RETRACT_CMD_DEPLOY`  | 1     | Move the gimbal to a deployed position  |
| `RETRACT_CMD_RETRACT` | 2     | Move the gimbal to a retracted position |



## <a name="ORION_PKT_RETRACT_CMD"></a>OrionRetractCommand packet

Retract or deploy command for a gimbal retraction mechanism.

- packet identifier: `ORION_PKT_RETRACT_CMD` : 160
- data length: 2


[OrionRetractCommand packet bytes]
| Bytes | Name                        | [Enc](#Enc) | Repeat | Description                                                            |
| ----- | --------------------------- | :---------: | :----: | ---------------------------------------------------------------------- |
| 0     | 1)[Cmd](#OrionRetractCmd_t) | U8          | 1      | The retraction mechanism command.                                      |
| 1     | 2)Reserved                  | U8          | 1      | Reserved for future use.<br>Data are given constant value on encode 0. |


## <a name="ORION_PKT_USER_DATA"></a>OrionUserData packet

This packet can be used to transfer user-defined data payloads between communication ports on the gimbal.

- packet identifier: `ORION_PKT_USER_DATA` : 178
- minimum data length: 6
- maximum data length: 134


[OrionUserData packet bytes]
| Bytes   | Name                      | [Enc](#Enc) | Repeat          | Description                                                            |
| ------- | ------------------------- | :---------: | :-------------: | ---------------------------------------------------------------------- |
| 0       | 1)[port](#UserDataPort_t) | U8          | 1               | Destination crown board communication port identifier.                 |
| 1       | 2)size                    | U8          | 1               | User data array size in bytes.                                         |
| 2...5   | 3)id                      | U32         | 1               | User-defined unique identifier or sequence number for this data block. |
| 6...133 | 4)data                    | U8          | size, up to 128 | User data byte array.                                                  |


## <a name="ORION_PKT_LASER_CMD"></a>OrionLaserCommand packet

Command to fire the laser (if installed).

- packet identifier: `ORION_PKT_LASER_CMD` : 3
- minimum data length: 11
- maximum data length: 29


[OrionLaserCommand packet bytes]
| Bytes   | Name            | [Enc](#Enc)                           | Repeat | Description                                                                                                                                                                               |
| ------- | --------------- | :-----------------------------------: | :----: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...1   | 1)Reserved      | U16                                   | 1      | Deprecated parameter - set to zero.<br>Data are given constant value on encode 0.                                                                                                         |
| 2       | 2)Index         | U8                                    | 1      | Index of the laser to apply this command to.                                                                                                                                              |
| 3...18  | 3)Password      | Zero-terminated string up to 16 bytes         || Password, if required by the laser. If a password is required, it must be provided in order to change any aspect of the laser's state (e.g., enable/disable, arm/disarm, fire/deactivate. |
| 19      | 4)Enable        | U8                                    | 1      | Set to zero to disable the laser, or non-zero to enable it.                                                                                                                               |
| 20      | 5)Arm           | U8                                    | 1      | Set to zero to disarm the laser, or non-zero to arm.                                                                                                                                      |
| 21      | 6)Fire          | U8                                    | 1      | Set to zero to deactivate the laser, or non-zero to activate it.                                                                                                                          |
| 22...23 | 7)OnTime        | U16                                   | 1      | Periodic laser pulse on time, in milliseconds, or zero to fire continuously.                                                                                                              |
| 24...25 | 8)OffTime       | U16                                   | 1      | Periodic laser pulse off time, in milliseconds, or zero to fire continuously.                                                                                                             |
| 26...27 | 9)PrfCode       | U16                                   | 1      | Laser PRF code, if applicable.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                        |
| 28      | 10)SafetyBypass | U8                                    | 1      | Set to a non-zero value to bypass all laser safety checks.<br>This field is optional. If it is not included then the value is assumed to be 0.                                            |


## <a name="ORION_PKT_LASER_STATES"></a>OrionLaserStates packet

Information about the current state of lasers installed in the gimbal.

- packet identifier: `ORION_PKT_LASER_STATES` : 6
- minimum data length: 1
- maximum data length: 13


[OrionLaserStates packet bytes]
| Bytes     | Name                          | [Enc](#Enc) | Repeat                                           | Description                                                                                    |
| --------- | ----------------------------- | :---------: | :----------------------------------------------: | ---------------------------------------------------------------------------------------------- |
| 0         | 1)NumLasers                   | U8          | 1                                                | Number of installed lasers.                                                                    |
| 1...12    | 2)State                                    || NumLasers, up to [MAX_LASERS](#OrionMaxLasers_t) |                                                                                                |
| 1         | 2.1)[Type](#OrionLaserType_t) | U8          | 1                                                | The type of this laser.                                                                        |
| 2:7       | 2.2)Enabled                   | B1          | 1                                                | Set to 1 if the laser is enabled, otherwise 0.                                                 |
| 2:6       | 2.3)Armed                     | B1          | 1                                                | Set to 1 if the laser is armed, otherwise 0.                                                   |
| 2:5       | 2.4)Active                    | B1          | 1                                                | Set to 1 if the laser is active, otherwise 0.                                                  |
| 2:4       | 2.5)GroundSpeedLock           | B1          | 1                                                | Set to 1 if the laser is locked due to low ground speed, otherwise 0.                          |
| 2:3       | 2.6)AltitudeLock              | B1          | 1                                                | Set to 1 if the laser is locked due to low GPS altitude, otherwise 0.                          |
| 2:2       | 2.7)PasswordLock              | B1          | 1                                                | Set to 1 if the laser is locked by password, otherwise 0.                                      |
| 2:1       | 2.8)ApCommLock                | B1          | 1                                                | Set to 1 if the laser is locked due to lost autopilot comms, otherwise 0.                      |
| 2:0       | 2.9)ApFlyingLock              | B1          | 1                                                | Set to 1 if the laser is locked because the autopilot is not in a 'flying' state, otherwise 0. |
| 3:7       | 2.10)BypassEnabled            | B1          | 1                                                | Set to 1 if the current command is bypassing the safety interlocks, otherwise 0.               |
| 3:6       | 2.11)PitchAngleLock           | B1          | 1                                                | Set to 1 if the laser is locked due to a positive pitch angle, otherwise 0.                    |
| 3:5...4:0 | 2.12)Reserved                 | B14         | 1                                                | Reserved for future use.<br>Data are given constant value on encode 0.                         |
| 5...6     | 2.13)WaitTimer                | U16         | 1                                                | Time remaining, in milliseconds, before the laser will accept another command.                 |

<div class="page-break"></div>


---


# Packets used for gimbal configuration

These packets are sent by the user to configure the gimbal. To request a configuration packet send the packet with zero length. Configuration information is stored onboard the gimbal and does not need to be re-sent if the gimbal is restarted.


## UserDataPort_t enumeration

Destination port identifiers

[<a name="UserDataPort_t"></a>UserDataPort_t enumeration]
| Name                      | Value | Description                                                |
| ------------------------- | :---: | ---------------------------------------------------------- |
| `USER_DATA_PORT_ETHERNET` | 0     | Payload Ethernet connection                                |
| `USER_DATA_PORT_PRIMARY`  | 2     | Primary front-panel RS-232 serial port                     |
| `USER_DATA_PORT_FP2`      | 3     | Front panel FP1 RS-232 serial port                         |
| `USER_DATA_PORT_FP1`      | 4     | Front panel FP2 RS-232 serial port                         |
| `USER_DATA_PORT_FP3`      | 8     | Front panel FP3 TTL serial port                            |
| `USER_DATA_PORT_FP4`      | 9     | Front panel FP4 TTL serial port                            |
| `USER_DATA_PORT_CURRENT`  | 255   | The port referenced is whatever port delivered this packet |



## <a name="ORION_PKT_UART_CONFIG"></a>OrionUartConfig packet

This packet is used to change or request the baud rate and protocol of a UART on the crown board. Send this packet with zero bytes to request the configuration of the port that receives the request, or with 1 byte to request the configuration of a specific port. Not all port configuration options are available, the ethernet port must run the PROTOCOL_ORION protocol at either 115200 or 230400 bits per second. In general users should not change the ethernet port setup. In addition the PROTOCOL_UBLOX_GPS is not supported on any user configurable port, it is included in the protocol list only for completeness.

- packet identifier: `ORION_PKT_UART_CONFIG` : 2
- minimum data length: 5
- maximum data length: 6

Protocols understood by the Orion UART configuration

[<a name="OrionProtocols_t"></a>OrionProtocols_t enumeration]
| Name                       | Value | Description                                                         |
| -------------------------- | :---: | ------------------------------------------------------------------- |
| `PROTOCOL_NO_CHANGE`       | 0     | Do not change the protocol                                          |
| `PROTOCOL_NONE`            | 1     | Disable the port                                                    |
| `PROTOCOL_DEFAULT`         | 2     | Use the default protocol. This is port dependent                    |
| `PROTOCOL_ORION`           | 3     | Orion packet communications protocol                                |
| `PROTOCOL_CLEVIS`          | 4     | Orion packet communications protocol specific to the clevis         |
| `PROTOCOL_CLEVIS_RAW`      | 5     | Clevis raw communications mode                                      |
| `PROTOCOL_NMEA_GPS`        | 6     | GPS input over NMEA                                                 |
| `PROTOCOL_UBLOX_GPS`       | 7     | uBox GPS                                                            |
| `PROTOCOL_NOVATEL_GPS`     | 8     | Novatel GPS                                                         |
| `PROTOCOL_MAVLINK_GPS`     | 9     | Mavlink GPS                                                         |
| `PROTOCOL_UM7_MAG`         | 10    | CH Robotics UM7 magnetometer                                        |
| `PROTOCOL_SIGHTLINE_VIDEO` | 11    | Sightline video processing                                          |
| `PROTOCOL_SENSONOR_IMU`    | 12    | Sensonor IMU                                                        |
| `PROTOCOL_DMU11_IMU`       | 13    | Silicon sensing IMU                                                 |
| `PROTOCOL_DISCOVERY`       | 14    | Discovery protocol that decides between novatel, UM7, and Sightline |
| `PROTOCOL_AIRROBOT`        | 15    | Air robot protocol                                                  |
| `PROTOCOL_PROPRIETARY1`    | 16    | Proprietary protocol                                                |
| `PROTOCOL_PROPRIETARY2`    | 17    | Proprietary protocol                                                |
| `PROTOCOL_FLIR`            | 18    | FLIR pass-through protocol                                          |
| `PROTOCOL_EPSON_IMU`       | 19    | EPSON IMU protocol                                                  |
| `PROTOCOL_PIKSI`           | 20    | Swift Piksi GPS                                                     |
| `PROTOCOL_WEPILOT`         | 21    | WePilot AP                                                          |
| `NUM_PROTOCOL`             | 22    |                                                                     |





[OrionUartConfig packet bytes]
| Bytes | Name                            | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                                               |
| ----- | ------------------------------- | :---------: | :----: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0     | 1)[port](#UserDataPort_t)       | U8          | 1      | The port whose configuration should be changed or requested, use USER_DATA_PORT_CURRENT to indicate the port which received this packet.                                                                                                  |
| 1...3 | 2)baud                          | U24         | 1      | Baud rate bits per second. Use 0 to select the default baud rate for this port and protocol.                                                                                                                                              |
| 4     | 3)[protocol](#OrionProtocols_t) | U8          | 1      | The protocol to use for this port. If PROTOCOL_NO_CHANGE or PROTOCOL_DEFAULT are used the reply to this packet will contain the actual protocol in use.                                                                                   |
| 5     | 4)temporary                     | U8          | 1      | A value of 0 indicates that the settings should persist across power cycles. Non-zero values will cause the new setting to be lost after power cycle.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_LIMITS"></a>OrionLimitsData packet

Gimbal limits of operation. This includes the max and min pan and tilt angles, as well as the angular rate and the maximum current consumption.

- packet identifier: `ORION_PKT_LIMITS` : 34
- minimum data length: 32
- maximum data length: 34


[OrionLimitsData packet bytes]
| Bytes   | Name          | [Enc](#Enc) | Repeat                           | Description                                                                                                                                                                             |
| ------- | ------------- | :---------: | :------------------------------: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3   | 1)MinPos      | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Minimum position in radians.<br>Scaled by 1000.0 from -32.767 to 32.767.                                                                                                                |
| 4...7   | 2)MaxPos      | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum position in radians.<br>Scaled by 1000.0 from -32.767 to 32.767.                                                                                                                |
| 8...11  | 3)MaxVel      | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum velocity in radians per second.<br>Scaled by 1000.0 from 0.0 to 65.535.                                                                                                         |
| 12...15 | 4)MaxAccel    | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum acceleration in radians per second per second.<br>Scaled by 1.0 from 0.0 to 65535.0.                                                                                            |
| 16...19 | 5)ContCur     | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum continuous current in Amps.<br>Scaled by 1000.0 from 0.0 to 65.535.                                                                                                             |
| 20...23 | 6)PeakCur     | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum peak current in Amps.<br>Scaled by 1000.0 from 0.0 to 65.535.                                                                                                                   |
| 24...27 | 7)PeakCurTime | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Peak current time in seconds.<br>Scaled by 1000.0 from 0.0 to 65.535.                                                                                                                   |
| 28...31 | 8)InitCur     | U16         | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Initialization current in Amps.<br>Scaled by 1000.0 from 0.0 to 65.535.                                                                                                                 |
| 32...33 | 9)MaxPower    | U8          | [NUM_GIMBAL_AXES](#GimbalAxis_t) | Maximum motor output power, in watts. Set to zero for no limit.<br>Scaled by 1.0 from 0.0 to 255.0.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_INS_OPTIONS"></a>InsOptions packet

INS options give configuration options for the inertial navigation system (INS) in the gunbal. Changing the INS options will cause the INS to be restarted if the platform rotation changes. When using platform rotation note the order: First yaw about the gimbal's z-axis to create an intermediate axis. Then pitch about the intermediate y-axis to create a second intermediate axis. Finally roll about the second intermediate x-axis to reach the platform orientation. When using an IMU in the payload ball the enablePlatformRotation can still be used, but will only effect heading observations and the GPS lever arm, it will not rotate the IMU data.

- packet identifier: `ORION_PKT_INS_OPTIONS` : 216
- minimum data length: 12
- maximum data length: 28


[InsOptions packet bytes]
| Bytes     | Name                           | [Enc](#Enc) | Repeat              | Description                                                                                                                                                                                                                                                                                                                             |
| --------- | ------------------------------ | :---------: | :-----------------: | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0:7       | 1)enablePlatformRotation       | B1          | 1                   | The INS will be run in the platform reference frame if this is set. The platform reference frame is defined by the rotation from the gimbal reference frame to the platform frame. Set this if the gimbal is installed such that the gimbal reference frame is often near pitch of +/- 90 degrees.                                      |
| 0:6       | 2)enableCourseIsHeading        | B1          | 1                   | If set then the INS will use the course of the GPS as a heading input for initialization and heading updates when the heading confidence is poor. This is appropriate for fixed wing aircraft if enablePlatformRotation is set (or if the gimbal native axis is aligned with the aircraft axis).                                        |
| 0:5       | 3)disableMagnetometer          | B1          | 1                   | If set then the INS will ignore any magnetometer measurements.                                                                                                                                                                                                                                                                          |
| 0:4       | 4)disableGPS                   | B1          | 1                   | If set then the INS will ignore any GPS measurements.                                                                                                                                                                                                                                                                                   |
| 0:3       | 5)resetINS                     | B1          | 1                   | If set then the INS will be reset, all other fields of this packet are ignored in that case.                                                                                                                                                                                                                                            |
| 0:2       | 6)disableGpsClockError         | B1          | 1                   | Set this to disable clock error estimation for tightly coupled INS.                                                                                                                                                                                                                                                                     |
| 0:1       | 7)enableAccelBias              | B1          | 1                   | Set this to enable accelerometer bias estimation.                                                                                                                                                                                                                                                                                       |
| 0:0       | 8)enableTightlyCoupled         | B1          | 1                   | Set this to enable the tightly coupled INS.                                                                                                                                                                                                                                                                                             |
| 1:7       | 9)usePhaseForVelocity          | B1          | 1                   | If set the INS will use the carrier phase rate for tightly coupled velocity observations. Otherwise the tightly coupled velocity observations come from doppler shifts.                                                                                                                                                                 |
| 1:6       | 10)disableL1                   | B1          | 1                   | If set the INS will ignore L1 band for tightly coupled observations.                                                                                                                                                                                                                                                                    |
| 1:5       | 11)disableL2                   | B1          | 1                   | If set the INS will ignore the L2 band for tightly coupled observations.                                                                                                                                                                                                                                                                |
| 1:4       | 12)disableL5                   | B1          | 1                   | If set the INS will ignore the L5 band for tightly coupled observations.                                                                                                                                                                                                                                                                |
| 1:3...1:0 | 13)elevationMask               | B4          | 1                   | Elevation mask for tightly coupled satellites.<br>Scaled by 15/(5&times;&pi/180) from 0.0 to 0.08726646259971647.                                                                                                                                                                                                                       |
| 2:7       | 14)enablePayloadIns            | B1          | 1                   | If set the IMU inputs to the INS are coming from the payload ball, not the gimbal crown. Note that enablePlatformRotation is interpreted differently in that case.                                                                                                                                                                      |
| 2:6       | 15)enableNonLinearHeading      | B1          | 1                   | If set then the INS will attempt to observe the heading use nonlinear methods. This may be helpful for hovering vehicles that do not have any other heading observation.                                                                                                                                                                |
| 2:5       | 16)enableSensonorMV            | B1          | 1                   | If set, and if enablePayloadIns is set, the Sensonor IMU inputs to the INS are coming from the payload ball in the HD80MV orientation.                                                                                                                                                                                                  |
| 2:4       | 17)enableHD25PayloadINS        | B1          | 1                   | If set, and if enablePayloadIns is set, the IMU inputs to the INS are coming from the payload ball in the HD25 special orientation.                                                                                                                                                                                                     |
| 2:3...2:0 | 18)opticalFlowSetting          | B4          | 1                   | Non-zero to enable optical flow.                                                                                                                                                                                                                                                                                                        |
| 3:7...3:0 | 19)reserved                    | B8          | 1                   | reserved space for future expansion.                                                                                                                                                                                                                                                                                                    |
| 4...9     | 20)gimbalToPlatformEuler       | I16         | [NUM_AXES](#Axis_t) | Roll, Pitch, Yaw angles in radians from -pi to pi. The angles give the Euler rotation (yaw first, then pitch, then roll) from the platform reference frame into the gimbal reference frame.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.                                                                                                |
| 10...11   | 21)initialHeading              | I16         | 1                   | Initial heading angle in radians for the INS.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.                                                                                                                                                                                                                                              |
| 12...17   | 22)gpsLeverArm                 | I16         | 3                   | X, Y, Z meters from first (primary) GPS antenna to IMU in the gimbal reference frame, or the platform reference frame if enablePlatformRotation is on.<br>Scaled by 1000 from -32.767 to 32.767.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                    |
| 18...19   | 23)headingObservationBiasAngle | I16         | 1                   | For external heading observations (including magnetometer) subtract this angle in radians from the heading observation to get the heading in INS or platform frame.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                    |
| 20...25   | 24)secondGPSLeverArm           | I16         | 3                   | X, Y, Z meters from second GPS antenna to IMU in the gimbal reference frame, or the platform reference frame if enablePlatformRotation is on. This only applies if Novatel Align data are being received.<br>Scaled by 1000 from -32.767 to 32.767.<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 26...27   | 25)alignHeadingNoise           | U16         | 1                   | Sensor noise in radians for align heading observation. Use zero for the default sensor noise from the Align hardware.<br>Scaled by 65535/(&pi) from 0.0 to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                    |


## <a name="ORION_PKT_NETWORK_SETTINGS"></a>OrionNetworkSettings packet

Change the IP network settings in the gimbal.

- packet identifier: `ORION_PKT_NETWORK_SETTINGS` : 228
- minimum data length: 12
- maximum data length: 15


[OrionNetworkSettings packet bytes]
| Bytes   | Name       | [Enc](#Enc) | Repeat | Description                                                                                                                                              |
| ------- | ---------- | :---------: | :----: | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3   | 1)Ip       | U32         | 1      | IPv4 address.                                                                                                                                            |
| 4...7   | 2)Mask     | U32         | 1      | IPv4 network mask.                                                                                                                                       |
| 8...11  | 3)Gateway  | U32         | 1      | IPv4 gateway address.                                                                                                                                    |
| 12      | 4)LowDelay | U8          | 1      | Set to 1 to use low-delay TCP mode (i.e. disable Nagle's Algorithm).<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 13...14 | 5)Mtu      | U16         | 1      | Ethernet MTU size, set to zero for hardware default.<br>This field is optional. If it is not included then the value is assumed to be 0.                 |


## <a name="ORION_PKT_NETWORK_SETTINGS"></a>OrionNetworkByteSettings packet

Change the IP network settings in the gimbal.

- packet identifier: `ORION_PKT_NETWORK_SETTINGS` : 228
- minimum data length: 12
- maximum data length: 15


[OrionNetworkByteSettings packet bytes]
| Bytes   | Name       | [Enc](#Enc) | Repeat | Description                                                                                                                                              |
| ------- | ---------- | :---------: | :----: | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3   | 1)Ip       | U8          | 4      | IPv4 address.                                                                                                                                            |
| 4...7   | 2)Mask     | U8          | 4      | IPv4 network mask.                                                                                                                                       |
| 8...11  | 3)Gateway  | U8          | 4      | IPv4 gateway address.                                                                                                                                    |
| 12      | 4)LowDelay | U8          | 1      | Set to 1 to use low-delay TCP mode (i.e. disable Nagle's Algorithm).<br>This field is optional. If it is not included then the value is assumed to be 0. |
| 13...14 | 5)Mtu      | U16         | 1      | Ethernet MTU size, set to zero for hardware default.<br>This field is optional. If it is not included then the value is assumed to be 0.                 |


## <a name="ORION_PKT_NETWORK_VIDEO"></a>OrionNetworkVideo packet

Change the IP settings for video delivery.

- packet identifier: `ORION_PKT_NETWORK_VIDEO` : 98
- minimum data length: 6
- maximum data length: 14

Network video output stream types

[<a name="StreamType_t"></a>StreamType_t enumeration]
| Name                | Value | Description                                  |
| ------------------- | :---: | -------------------------------------------- |
| `STREAM_TYPE_H264`  | 0     | h.264+KLV stream in an MPEG transport stream |
| `STREAM_TYPE_MJPEG` | 1     | MJPEG multipart video stream                 |
| `STREAM_TYPE_RAW`   | 2     | Raw multipart video stream                   |
| `STREAM_TYPE_YUV`   | 3     | YUV (NV12) multipart video stream            |
| `NUM_STREAM_TYPES`  | 4     | Number of video stream type IDs              |





[OrionNetworkVideo packet bytes]
| Bytes       | Name                          | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                            |
| ----------- | ----------------------------- | :---------: | :----: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3       | 1)DestIp                      | U32         | 1      | destination IPv4 address.                                                                                                                                                                                              |
| 4...5       | 2)Port                        | U16         | 1      | desitnation port.                                                                                                                                                                                                      |
| 6...9       | 3)Bitrate                     | U32         | 1      | Video encoding target bitrate, in bits per second.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                 |
| 10          | 4)Ttl                         | I8          | 1      | TTL value for UDP video packets. Values less than or equal to zero will use default values of 64 for unicast and 1 for multicast.<br>This field is optional. If it is not included then the value is assumed to be -1. |
| 11          | 5)[StreamType](#StreamType_t) | U8          | 1      | Output stream format specifier.<br>This field is optional. If it is not included then the value is assumed to be STREAM_TYPE_H264.                                                                                     |
| 12          | 6)MjpegQuality                | U8          | 1      | MJPEG quality value, from 10 to 95.<br>This field is optional. If it is not included then the value is assumed to be 30.                                                                                               |
| 13:7        | 7)SaveSettings                | B1          | 1      | Set to 1 to make h.264 settings non-volatile, or 0 to reset to defaults on startup.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                |
| 13:6...13:3 | 8)TsPacketCount               | B4          | 1      | Number of TS packets per UDP packet, or 0 for no change.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                           |


## <a name="ORION_PKT_NETWORK_VIDEO"></a>OrionNetworkByteVideo packet

Change the IP settings for video delivery.

- packet identifier: `ORION_PKT_NETWORK_VIDEO` : 98
- minimum data length: 6
- maximum data length: 14


[OrionNetworkByteVideo packet bytes]
| Bytes       | Name                          | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                            |
| ----------- | ----------------------------- | :---------: | :----: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...3       | 1)DestIp                      | U8          | 4      | destination IPv4 address.                                                                                                                                                                                              |
| 4...5       | 2)Port                        | U16         | 1      | desitnation port.                                                                                                                                                                                                      |
| 6...9       | 3)Bitrate                     | U32         | 1      | Video encoding target bitrate, in bits per second.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                                 |
| 10          | 4)Ttl                         | I8          | 1      | TTL value for UDP video packets. Values less than or equal to zero will use default values of 64 for unicast and 1 for multicast.<br>This field is optional. If it is not included then the value is assumed to be -1. |
| 11          | 5)[StreamType](#StreamType_t) | U8          | 1      | Output stream format specifier.<br>This field is optional. If it is not included then the value is assumed to be STREAM_TYPE_H264.                                                                                     |
| 12          | 6)MjpegQuality                | U8          | 1      | MJPEG quality value, from 10 to 95.<br>This field is optional. If it is not included then the value is assumed to be 30.                                                                                               |
| 13:7        | 7)SaveSettings                | B1          | 1      | Set to 1 to make h.264 settings non-volatile, or 0 to reset to defaults on startup.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                |
| 13:6...13:3 | 8)TsPacketCount               | B4          | 1      | Number of TS packets per UDP packet, or 0 for no change.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                           |


## <a name="ORION_PKT_CAMERAS"></a>OrionCameras packet

Camera definitions for the Orion gimbal

- packet identifier: `ORION_PKT_CAMERAS` : 99
- minimum data length: 4
- maximum data length: 52

The maximum number of cameras in an Orion Gimbal

[<a name="OrionNumberOfCameras_t"></a>OrionNumberOfCameras_t enumeration]
| Name          | Value | Description |
| ------------- | :---: | ----------- |
| `NUM_CAMERAS` | 2     |             |




The type of camera

[<a name="OrionCameraType_t"></a>OrionCameraType_t enumeration]
| Name                  | Value | Description                |
| --------------------- | :---: | -------------------------- |
| `CAMERA_TYPE_NONE`    | 0     | No camera                  |
| `CAMERA_TYPE_VISIBLE` | 1     | Visible light camera       |
| `CAMERA_TYPE_SWIR`    | 2     | Short wave infrared camera |
| `CAMERA_TYPE_MWIR`    | 3     | Mid wave infrared camera   |
| `CAMERA_TYPE_LWIR`    | 4     | Long wave infrared camera  |




The camera communications protocol

[<a name="OrionCameraProtocol_t"></a>OrionCameraProtocol_t enumeration]
| Name                   | Value | Description                          |
| ---------------------- | :---: | ------------------------------------ |
| `CAMERA_PROTO_UNKNOWN` | 0     | Protocol unknown or unsupported      |
| `CAMERA_PROTO_FLIR`    | 1     | FLIR camera                          |
| `CAMERA_PROTO_APTINA`  | 2     | Aptina camera                        |
| `CAMERA_PROTO_ZAFIRO`  | 3     | Zafiro camera                        |
| `CAMERA_PROTO_HITACHI` | 4     | Hitachi camera                       |
| `CAMERA_PROTO_BAE`     | 5     | BAE camera                           |
| `CAMERA_PROTO_SONY`    | 6     | Sony camera                          |
| `CAMERA_PROTO_KTNC`    | 7     | KTnC camera                          |
| `NUM_CAMERA_PROTOS`    | 8     | Number of available camera protocols |





[OrionCameras packet bytes]
| Bytes   | Name                                | [Enc](#Enc) | Repeat                                                   | Description                                                                          |
| ------- | ----------------------------------- | :---------: | :------------------------------------------------------: | ------------------------------------------------------------------------------------ |
| 0       | 1)NumCameras                        | U8          | 1                                                        | The number of cameras installed.                                                     |
| 1...3   | 2)reserved                          | U8          | 3                                                        | Reserved space for future expansion.                                                 |
| 4...51  | 3)OrionCamSettings                               || NumCameras, up to [NUM_CAMERAS](#OrionNumberOfCameras_t) | Camera settings information                                                          |
| 4       | 3.1)[Type](#OrionCameraType_t)      | U8          | 1                                                        | The type of camera.                                                                  |
| 5       | 3.2)[Proto](#OrionCameraProtocol_t) | U8          | 1                                                        | The communications protocol of the camera.                                           |
| 6...9   | 3.3)MinFocalLength                  | U32         | 1                                                        | Minimum focal length in millimeters.<br>Scaled by 1000 from 0.0 to 4294967.295.      |
| 10...13 | 3.4)MaxFocalLength                  | U32         | 1                                                        | Minimum focal length in millimeters.<br>Scaled by 1000 from 0.0 to 4294967.295.      |
| 14...15 | 3.5)PixelPitch                      | U16         | 1                                                        | Spacing of pixels in millimeters.<br>Scaled by 1000000 from 0.0 to 0.065535.         |
| 16...17 | 3.6)ArrayWidth                      | U16         | 1                                                        | Number of horizontal pixels in the focal plane array.                                |
| 18...19 | 3.7)ArrayHeight                     | U16         | 1                                                        | Number of vertical pixels in the focal plane array.                                  |
| 20...23 | 3.8)AlignMin                        | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t)                         | boresight alignment offsets in radians.<br>Scaled by 32767/(&pi) from -&pi; to &pi;. |
| 24...27 | 3.9)AlignMax                        | I16         | [NUM_GIMBAL_AXES](#GimbalAxis_t)                         | boresight alignment offsets in radians.<br>Scaled by 32767/(&pi) from -&pi; to &pi;. |


# Packets used to determine manufacturing and software information about the gimbal

These informational packets are sent by the gimbal in response to user request. To request a specific packet, send that packet to the gimbal with zero length.


## <a name="ORION_PKT_CLEVIS_VERSION"></a>OrionClevisVersion packet

Software version for the clevis board in the Orion gimbal.

- packet identifier: `ORION_PKT_CLEVIS_VERSION` : 37
- minimum data length: 32
- maximum data length: 36


[OrionClevisVersion packet bytes]
| Bytes   | Name         | [Enc](#Enc)                                | Repeat | Description                                                                                                   |
| ------- | ------------ | :----------------------------------------: | :----: | ------------------------------------------------------------------------------------------------------------- |
| 0...15  | 1)Version    | Zero terminated string of exactly 16 bytes         || Version of the clevis board software.                                                                         |
| 16...31 | 2)PartNumber | Zero terminated string of exactly 16 bytes         || Hardware details of the clevis board.                                                                         |
| 32...35 | 3)OnTime     | U32                                        | 1      | Board on time in minutes.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_CROWN_VERSION"></a>OrionCrownVersion packet

Software version for the crown board in the Orion gimbal.

- packet identifier: `ORION_PKT_CROWN_VERSION` : 40
- minimum data length: 32
- maximum data length: 36


[OrionCrownVersion packet bytes]
| Bytes   | Name         | [Enc](#Enc)                                | Repeat | Description                                                                                                   |
| ------- | ------------ | :----------------------------------------: | :----: | ------------------------------------------------------------------------------------------------------------- |
| 0...15  | 1)Version    | Zero terminated string of exactly 16 bytes         || Version of the crown board software.                                                                          |
| 16...31 | 2)PartNumber | Zero terminated string of exactly 16 bytes         || Hardware details of the crown board.                                                                          |
| 32...35 | 3)OnTime     | U32                                        | 1      | Board on time in minutes.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_PAYLOAD_VERSION"></a>OrionPayloadVersion packet

Software version for the payload board in the Orion gimbal.

- packet identifier: `ORION_PKT_PAYLOAD_VERSION` : 41
- minimum data length: 40
- maximum data length: 44


[OrionPayloadVersion packet bytes]
| Bytes   | Name         | [Enc](#Enc)                                | Repeat | Description                                                                                                   |
| ------- | ------------ | :----------------------------------------: | :----: | ------------------------------------------------------------------------------------------------------------- |
| 0...23  | 1)Version    | Zero terminated string of exactly 24 bytes         || Version of the payload board software.                                                                        |
| 24...39 | 2)PartNumber | Zero terminated string of exactly 16 bytes         || Hardware details of the payload board.                                                                        |
| 40...43 | 3)OnTime     | U32                                        | 1      | Board on time in minutes.<br>This field is optional. If it is not included then the value is assumed to be 0. |


## <a name="ORION_PKT_TRACKER_VERSION"></a>OrionTrackerVersion packet

Software version for the tracker board in the Orion Gimbal (if applicable)

- packet identifier: `ORION_PKT_TRACKER_VERSION` : 44
- data length: 32


[OrionTrackerVersion packet bytes]
| Bytes   | Name         | [Enc](#Enc)                                | Repeat | Description                            |
| ------- | ------------ | :----------------------------------------: | :----: | -------------------------------------- |
| 0...15  | 1)Version    | Zero terminated string of exactly 16 bytes         || Version of the tracker board software. |
| 16...31 | 2)PartNumber | Zero terminated string of exactly 16 bytes         || Hardware details of the tracker board. |


## <a name="ORION_PKT_RETRACT_VERSION"></a>OrionRetractVersion packet

Software version for the retract board in the Retract Unit (if applicable)

- packet identifier: `ORION_PKT_RETRACT_VERSION` : 248
- data length: 16


[OrionRetractVersion packet bytes]
| Bytes  | Name      | [Enc](#Enc)                                | Repeat | Description                            |
| ------ | --------- | :----------------------------------------: | :----: | -------------------------------------- |
| 0...15 | 1)Version | Zero terminated string of exactly 16 bytes         || Version of the retract board software. |


## <a name="ORION_PKT_LENSCTL_VERSION"></a>OrionLensCtlVersion packet

Software version for the retract board in the Retract Unit (if applicable)

- packet identifier: `ORION_PKT_LENSCTL_VERSION` : 249
- data length: 16


[OrionLensCtlVersion packet bytes]
| Bytes  | Name      | [Enc](#Enc)                                | Repeat | Description                            |
| ------ | --------- | :----------------------------------------: | :----: | -------------------------------------- |
| 0...15 | 1)Version | Zero terminated string of exactly 16 bytes         || Version of the retract board software. |


## <a name="ORION_PKT_BOARD"></a>OrionBoard packet

Information for a specific board in the gimbal.

- packet identifier: `ORION_PKT_BOARD` : 39
- data length: 20


[OrionBoard packet bytes]
| Bytes       | Name                                    | [Enc](#Enc) | Repeat | Description                                                          |
| ----------- | --------------------------------------- | :---------: | :----: | -------------------------------------------------------------------- |
| 0...3       | 1)boardSN                               | U32         | 1      | Serial number of this board.                                         |
| 4...7       | 2)assemblySN                            | U32         | 1      | Serial number of the whole assembly.                                 |
| 8...11      | 3)config                                | U32         | 1      | Configuration bits.                                                  |
| 12...14     | 4)reserved                              | U8          | 3      | Reserved space for future things.                                    |
| 15          | 5)[boardEnum](#OrionBoardEnumeration_t) | U8          | 1      | Enumeration defining which board in the gimbal this data applies to. |
| 16...17     | 6)manufactureDate                                    || 1      | Date the board was manufactured                                      |
| 16:7...16:1 | 6.1)year                                | B7          | 1      | Years after 2000.                                                    |
| 16:0...17:5 | 6.2)month                               | B4          | 1      | Month of the year, from 1 to 12.                                     |
| 17:4...17:0 | 6.3)day                                 | B5          | 1      | Day of the month, from 1 to 31.                                      |
| 18...19     | 7)calibrationDate                                    || 1      | Date the board was calibrated                                        |
| 18:7...18:1 | 7.1)year                                | B7          | 1      | Years after 2000.                                                    |
| 18:0...19:5 | 7.2)month                               | B4          | 1      | Month of the year, from 1 to 12.                                     |
| 19:4...19:0 | 7.3)day                                 | B5          | 1      | Day of the month, from 1 to 31.                                      |


# Packets for external sensor inputs

These optional packets can be sent to the gimbal to provide external sensor information.


## <a name="ORION_PKT_AUTOPILOT_DATA"></a>OrionAutopilotData packet

Autopilot state data. The gimbal will use this information to limit certain operations, for example some lasers will not fire unless the autopilot is flying with sufficient altitude above ground.

- packet identifier: `ORION_PKT_AUTOPILOT_DATA` : 128
- data length: 4


[OrionAutopilotData packet bytes]
| Bytes | Name       | [Enc](#Enc) | Repeat | Description                                                       |
| ----- | ---------- | :---------: | :----: | ----------------------------------------------------------------- |
| 0     | 1)IsFlying | U8          | 1      | Set to 1 if flying, otherwise 0.                                  |
| 1     | 2)CommGood | U8          | 1      | Set to 1 if AP comms are good, otherwise 0.                       |
| 2...3 | 3)Agl      | I16         | 1      | Altitude above ground level in meters. Invalid if less than zero. |


## <a name="ORION_PKT_EXT_HEADING_DATA"></a>OrionExtHeadingData packet

Externally supplied heading information. Send this packet to supply a correction to the heading of the inertial navigation system of the gimbal. This packet will be echoed back to the user if INS logging is enabled.

- packet identifier: `ORION_PKT_EXT_HEADING_DATA` : 210
- minimum data length: 4
- maximum data length: 8


[OrionExtHeadingData packet bytes]
| Bytes     | Name                  | [Enc](#Enc) | Repeat | Description                                                                                                                                                                                                                                                                         |
| --------- | --------------------- | :---------: | :----: | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0...1     | 1)extHeading          | I16         | 1      | External true heading observation in radians from -pi to pi.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.                                                                                                                                                                           |
| 2...3     | 2)noise               | U16         | 1      | Sensor noise of the heading observation in radians. Send 0 to cause the INS to simply shift its heading. Otherwise the extHeading will be fed into the INS kalman filter with this sensor noise.<br>Scaled by 65535/(2&times;&pi) from 0.0 to 2&pi;.                                |
| 4:7       | 3)headingInGimbalAxis | B1          | 1      | Set this if the provided heading is in gimbal native axis rather than the platform axis. This will cause the gimbal to rotate the heading reference as needed to the platform axis used by the INS. This will have no effect if the INS options do not specify a platform rotation. |
| 4:6       | 4)headingFromAlign    | B1          | 1      | Set this if the provided heading came from an align system.                                                                                                                                                                                                                         |
| 4:5...5:0 | 5)reserved            | B14         | 1      | Reserved bits in the packet.                                                                                                                                                                                                                                                        |
| 6...7     | 6)pitch               | I16         | 1      | Pitch angle in radians from -pi to pi of the vector used to compute heading.<br>Scaled by 32767/(&pi) from -&pi; to &pi;.<br>This field is optional. If it is not included then the value is assumed to be 0.                                                                       |


## <a name="ORION_PKT_RANGE_DATA"></a>OrionRangeData packet

Ranging information from the gimbal to its target. The gimbal will use this information to compute the target location and populate that informtaion in its telemetry output.

- packet identifier: `ORION_PKT_RANGE_DATA` : 214
- data length: 7


[OrionRangeData packet bytes]
| Bytes | Name                        | [Enc](#Enc) | Repeat | Description                                                          |
| ----- | --------------------------- | :---------: | :----: | -------------------------------------------------------------------- |
| 0...3 | 1)range                     | U32         | 1      | Range to target in meters.<br>Scaled by 100 from 0.0 to 42949672.95. |
| 4...5 | 2)maxAgeMs                  | U16         | 1      | Maximum data age before expiration in milliseconds.                  |
| 6     | 3)[source](#RangeDataSrc_t) | U8          | 1      | Source of range information.                                         |

<div class="page-break"></div>


----------------------------

# About this ICD

This is the interface control document for data *on the wire*, not data in memory. This document was automatically generated by the [ProtoGen software](https://github.com/billvaglienti/ProtoGen), version 2.12.c. ProtoGen also generates C source code for doing most of the work of encoding data from memory to the wire, and vice versa.

## Encodings

Data for this protocol are sent in BIG endian format. Any field larger than one byte is sent with the most signficant byte first, and the least significant byte last.

Data can be encoded as unsigned integers, signed integers (two's complement), bitfields, and floating point.

| <a name="Enc"></a>Encoding | Interpretation                        | Notes                                                                       |
| :--------------------------: | ------------------------------------- | --------------------------------------------------------------------------- |
| UX                           | Unsigned integer X bits long          | X must be: 8, 16, 24, 32, 40, 48, 56, or 64                                 |
| IX                           | Signed integer X bits long            | X must be: 8, 16, 24, 32, 40, 48, 56, or 64                                 |
| BX                           | Unsigned integer bitfield X bits long | X must be greater than 0 and less than 32                                   |
| F16:X                        | 16 bit float with X significand bits  | 1 sign bit : 15-X exponent bits : X significant bits with implied leading 1 |
| F24:X                        | 24 bit float with X significand bits  | 1 sign bit : 23-X exponent bits : X significant bits with implied leading 1 |
| F32                          | 32 bit float (IEEE-754)               | 1 sign bit : 8 exponent bits : 23 significant bits with implied leading 1   |
| F64                          | 64 bit float (IEEE-754)               | 1 sign bit : 11 exponent bits : 52 significant bits with implied leading 1  |

## Size of fields
The encoding tables give the bytes for each field as X...Y; where X is the first byte (counting from 0) and Y is the last byte. For example a 4 byte field at the beginning of a packet will give 0...3. If the field is 1 byte long then only the starting byte is given. Bitfields are more complex, they are displayed as Byte:Bit. A 3-bit bitfield at the beginning of a packet will give 0:7...0:5, indicating that the bitfield uses bits 7, 6, and 5 of byte 0. Note that the most signficant bit of a byte is 7, and the least signficant bit is 0. If the bitfield is 1 bit long then only the starting bit is given.

The byte count in the encoding tables are based on the maximum length of the field(s). If a field is variable length then the actual byte location of the data may be different depending on the value of the variable field. If the field is a variable length character string this will be indicated in the encoding column of the table. If the field is a variable length array this will be indicated in the repeat column of the encoding table. If the field depends on the non-zero value of another field this will be indicated in the description column of the table.

