/**
 * @copyright Copyright (c) 2021 Sagetech, Inc. All rights reserved.
 *
 * @file   sg.h
 * @author jimb
 *
 * @date   Feb 10, 2021
 *
 * Sagetech protocol for host message building and parsing.
 *
 * This module performs both the following:
 *    1. Parses raw Sagetech host messages defined in the SDIM and
 *       returns a populated struct dataset of the message type.
 *    2. Receives a populated struct dataset of the desired host message
 *       and returns the corresponding raw message data buffer.
 */

#ifndef SG_H
#define SG_H

#include <stdint.h>
#include <stdbool.h>

/// Host Message Lengths (bytes)
#define SG_MSG_LEN_INSTALL 41
#define SG_MSG_LEN_FLIGHT 17
#define SG_MSG_LEN_OPMSG 17
#define SG_MSG_LEN_GPS 68
#define SG_MSG_LEN_DATAREQ 9
#define SG_MSG_LEN_TARGETREQ 12
#define SG_MSG_LEN_MODE 10

/// Host Message Types
#define SG_MSG_TYPE_HOST_INSTALL 0x01
#define SG_MSG_TYPE_HOST_FLIGHT 0x02
#define SG_MSG_TYPE_HOST_OPMSG 0x03
#define SG_MSG_TYPE_HOST_GPS 0x04
#define SG_MSG_TYPE_HOST_DATAREQ 0x05
#define SG_MSG_TYPE_HOST_TARGETREQ 0x0B
#define SG_MSG_TYPE_HOST_MODE 0x0C

/// XPNDR Message Types
#define SG_MSG_TYPE_XPNDR_ACK 0x80
#define SG_MSG_TYPE_XPNDR_INSTALL 0x81
#define SG_MSG_TYPE_XPNDR_FLIGHT 0x82
#define SG_MSG_TYPE_XPNDR_STATUS 0x83
#define SG_MSG_TYPE_XPNDR_COMMA 0x85
#define SG_MSG_TYPE_XPNDR_MODE 0x8C
#define SG_MSG_TYPE_XPNDR_VERSION 0x8E
#define SG_MSG_TYPE_XPNDR_SERIALNUM 0x8F

/// ADS-B Message Types
#define SG_MSG_TYPE_ADSB_TSUMMARY 0x90
#define SG_MSG_TYPE_ADSB_SVR 0x91
#define SG_MSG_TYPE_ADSB_MSR 0x92
#define SG_MSG_TYPE_ADSB_TSTATE 0x97
#define SG_MSG_TYPE_ADSB_ARVR 0x98

/// Start byte for all host messages
#define SG_MSG_START_BYTE 0xAA

/// Emitter category set byte values
#define SG_EMIT_GROUP_A 0x00
#define SG_EMIT_GROUP_B 0x01
#define SG_EMIT_GROUP_C 0x02
#define SG_EMIT_GROUP_D 0x03

/// Emitter category enumeration offsets
#define SG_EMIT_OFFSET_A 0x00
#define SG_EMIT_OFFSET_B 0x10
#define SG_EMIT_OFFSET_C 0x20
#define SG_EMIT_OFFSET_D 0x30

/**
 * Available COM port baud rates.
 */
typedef enum
{
    baud38400 = 0,
    baud600,
    baud4800,
    baud9600,
    baud28800,
    baud57600,
    baud115200,
    baud230400,
    baud19200,
    baud460800,
    baud921600
} sg_baud_t;

/**
 * Transponder ethernet configuration
 */
typedef struct
{
    uint32_t ipAddress;  /// The transponder ip address
    uint32_t subnetMask; /// The transponder subnet mask
    uint16_t portNumber; /// The transponder port number
} sg_ethernet_t;

/**
 * Available GPS integrity SIL values
 */
typedef enum
{
    silUnknown = 0,
    silLow,
    silMedium,
    silHigh
} sg_sil_t;

/**
 * Available GPS integrity SDA values
 */
typedef enum
{
    sdaUnknown = 0,
    sdaMinor,
    sdaMajor,
    sdaHazardous
} sg_sda_t;

/**
 * Available emitter types
 */
typedef enum
{
    aUnknown = SG_EMIT_OFFSET_A,
    aLight,
    aSmall,
    aLarge,
    aHighVortex,
    aHeavy,
    aPerformance,
    aRotorCraft,
    bUnknown = SG_EMIT_OFFSET_B,
    bGlider,
    bAir,
    bParachutist,
    bUltralight,
    bUAV = SG_EMIT_OFFSET_B + 6,
    bSpace,
    cUnknown = SG_EMIT_OFFSET_C,
    cEmergency,
    cService,
    cPoint,
    cCluster,
    cLine,
    dUnknown = SG_EMIT_OFFSET_D
} sg_emitter_t;

/**
 * Available aircraft sizes in meters
 */
typedef enum
{
    sizeUnknown = 0, /// Dimensions unknown
    sizeL15W23,      /// Length <= 15m & Width <= 23m
    sizeL25W28,      /// Length <= 25m & Width <= 28.5m
    sizeL25W34,      /// Length <= 25m & Width <= 34m
    sizeL35W33,      /// Length <= 35m & Width <= 33m
    sizeL35W38,      /// Length <= 35m & Width <= 38m
    sizeL45W39,      /// Length <= 45m & Width <= 39.5m
    sizeL45W45,      /// Length <= 45m & Width <= 45m
    sizeL55W45,      /// Length <= 55m & Width <= 45m
    sizeL55W52,      /// Length <= 55m & Width <= 52m
    sizeL65W59,      /// Length <= 65m & Width <= 59.5m
    sizeL65W67,      /// Length <= 65m & Width <= 67m
    sizeL75W72,      /// Length <= 75m & Width <= 72.5m
    sizeL75W80,      /// Length <= 75m & Width <= 80m
    sizeL85W80,      /// Length <= 85m & Width <= 80m
    sizeL85W90       /// Length <= 85m & Width <= 90m
} sg_size_t;

/**
 * Available aircraft maximum airspeeds
 */
typedef enum
{
    speedUnknown = 0, /// Max speed unknown
    speed75kt,        /// 0 knots    < Max speed < 75 knots
    speed150kt,       /// 75 knots   < Max speed < 150 knots
    speed300kt,       /// 150 knots  < Max speed < 300 knots
    speed600kt,       /// 300 knots  < Max speed < 600 knots
    speed1200kt,      /// 600 knots  < Max speed < 1200 knots
    speedGreater      /// 1200 knots < Max speed
} sg_airspeed_t;

/**
 * Available antenna configurations
 */
typedef enum
{
    antBottom = 1, /// bottom antenna only
    antBoth = 3    /// both top and bottom antennae
} sg_antenna_t;

/**
 * The XPNDR Installation Message.
 * Host --> XPNDR.
 * XPNDR --> Host.
 * Use 'strcpy(install.reg, "REGVAL1")' to assign the registration.
 */
typedef struct
{
    uint32_t icao;          /// The aircraft's ICAO address
    char reg[8];            /// The aircraft's registration (left-justified alphanumeric characters padded with spaces)
    sg_baud_t com0;         /// The baud rate for COM Port 0
    sg_baud_t com1;         /// The baud rate for COM Port 1
    sg_ethernet_t eth;      /// The ethernet configuration
    sg_sil_t sil;           /// The gps integrity SIL parameter
    sg_sda_t sda;           /// The gps integrity SDA parameter
    sg_emitter_t emitter;   /// The platform's emitter type
    sg_size_t size;         /// The platform's dimensions
    sg_airspeed_t maxSpeed; /// The platform's maximum airspeed
    int16_t altOffset;      /// The altitude encoder offset is a legacy field that should always = 0
    sg_antenna_t antenna;   /// The antenna configuration
    bool altRes100;         /// Altitude resolution. true = 100 foot, false = 25 foot
    bool hdgTrueNorth;      /// Heading type. true = true north, false = magnetic north
    bool airspeedTrue;      /// Airspeed type. true = true speed, false = indicated speed
    bool heater;            /// true = heater enabled, false = heater disabled
    bool wowConnected;      /// Weight on Wheels sensor. true = connected, false = not connected
} sg_install_t;

/**
 * The XPNDR Flight ID Message.
 * Host --> XPNDR.
 * XPNDR --> Host.
 * * Use 'strcpy(id.flightID, "FLIGHTNO")' to assign the flight identification.
 */
typedef struct
{
    char flightId[9]; /// The flight identification (left-justified alphanumeric characters padded with spaces)
} sg_flightid_t;

/**
 * Available transponder operating modes. The enumerated values are
 * offset from the host message protocol values.
 */
typedef enum
{
    modeOff = 0, /// 'Off' Mode:     Xpdr will not transmit
    modeOn,      /// 'On' Mode:      Full functionality with Altitude = Invalid
    modeStby,    /// 'Standby' Mode: Reply to lethal interrogations, only
    modeAlt      /// 'Alt' Mode:     Full functionality
} sg_op_mode_t;

/**
 * Available emergency status codes.
 */
typedef enum
{
    emergcNone = 0, /// No Emergency
    emergcGeneral,  /// General Emergency
    emergcMed,      /// Lifeguard/Medical Emergency
    emergcFuel,     /// Minimum Fuel
    emergcComm,     /// No Communications
    emergcIntrfrc,  /// Unlawful Interference
    emergcDowned    /// Downed Aircraft
} sg_emergc_t;

/**
 * The XPNDR Operating Message.
 * Host --> XPNDR.
 */
typedef struct
{
    uint16_t squawk;        /// 4-digit octal Mode A code
    sg_op_mode_t opMode;    /// Operational mode
    bool savePowerUp;       /// Save power-up state in non-volatile
    bool enableSqt;         /// Enable extended squitters
    bool enableXBit;        /// Enable the x-bit
    bool milEmergency;      /// Broadcast a military emergency
    sg_emergc_t emergcType; /// Enumerated civilian emergency type
    bool identOn;           /// Set the identification switch = On
    bool altUseIntrnl;      /// True = Report altitude from internal pressure sensor (will ignore other bits in the field)
    bool altHostAvlbl;      /// True = Host Altitude is being provided
    bool altRes25;          /// Host Altitude Resolution from install message, True = 25 ft, False = 100 ft
    int32_t altitude;       /// Sea-level altitude in feet. Field is ignored when internal altitude is selected.
    bool climbValid;        /// Climb rate is provided;
    int16_t climbRate;      /// Climb rate in ft/min. Limits are +/- 16,448 ft/min.
    bool headingValid;      /// Heading is valid.
    double heading;         /// Heading in degrees
    bool airspdValid;       /// Airspeed is valid.
    uint16_t airspd;        /// Airspeed in knots.
} sg_operating_t;

/**
 * Avaiable NACp values.
 */
typedef enum
{
    nacpUnknown, /// >= 18.52  km ( 10  nmile)
    nacp10dot0,  /// <  18.52  km ( 10  nmile)
    nacp4dot0,   /// <   7.408 km (  4  nmile)
    nacp2dot0,   /// <   3.704 km (  2  nmile)
    nacp1dot0,   /// <   1.852 km (  1  nmile)
    nacp0dot5,   /// <   0.926 km (0.5  nmile)
    nacp0dot3,   /// <   0.556 km (0.3  nmile)
    nacp0dot1,   /// <   0.185 km (0.1  nmile)
    nacp0dot05,  /// <    92.6 m  (0.05 nmile)
    nacp30,      /// <    30.0 m
    nacp10,      /// <    10.0 m
    nacp3        /// <     3.0 m
} sg_nacp_t;

/**
 * Available NACv values (m/s)
 */
typedef enum
{
    nacvUnknown = 0, /// 10   <= NACv (or NACv is unknown)
    nacv10dot0,      ///  3   <= NACv < 10
    nacv3dot0,       ///  1   <= NACv <  3
    nacv1dot0,       ///  0.3 <= NACv <  1
    nacv0dot3        ///  0.0 <= NACv <  0.3
} sg_nacv_t;

/**
 * The XPNDR Simulated GPS Message.
 * Host --> XPNDR.
 */
typedef struct
{
    char longitude[12]; /// The absolute value of longitude (degree and decimal minute)
    char latitude[11];  /// The absolute value of latitude (degree and decimal minute)
    char grdSpeed[7];   /// The GPS over-ground speed (knots)
    char grdTrack[9];   /// The GPS track referenced from True North (degrees, clockwise)
    bool latNorth;      /// The aircraft is in the northern hemisphere
    bool lngEast;       /// The aircraft is in the eastern hemisphere
    bool fdeFail;       /// True = A satellite error has occurred
    bool gpsValid;      /// True = GPS data is valid
    char timeOfFix[11]; /// Time, relative to midnight UTC (can optionally be filled spaces)
    float height;       /// The height above the WGS-84 ellipsoid (meters)
    float hpl;          /// The Horizontal Protection Limit (meters)
    float hfom;         /// The Horizontal Figure of Merit (meters)
    float vfom;         /// The Vertical Figure of Merit (meters)
    sg_nacv_t nacv;     /// Navigation Accuracy for Velocity (meters/second)
} sg_gps_t;

/**
 * Available data request types
 */
typedef enum
{
    dataInstall = 0x81,    /// Installation data
    dataFlightID = 0x82,   /// Flight Identification data
    dataStatus = 0x83,     /// Status Response data
    dataMode = 0x8C,       /// Mode Settings data
    dataHealth = 0x8D,     /// Health Monitor data
    dataVersion = 0x8E,    /// Version data
    dataSerialNum = 0x8F,  /// Serial Number data
    dataTOD = 0xD2,        /// Time of Day data
    dataMode5 = 0xD3,      /// Mode 5 Indication data
    dataCrypto = 0xD4,     /// Crypto Status data
    dataMilSettings = 0xD7 /// Military Settings data
} sg_datatype_t;

/**
 * The Data Request message.
 * Host --> XPDR.
 */
typedef struct
{
    sg_datatype_t reqType; /// The desired data response
    uint8_t resv[3];
} sg_datareq_t;

/**
 * Available target request types
 */
typedef enum
{
    reportAuto = 0, /// Enable auto output of all target reports
    reportSummary,  /// Report list of all tracked targets (disables auto-output)
    reportIcao,     /// Generate reports for specific target, only (disables auto-output)
    reportNone      /// Disable all target reports
} sg_reporttype_t;

/**
 * Available target report transmission ports
 */
typedef enum
{
    transmitSource = 0, /// Transmit reports on channel where target request was received
    transmitCom0,       /// Transmit reports on Com0
    transmitCom1,       /// Transmit reports on Com1
    transmitEth         /// Transmit reports on Ethernet
} sg_transmitport_t;

/**
 * The Target Request message for ADS-B 'in' data.
 * Host --> XPDR.
 */
typedef struct
{
    sg_reporttype_t reqType;        /// The desired report mode
    sg_transmitport_t transmitPort; /// The communication port used for report transmission
    uint16_t maxTargets;            /// The maximum number of targets to track (max value: 404)
    uint32_t icao;                  /// The desired target's ID, if applicable
    bool stateVector;               /// Transmit state vector reports
    bool modeStatus;                /// Transmit mode status reports
    bool targetState;               /// Transmit target state reports
    bool airRefVel;                 /// Transmit air referenced velocity reports
    bool tisb;                      /// Transmit raw TIS-B message reports (requires auto-output)
    bool military;                  /// Enable tracking of military aircraft
    bool commA;                     /// Transmit Comm-A Reports (requires auto-output)
    bool ownship;                   /// Transmit reports about own aircraft
} sg_targetreq_t;

/**
 * The Mode message.
 * Host --> XPDR.
 */
typedef struct
{
    bool reboot; /// Reboot the MX
} sg_mode_t;

/**
 * The XPNDR Acknowledge Message following all host messages.
 * XPNDR --> Host.
 */
typedef struct
{
    uint8_t ackType;     /// Message type being acknowledged
    uint8_t ackId;       /// Message ID being acknowledged
    bool failXpdr;       /// Built-in-test failure
    bool failSystem;     /// Required system input missing
    bool failCrypto;     /// Crypto status failure
    bool wow;            /// Weight-on-wheels indicates aircraft is on-ground
    bool maint;          /// Maintenance mode enabled
    bool isHostAlt;      /// False = Pressure sensor altitude, True = Host provided value
    sg_op_mode_t opMode; /// Operational mode
    int32_t alt;         /// Altitude (feet)
    bool altValid;       /// Altitude is valid
} sg_ack_t;

/**
 * The XPNDR Status Response Message following a Data Request for Status.
 * XPNDR --> Host.
 */
typedef struct
{
    uint8_t versionSW; /// SW Version # installed on the XPNDR
    uint8_t versionFW; /// FW Version # installed on the XPNDR
    uint32_t crc;      /// CRC Checksum for the installed XPNDR SW/FW versions

    bool powerUp : 1;    /// Integrity of CPU and Non-Volatile data at power-up
    bool continuous : 1; /// Set by any other B.I.T. failures during operation
    bool processor : 1;  /// One-time processor instruction set test at power-up
    bool crcValid : 1;   /// Calculate then verifies the CRC against the stored value
    bool memory : 1;     /// Processor RAM is functional
    bool calibrated : 1; /// Transponder is calibrated
    bool receiver : 1;   /// RF signals travel through hardware correctly
    bool power53v : 1;   /// Voltage at the 53V power supply is correct
    bool adc : 1;        /// Analog-to-Digital Converter is functional
    bool pressure : 1;   /// Internal pressure transducer is functional
    bool fpga : 1;       /// FPGA I/O operations are functional
    bool rxLock : 1;     /// Rx oscillator reporting PLL Lock at reference frequency
    bool txLock : 1;     /// Tx oscillator reporting PLL Lock at reference frequency
    bool mtSuppress : 1; /// Mutual suppression is operating correctly
    bool temp : 1;       /// Internal temperature is within range (< 110 C)
    bool sqMonitor : 1;  /// Squitters are transmitting at their nominal rates
    bool txRate : 1;     /// Transmission duty cycle is in the safe range
    bool sysLatency : 1; /// Systems events occurred within expected time limits
    bool txPower : 1;    /// Transmission power is in-range
    bool voltageIn : 1;  /// Input voltage is in-range (10V-32V)
    bool icao : 1;       /// ICAO Address is valid (fail at '000000' or 'FFFFFF')
    bool gps : 1;        /// Valid GPS data is received at 1Hz, minimum
} sg_status_t;

/**
 * The XPNDR Health Monitor Response Message.
 * XPNDR --> Host.
 */
typedef struct
{
    int8_t socTemp; /// System on a Chip temperature
    int8_t rfTemp;  /// RF Board temperature
    int8_t ptTemp;  /// Pressure Transducer temperature
} sg_healthmonitor_t;

/**
 * The XPNDR Version Response Message.
 * XPNDR --> Host.
 */
typedef struct
{
    uint8_t swVersion;      /// The SW Version major revision number
    uint8_t fwVersion;      /// The FW Version major revision number
    uint16_t swSvnRevision; /// The SW Repository version number
    uint16_t fwSvnRevision; /// The FW Repository version number
} sg_version_t;

/**
 * The XPNDR Serial Number Response Message.
 * XPNDR --> Host.
 */
typedef struct
{
    char ifSN[33];    /// The Interface Board serial number
    char rfSN[33];    /// The RF Board serial number
    char xpndrSN[33]; /// The Transponder serial number
} sg_serialnumber_t;

/// The state vector report type.
typedef enum
{
    svrAirborne = 1, /// Airborne state vector report type.
    svrSurface       /// Surface state vector report type.
} sg_svr_type_t;

/// The state vector report participant address type.
typedef enum
{
    svrAdrIcaoUnknown,    /// ICAO address unknown emitter category.
    svrAdrNonIcaoUnknown, /// Non-ICAO address unknown emitter category.
    svrAdrIcao,           /// ICAO address aircraft.
    svrAdrNonIcao,        /// Non-ICAO address aircraft.
    svrAdrIcaoSurface,    /// ICAO address surface vehicle, fixed ground, tethered obstruction.
    svrAdrNonIcaoSurface, /// Non-ICAO address surface vehicle, fixed ground, tethered obstruction.
    svrAdrDup,            /// Duplicate target of another ICAO address.
    svrAdrAdsr            /// ADS-R target.
} sg_addr_type_t;

/// The surface part of a state vector report.
typedef struct
{
    int16_t speed;   /// Surface speed.
    int16_t heading; /// Surface heading.
} sg_svr_surface_t;

/// The airborne part of a state vector report.
typedef struct
{
    int16_t velNS;   /// The NS speed vector component. [knots]
    int16_t velEW;   /// The EW speed vector component. [knots]
    int16_t speed;   /// Speed from N/S and E/W velocity. [knots]
    int16_t heading; /// Heading from N/S and E/W velocity. [deg from N]
    int32_t geoAlt;  /// Geometric altitude. [ft]
    int32_t baroAlt; /// Barometric altitude. [ft]
    int16_t vrate;   /// Vertical rate. [ft/min]
    float estLat;    /// Estimated latitude. [deg N]
    float estLon;    /// Estimated longitude. [deg E]
} sg_svr_airborne_t;

typedef struct
{
    bool baroVRate : 1;   /// Barometric vertical rate valid.
    bool geoVRate : 1;    /// Geometric vertical rate valid.
    bool baroAlt : 1;     /// Barometric altitude valid.
    bool surfHeading : 1; /// Surface heading valid.
    bool surfSpeed : 1;   /// Surface speed valid.
    bool airSpeed : 1;    /// Airborne speed and heading valid.
    bool geoAlt : 1;      /// Geometric altitude valid.
    bool position : 1;    /// Lat and lon data valid.
} sg_svr_validity_t;

typedef struct
{
    uint8_t reserved : 6; /// Reserved.
    bool estSpeed : 1;    /// Estimated N/S and E/W velocity.
    bool estPosition : 1; /// Estimated lat/lon position.
} sg_svr_est_validity_t;

/**
 * The XPDR ADS-B state vector report Message.
 * Host --> XPDR.
 *
 * @note The time of applicability values are based on the MX system clock that starts
 * at 0 on power up. The time is the floating point number that is the seconds since
 * power up. The time number rolls over at 512.0.
 */
typedef struct
{
    sg_svr_type_t type; /// Report type.
    union
    {
        uint8_t flags;
        sg_svr_validity_t validity; /// Field validity flags.
    };
    union
    {
        uint8_t eflags;
        sg_svr_est_validity_t evalidity; /// Estimated field validity flags.
    };
    uint32_t addr;           /// Participant address.
    sg_addr_type_t addrType; /// Participant address type.
    float toaEst;            /// Report estimated position and speed time of applicability.
    float toaPosition;       /// Report position time of applicability.
    float toaSpeed;          /// Report speed time of applicability.
    uint8_t survStatus;      /// Surveillance status.
    uint8_t mode;            /// Report mode.
    uint8_t nic;             /// Navigation integrity category.
    float lat;               /// Latitude.
    float lon;               /// Longitude.
    union
    {
        sg_svr_surface_t surface;   /// Surface SVR data.
        sg_svr_airborne_t airborne; /// Airborne SVR data.
    };
} sg_svr_t;

typedef enum
{
    msrTypeV0,
    msrTypeV1Airborne,
    msrTypeV1Surface,
    msrTypeV2Airborne,
    msrTypeV2Surface
} sg_msr_type_t;

typedef struct
{
    uint8_t reserved : 2;
    bool priority : 1;
    bool sil : 1;
    bool nacv : 1;
    bool nacp : 1;
    bool opmode : 1;
    bool capcodes : 1;
} sg_msr_validity_t;

typedef enum
{
    adsbVerDO260,
    adsbVerDO260A,
    adsbVerDO260B
} sg_adsb_version_t;

typedef enum
{
    adsbUnknown,
    adsbLight,
    adsbSmall = 0x3,
    adsbLarge = 0x5,
    adsbHighVortex,
    adsbHeavy,
    adsbPerformance,
    adsbRotorcraft = 0x0A,
    adsbGlider,
    adsbAir,
    adsbUnmaned,
    adsbSpace,
    adsbUltralight,
    adsbParachutist,
    adsbVehicle_emg = 0x14,
    adsbVehicle_serv,
    adsbObsticlePoint,
    adsbObsticleCluster,
    adsbObsticleLinear
} sg_adsb_emitter_t;

typedef enum
{
    priNone,
    priGeneral,
    priMedical,
    priFuel,
    priComm,
    priUnlawful,
    priDowned
} sg_priority_t;

typedef enum
{
    tcrNone,
    tcrSingle,
    tcrMultiple
} sg_tcr_t;

typedef struct
{
    bool b2low : 1;
    bool uat : 1;
    bool arv : 1;
    bool tsr : 1;
    bool adsb : 1;
    bool tcas : 1;
    sg_tcr_t tcr;
} sg_capability_t;

typedef enum
{
    gpsLonNodata,
    gpsLonSensorSupplied,
    gpsLon2m,
    gpsLon4m,
    gpsLon6m,
    gpsLon8m,
    gpsLon10m,
    gpsLon12m,
    gpsLon14m,
    gpsLon16m,
    gpsLon18m,
    gpsLon20m,
    gpsLon22m,
    gpsLon24m,
    gpsLon26m,
    gpsLon28m,
    gpsLon30m,
    gpsLon32m,
    gpsLon34m,
    gpsLon36m,
    gpsLon38m,
    gpsLon40m,
    gpsLon42m,
    gpsLon44m,
    gpsLon46m,
    gpsLon48m,
    gpsLon50m,
    gpsLon52m,
    gpsLon54m,
    gpsLon56m,
    gpsLon58m,
    gpsLon60m
} sg_gps_lonofs_t;

typedef enum
{
    gpslatNodata,
    gpslatLeft2m,
    gpslatLeft4m,
    gpslatLeft6m,
    gpslatRight0m,
    gpslatRight2m,
    gpslatRight4m,
    gpslatRight6m,
} sg_gps_latofs_t;

typedef struct
{
    bool gpsLatFmt;
    sg_gps_latofs_t gpsLatOfs;
    bool gpsLonFmt;
    sg_gps_lonofs_t gpsLonOfs;
    bool tcasRA : 1;
    bool ident : 1;
    bool singleAnt : 1;
} sg_adsb_opmode_t;

typedef enum
{
    gvaUnknown,
    gvaLT150m,
    gvaLT45m
} sg_gva_t;

typedef enum
{
    nicGolham,
    nicNonGilham
} sg_nicbaro_t;

typedef enum
{
    svsilUnknown,
    svsilPow3,
    svsilPow5,
    svsilPow7
} sg_svsil_t;

typedef struct
{
    sg_nacp_t nacp;
    sg_nacv_t nacv;
    sg_sda_t sda;
    bool silSupp;
    sg_svsil_t sil;
    sg_gva_t gva;
    sg_nicbaro_t nicBaro;
} sg_sv_qual_t;

typedef enum
{
    trackTrueNorth,
    trackMagNorth,
    headingTrueNorth,
    headingMagNorth
} sg_trackheading_t;

typedef enum
{
    vrateBaroAlt,
    vrateGeoAlt
} sg_vratetype_t;

/**
 * The XPDR ADS-B mode status report Message.
 * Host --> XPDR.
 *
 * @note The time of applicability values are based on the MX system clock that starts
 * at 0 on power up. The time is the floating point number that is the seconds since
 * power up. The time number rolls over at 512.0.
 */
typedef struct
{
    sg_msr_type_t type; /// Report type.

    union
    {
        uint8_t flags;
        sg_msr_validity_t validity; /// Field validity flags.
    };

    uint32_t addr;           /// Participant address.
    sg_addr_type_t addrType; /// Participant address type.

    float toa;
    sg_adsb_version_t version;
    char callsign[9];
    sg_adsb_emitter_t emitter;
    sg_size_t size;
    sg_priority_t priority;
    sg_capability_t capability;
    sg_adsb_opmode_t opMode;
    sg_sv_qual_t svQuality;
    sg_trackheading_t trackHeading;
    sg_vratetype_t vrateType;
} sg_msr_t;

/**
 * Convert install message struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw install message.
 * @param[in]  stl    The install message struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in stl parameter must be pre-validated.
 */
bool sgEncodeInstall(uint8_t *buffer, sg_install_t *stl, uint8_t msgId);

/**
 * Convert flight identification struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw flight identification message.
 * @param[in]  id     The flight id struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in id parameter must be pre-validated.
 */
bool sgEncodeFlightId(uint8_t *buffer, sg_flightid_t *id, uint8_t msgId);

/**
 * Convert operating message struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw operating message.
 * @param[in]  op     The operating message struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in op parameter must be pre-validated.
 */
bool sgEncodeOperating(uint8_t *buffer, sg_operating_t *op, uint8_t msgId);

/* TODO: Create GPS helper functions to convert other data types --> char buffers */

/**
 * Convert GPS message struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw GPS message.
 * @param[in]  gps    The GPS message struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in gps parameter must be pre-validated.
 */
bool sgEncodeGPS(uint8_t *buffer, sg_gps_t *gps, uint8_t msgId);

/**
 * Convert data request message struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw target request message.
 * @param[in]  data   The data request message struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in data parameter must be pre-validated.
 */
bool sgEncodeDataReq(uint8_t *buffer, sg_datareq_t *data, uint8_t msgId);

/**
 * Convert target request message struct to the raw buffer format.
 *
 * @param[out] buffer An empty buffer to contain the raw target request message.
 * @param[in]  tgt    The target request message struct with fields populated.
 * @param[in]  msgId  The sequence number for the message.
 *
 * @return true if successful or false on failure.
 *
 * @warning data in tgt parameter must be pre-validated.
 */
bool sgEncodeTargetReq(uint8_t *buffer, sg_targetreq_t *tgt, uint8_t msgId);

/**
 * Process the ACK message response from the transponder.
 *
 * @param[in]  buffer The raw ACK message buffer.
 * @param[out] ack    The parsed message results.
 *
 * @return true if successful or false on failure.
 */
bool sgDecodeAck(uint8_t *buffer, sg_ack_t *ack);

/**
 * Process the Install message response from the transponder.
 *
 * @param[in]  buffer The raw Install message buffer.
 * @param[out] stl    The parsed message results.
 *
 * @return true if successful or false on failure.
 */
bool sgDecodeInstall(uint8_t *buffer, sg_install_t *stl);

/**
 * Process the Flight ID message response from the transponder.
 *
 * @param[in]  buffer The raw Flight ID message buffer.
 * @param[out] id     The parsed message results.
 *
 * @return true if successful or false on failure.
 */
bool sgDecodeFlightId(uint8_t *buffer, sg_flightid_t *id);

/**
 * Process the state vector report message.
 *
 * @param[in]  buffer The raw SVR message buffer.
 * @param[out] svr    The parsed SVR message.
 *
 * @return true if successful or false on failure.
 */
bool sgDecodeSVR(uint8_t *buffer, sg_svr_t *svr);

/**
 * Process the mode status report message.
 *
 * @param buffer The raw MSR message buffer.
 * @param msr    The parsed MSR message.
 *
 * @return true if successful or false on failure.
 */
bool sgDecodeMSR(uint8_t *buffer, sg_msr_t *msr);

#endif /* SG_H */
