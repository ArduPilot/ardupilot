// SPDX-License-Identifier: GPL-3.0-or-later
//! SHOW_ parameters, with the Skybrush fork's names, indices, defaults and docs.

ap_rust_sdk::param_table! {
    pub struct Params, enum P;
    key = 112, prefix = "SHOW_";

    // @Param: SHOW_START_TIME
    // @DisplayName: Start time
    // @Description: Start time of drone show as a GPS time of week timestamp (sec), negative if unset. See also SHOW_START_MSEC.
    // @Range: -1 604799
    // @Increment: 1
    // @Units: s
    // @Volatile: True
    // @User: Standard
    StartTime = 1, "START_TIME", i32: -1;

    // @Param: SHOW_START_MSEC
    // @DisplayName: Start time, extra millisecond offset
    // @Description: Number of milliseconds to add to the start time of the show in the SHOW_START_TIME parameter.
    // @Range: 0 999
    // @Increment: 1
    // @Units: ms
    // @Volatile: True
    // @User: Standard
    StartMsec = 40, "START_MSEC", f32: 0.0;

    // @Param: SHOW_ORIGIN_LAT
    // @DisplayName: Show origin (latitude)
    // @Description: Latitude of the origin of the drone show coordinate system in 1e-7 degrees, zero if unset
    // @Range: -900000000 900000000
    // @Increment: 1
    // @User: Standard
    OriginLat = 2, "ORIGIN_LAT", i32: 0;

    // @Param: SHOW_ORIGIN_LNG
    // @DisplayName: Show origin (longitude)
    // @Description: Longitude of the origin of the drone show coordinate system in 1e-7 degrees, zero if unset
    // @Range: -1800000000 1800000000
    // @Increment: 1
    // @User: Standard
    OriginLng = 3, "ORIGIN_LNG", i32: 0;

    // @Param: SHOW_ORIGIN_AMSL
    // @DisplayName: Show origin (altitude)
    // @Description: AMSL altitude of the origin of the drone show coordinate system, -10000000 or smaller if unset
    // @Range: -10000000 10000000
    // @Increment: 1
    // @Units: mm
    // @User: Standard
    OriginAmsl = 12, "ORIGIN_AMSL", i32: -10000000;

    // @Param: SHOW_ORIENTATION
    // @DisplayName: Show orientation
    // @Description: Orientation of the X axis of the show coordinate system in CW direction relative to North, -1 if unset
    // @Range: -1 360
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    Orientation = 4, "ORIENTATION", f32: -1.0;

    // @Param: SHOW_START_AUTH
    // @DisplayName: Authorization to start
    // @Description: Whether the drone is authorized to start the show
    // @Values: 0:Revoked, 1:Granted, 2:Granted in rehearsal mode, 3:Granted with lights only
    // @Volatile: True
    // @User: Standard
    StartAuth = 5, "START_AUTH", f32: 0.0;

    // @Param: SHOW_LED0_TYPE
    // @DisplayName: Assignment of LED channel 0 to a LED output type
    // @Description: Specifies where the output of the main LED light track of the show should be sent
    // @Values: 0:Off, 1:MAVLink, 2:NeoPixel, 3:ProfiLED, 4:Debug, 6:Servo, 7:I2C RGB, 8:Inverted servo, 9:UART (WGDrones), 10:NeoPixel RGBW, 11:I2C RGBW, 12:Notification LED, 13:Servo with limits (off=0), 14:Servo with limits
    // @User: Advanced
    Led0Type = 6, "LED0_TYPE", f32: 0.0;

    // @Param: SHOW_LED0_CHAN
    // @DisplayName: PWM, MAVLink or UART channel to use for the LED output
    // @Description: PWM channel to use for the LED output (1-based) if the LED type is "NeoPixel", "ProfiLED" or "NeoPixel RGBW"; the MAVLink channel to use if the LED type is "MAVLink"; the I2C address of the LED if the LED type is "I2C"; the UART index if the LED type is "WGDrones". For UART-driven LEDs, you also need to set the baud rate in SERIALx_BAUD and set SERIALx_PROTOCOL to "Scripting" to ensure that the UART is initialized.
    // @User: Advanced
    Led0Chan = 8, "LED0_CHAN", f32: 0.0;

    // @Param: SHOW_LED0_COUNT
    // @DisplayName: Number of individual LEDs on a LED channel
    // @Description: For NeoPixel or ProfiLED LED strips: specifies how many LEDs there are on the strip. For I2C LEDs: specifies the index of the bus that the LED is attached to.
    // @User: Advanced
    Led0Count = 7, "LED0_COUNT", f32: 16.0;

    // @Param: SHOW_LED0_GAMMA
    // @DisplayName: Gamma correction factor for the LED channel
    // @Description: Specifies the exponent of the gamma correction to apply on the RGB values of this channel. Set this to 1 if you do not want to use gamma correction or if the LEDs perform gamma correction on their own; otherwise typical values are in the range 2.2 to 2.8 for LEDs. Set a value that provides an approximately linear perceived brightness response when the LEDs are faded from full black to full white.
    // @Range: 1 5
    // @Increment: 0.1
    // @User: Advanced
    Led0Gamma = 19, "LED0_GAMMA", f32: 1.0;

    // @Param: SHOW_LED0_WTEMP
    // @DisplayName: Color temperature of the white LED of the channel
    // @Description: Specifies the color temperature of the white LED of the channel if the channel makes use of an additional white LED. Set to zero if you don't know the color temperature of the white LED or if there is no white LED.
    // @Range: 0 15000
    // @Increment: 100
    // @User: Advanced
    Led0Wtemp = 23, "LED0_WTEMP", f32: 0.0;

    // @Param: SHOW_LED0_MINBRI
    // @DisplayName: Minimum LED brightness threshold
    // @Description: Minimum brightness threshold (as a ratio 0.0-1.0) below which LED is turned off completely
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Advanced
    Led0Minbri = 32, "LED0_MINBRI", f32: 0.0;

    // @Param: SHOW_MODE_BOOT
    // @DisplayName: Conditions for entering show mode
    // @Description: Bitfield that specifies when the drone should switch to show mode automatically
    // @Bitmask: 0:At boot,1:When authorized
    // @User: Standard
    ModeBoot = 9, "MODE_BOOT", f32: 2.0;

    // @Param: SHOW_PRE_LIGHTS
    // @DisplayName: Brightness of preflight check related lights
    // @Description: Controls the brightness of light signals on the drone that are used to report status information when the drone is on the ground. 0 is off, 1 is low brightness (25%), 2 is medium brightness (50%), 3 is full brightness (100%). Values greater than 3 and less than or equal to 100 are interpreted as percentages. Negative values are treated as zero.
    // @Range: 0 100
    // @Increment: 1
    // @User: Standard
    PreLights = 10, "PRE_LIGHTS", f32: 2.0;

    // @Param: SHOW_CTRL_MODE
    // @DisplayName: Flags to configure the show position control algorithm
    // @Description: Controls various aspects of the position control algorithm built into the firmware
    // @Bitmask: 0:Enable velocity control,1:Unused (was acceleration control)
    // @User: Advanced
    CtrlMode = 11, "CTRL_MODE", f32: 1.0;

    // @Param: SHOW_GROUP
    // @DisplayName: Show group index
    // @Description: Index of the group that this drone belongs to
    // @Range: 0 63
    // @Increment: 1
    // @User: Standard
    Group = 13, "GROUP", f32: 0.0;

    // @Param: SHOW_CTRL_RATE
    // @DisplayName: Target update rate
    // @Description: Update rate of the target position and velocity during the show
    // @Range: 1 50
    // @Increment: 1
    // @Units: Hz
    // @User: Advanced
    CtrlRate = 14, "CTRL_RATE", f32: 10.0;

    // @Param: SHOW_VEL_FF_GAIN
    // @DisplayName: Velocity feed-forward gain
    // @Description: Multiplier used when mixing the desired velocity of the drone into the velocity target of the position controller. Lower values will result in more relaxed/stable behaviour, at the price of a smoothed trajectory with rounded corners, less accuracy and more lag behind desired position. Higher values will decrease lag, make trajectory following more accurate, sharp and aggressive, but might increase overshoot at corners and decrease stability if general attitude control is not tuned well.
    // @Range: 0 1
    // @Increment: 0.1
    // @User: Advanced
    VelFfGain = 16, "VEL_FF_GAIN", f32: 1.0;

    // @Param: SHOW_TAKEOFF_ALT
    // @DisplayName: Takeoff altitude
    // @Description: Altitude above current position to take off to when starting the show
    // @Range: 0 5
    // @Increment: 0.1
    // @Units: m
    // @User: Advanced
    // @RebootRequired: True
    TakeoffAlt = 17, "TAKEOFF_ALT", f32: 2.5;

    // @Param: SHOW_TAKEOFF_ERR
    // @DisplayName: Maximum placement error in XY direction
    // @Description: Maximum placement error that we tolerate before takeoff, in meters. Zero to turn off XY placement accuracy checks.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    TakeoffErr = 15, "TAKEOFF_ERR", f32: 3.0;

    // @Param: SHOW_SYNC_MODE
    // @DisplayName: Time synchronization mode
    // @Description: Time synchronization mode to use when starting the show
    // @Values: 0:Countdown, 1:GPS time
    // @User: Advanced
    // @RebootRequired: True
    SyncMode = 18, "SYNC_MODE", f32: 1.0;

    // @Param: SHOW_HFENCE_EN
    // @DisplayName: Hard fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the hard fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    HfenceEn = 20, "HFENCE_EN", f32: 0.0;

    // @Param: SHOW_HFENCE_DIST
    // @DisplayName: Hard fence minimum distance
    // @Description: Minimum distance that the hard fence extends beyond the standard geofence
    // @Units: m
    // @Range: 1 1000
    // @User: Standard
    HfenceDist = 21, "HFENCE_DIST", f32: 25.0;

    // @Param: SHOW_HFENCE_TO
    // @DisplayName: Hard fence timeout
    // @Description: Minimum time that the vehicle needs to spend outside the hard geofence to trigger a motor shutdown
    // @Units: s
    // @Range: 0 120
    // @User: Standard
    HfenceTo = 22, "HFENCE_TO", f32: 5.0;

    // @Param: SHOW_MAX_XY_ERR
    // @DisplayName: Maximum allowed drift in XY direction during show
    // @Description: Maximum allowed drift from planned trajectory in XY plane that we tolerate during show, in meters. Zero to turn off XY checks. Drifts exceeding the threshold will trigger a status flag but do not abort the show.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    MaxXyErr = 24, "MAX_XY_ERR", f32: 3.0;

    // @Param: SHOW_MAX_Z_ERR
    // @DisplayName: Maximum allowed drift in Z direction during show
    // @Description: Maximum allowed drift from planned trajectory in Z direction that we tolerate during show, in meters. Zero to turn off Z checks. Drifts exceeding the threshold will trigger a status flag but do not abort the show.
    // @Range: 0 20
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    MaxZErr = 25, "MAX_Z_ERR", f32: 3.0;

    // @Param: SHOW_POST_ACTION
    // @DisplayName: Action to perform when the show trajectory ends
    // @Description: Specifies what to do at the end of the show trajectory
    // @Values: 3:RTL if above takeoff position and land otherwise,2:RTL unconditionally,1:Land,0:Loiter (position hold)
    // @User: Advanced
    PostAction = 26, "POST_ACTION", f32: 1.0;

    // @Param: SHOW_BFENCE_EN
    // @DisplayName: Bubble fence enable/disable
    // @Description: Allows you to enable (1) or disable (0) the bubble fence functionality
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    BfenceEn = 27, "BFENCE_EN", f32: 1.0;

    // @Param: SHOW_BFENCE_DXY
    // @DisplayName: Bubble fence XY distance
    // @Description: Maximum allowed deviation from the flight path in the XY plane. Set to zero to disable XY checks.
    // @Units: m
    // @Range: 0 1000
    // @User: Standard
    BfenceDxy = 28, "BFENCE_DXY", f32: 10.0;

    // @Param: SHOW_BFENCE_DZ
    // @DisplayName: Bubble fence Z distance
    // @Description: Maximum allowed deviation from the flight path along the Z axis. Set to zero to disable Z checks.
    // @Units: m
    // @Range: 0 1000
    // @User: Standard
    BfenceDz = 29, "BFENCE_DZ", f32: 10.0;

    // @Param: SHOW_BFENCE_TO
    // @DisplayName: Bubble fence timeout
    // @Description: Minimum time that the bubble fence needs to be breached to trigger the associated action
    // @Units: s
    // @Range: 0 120
    // @User: Standard
    BfenceTo = 30, "BFENCE_TO", f32: 5.0;

    // @Param: SHOW_BFENCE_ACT
    // @DisplayName: Bubble fence action
    // @Description: Action to take when the bubble fence is breached beyond the timeout
    // @Values: 0:None, 1:Report only, 2:Flash lights, 3:RTL, 4:Land, 5:Disarm
    // @User: Standard
    BfenceAct = 31, "BFENCE_ACT", f32: 1.0;

    // @Param: SHOW_PYRO_MINALT
    // @DisplayName: Minimum altitude for pyro events
    // @Description: Minimum altitude above the takeoff position required for triggering pyrotechnic effects
    // @User: Standard
    PyroMinalt = 33, "PYRO_MINALT", f32: 20.0;

    // @Param: SHOW_PYRO_TYPE
    // @DisplayName: Pyrotechnic device type
    // @Description: Specifies the type of the pyrotechnic device that is used to trigger pyrotechnic effects during the show. For Cobra devices, set the protocol of the corresponding UART to "Volz servo out".
    // @Values: 0:None, 1:Debug, 2:SingleServo, 3:MultipleServos, 4:Cobra, 5:Relay
    // @User: Advanced
    PyroType = 34, "PYRO_TYPE", f32: 0.0;

    // @Param: SHOW_PYRO_TCOMP
    // @DisplayName: Time compensation for pyrotechnic events
    // @Description: Time compensation to apply to the pyrotechnic events, in milliseconds. This is used to compensate for the delay between the moment when the command is sent and the moment when the pyrotechnic device actually fires.
    // @User: Advanced
    PyroTcomp = 35, "PYRO_TCOMP", i32: 0;

    // @Param: SHOW_PYRO_ITIME
    // @DisplayName: Ignition duration for pyrotechnic events
    // @Description: Duration for which the pyrotechnic device is ignited, in milliseconds. Zero means forever. May be ignored by the pyrotechnic device if it manages ignition time on its own.
    // @User: Advanced
    PyroItime = 36, "PYRO_ITIME", i32: 3000;

    // @Param: SHOW_OPTIONS
    // @DisplayName: Show flight mode options
    // @Description: General options related to the show flight mode
    // @Bitmask: 0:Disable failsafe lights, 1:Attempt to adjust the landing segment of circular trajectories to land at the exact takeoff position
    // @User: Advanced
    Options = 37, "OPTIONS", f32: 0.0;

    // @Param: SHOW_TEL_PROFILE
    // @DisplayName: Telemetry profile
    // @Description: Specifies the telemetry profile to use for the show. This controls which telemetry messages are sent with what frequency.
    // @Values: 0:Standard, 1:Compact
    // @User: Advanced
    // @RebootRequired: True
    TelProfile = 38, "TEL_PROFILE", f32: 0.0;

    // @Param: SHOW_MAX_ESC_ERR
    // @DisplayName: Maximum allowed ESC error rate percentage
    // @Description: Maximum allowed ESC error rate percentage on any of the ESCs. Requires ESC telemetry. Should be in the range 0-100. Negative values disable the check. Error rates exceeding the threshold will trigger a status flag but do not abort the show.
    // @Range: -1 100
    // @Increment: 1
    // @Units: %
    // @User: Advanced
    MaxEscErr = 39, "MAX_ESC_ERR", f32: -1.0;
}

/// Vehicle parameters the show reads, bound once like a Lua script's
/// bind_param() so later reads need no name search.
pub struct VehicleParams<'a, 'h> {
    pub fs_ekf_action: Option<ap_rust_sdk::Parameter<'a, 'h>>,
    pub disarm_delay: Option<ap_rust_sdk::Parameter<'a, 'h>>,
    pub land_speed: Option<ap_rust_sdk::Parameter<'a, 'h>>,
    pub wp_speed_up: Option<ap_rust_sdk::Parameter<'a, 'h>>,
    pub wp_accel_z: Option<ap_rust_sdk::Parameter<'a, 'h>>,
    pub spool_time: Option<ap_rust_sdk::Parameter<'a, 'h>>,
}

impl<'a, 'h> VehicleParams<'a, 'h> {
    pub fn bind(c: &'a ap_rust_sdk::Context<'h>) -> ap_rust_sdk::Result<Self> {
        Ok(Self {
            fs_ekf_action: c.parameter("FS_EKF_ACTION")?,
            disarm_delay: c.parameter("DISARM_DELAY")?,
            land_speed: c.parameter("LAND_SPD_MS")?,
            wp_speed_up: c.parameter("WP_SPD_UP")?,
            wp_accel_z: c.parameter("WP_ACC_Z")?,
            spool_time: c.parameter("MOT_SPOOL_TIME")?,
        })
    }
}

/// Value of a bound vehicle parameter, None if it does not exist.
pub fn value(p: &Option<ap_rust_sdk::Parameter<'_, '_>>) -> Option<f32> {
    p.as_ref().and_then(|p| p.get().ok().flatten())
}

impl Params<'_, '_> {
    /// The fork treats start time and authorization as volatile.
    pub fn reset_volatile(&mut self) -> ap_rust_sdk::Result {
        self.set_i32(P::StartTime, -1)?;
        self.set(P::StartMsec, 0.0)?;
        self.set(P::StartAuth, 0.0)
    }
}
