--[[
    adsb_send.lua: send a simulated ADSB vehicle flying a 500m radius circle

    This script demonstrates how to send ADSB_VEHICLE MAVLink messages using
    -- mavlink:send_chan() with manual struct packing (no MAVLink module required)
--]]

-- initialise mavlink (no rx needed, only tx)
mavlink:init(1, 0)

local ADSB_VEHICLE_MSGID = 246

-- ADSB flags bitmask values
local ADSB_FLAGS_VALID_COORDS              = 1
local ADSB_FLAGS_VALID_ALTITUDE            = 2
local ADSB_FLAGS_VALID_HEADING             = 4
local ADSB_FLAGS_VALID_VELOCITY            = 8
local ADSB_FLAGS_VALID_CALLSIGN            = 16
local ADSB_FLAGS_SIMULATED                 = 64
local ADSB_FLAGS_VERTICAL_VELOCITY_VALID   = 128

--
-- Send an ADSB_VEHICLE message on all MAVLink channels
--
-- lat_deg      : latitude in degrees
-- lng_deg      : longitude in degrees
-- alt_amsl_m   : altitude AMSL in metres
-- heading_deg  : course over ground in degrees (0=north, clockwise)
-- speed_mps    : horizontal speed in m/s
-- vspeed_mps   : vertical speed in m/s (positive = up)
-- icao_address : 24-bit ICAO address
-- callsign     : up to 8 character string
-- squawk       : squawk code in decimal (e.g. 1200 for VFR)
-- emitter_type : ADSB_EMITTER_TYPE enum value
-- flags        : ADSB_FLAGS bitmask
--
local function send_ADSB_VEHICLE(lat_deg, lng_deg, alt_amsl_m, heading_deg,
                                 speed_mps, vspeed_mps, icao_address,
                                 callsign, squawk, emitter_type, flags)
    -- pad or truncate callsign to exactly 9 bytes (8 chars + null)
    local cs = callsign .. string.rep("\0", 9)
    cs = string.sub(cs, 1, 9)

    -- MAVLink wire order: fields sorted by type size descending, then XML order
    --   uint32: ICAO_address, lat, lon, altitude          (4 x I4/i4)
    --   uint16: heading, hor_velocity, ver_velocity,      (5 x I2/i2)
    --           flags, squawk
    --   uint8:  altitude_type                             (1 x B)
    --   char[9]: callsign                                 (c9)
    --   uint8:  emitter_type, tslc                        (2 x B)
    local payload = string.pack("<I4i4i4i4 I2I2i2I2I2 Bc9BB",
        icao_address,
        math.floor(lat_deg * 1e7),          -- degE7
        math.floor(lng_deg * 1e7),          -- degE7
        math.floor(alt_amsl_m * 1000),      -- mm
        math.floor(heading_deg * 100),      -- cdeg
        math.floor(speed_mps * 100),        -- cm/s
        math.floor(vspeed_mps * 100),       -- cm/s
        flags,
        squawk,
        0,                                  -- altitude_type: PRESSURE_QNH
        cs,
        emitter_type,
        0)                                  -- tslc

    for chan = 0, 5 do
        mavlink:send_chan(chan, ADSB_VEHICLE_MSGID, payload)
    end
end

-- circle parameters
local RADIUS_M   = 500       -- circle radius in metres
local ALT_AMSL_M = 100       -- altitude AMSL in metres
local SPEED_MS   = 50         -- ground speed in m/s
local ICAO_ADDR  = 0xFABCDE  -- fake ICAO address (0xF00000-0xFFFFFF is unallocated)
local CALLSIGN   = "SIM12345"
local UPDATE_HZ  = 5          -- send rate

local FLAGS = ADSB_FLAGS_VALID_COORDS + ADSB_FLAGS_VALID_ALTITUDE
            + ADSB_FLAGS_VALID_HEADING + ADSB_FLAGS_VALID_VELOCITY
            + ADSB_FLAGS_VALID_CALLSIGN + ADSB_FLAGS_SIMULATED
            + ADSB_FLAGS_VERTICAL_VELOCITY_VALID

-- angular rate in rad/s
local OMEGA = SPEED_MS / RADIUS_M
local angle_rad = 0

local function update()
    local home = ahrs:get_home()
    if not home then
        return update, 1000
    end

    local center_lat = home:lat()
    local center_lng = home:lng()
    local alt_m = ALT_AMSL_M + home:alt()*0.01

    local dt = 1.0 / UPDATE_HZ
    angle_rad = angle_rad + OMEGA * dt
    if angle_rad > 2 * math.pi then
        angle_rad = angle_rad - 2 * math.pi
    end

    -- compute position on circle
    local pos = Location()
    pos:lat(center_lat)
    pos:lng(center_lng)
    pos:alt(math.floor(alt_m * 100))
    pos:offset_bearing(math.deg(angle_rad), RADIUS_M)

    -- heading is tangent to circle (90 degrees ahead of radial)
    local heading_deg = math.deg(angle_rad) + 90
    if heading_deg >= 360 then
        heading_deg = heading_deg - 360
    end

    send_ADSB_VEHICLE(
        pos:lat() / 1e7,    -- back to degrees
        pos:lng() / 1e7,    -- back to degrees
        alt_m,
        heading_deg,
        SPEED_MS,
        0,                  -- no vertical speed
        ICAO_ADDR,
        CALLSIGN,
        1200,               -- VFR squawk
        1,                  -- ADSB_EMITTER_TYPE_LIGHT
        FLAGS
    )

    return update, math.floor(1000 / UPDATE_HZ)
end

return update()
