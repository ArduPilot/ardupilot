--[[
    Radio link simulator example

    This script provides some examples of how to use the Sn_PKT_LOSS and
    Sn_DELAY parameters to simulate different types of radio links to the
    aircaft.

    1. Direct link
      - This is a direct RF link between the aircraft and the ground station.
        There is no delay, but packet loss increases as the aircraft moves
        further away from the ground station.
    2. Cellular link
      - This simulates a cellular link with a 200ms delay. At random intervals,
        there is 100% loss on the link for a period of time to simulate
        something like tower switching. The packet loss is independent of the
        aircraft location (i.e., the script assumes perfect coverage).
    3. Satellite link
      - This simulates a link with a constant delay, but is otherwise perfect.
        The stream rates are set to 1Hz for all streams.

    All three of these links are simulated at the same time. Set serial ports
    1-3 up as MAVLink and connect to port 5760 for direct link, 5762 for
    cellular, and 5763 for satellite.
--]]

-- bind a parameter to a variable
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

local DIRECT_PACKET_LOSS = bind_param("SIM_S0_PKT_LOSS")
local DIRECT_DELAY = bind_param("SIM_S0_DELAY")
local LTE_PACKET_LOSS = bind_param("SIM_S1_PKT_LOSS")
local LTE_DELAY = bind_param("SIM_S1_DELAY")
local SATCOM_DELAY = bind_param("SIM_S2_DELAY")

--[[
    Constants for direct radio link

    Loss is modelled (too) simply as a piecewise linear function of discance
    from the home position. The loss percentage grows to DIRECT_LINK_RANGE1_LOSS
    at a distance of DIRECT_LINK_RANGE1, and then grows linearly to 100% at a
    distance of DIRECT_LINK_MAX_RANGE.
--]]
DIRECT_DELAY:set(0)                -- No delay
local DIRECT_LINK_RANGE1 = 5000    -- Range (m) at which the link rapidly degrades
local DIRECT_LINK_RANGE1_LOSS = 2  -- Packet loss (%) at RANGE1
local DIRECT_LINK_MAX_RANGE = 8000 -- Range (m) at which the link is completely lost

-- Constants for cellular link
LTE_DELAY:set(0.2)             -- 200ms delay
local LTE_SHORT_INTERVAL = 120 -- Time (s), on average, between short loss events
local LTE_SHORT_DURATION = 5   -- Time (s) that short loss events last
local LTE_LONG_INTERVAL = 600  -- Time (s), on average, between long loss events
local LTE_LONG_DURATION = 25   -- Time (s) that long loss events last

-- Constants for satellite link
SATCOM_DELAY:set(2) -- 2s delay

-- Force satcom serial options to ignore GCS stream rate requests
local SATCOM_OPTIONS = bind_param("SERIAL2_OPTIONS")
SATCOM_OPTIONS:set_and_save(SATCOM_OPTIONS:get() | 0x1000)

-- Set satcom stream rates
-- (The 2 here is the number of the MAVLink stream, not the serial port number;
-- in this case, that happens to be the same, because Serials 0-2 are MAVLink,
-- but watch out)
param:set_and_save("SR2_RAW_SENS", 1)
param:set_and_save("SR2_EXT_STAT", 1)
param:set_and_save("SR2_RC_CHAN", 1)
param:set_and_save("SR2_RAW_CTRL", 1)
param:set_and_save("SR2_POSITION", 1)
param:set_and_save("SR2_EXTRA1", 1)
param:set_and_save("SR2_EXTRA2", 1)
param:set_and_save("SR2_EXTRA3", 1)
param:set_and_save("SR2_PARAMS", 0) -- Don't stream params
param:set_and_save("SR2_ADSB", 0)   -- Don't stream ADSB

local MAV_SEVERITY_INFO = 6

-- Function to update packet loss of the direct link based on distance
local function update_direct_link_loss()
    local distance = 0
    local home_pos = ahrs:get_relative_position_NED_home()
    if home_pos then
        distance = home_pos:length()
    end
    local loss
    if distance < DIRECT_LINK_RANGE1 then
        loss = (distance / DIRECT_LINK_RANGE1) * DIRECT_LINK_RANGE1_LOSS
    elseif distance < DIRECT_LINK_MAX_RANGE then
        loss = DIRECT_LINK_RANGE1_LOSS +
            ((distance - DIRECT_LINK_RANGE1) / (DIRECT_LINK_MAX_RANGE - DIRECT_LINK_RANGE1)) *
            (100 - DIRECT_LINK_RANGE1_LOSS)
    else
        loss = 100
    end
    DIRECT_PACKET_LOSS:set(loss)
end

-- Timers for packet loss events
local next_short_loss_time = -math.log(1.0 - math.random()) * LTE_SHORT_INTERVAL
local next_long_loss_time = -math.log(1.0 - math.random()) * LTE_LONG_INTERVAL
local short_loss_timer = 0
local long_loss_timer = 0

local function update_lte_loss_events()
    -- Decrement timers
    next_short_loss_time = next_short_loss_time - 1
    next_long_loss_time = next_long_loss_time - 1

    -- Handle short loss event
    if next_short_loss_time <= 0 then
        next_short_loss_time = -math.log(1.0 - math.random()) * LTE_SHORT_INTERVAL
        -- Print the time of the next loss event
        gcs:send_text(MAV_SEVERITY_INFO, string.format("LTE short packet loss. Next: %.0f", next_short_loss_time))
        LTE_PACKET_LOSS:set(100)
        short_loss_timer = LTE_SHORT_DURATION
    end

    -- Handle long loss event
    if next_long_loss_time <= 0 then
        next_long_loss_time = -math.log(1.0 - math.random()) * LTE_LONG_INTERVAL
        gcs:send_text(MAV_SEVERITY_INFO, string.format("LTE long packet loss. Next: %.0f", next_long_loss_time))
        LTE_PACKET_LOSS:set(100)
        long_loss_timer = LTE_LONG_DURATION
    end

    -- Reset packet loss after duration
    if short_loss_timer > 0 then
        short_loss_timer = short_loss_timer - 1
        if short_loss_timer <= 0 then
            LTE_PACKET_LOSS:set(0)
        end
    end

    if long_loss_timer > 0 then
        long_loss_timer = long_loss_timer - 1
        if long_loss_timer <= 0 then
            LTE_PACKET_LOSS:set(0)
        end
    end
end

local function update()
    update_direct_link_loss()
    update_lte_loss_events()
end

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(0, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 5000
    end
    return protected_wrapper, 1000
end

gcs:send_text(MAV_SEVERITY_INFO, "Loaded radio simulator")

-- start running update loop
return protected_wrapper()
