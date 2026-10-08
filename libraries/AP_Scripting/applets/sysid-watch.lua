--[[
   Periodically report the MAVLink system IDs this vehicle can see, and
   optionally collect receive statistics for a single system ID.

   Systems are discovered from the HEARTBEAT messages arriving on any
   MAVLink channel.  Setting SYSW_STATS_ID to a system ID additionally
   counts a fixed set of common telemetry messages from that system.
--]]

-- load mavlink message definitions from modules/MAVLink directory
local mavlink_msgs = require("MAVLink/mavlink_msgs")

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 110
local PARAM_TABLE_PREFIX = "SYSW_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 4), 'could not add param table')

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', PARAM_TABLE_PREFIX .. name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

--[[
  // @Param: SYSW_ENABLE
  // @DisplayName: System ID watch enable
  // @Description: Enables periodic reporting of the MAVLink system IDs seen by this vehicle
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SYSW_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: SYSW_PERIOD
  // @DisplayName: System ID watch report period
  // @Description: Time between reports of the system IDs seen, and of the statistics for SYSW_STATS_ID
  // @Units: s
  // @Range: 1 600
  // @User: Standard
--]]
local SYSW_PERIOD = bind_add_param("PERIOD", 2, 10)

--[[
  // @Param: SYSW_TIMEOUT
  // @DisplayName: System ID watch timeout
  // @Description: A system or component is no longer reported once no HEARTBEAT has been received from it for this long
  // @Units: s
  // @Range: 1 600
  // @User: Standard
--]]
local SYSW_TIMEOUT = bind_add_param("TIMEOUT", 3, 10)

--[[
  // @Param: SYSW_STATS_ID
  // @DisplayName: System ID watch statistics system ID
  // @Description: MAVLink system ID to collect and report received packet statistics for. Zero disables statistics collection. Changing this value resets the statistics.
  // @Range: 0 16777215
  // @User: Standard
--]]
local SYSW_STATS_ID = bind_add_param("STATS_ID", 4, 0)

-- statistics are only gathered for messages registered with the
-- scripting MAVLink interface, which accepts at most 25 IDs.
-- HEARTBEAT is always registered; the others only once statistics
-- are first requested
local HEARTBEAT_ID = 0
local MSG_NAMES = {
    [0] = "HEARTBEAT",
    [1] = "SYS_STATUS",
    [2] = "SYSTEM_TIME",
    [22] = "PARAM_VALUE",
    [24] = "GPS_RAW_INT",
    [27] = "RAW_IMU",
    [29] = "SCALED_PRESSURE",
    [30] = "ATTITUDE",
    [33] = "GLOBAL_POSITION_INT",
    [36] = "SERVO_OUTPUT_RAW",
    [42] = "MISSION_CURRENT",
    [62] = "NAV_CONTROLLER_OUTPUT",
    [65] = "RC_CHANNELS",
    [74] = "VFR_HUD",
    [75] = "COMMAND_INT",
    [76] = "COMMAND_LONG",
    [77] = "COMMAND_ACK",
    [109] = "RADIO_STATUS",
    [111] = "TIMESYNC",
    [147] = "BATTERY_STATUS",
    [241] = "VIBRATION",
    [242] = "HOME_POSITION",
    [245] = "EXTENDED_SYS_STATE",
    [253] = "STATUSTEXT",
}
local HEARTBEAT_MAP = {[HEARTBEAT_ID] = "HEARTBEAT"}

local RX_QUEUE_LEN = 20
-- CRC-checking a HEARTBEAT is expensive, so handling a full queue in
-- one update could exceed SCR_VM_I_COUNT
local MAX_MSGS_PER_UPDATE = 8
local MAX_LINE_LEN = 50   -- STATUSTEXT text length

local initialised = false
local enabled = false
local stats_msgs_registered = false
local foreign_msg_warned = false

-- seen[sysid] = {last_ms=, comps={[compid]=last_ms}, chans={[chan]=last_ms}}
local seen = {}

local last_report_ms = 0
local queue_full_count = 0
-- when the queue was last found empty, and how many of the messages
-- handled since were already queued by then
local drain_start_ms = nil
local drain_queued = 0

local function now_ms()
    return millis():toint()
end

local function new_stats()
    return {
        start_ms = now_ms(),
        period_start_ms = now_ms(),
        total = 0,
        period_total = 0,
        msg_period = {},
        bad_hb = 0,
        comps = {},
        chans = {},
        last_ms = nil,
        last_hb_ms = nil,
        hb_gap_max_ms = 0,
    }
end

-- statistics for SYSW_STATS_ID, only gathered while stats_sysid is non-zero
local stats = new_stats()
local stats_sysid = 0

local function reset_stats(sysid)
    stats_sysid = sysid
    stats = new_stats()
end

-- send a list of strings as STATUSTEXTs, packing as many as fit on each line
local function send_wrapped(prefix, items)
    local line = prefix
    for _, item in ipairs(items) do
        if #line + 1 + #item > MAX_LINE_LEN and line ~= prefix then
            gcs:send_text(MAV_SEVERITY.INFO, line)
            line = prefix
        end
        line = line .. " " .. item
    end
    if line ~= prefix then
        gcs:send_text(MAV_SEVERITY.INFO, line)
    end
end

local function sorted_keys(t)
    local keys = {}
    for k in pairs(t) do
        keys[#keys+1] = k
    end
    table.sort(keys)
    return keys
end

local function update_stats(header, chan, rx_ms, valid)
    if header.msgid == HEARTBEAT_ID and not valid then
        stats.bad_hb = stats.bad_hb + 1
        return
    end
    stats.total = stats.total + 1
    stats.period_total = stats.period_total + 1
    stats.msg_period[header.msgid] = (stats.msg_period[header.msgid] or 0) + 1
    stats.comps[header.compid] = (stats.comps[header.compid] or 0) + 1
    stats.chans[chan] = true
    stats.last_ms = rx_ms
    if header.msgid == HEARTBEAT_ID and header.compid == 1 then
        -- track dropouts using the autopilot's 1Hz heartbeat
        if stats.last_hb_ms ~= nil then
            stats.hb_gap_max_ms = math.max(stats.hb_gap_max_ms, rx_ms - stats.last_hb_ms)
        end
        stats.last_hb_ms = rx_ms
    end
end

local function handle_message(msg, chan, rx_ms)
    local header = mavlink_msgs.decode_header(msg)
    if header == nil then
        return
    end
    if MSG_NAMES[header.msgid] == nil then
        -- the receive queue is shared between all scripts, so we are
        -- consuming messages another script registered for
        if not foreign_msg_warned then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("SYSW: other script receiving MAVLink (msgid %u)", header.msgid))
            foreign_msg_warned = true
        end
        return
    end

    local valid = true
    if header.msgid == HEARTBEAT_ID then
        -- only trust heartbeats with a good CRC to discover systems
        valid = mavlink_msgs.decode(msg, HEARTBEAT_MAP) ~= nil
        if valid then
            local s = seen[header.sysid]
            if s == nil then
                s = {comps = {}, chans = {}}
                seen[header.sysid] = s
            end
            s.last_ms = rx_ms
            s.comps[header.compid] = rx_ms
            s.chans[chan] = rx_ms
        end
    end

    if stats_sysid ~= 0 and header.sysid == stats_sysid then
        update_stats(header, chan, rx_ms, valid)
    end
end

local function expire_seen(now)
    local timeout_ms = math.max(SYSW_TIMEOUT:get(), 1) * 1000
    for sysid, s in pairs(seen) do
        if now - s.last_ms > timeout_ms then
            seen[sysid] = nil
        else
            for compid, t in pairs(s.comps) do
                if now - t > timeout_ms then
                    s.comps[compid] = nil
                end
            end
            for c, t in pairs(s.chans) do
                if now - t > timeout_ms then
                    s.chans[c] = nil
                end
            end
        end
    end
end

local function report_seen()
    local items = {}
    for _, sysid in ipairs(sorted_keys(seen)) do
        local s = seen[sysid]
        items[#items+1] = string.format("%d[%s]", sysid, table.concat(sorted_keys(s.comps), ","))
    end
    if #items == 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "SYSW: no systems seen")
        return
    end
    send_wrapped(string.format("SYSW: %d sys:", #items), items)
end

local function report_stats(now)
    local prefix = string.format("SYSW %d:", stats_sysid)
    if stats.last_ms == nil then
        gcs:send_text(MAV_SEVERITY.INFO, string.format("%s nothing received for %.0fs", prefix, (now - stats.start_ms) * 0.001))
        return
    end

    local period_s = math.max(now - stats.period_start_ms, 1) * 0.001
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s %.1fpkt/s total %d chan %s",
        prefix, stats.period_total / period_s, stats.total, table.concat(sorted_keys(stats.chans), ",")))
    gcs:send_text(MAV_SEVERITY.INFO, string.format("%s last %.1fs ago HB gap %.1fs badHB %d",
        prefix, (now - stats.last_ms) * 0.001, stats.hb_gap_max_ms * 0.001, stats.bad_hb))

    local comps = {}
    for _, compid in ipairs(sorted_keys(stats.comps)) do
        comps[#comps+1] = string.format("%d:%d", compid, stats.comps[compid])
    end
    send_wrapped(prefix .. " comp", comps)

    local rates = {}
    for _, msgid in ipairs(sorted_keys(stats.msg_period)) do
        rates[#rates+1] = string.format("%s=%.1f", MSG_NAMES[msgid], stats.msg_period[msgid] / period_s)
    end
    send_wrapped(prefix, rates)

    stats.period_start_ms = now
    stats.period_total = 0
    stats.msg_period = {}
    stats.hb_gap_max_ms = 0
end

-- registrations are shared between all scripts and sized by the
-- first script to call mavlink:init, so may already be exhausted.
-- Returns true if msgid will be received
local function register_rx_msgid(msgid)
    return pcall(mavlink.register_rx_msgid, mavlink, msgid)
end

local function init()
    mavlink:init(RX_QUEUE_LEN, 24)
    if not register_rx_msgid(HEARTBEAT_ID) then
        gcs:send_text(MAV_SEVERITY.ERROR, "SYSW: no MAVLink rx registrations free")
        return false
    end
    initialised = true
    return true
end

local function register_stats_msgs()
    local failed = 0
    for msgid in pairs(MSG_NAMES) do
        if msgid ~= HEARTBEAT_ID and not register_rx_msgid(msgid) then
            failed = failed + 1
        end
    end
    if failed > 0 then
        gcs:send_text(MAV_SEVERITY.WARNING, string.format("SYSW: %d stats msgs not registered", failed))
    end
    stats_msgs_registered = true
end

local function discard_queue()
    while mavlink:receive_chan() ~= nil do
    end
end

local function update()
    if SYSW_ENABLE:get() <= 0 then
        enabled = false
        if initialised then
            -- don't let messages build up while disabled
            discard_queue()
        end
        return update, 1000
    end
    if not initialised and not init() then
        -- stop running
        return
    end
    if not enabled then
        -- start afresh, with the first report a full period away
        enabled = true
        discard_queue()
        seen = {}
        stats_sysid = 0
        last_report_ms = now_ms()
    end

    local want_sysid = math.max(math.floor(SYSW_STATS_ID:get()), 0)
    if want_sysid ~= stats_sysid then
        if want_sysid ~= 0 and not stats_msgs_registered then
            register_stats_msgs()
        end
        reset_stats(want_sysid)
        -- next report covers a full period of the new statistics
        last_report_ms = now_ms()
    end

    -- drain the receive queue, a limited number of messages at a time
    if drain_start_ms == nil then
        drain_start_ms = now_ms()
    end
    local count = 0
    while count < MAX_MSGS_PER_UPDATE do
        local msg, chan, rx_time = mavlink:receive_chan()
        if msg == nil then
            break
        end
        local rx_ms = rx_time:toint()
        handle_message(msg, chan, rx_ms)
        if rx_ms <= drain_start_ms then
            drain_queued = drain_queued + 1
        end
        count = count + 1
    end
    if count == MAX_MSGS_PER_UPDATE then
        -- more may be waiting; come straight back for them
        return update, 1
    end
    if drain_queued >= RX_QUEUE_LEN then
        -- the queue may have overflowed and messages been dropped
        queue_full_count = queue_full_count + 1
    end
    drain_start_ms = nil
    drain_queued = 0

    local now = now_ms()
    if now - last_report_ms >= math.max(SYSW_PERIOD:get(), 1) * 1000 then
        last_report_ms = now
        expire_seen(now)
        report_seen()
        if stats_sysid ~= 0 then
            report_stats(now)
        end
        if queue_full_count > 0 then
            gcs:send_text(MAV_SEVERITY.WARNING, string.format("SYSW: rx queue full %d times", queue_full_count))
            queue_full_count = 0
        end
    end

    -- drain faster while gathering statistics as more messages are queued
    if stats_sysid ~= 0 then
        return update, 20
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "SYSW: sysid-watch loaded")

-- initialise after every script's top level has run, so the outcome
-- does not depend on the order in which scripts call mavlink:init
return update, 1000
