--[[
    update and log battery tag information from BatteryTag periph nodes
--]]

local GLOBALTIME_ID = 344
local GLOBALTIME_SIGNATURE = uint64_t(0xA5517744, 0x8A490F33)

local BATTERYTAG_ID = 20500
local BATTERYTAG_SIGNATURE = uint64_t(0x4A5A9B42, 0x099F73E1)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 49
PARAM_TABLE_PREFIX = "BTAG_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'BatteryTag: could not add param table')

--[[
  // @Param: BTAG_ENABLE
  // @DisplayName: enable battery info support
  // @Description: enable battery info support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local BTAG_ENABLE = bind_add_param('ENABLE',  1, 1)

--[[
  // @Param: BTAG_MAX_CYCLES
  // @DisplayName: max battery cycles
  // @Description: max battery cycles for arming
  // @Range: 0 10000
  // @User: Standard
--]]
local BTAG_MAX_CYCLES = bind_add_param('MAX_CYCLES',  2, 400)

--[[
  // @Param: BTAG_CUR_CYCLES
  // @DisplayName: current battery cycles
  // @Description: this is the highest value for battery cycles for all connected batteries
  // @Range: 0 10000
  // @User: Advanced
--]]
local BTAG_CUR_CYCLES = bind_add_param('CUR_CYCLES',  3, 0)

if BTAG_ENABLE:get() == 0 then
    return
end

-- a handle for receiving BatteryTag messages
local batterytag_handles = {
    DroneCAN_Handle(0, BATTERYTAG_SIGNATURE, BATTERYTAG_ID),
    DroneCAN_Handle(1, BATTERYTAG_SIGNATURE, BATTERYTAG_ID)
}
for bus, _ in pairs(batterytag_handles) do
    batterytag_handles[bus]:subscribe()
end

-- a handle for sending GlobalTime messages
local globaltime_handles = {
    DroneCAN_Handle(0, GLOBALTIME_SIGNATURE, GLOBALTIME_ID),
    DroneCAN_Handle(1, GLOBALTIME_SIGNATURE, GLOBALTIME_ID)
}

-- ID for an arming check
local auth_id = arming:get_aux_auth_id()

local highest_cycles = 0

local node_cycles = {}

local gcs_connect_time = nil
local sent_report = false
local have_tag = false

-- report battery tags to GCS at 30s after first GCS connection
local GCS_REPORT_TIME_S = 30

--[[
    check for BatteryTag messages
--]]
local function check_batterytag(bus)
    local payload, nodeid = batterytag_handles[bus]:check_message()
    if not payload then
        return
    end
    local serial_num, num_cycles, arm_hours, capacity, first_use, last_arm = string.unpack("IIfIII", payload)
    if not serial_num then
        return
    end

    have_tag = true

    if num_cycles > highest_cycles then
        highest_cycles = num_cycles
        BTAG_CUR_CYCLES:set_and_save(highest_cycles)
    end
    local node_tag = string.format("%d:%d", bus, nodeid)
    if not node_cycles[node_tag] then
       gcs:send_text(MAV_SEVERITY.INFO, string.format("BatteryTag: Node %s, Cycles %d", node_tag, num_cycles))
    end
    node_cycles[node_tag] = num_cycles

    -- log battery information
    logger:write("BTAG",
                 'Node,Ser,NCycle,ArmHr,Cap,FirstUse,LastArm',
                 'BIIfIII',
                 '#------',
                 '-------',
                 nodeid,
                 serial_num, num_cycles, arm_hours,
                 capacity, first_use, last_arm)

    if auth_id then
        if highest_cycles > BTAG_MAX_CYCLES:get() then
            arming:set_aux_auth_failed(auth_id, string.format("Battery cycles too high: %d", highest_cycles))
        else
            arming:set_aux_auth_passed(auth_id)
        end
    end
end

local last_globaltime_send = millis()

--[[
    see if we should send GlobalTime message
--]]
local function check_globaltime()
    local now = millis()
    if now - last_globaltime_send < 1000 then
        return
    end
    if gps:num_sensors() < 1 or gps:status(0) < 3 then
        return
    end
    last_globaltime_send = now

    -- create 56 bit UTC microsecond timestamp
    local utc_usec = gps:time_epoch_usec(0)
    if utc_usec == 0 then
        return
    end
    local usec_hi,usec_lo = utc_usec:split()
    local payload8 = string.pack("II", usec_lo:toint(), usec_hi:toint())
    local payload7 = string.sub(payload8, 1, 7)
    for bus, _ in pairs(globaltime_handles) do
        globaltime_handles[bus]:broadcast(payload7)
    end
end

--[[
   update GCS connection status
--]]
local function check_GCS()
   if not gcs_connect_time then
      local last_seen = gcs:last_seen()
      if last_seen ~= 0 then
         gcs_connect_time = last_seen
      end
   elseif not sent_report and #node_cycles and
      millis() - gcs_connect_time >= GCS_REPORT_TIME_S*1000
   then
      -- report battery tags at GCS_REPORT_TIME_S seconds
      sent_report = true
      for node_tag, cycles in pairs(node_cycles) do
         gcs:send_text(MAV_SEVERITY.INFO, string.format("BatteryTag: Node %s, Cycles %d", node_tag, cycles))
      end
   end
end

local function update()
    if BTAG_ENABLE:get() ~= 0 then
        for bus, _ in pairs(batterytag_handles) do
            check_batterytag(bus)
        end
        check_globaltime()
        check_GCS()
        if auth_id and not have_tag and millis():tofloat() * 0.001 > 15 then
            -- no tag found after 15s, setup arming failure
            arming:set_aux_auth_failed(auth_id, string.format("Battery Tag not connected"))
        end
    end
    return update, 200
end

gcs:send_text(MAV_SEVERITY.INFO, "BatteryTag loaded")

return update, 1000
