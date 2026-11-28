--[[
   Companion computer failsafe for Copter

   When a companion computer flies the vehicle in GUIDED mode and its
   software stops (crash, power loss, cable fault) the vehicle is left
   holding position with nothing in control. This script watches for the
   companion's MAVLink HEARTBEAT and changes flight mode if it goes
   missing while the vehicle is armed and in GUIDED.
--]]

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local PARAM_TABLE_KEY = 109
local PARAM_TABLE_PREFIX = "CC_FS_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local HEARTBEAT_ID = mavlink_msgs.get_msgid("HEARTBEAT")
local MODE_GUIDED = 4
local MODE_LAND = 9
local UPDATE_MS = 100

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

--[[
  // @Param: CC_FS_ENABLE
  // @DisplayName: Companion failsafe enable
  // @Description: Enable the companion computer failsafe
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local CC_FS_ENABLE = bind_add_param('ENABLE', 1, 1)

--[[
  // @Param: CC_FS_TIMEOUT
  // @DisplayName: Companion failsafe timeout
  // @Description: Time without a companion HEARTBEAT, while armed in GUIDED mode, before the flight mode is changed
  // @Range: 1 30
  // @Units: s
  // @User: Standard
--]]
local CC_FS_TIMEOUT = bind_add_param('TIMEOUT', 2, 3)

--[[
  // @Param: CC_FS_SYSID
  // @DisplayName: Companion failsafe system ID
  // @Description: MAVLink system ID of the companion computer. Zero matches any system ID
  // @Range: 0 255
  // @User: Standard
--]]
local CC_FS_SYSID = bind_add_param('SYSID', 3, 0)

--[[
  // @Param: CC_FS_COMPID
  // @DisplayName: Companion failsafe component ID
  // @Description: MAVLink component ID of the companion computer. The default is MAV_COMP_ID_ONBOARD_COMPUTER. Zero matches any component ID
  // @Range: 0 255
  // @User: Standard
--]]
local CC_FS_COMPID = bind_add_param('COMPID', 4, 191)

--[[
  // @Param: CC_FS_MODE
  // @DisplayName: Companion failsafe flight mode
  // @Description: Flight mode to change to when the companion HEARTBEAT is lost. If this mode is refused the vehicle changes to LAND
  // @Values: 5:Loiter,6:RTL,9:Land,17:Brake,21:SmartRTL
  // @User: Standard
--]]
local CC_FS_MODE = bind_add_param('MODE', 5, 9)

mavlink:init(10, 1)
mavlink:register_rx_msgid(HEARTBEAT_ID)

local last_heartbeat_ms = nil  -- time of the last matching HEARTBEAT, nil if never seen
local companion_present = false -- matching HEARTBEAT seen within the timeout
local monitoring = false       -- companion has been seen during this GUIDED flight
local was_guided = false
local triggered = false

local function timeout_ms()
   return math.max(CC_FS_TIMEOUT:get(), 1) * 1000
end

-- read all queued HEARTBEAT messages, noting any from the companion
local function read_heartbeats(now)
   for _ = 1, 10 do
      local msg = mavlink:receive_chan()
      if not msg then
         return
      end
      local header = mavlink_msgs.decode_header(msg)
      if header and header.msgid == HEARTBEAT_ID then
         local sysid = CC_FS_SYSID:get()
         local compid = CC_FS_COMPID:get()
         if (sysid == 0 or header.sysid == sysid) and (compid == 0 or header.compid == compid) then
            last_heartbeat_ms = now
         end
      end
   end
end

-- change to CC_FS_MODE, falling back to LAND if that mode is refused.
-- Called once per GUIDED flight, so a refusal is reported once rather than retried.
local function trigger_failsafe()
   local mode = math.floor(CC_FS_MODE:get())
   if vehicle:set_mode(mode) then
      gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Companion FS: heartbeat lost, mode %d", mode))
      return
   end
   if mode ~= MODE_LAND and vehicle:set_mode(MODE_LAND) then
      gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("Companion FS: lost, mode %d refused, using LAND", mode))
      return
   end
   gcs:send_text(MAV_SEVERITY.CRITICAL, "Companion FS: heartbeat lost, mode change refused")
end

local function update()
   local now = millis()
   read_heartbeats(now)

   local present = last_heartbeat_ms ~= nil and (now - last_heartbeat_ms):toint() < timeout_ms()
   if present and not companion_present then
      gcs:send_text(MAV_SEVERITY.INFO, "Companion FS: companion heartbeat detected")
   end
   companion_present = present

   local guided = arming:is_armed() and vehicle:get_mode() == MODE_GUIDED
   if not guided then
      was_guided = false
      monitoring = false
      triggered = false
      return update, UPDATE_MS
   end
   if not was_guided then
      -- only start monitoring once the companion has been seen, so that
      -- GUIDED flown from a ground station alone is not affected
      was_guided = true
      monitoring = present
   end
   if present then
      monitoring = true
   elseif monitoring and not triggered and CC_FS_ENABLE:get() >= 1 then
      triggered = true
      trigger_failsafe()
   end

   return update, UPDATE_MS
end

if FWVersion:type() ~= 2 then
   gcs:send_text(MAV_SEVERITY.ERROR, "Companion FS: this script only supports Copter")
   return
end

gcs:send_text(MAV_SEVERITY.INFO, "Companion FS: loaded")

return update()
