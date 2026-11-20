--[[ 
  Freeflight AltaX8 CAN ESC Feedback Driver

  Originally written here: https://github.com/magicrub/ardupilot/commit/550ba73fef998bfcc7fb94460f8214f242469a11

  Params for CAN1 showing telem on BATT1
    * SCR_ENABLE = 1
    * CAN_P1_DRIVER = 1
    * CAN_D1_PROTOCOL 10 (scripting)
    * ALTAX_MOT_RO 0
   And, configuration for ESC telem to inform battery measurements
   https://ardupilot.org/copter/docs/common-esc-telemetry.html#use-as-battery-monitor
    * BATT_MONITOR 9 # ESC 
    * BATT_ESC_MASK 0 # use all connected ESC's


   Wiring: On the freefly carrier, "Motor CAN" is CAN1 of Cube Orange+.


   IE:

         CAN_D1_PROTOCOL  10          # Scripting
         CAN_D1_PROTOCOL2 0           # Disabled
         CAN_D2_PROTOCOL  1           # DroneCAN
         CAN_D2_PROTOCOL2 0           # Disabled
         CAN_LOGLEVEL     0           # Log None
         CAN_P1_BITRATE   1000000
         CAN_P1_DRIVER    1           # First driver
         CAN_P1_FDBITRATE 8           # 8M
         CAN_P1_OPTIONS   0           # 
         CAN_P2_DRIVER    0           # Disabled
         CAN_SLCAN_CPORT  0           # Disabled
         CAN_SLCAN_SDELAY 1
         CAN_SLCAN_SERNUM -1          # Disabled
         CAN_SLCAN_TIMOUT 0

   Params for CAN2 (different from above), not used for freefly's hardware.
    * CAN_D1_PROTOCOL 0
    * CAN_D2_PROTOCOL 10
    * CAN_P2_DRIVER 2

   Then, install the script (using mavproxy)

   ftp put libraries/AP_Scripting/drivers/AltaX.lua /APM/scripts/

   reboot
   
--]]

local SCRIPT_NAME = "AltaX8 CAN"

local PARAM_TABLE_KEY = 91
local PARAM_TABLE_PREFIX = "ALTAX_"

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 1), 'could not add param table')

--[[
  // @Param: ALTAX_MOT_RO
  // @DisplayName: AltaX motor CAN bus read-only mode
  // @Description: If set, this script only reads CAN feedback frames and never writes the init or request-feedback frames. Lets you run it passively alongside another CAN master (e.g. the stock Freefly/PX4 driver) to sniff/verify telemetry parsing without contending for the bus.
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local ALTAX_MOT_RO = bind_add_param('MOT_RO', 1, 0)

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local TelemetryType = {
      TEMPERATURE = 1 << 0,
      MOTOR_TEMPERATURE  = 1 << 1,
      VOLTAGE     = 1 << 2,
      CURRENT     = 1 << 3,
      CONSUMPTION = 1 << 4,
      USAGE       = 1 << 5,
      TEMPERATURE_EXTERNAL = 1 << 6,
      MOTOR_TEMPERATURE_EXTERNAL  = 1 << 7,
      EDT2_STATUS = 1 << 8,
      EDT2_STRESS = 1 << 9,
      INPUT_DUTY  = 1 << 10,
      OUTPUT_DUTY = 1 << 11,
      FLAGS       = 1 << 12,
      POWER_PERCENTAGE = 1 << 13
      }

-- timer constants
local UPDATE_INTERVAL_MS = 10 -- This controls the delay between fetching feedback msg and checking for it's response
-- 20ms between individual ESC feedback requests (80ms full round-robin cycle) matches the
-- reference PX4/Freefly driver's cadence, confirmed against a CAN capture (see request-feedback loop below)
local SEND_GET_FEEDBACK_MSG_INTERVAL_MS = uint32_t(20)
local SEND_INIT_MSG_INTERVAL_MS = uint32_t(5000)
local FEEDBACK_TIMEOUT_MS = uint32_t(1000)

-- timer variables
local feedback_msg_timestamps = {uint32_t(0), uint32_t(0), uint32_t(0), uint32_t(0)} -- index count acts as ESC count
local init_ms = uint32_t(0)
local send_get_feedback_msg_ms = uint32_t(0)
local next_feedback_esc_index = 1 -- round-robins 1..ESC_COUNT, one esc requested per SEND_GET_FEEDBACK_MSG_INTERVAL_MS
local now_ms = millis() -- might as well make it global file-wide so we only need to fetch it once per tick and all functions can see it

-- first-feedback-seen tracking, purely for the one-shot gcs:send_text below - lets you confirm
-- at a glance (e.g. in the SITL console) that each ESC's CAN telemetry is actually being read
local esc_first_seen  = {false, false, false, false}
local esc_has_voltage = {false, false, false, false}
local esc_has_current = {false, false, false, false}
local esc_last_voltage = {0, 0, 0, 0}
local esc_last_rpm     = {0, 0, 0, 0}
local esc_last_current = {0, 0, 0, 0}


local ESC_COUNT = #feedback_msg_timestamps

local CAN_BUF_LEN = 25
local can_driver = CAN:get_device(CAN_BUF_LEN)
if not can_driver then
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: Failed to load driver", SCRIPT_NAME))
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: Check these params:", SCRIPT_NAME))
    gcs:send_text(MAV_SEVERITY.CRITICAL, string.format("%s: CAN_P1_DRIVER=1 and CAN_D1_PROTOCOL=10", SCRIPT_NAME))
    return
end


-- Type conversion
function get_uint16(frame, indexMSB, indexLSB)
    return (frame:data(indexMSB) << 8) + frame:data(indexLSB)
end


function handle_frame(frame)
--  Example frames:
--  RX    22:32:15.144685    NFD         04D    02 15 B3 01 00 00 00 00
--  RX    22:32:15.144685    NFD         04E    02 00 00 00 00 00 00 00
--  RX    22:32:15.144685    NFD         04D    03 15 AF 01 00 00 00 00
--  RX    22:32:15.144685    NFD         04E    03 00 00 00 00 00 00 00
--  RX    22:32:15.145685    NFD         04D    04 16 B1 01 00 00 00 00
--  RX    22:32:15.145685    NFD         04E    04 00 00 00 00 00 00 00
--  RX    22:32:15.207698    NFD         04D    01 15 B3 01 AA 02 43 00
--  RX    22:32:15.207698    NFD         04E    51 07 B0 00 00 00 00 00
   if frame:isExtended() then
      -- This is not the packet you're looking for...
      return
   end

   -- frame:id() returns a uint32_t userdata, not a plain Lua number - Lua's == only calls a
   -- userdata's __eq metamethod when BOTH sides are the same basic type, so comparing it directly
   -- against a plain number literal (e.g. `frame:id() == 0x4D`) silently always evaluates false.
   -- :toint() converts it to a plain number first so these comparisons actually work.
   local frame_id = frame:id():toint()

   local esc_index = (frame:data(0) & 0x0F);
   if esc_index == 0 or esc_index > ESC_COUNT then
      -- TEMP DEBUG: see docs/comment above the rx print in update() - remove once feedback is confirmed working
      gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: rejected id=0x%03X data0=0x%02X esc_index=%d",
         SCRIPT_NAME, frame_id, frame:data(0), esc_index))
      -- invalid esc index
      return
   end

   local telem_data = ESCTelemetryData()
   feedback_msg_timestamps[esc_index] = now_ms

   if frame_id == 0x4D then
      -- Voltage and RPM
      -- note: 16bit data is LSB first
      local voltage = get_uint16(frame, 3, 2) * 0.1
      local rpm = get_uint16(frame, 5, 4)
      telem_data:voltage(voltage)
      -- esc_telem's index is 0-based (ESC 1 = index 0, see AP_ESC_Telem.cpp's _rpm_data[esc_index]/
      -- _telem_data[esc_index]), while our esc_index here is the 1-based value the AltaX CAN
      -- protocol itself encodes - hence the -1. Confirmed via motortest: without it, commanding
      -- motor 1 showed up as ESC 2 in ESC_TELEMETRY_1_TO_4 (one slot too high), with ESC 1 empty.
      esc_telem:update_telem_data(esc_index - 1, telem_data, TelemetryType.VOLTAGE)
      esc_telem:update_rpm(esc_index - 1, rpm, 0)

      esc_last_voltage[esc_index] = voltage
      esc_last_rpm[esc_index] = rpm
      esc_has_voltage[esc_index] = true

   elseif frame_id == 0x4E then
      -- Current
       -- note: 16bit data is MSB first
      local current = get_uint16(frame, 1, 2) * 0.0001
      telem_data:current(current)
      esc_telem:update_telem_data(esc_index - 1, telem_data, TelemetryType.CURRENT)

      esc_last_current[esc_index] = current
      esc_has_current[esc_index] = true
   else
      gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: unparsed id=0x%03X data0=0x%02X",
         SCRIPT_NAME, frame_id, frame:data(0)))
   end

   -- one-shot: report the first time we have a full voltage+current+rpm reading for this ESC
   if not esc_first_seen[esc_index] and esc_has_voltage[esc_index] and esc_has_current[esc_index] then
      esc_first_seen[esc_index] = true
      gcs:send_text(MAV_SEVERITY.INFO, string.format(
         "%s: ESC %d feedback: %.1fV  %.4fA  %d RPM",
         SCRIPT_NAME, esc_index, esc_last_voltage[esc_index], esc_last_current[esc_index], esc_last_rpm[esc_index]))
   end
end



function send_init_msg()
   local init_msgs = {
      {0x4C, 0x00, 0x80, 0x00, 0x08},
      {0x55, 0x00, 0x80, 0x00, 0x08},
      {0x77, 0x00, 0x70, 0x00, 0x08}
   }

   for row=1, 3 do
      local msg = CANFrame()
      msg:id(0x010)
      for col=1, 5 do
         -- CANFrame:data() is 0-indexed (see frame:data(0) for esc_index in handle_frame above),
         -- but init_msgs[row] is a 1-indexed Lua table - without the -1 here every byte lands one
         -- slot too high (data(1..5) instead of data(0..4)), shifting the whole payload right by
         -- one and truncating the last byte off since dlc(5) only sends indices 0-4. Confirmed via
         -- a real capture (altax_ardupilot_bootup.pcapng): "4C 00 80 00 08" was going out as
         -- "00 4C 00 80 00" - which is exactly why the ESCs never responded to it.
         msg:data(col - 1, init_msgs[row][col])
      end
      msg:dlc(5)
      can_driver:write_frame(msg, 10000)
   end
end


function update()
   now_ms = millis()

   for _ = 1, CAN_BUF_LEN do
      local frame = can_driver:read_frame()
      if not frame then
         -- buffer is empty
         break
      end
      -- TEMP DEBUG: print every frame this script's CAN device actually receives, so we can tell
      -- whether frames are reaching the "Scripting" CAN device at all vs. being rejected/misparsed
      -- by handle_frame() below. Remove once feedback is confirmed working.
      -- gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: rx id=0x%03X ext=%s dlc=%d",
      --    SCRIPT_NAME, frame:id():toint(), tostring(frame:isExtended()), frame:dlc()))
      handle_frame(frame)
   end

   -- check timeouts and re-init as needed
   -- NOTE: this retry-the-init-sequence-on-timeout behavior is an ArduPilot-side addition, not
   -- something the stock Freefly/PX4 driver does: a capture with the ESCs disconnected for over
   -- 20s of continuous silence (altax_px4_carrier_only_long.pcapng) showed PX4 send the init burst
   -- exactly once at boot and never again, just polling for feedback forever regardless of response.
   -- We add this re-init as extra protection in case an ESC/CAN transceiver needs to be kicked back
   -- into a working state after a dropout.
   -- ALTAX_MOT_RO: read-only mode never writes to the bus (no init, no re-init, no feedback
   -- requests) - lets this run passively alongside another CAN master, e.g. stock PX4/Freefly.
   if ALTAX_MOT_RO:get() == 0 then
      if (now_ms - init_ms >= SEND_INIT_MSG_INTERVAL_MS) then
         -- don't send the init msg too often but if it's expired then lets check the timeouts at full-speed
         for esc_index=1, ESC_COUNT do
            -- Check for timeout on each ESC
            if (now_ms - feedback_msg_timestamps[esc_index] > FEEDBACK_TIMEOUT_MS) then
               gcs:send_text(MAV_SEVERITY.WARNING, string.format("%s: re-init, ESC %d feedback timed out ", SCRIPT_NAME, esc_index))
               init_ms = now_ms
               send_init_msg()

               -- only send one init msg for when ANY ESC is timed out. All ESCs will get re-init msgs
               return update, UPDATE_INTERVAL_MS
            end
         end
      end

      -- Send request-feedback message at regular interval, one ESC per interval (round-robin)
      if (now_ms - send_get_feedback_msg_ms >= SEND_GET_FEEDBACK_MSG_INTERVAL_MS) then
         send_get_feedback_msg_ms = now_ms
         local requestFeedbackMsg = CANFrame()
         requestFeedbackMsg:id(0x02A)
         requestFeedbackMsg:data(0, next_feedback_esc_index)
         requestFeedbackMsg:dlc(1)
         can_driver:write_frame(requestFeedbackMsg, 10000)
         next_feedback_esc_index = (next_feedback_esc_index % ESC_COUNT) + 1
      end
   end

   return update, UPDATE_INTERVAL_MS
end


function init()
   gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: Starting Driver", SCRIPT_NAME))
   init_ms = millis()
   if ALTAX_MOT_RO:get() == 0 then
      send_init_msg()
   else
      gcs:send_text(MAV_SEVERITY.INFO, string.format("%s: ALTAX_MOT_RO set, running read-only (no writes to CAN bus)", SCRIPT_NAME))
   end
   return update()
end

return init()
