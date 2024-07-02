--[[
   EFI driver using NMEA 2000 marine CAN protocol
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 48
PARAM_TABLE_PREFIX = "EFI_2K_"

local efi_backend = nil
local efi_state = EFI_State()
local cylinder_state = Cylinder_Status()
if not efi_state or not cylinder_state then
   return
end

-- bind a parameter to a variable given
function bind_param(name)
    local p = Parameter(name)
    assert(p, string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add EFI_2K param table')

--[[
  // @Param: EFI_2K_ENABLE
  // @DisplayName: Enable NMEA 2000 EFI driver
  // @Description: Enable NMEA 2000 EFI driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_2K_ENABLE = bind_add_param('ENABLE',  1, 1)
if EFI_2K_ENABLE:get() < 1 then
   return
end

--[[
  // @Param: EFI_2K_CANDRV
  // @DisplayName: NMEA 2000 CAN driver
  // @Description: NMEA 2000 CAN driver. Use 1 for first CAN scripting driver, 2 for 2nd driver
  // @Values: 0:Disabled,1:FirstCAN,2:SecondCAN
  // @User: Standard
--]]
local EFI_2K_CANDRV = bind_add_param('CANDRV',  2, 0)   -- CAN driver number

--[[
  // @Param: EFI_2K_OPTIONS
  // @DisplayName: NMEA 2000 options
  // @Description: NMEA 2000 driver options
  // @Bitmask: 0:EnableLogging
  // @User: Standard
--]]
EFI_2K_OPTIONS = bind_add_param("OPTIONS", 3, 0)

local OPTION_LOGGING = (1<<0)

--[[
   return true if an option is enabled
--]]
local function option_enabled(option)
   return (EFI_2K_OPTIONS:get() & option) ~= 0
end

-- Register for the CAN drivers
local CAN_BUF_LEN = 25
local can_driver = nil

if EFI_2K_CANDRV:get() == 1 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_2K: attaching to CAN1"))
   can_driver = CAN:get_device(CAN_BUF_LEN)
elseif EFI_2K_CANDRV:get() == 2 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_2K: attaching to CAN2"))
   can_driver = CAN:get_device2(CAN_BUF_LEN)
end

if not can_driver then
   gcs:send_text(MAV_SEVERITY.ERROR, string.format("EFI_2K: invalid CAN driver"))
   return
end

-- load NMEA_2000 module
local NMEA_2000 = require("NMEA_2000")
if not NMEA_2000 then
   gcs:send_text(MAV_SEVERITY.ERROR, "EFI_2K: Unable to load NMEA_2000.lua module")
   return
end

--[[
   create PGN table
--]]
local PGN_ENGINE_PARAM_RAPID = 0x1F200
local PGN_ENGINE_PARAM_DYNAMIC = 0x1F201

local PGN_TABLE = {
   [PGN_ENGINE_PARAM_RAPID] = 8,
   [PGN_ENGINE_PARAM_DYNAMIC] = 26
}

NMEA_2000.set_PGN_table(PGN_TABLE)

local frame_count = 0
local state = {}

local function log_frame(frame)
   local id = frame:id()
   logger:write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                id,
                frame:dlc(),
                frame_count,
                frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                frame:data(4), frame:data(5), frame:data(6), frame:data(7))
   frame_count = frame_count + 1
end

--[[
   parse the higher rate engine data, giving RPM and pressure
--]]
local function parse_engine_param_rapid(data)
   state.instance, state.speed, state.boost_presssure, state.tilt, _ = string.unpack("<BHHbH", data)
   if not state.instance then
      return
   end

   efi_state:engine_speed_rpm(uint32_t(state.speed/4))
   efi_state:atmospheric_pressure_kpa(state.boost_presssure*0.001)
   efi_state:cylinder_status(cylinder_state)
   efi_state:last_updated_ms(millis())

   -- Set the EFI_State into the EFI scripting driver
   if efi_backend then
      efi_backend:handle_scripting(efi_state)
   end
end

--[[
   parse the lower rate engine data, giving other engine data
--]]
local function parse_engine_param_dynamic(data)
   state.instance, state.oil_press, state.oil_temp, state.temp_K, state.alt_V, state.fuel_rate, state.engine_s, state.cool_press,
      state.fuel_press, state.res1, state.dstat1, state.dstat2, state.eload, state.etorque = string.unpack("<BHHHhhIHHBHHbb", data)
   if not state.instance then
      return
   end
   efi_state:intake_manifold_temperature(state.temp_K*0.01)
   efi_state:oil_pressure(state.oil_press*0.01*0.001)
   efi_state:oil_temperature(state.oil_temp*0.1)
   cylinder_state:cylinder_head_temperature(state.temp_K*0.01)
   efi_state:ignition_voltage(state.alt_V*0.01)
   efi_state:fuel_consumption_rate_cm3pm(state.fuel_rate*0.1*1000.0/3600.0)
   efi_state:intake_manifold_pressure_kpa(state.cool_press*100*0.001)
   efi_state:fuel_pressure(state.fuel_press)
   efi_state:engine_load_percent(state.eload)
end

function update()
   if EFI_2K_ENABLE:get() < 1 then
      return update, 500
   end
   if not efi_backend then
      efi_backend = efi:get_backend(0)
   end
   if not efi_backend then
      return update, 500
   end

   -- read up to 25 frames per update() call
   for _ = 1, 25 do
      local frame = can_driver:read_frame()
      if not frame then
         break
      end
      if option_enabled(OPTION_LOGGING) then
         log_frame(frame)
      end
      pgn, data = NMEA_2000.parse(frame)
      if pgn then
         if pgn == PGN_ENGINE_PARAM_RAPID then
            parse_engine_param_rapid(data)
         elseif pgn == PGN_ENGINE_PARAM_DYNAMIC then
            parse_engine_param_dynamic(data)
         end
      end
   end

   return update, 2
end

return update()
