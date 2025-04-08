--[[
 DLA serial EFI protocol

 Note that this protocol is gap framed, no CRC

 https://www.austars-model.com/dla-232cc-uavuas-engine-optional-one-key-startauto-startergenerator_g17937.html
--]]

local PARAM_TABLE_KEY = 41
local PARAM_TABLE_PREFIX = "EFI_DLA_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- bind a parameter to a variable given
local function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: EFI_DLA_ENABLE
  // @DisplayName: EFI DLA enable
  // @Description: Enable EFI DLA driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_DLA_ENABLE = bind_add_param("ENABLE", 1, 0)

--[[
  // @Param: EFI_DLA_LPS
  // @DisplayName: EFI DLA fuel scale
  // @Description: EFI DLA litres of fuel per second of injection time
  // @Range: 0.00001 1
  // @Units: litres
  // @User: Standard
--]]
EFI_DLA_LPS = bind_add_param("LPS", 2, 0.001)

if EFI_DLA_ENABLE:get() ~= 1 then
   return
end

local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "EFI_DLA: unable to find scripting serial")
   return
end
uart:begin(115200)

local efi_backend = efi:get_backend(0)
if not efi_backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "EFI_DLA: unable to find EFI backend")
   return
end

--[[
   discard n bytes
--]]
local function discard_bytes(n)
   for _ = 1, n do
      uart:read()
   end
end

local function read_bytes(n)
   local ret = ""
   for _ = 1, n do
      ret = ret .. string.char(uart:read())
   end
   return ret
end

local state = {}
state.last_read_us = uint32_t(0)
state.total_fuel_cm3 = 0.0

--[[
   check for input and parse data
--]]
local function check_input()
   local n_bytes = uart:available():toint()
   --gcs:send_text(MAV_SEVERITY.INFO, string.format("n_bytes=%u %.2f", n_bytes, millis():tofloat()*0.001))
   if n_bytes < 82 then
      return
   end
   if n_bytes > 82 then
      discard_bytes(n_bytes)
      return
   end

   state.seconds, state.pw1, state.pw2 = string.unpack("<HHH", read_bytes(6))
   state.rpm, state.adv_deg, state.squirt = string.unpack("<HhB", read_bytes(5))
   state.engine, state.afrtgt1, state.afrtgt2 = string.unpack("<BBB", read_bytes(3))
   state.wbo2_en1, state.wbo2_en2, state.baro = string.unpack("<BBh", read_bytes(4))
   state.map, state.mat, state.clt = string.unpack("<hhh", read_bytes(6))
   state.tps, state.batt = string.unpack("<hh", read_bytes(4))

   state.last_read_us = micros()

   -- discard the rest
   discard_bytes(uart:available():toint())
end

--[[
   request more data
--]]
local function request_data()
   --uart:write(string.byte("a"))
   uart:write(0x61)
end

local function farenheight_to_C(v)
   return (v + 459.67) * 0.55556
end

--[[
   update EFI state
--]]
local function update_EFI()
   if state.last_read_us == uint32_t(0) then
      return
   end
   local cylinder_state = Cylinder_Status()
   local efi_state = EFI_State()

   -- 4.3.x incorrectly uses C instead of kelvin
   -- local C_TO_KELVIN = 273.2

   cylinder_state:cylinder_head_temperature(farenheight_to_C(state.clt*0.1))
   cylinder_state:exhaust_gas_temperature(farenheight_to_C(state.mat*0.1))
   cylinder_state:ignition_timing_deg(state.adv_deg*0.1)

   local inj_time_ms = (state.pw1+state.pw2)*0.001
   cylinder_state:injection_time_ms(inj_time_ms)

   efi_state:engine_speed_rpm(state.rpm)

   efi_state:atmospheric_pressure_kpa(state.baro*0.1)
   efi_state:intake_manifold_pressure_kpa(state.map*0.1)
   efi_state:intake_manifold_temperature(farenheight_to_C(state.mat*0.1))
   efi_state:throttle_position_percent(math.floor(state.tps*0.1))
   efi_state:ignition_voltage(state.batt*0.1)

   local now_us = micros()
   local dt = (now_us - state.last_read_us):tofloat()*1.0e-6
   state.last_read_us = now_us

   local revs = state.rpm * 60.0 * dt
   local inj_time = revs * inj_time_ms * 0.001
   local fuel_used_cm3 = EFI_DLA_LPS:get() * 0.001 * inj_time

   state.total_fuel_cm3 = state.total_fuel_cm3 + fuel_used_cm3

   efi_state:fuel_consumption_rate_cm3pm((fuel_used_cm3 / dt) * 60.0)
   efi_state:estimated_consumed_fuel_volume_cm3(state.total_fuel_cm3)
   
   efi_state:cylinder_status(cylinder_state)
   efi_state:last_updated_ms(millis())

   -- Set the EFI_State into the EFI scripting driver
   efi_backend:handle_scripting(efi_state)
end


--[[
   main update function
--]]
local function update()
   check_input()
   update_EFI()
   request_data()

   return update, 100
end

gcs:send_text(MAV_SEVERITY.INFO, "EFI_DLA: loaded")

return update()
