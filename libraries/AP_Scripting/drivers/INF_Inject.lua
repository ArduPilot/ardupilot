--[[
 Driver for INF_Inject EFI system

 https://innoflighttechnology.com/efi/
--]]

local PARAM_TABLE_KEY = 43
local PARAM_TABLE_PREFIX = "EFI_INF_"

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
  // @Param: EFI_INF_ENABLE
  // @DisplayName: EFI INF-Inject enable
  // @Description: Enable EFI INF-Inject driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_INF_ENABLE = bind_add_param("ENABLE", 1, 1)

if EFI_INF_ENABLE:get() ~= 1 then
   return
end

local EFI_FUEL_DENS = bind_param("EFI_FUEL_DENS")

local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "EFI_INF: unable to find scripting serial")
   return
end
uart:begin(9600)

local efi_backend = efi:get_backend(0)
if not efi_backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "EFI_INF: unable to find EFI backend")
   return
end

local state = {}
state.last_read_us = uint32_t(0)
state.chk0 = 0
state.chk1 = 0
state.total_fuel_g = 0.0

local function read_bytes(n)
   local ret = ""
   for _ = 1, n do
      local b = uart:read()
      state.chk0 = state.chk0 ~ b
      state.chk1 = state.chk1 ~ state.chk0
      ret = ret .. string.char(b)
   end
   return ret
end

--[[
   convert grams of fuel to cm3
--]]
local function gram_to_cm3(g)
   local kg = g * 0.001
   local kg_per_m3 = EFI_FUEL_DENS:get()
   if kg_per_m3 <= 0 then
      kg_per_m3 = 742.9
   end
   local m3 = kg / kg_per_m3
   return m3 * 1.0e6
end

--[[
   check for input and parse data
--]]
local function check_input()
   local packet_size = 83
   local n_bytes = uart:available():toint()
   if n_bytes < packet_size then
      return false
   end

   local tus = micros()
   state.chk0 = 0
   state.chk1 = 0

   -- sync on header start
   local header_ok = false
   while n_bytes >= packet_size and not header_ok do
      local header0 = string.unpack("<B", read_bytes(1))
      n_bytes = n_bytes - 1
      if header0 == 0xB5 then
         local header1 = string.unpack("<B", read_bytes(1))
         n_bytes = n_bytes - 1
         if header1 == 0x62 then
            header_ok = true
         end
      end
   end
   if not header_ok or n_bytes < packet_size-2 then
      return false
   end

   -- look for basic data table 2
   local _, _, dtype, num, _, ack = string.unpack("<BBBBIB", read_bytes(9))
   if dtype ~= 0x02 then
      return false
   end
   if ack ~= 0x50 then
      return false
   end
   if num < packet_size then
      return false
   end

   --gcs:send_text(MAV_SEVERITY.INFO, string.format("packet start"))

   state.mode, state.sta, state.sta1, state.bus_thr = string.unpack("<BBBB", read_bytes(4))
   state.bus_thr = state.bus_thr * 0.1
   state.svr_pwm, state.rpm_out, state.rpm1, state.rpm2 = string.unpack("<BHHH", read_bytes(7))
   state.svr_pwm = state.svr_pwm * 0.1
   state.tmp_env, state.tmp0, state.tmp1, state.tmp2, state.tmp3 = string.unpack("<hhhhh", read_bytes(10))
   state.vol_power, state.vol_svr, state.vol_pump, state.amp_pump = string.unpack("<HBBB", read_bytes(5))
   state.vol_power = state.vol_power * 0.1
   state.vol_svr = state.vol_svr * 0.1
   state.vol_pump = state.vol_pump * 0.1
   state.amp_pump = state.amp_pump * 0.1
   state.ADC1, state.ADC2, state.pre_gas, state.pre_alt = string.unpack("<BBfh", read_bytes(8))
   state.ADC1 = state.ADC1 * 0.1
   state.ADC2 = state.ADC2 * 0.1
   state.PWM_IN1, state.PWM_IN2, state.PWM_IN3 = string.unpack("<BBB", read_bytes(3))
   state.PWM_IN1 = state.PWM_IN1 * 10
   state.PWM_IN2 = state.PWM_IN2 * 10
   state.PWM_IN3 = state.PWM_IN3 * 10
   state.PWM_OUT1, state.PWM_OUT2, state.PWM_OUT3 = string.unpack("<BBB", read_bytes(3))
   state.pre_oil, state.inj1_ms, state.inj2_ms = string.unpack("<fhh", read_bytes(8))
   state.inj1_mg, state.inj2_mg = string.unpack("<hh", read_bytes(4))
   state.adc1_thr, state.adc2_thr = string.unpack("<BB", read_bytes(2))
   state.adc1_thr = state.adc1_thr * 10
   state.adc2_thr = state.adc2_thr * 10
   state.ecu_run_time, state.err_flg = string.unpack("<IH", read_bytes(6))
   state.oil1_thr, state.oil2_thr = string.unpack("<BB", read_bytes(2))
   state.oil1_thr = state.oil1_thr * 10
   state.oil2_thr = state.oil2_thr * 10
   state.reserve1, state.reserve2 = string.unpack("<BB", read_bytes(2))
   state.tmp4, state.tmp5 = string.unpack("<hh", read_bytes(4))

   local chk0 = state.chk0
   local chk1 = state.chk1
   state.check0, state.check1 = string.unpack("<BB", read_bytes(2))
   --[[
      we accept 0 or the right value for check1 as some devices always send 0 for chk1
   --]]
   local checksum_ok = chk0 == state.check0 and (0x00 == state.check1 or chk1 == state.check1)
   if not checksum_ok then
      gcs:send_text(MAV_SEVERITY.INFO, string.format("chksum wrong (0x%02x,0x%02x) (0x%02x,0x%02x)", chk0, chk1, state.check0, state.check1))
      return false
   end
   state.end0, state.end1 = string.unpack("<BB", read_bytes(2))

   if state.end0 ~= 0x0d or state.end1 ~= 0x0a then
      return false
   end

   local dt = (tus - state.last_read_us):tofloat()*1.0e-6

   local fuel_g_rev = (state.inj1_mg + state.inj2_mg)
   state.fuel_g_per_min = fuel_g_rev * state.rpm1
   state.total_fuel_g = state.total_fuel_g + (state.fuel_g_per_min * dt / 60.0)

   state.last_read_us = micros()

   gcs:send_named_float('VOL_SRV', state.vol_svr)
   gcs:send_named_float('VOL_PUMP', state.vol_pump)
   gcs:send_named_float('INF_ETEMP', state.tmp_env)
   gcs:send_named_float('INF_TEMP1', state.tmp0)
   gcs:send_named_float('INF_TEMP2', state.tmp1)

   return true
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

   cylinder_state:cylinder_head_temperature(state.tmp0)
   cylinder_state:exhaust_gas_temperature(state.tmp1)
   cylinder_state:injection_time_ms(state.inj1_ms)

   efi_state:engine_speed_rpm(state.rpm_out)

   efi_state:atmospheric_pressure_kpa(state.pre_gas*0.01)
   efi_state:intake_manifold_temperature(state.tmp2)
   efi_state:throttle_position_percent(math.floor(state.bus_thr*0.1))
   efi_state:ignition_voltage(state.vol_power)

   efi_state:cylinder_status(cylinder_state)
   efi_state:last_updated_ms(millis())
   efi_state:fuel_pressure(state.pre_oil)

   efi_state:fuel_consumption_rate_cm3pm(gram_to_cm3(state.fuel_g_per_min))
   efi_state:estimated_consumed_fuel_volume_cm3(gram_to_cm3(state.total_fuel_g))

   -- Set the EFI_State into the EFI scripting driver
   efi_backend:handle_scripting(efi_state)
end


--[[
   main update function
--]]
local function update()
   if check_input() then
      update_EFI()
   end

   return update, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "EFI_INF: loaded")

return update()
