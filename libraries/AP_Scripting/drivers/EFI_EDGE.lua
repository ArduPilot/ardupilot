--[[

Name: EFI Scripting backend driver for EDGE Autonomy EFI
--]]

local SCRIPT_NAME = "EFI: EDGE CAN"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local K_THROTTLE = 70
local K_HELIRSC = 31

PARAM_TABLE_KEY = 41
PARAM_TABLE_PREFIX = "EFI_ED_"

-- bind a parameter to a variable given
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
end

function get_time_sec()
    return millis():tofloat() * 0.001
end

-- Type conversion functions

function get_uint8(frame, ofs)
   return frame:data(ofs)
end

function get_uint16(frame, ofs)
   -- protocol is big endian
   return (frame:data(ofs)<<8) + frame:data(ofs + 1)
end

function get_int16(frame, ofs)
   local v = get_uint16(frame, ofs)
   if v & 0x8000 ~= 0 then
      return v - 65536
   end
   return v
end

function get_int8(frame, ofs)
   local v = get_uint8(frame, ofs)
   if v & 0x80 then
      return v - 256
   end
   return v
end

function constrain(v, vmin, vmax)
    if v < vmin then
        v = vmin
    end
    if v > vmax then
        v = vmax
    end
    return v
end

local efi_backend = nil

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add EFI_ED param table')

--[[
  // @Param: EFI_ED_ENABLE
  // @DisplayName: Enable EDGE EFI support
  // @Description: Enable EDGE EFI support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_ED_ENABLE     = bind_add_param('ENABLE',     1, 0)

--[[
  // @Param: EFI_ED_CANDRV
  // @DisplayName: Set EDGE EFI CAN driver
  // @Description: Set EDGE EFI CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local EFI_ED_CANDRV     = bind_add_param('CANDRV',     2, 1)    -- CAN driver to use

--[[
  // @Param: EFI_ED_UPDATE_HZ
  // @DisplayName: EDGE EFI update rate
  // @Description: EDGE EFI update rate
  // @Range: 10 200
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_ED_UPDATE_HZ  = bind_add_param('UPDATE_HZ',  3, 200)  -- Script update frequency in Hz

--[[
  // @Param: EFI_ED_THR_FN
  // @DisplayName: EDGE EFI throttle function
  // @Description: EDGE EFI throttle function. This sets which SERVOn_FUNCTION to use for the target throttle. This should be 70 for fixed wing aircraft and 31 for helicopter rotor speed control
  // @Values: 0:Disabled,70:FixedWing,31:HeliRSC
  // @User: Standard
--]]
local EFI_ED_THR_FN     = bind_add_param('THR_FN',     4, 0)    -- servo function for throttle

--[[
  // @Param: EFI_ED_THR_RATE
  // @DisplayName: EDGE EFI throttle rate
  // @Description: EDGE EFI throttle rate. This sets rate at which throttle updates are sent to the engine
  // @Range: 10 100
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_ED_THR_RATE   = bind_add_param('THR_RATE',   5, 50)   -- throttle update rate

--[[
  // @Param: EFI_ED_START_FN
  // @DisplayName: EDGE EFI start function
  // @Description: EDGE EFI start function. This is the RCn_OPTION value to use to find the R/C channel used for controlling engine start
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFI_ED_START_FN   = bind_add_param('START_FN',   6, 0)    -- start control function (RC option)

--[[
  // @Param: EFI_ED_MIN_RPM
  // @DisplayName: EDGE EFI minimum RPM
  // @Description: EDGE EFI minimum RPM. This is the RPM below which the engine is considered to be stopped
  // @Range: 1 1000
  // @User: Advanced
--]]
local EFI_ED_MIN_RPM    = bind_add_param('MIN_RPM',    7, 100)  -- min RPM, for engine restart

--[[
  // @Param: EFI_ED_TLM_RT
  // @DisplayName: EDGE EFI telemetry rate
  // @Description: EDGE EFI telemetry rate. This is the rate at which extra telemetry values are sent to the GCS
  // @Range: 1 10
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_ED_TLM_RT     = bind_add_param('TLM_RT',     8, 2)    -- rate for extra telemetry values

--[[
  // @Param: EFI_ED_LOG_RT
  // @DisplayName: EDGE EFI log rate
  // @Description: EDGE EFI log rate. This is the rate at which extra logging of the EDGE EFI is performed
  // @Range: 1 50
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_ED_LOG_RT     = bind_add_param('LOG_RT',     9, 10)   -- rate for logging

--[[
  // @Param: EFI_ED_ST_DISARM
  // @DisplayName: EDGE EFI allow start disarmed
  // @Description: EDGE EFI allow start disarmed. This controls if starting the engine while disarmed is allowed
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_ED_ST_DISARM  = bind_add_param('ST_DISARM', 10, 0)    -- allow start when disarmed

if EFI_ED_ENABLE:get() == 0 then
   gcs:send_text(0, string.format("EFI_EDGE: disabled"))
   return
end

EFI_FUEL_DENS = Parameter("EFI_FUEL_DENS")

-- Register for the CAN drivers
local driver1

local CAN_BUF_LEN = 25
if EFI_ED_CANDRV:get() == 1 then
   driver1 = CAN.get_device(CAN_BUF_LEN)
elseif EFI_ED_CANDRV:get() == 2 then
   driver1 = CAN.get_device2(CAN_BUF_LEN)
end

if not driver1 then
    gcs:send_text(0, string.format("EFI_EDGE: Failed to load driver"))
    return
end

function C_TO_KELVIN(temp)
   return temp + 273.15
end

--[[
   we allow the engine to run if either armed or the EFI_ED_ST_DISARM parameter is 1
--]]
function allow_run_engine()
  return arming:is_armed() or EFI_ED_ST_DISARM:get() == 1
end

function can_log(frame)
   local id = frame:id_signed()
   logger.write('EFCN','Id,B0,B1,B2,B3,B4,B5,B6,B7', 'iBBBBBBBB',
                id,
                frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                frame:data(4), frame:data(5), frame:data(6), frame:data(7))
end

--[[
   convert grams to cm3 of fuel
--]]
function gram_to_cm3(g)
   local fuel_density = EFI_FUEL_DENS:get()
   if fuel_density <= 0 then
      fuel_density = 800.0
   end
   return g * fuel_density * 0.001
end

--[[
   EFI Engine Object
--]]
local function engine_control(_driver, _idx)
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local driver = _driver
    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()
    local last_thr_update = get_time_sec()
    local last_telem_update = get_time_sec()
    local last_log_t = get_time_sec()
    local engine_started = false
    local engine_start_t = 0.0
    local last_throttle = 0.0

    local tlm = {rpm = 0, advance = 0, barometer_kPa = 0, map_kPa = 0, mat_C = 0, coolent_C = 0, eng_status = 0,
                 tps = 0, afr = 0, voltage = 0, warmup_enr = 0, baro_corr = 0, ve_curr = 0,
                 sys_status = 0, thr_source = 0, duct_pos = 0, thr_pos = 0, analog_voltage = 0,
                 duct_current = 0, thr_servo_current = 0, fp_current = 0, ecu_temp = 0, gen_temp = 0,
                 total_fuel_g = 0, fuel_restart_g = 0, ewt = 0, rem_fuel_g = 0,
                 ignition_current = 0, injector_current = 0, rpm2 = 0, fuel_flow_gph = 0, voltage2 = 0,
                 tps2 = 0, cht = 0, map2_Pa = 0, injection_time_us = 0, ignition_angle = 0, inlet_temp = 0
    }


    -- read telemetry packets
    function self.update_telemetry()
        local max_packets = 25
        local count = 0
        while count < max_packets do
            frame = driver:read_frame()
            count = count + 1
            if not frame then
                break
            end

            -- All Frame IDs for this EFI Engine are extended
            if frame:isExtended() then
                self.handle_EFI_packet(frame, _idx)
            end
        end
        if last_rpm_t > last_state_update_t then
           -- update state if we have an updated RPM
           last_state_update_t = last_rpm_t
           self.set_EFI_State()
        end
    end

    -- handle an EFI packet
    function self.handle_EFI_packet(frame, _)
       local id = frame:id_signed()
       can_log(frame)
       if id == 0x19000000 then
          tlm.rpm = get_uint16(frame, 2)
          tlm.advance = get_uint16(frame, 4)
          tlm.barometer_kPa = get_uint16(frame, 6) * 0.1
          last_rpm_t = get_time_sec()
       elseif id == 0x19000001 then
          tlm.map_kPa = get_int16(frame, 0) * 0.1
          tlm.mat_C = get_int16(frame, 2) * 0.05555
          tlm.coolent_C = get_int16(frame, 4) * 0.05555
          tlm.eng_status = get_uint8(frame, 6)
       elseif id == 0x19000002 then
          tlm.tps = get_int16(frame, 0) * 0.1
          tlm.afr = get_int16(frame, 2) * 0.1
          tlm.voltage = get_int16(frame, 4) * 0.1
          tlm.warmup_enr = get_int16(frame, 6)
       elseif id == 0x19000003 then
          tlm.baro_corr = get_int16(frame, 0)
          tlm.ve_curr = get_int16(frame, 2) * 0.1
       elseif id == 0x19000004 then
          tlm.sys_status = get_uint16(frame, 0)
          tlm.thr_source = get_uint8(frame, 2)
          tlm.duct_pos = get_uint8(frame, 3)
          tlm.thr_pos = get_uint8(frame, 4)
          tlm.analog_voltage = get_uint16(frame, 5) * 0.002
       elseif id == 0x19000005 then
          tlm.duct_current = get_uint16(frame, 0) * 0.001
          tlm.thr_servo_current = get_uint16(frame, 2) * 0.001
          tlm.fp_current = get_uint16(frame, 4) * 0.001
          tlm.ecu_temp = get_int8(frame, 6)
          tlm.gen_temp = get_int8(frame, 7)
       elseif id == 0x19000006 then
          tlm.total_fuel_g = get_uint16(frame, 0) * 100.0
          tlm.fuel_restart_g = get_uint16(frame, 2)
          tlm.ewt = get_uint16(frame, 4) * 0.01
          tlm.rem_fuel_g = get_uint16(frame, 6)
       elseif id == 0x19000007 then
          tlm.ignition_current = get_uint16(frame, 0) * 0.001
          tlm.injector_current = get_uint16(frame, 2) * 0.001
       elseif id == 0x08800000 then
          tlm.rpm2 = get_uint16(frame, 0)
          tlm.fuel_flow_gph = get_uint16(frame, 2)
       elseif id == 0x08810000 then
          tlm.voltage2 = get_uint8(frame, 0) * 0.1
          tlm.tps2 = get_uint8(frame, 1)
          tlm.cht = get_int16(frame, 2) * 0.1
          tlm.map2_Pa = get_uint16(frame, 6) * 2
       elseif id == 0x08820000 then
          tlm.injection_time_us = get_uint16(frame, 0)
          tlm.ignition_angle = get_int16(frame, 2)
          tlm.inlet_temp = get_uint8(frame, 5)
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:ignition_timing_deg(tlm.ignition_angle)
       cylinder_state:injection_time_ms(tlm.injection_time_us*0.001)
       cylinder_state:cylinder_head_temperature(C_TO_KELVIN(tlm.cht))
       cylinder_state:exhaust_gas_temperature(C_TO_KELVIN(tlm.coolent_C))

       efi_state:engine_speed_rpm(uint32_t(tlm.rpm))
       efi_state:atmospheric_pressure_kpa(tlm.barometer_kPa)
       efi_state:intake_manifold_pressure_kpa(tlm.map_kPa)
       efi_state:intake_manifold_temperature(C_TO_KELVIN(tlm.mat_C))
       efi_state:coolant_temperature(C_TO_KELVIN(tlm.coolent_C))

       efi_state:fuel_consumption_rate_cm3pm(gram_to_cm3(tlm.fuel_flow_gph) / 60.0)
       efi_state:estimated_consumed_fuel_volume_cm3(gram_to_cm3(tlm.fuel_restart_g))

       efi_state:throttle_position_percent(math.floor(constrain(tlm.tps,0,100)))
       efi_state:ignition_voltage(tlm.voltage)
       efi_state:throttle_out(last_throttle * 100)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)

       local last_efi_state_time = millis()
       efi_state:last_updated_ms(last_efi_state_time)


        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    --- send throttle command, thr is 0 to 1
    function self.send_throttle(_)
    end

    -- send an engine start command
    function self.send_engine_start()
    end

    -- send an engine stop command
    function self.send_engine_stop()
    end

    -- update starter control
    function self.update_starter()
       local start_fn = EFI_ED_START_FN:get()
       if start_fn == 0 then
          return
       end
       local start_state = rc:get_aux_cached(start_fn)
       if start_state == nil then
          start_state = 0
       end
       local should_be_running = false
       if start_state == 0 and engine_started then
          engine_started = false
          engine_start_t = 0
          self.send_engine_stop()
       end
       if start_state == 2 and not engine_started and allow_run_engine() then
          engine_started = true
          gcs:send_text(0, string.format("EFI_EDGE: starting engine"))
          engine_start_t = get_time_sec()
          self.send_engine_start()
          should_be_running = true
       end
       if start_state > 0 and engine_started and allow_run_engine() then
          should_be_running = true
       end
       local min_rpm = EFI_ED_MIN_RPM:get()
       if min_rpm > 0 and engine_started and rpm < min_rpm and allow_run_engine() then
          local now = get_time_sec()
          local dt = now - engine_start_t
          if dt > 2.0 then
             gcs:send_text(0, string.format("EFI_EDGE: re-starting engine"))
             self.send_engine_start()
             engine_start_t = get_time_sec()
          end
       end
       --[[
          cope with lost engine stop packets
       --]]
       if rpm > min_rpm and not should_be_running then
          engine_started = false
          engine_start_t = 0
          self.send_engine_stop()
       end
    end

    -- update throttle output
    function self.update_throttle()
       local thr_func = EFI_ED_THR_FN:get()
       local thr_rate = EFI_ED_THR_RATE:get()
       if thr_func == 0 or thr_rate == 0 then
          return
       end
       local now = get_time_sec()
       if now - last_thr_update < 1.0 / thr_rate then
          return
       end
       last_thr_update = now
       local thr = 0.0
       local scaled = SRV_Channels:get_output_scaled(thr_func)
       if thr_func == K_THROTTLE then
          thr = scaled * 0.01
       elseif thr_func == K_HELIRSC then
          thr = scaled * 0.001
       end
       self.send_throttle(thr)
    end

    -- update telemetry output for extra telemetry values
    function self.update_telem_out()
       local rate = EFI_ED_TLM_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_telem_update < 1.0 / rate then
          return
       end
       last_telem_update = now
       gcs:send_named_float('EFI_VOLT2', tlm.voltage2)
       gcs:send_named_float('EFI_STATUS', tlm.sys_status)
    end

    -- update custom logging
    function self.update_logging()
       local rate = EFI_ED_LOG_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_log_t < 1.0 / rate then
          return
       end
       last_log_t = now
       logger.write('EFED','Status,ECUTmp,InTmp', 'Hff',
                    tlm.sys_status, tlm.ecu_temp, tlm.inlet_temp)
    end
    
    -- return the instance
    return self
end -- end function engine_control(_driver, _idx)

local engine1 = engine_control(driver1, 1)

function update()
   if not efi_backend then
      efi_backend = efi:get_backend(0)
      if not efi_backend then
         return
      end
   end

   -- Parse Driver Messages
   engine1.update_telemetry()
   engine1.update_starter()
   engine1.update_throttle()
   engine1.update_telem_out()
   engine1.update_logging()
end

gcs:send_text(0, SCRIPT_NAME .. string.format(" loaded"))

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / EFI_ED_UPDATE_HZ:get()
end

-- start running update loop
return protected_wrapper()
