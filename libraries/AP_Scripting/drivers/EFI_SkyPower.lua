--[[ 
Name: EFI Scripting backend driver for SkyPower
Authors: Andrew Tridgell & Josh Henderson

The protocol has high CAN utilization due to CAN packets that are used
internal to the enigne being published externally, as well as being
limited to 500 kbit/s.

CAN_D1_PROTOCOL 10 (Scripting Driver 1)
CAN_P1_DRIVER 1 (First driver)
CAN_D1_BITRATE 500000 (500 kbit/s)

--]]

-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = 4.3
local SCRIPT_NAME       = "EFI: Skypower CAN"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s Requires: %s:%.1f. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))


local MAV_SEVERITY_ERROR = 3

local K_THROTTLE = 70
local K_HELIRSC = 31

local MODEL_DEFAULT = 0
local MODEL_SP_275 = 1

PARAM_TABLE_KEY = 36
PARAM_TABLE_PREFIX = "EFI_SP_"

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
function get_uint16(frame, ofs)
    return frame:data(ofs) + (frame:data(ofs + 1) << 8)
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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add EFI_SP param table')

--[[
  // @Param: EFI_SP_ENABLE
  // @DisplayName: Enable SkyPower EFI support
  // @Description: Enable SkyPower EFI support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_SP_ENABLE     = bind_add_param('ENABLE',     1, 0)

--[[
  // @Param: EFI_SP_CANDRV
  // @DisplayName: Set SkyPower EFI CAN driver
  // @Description: Set SkyPower EFI CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local EFI_SP_CANDRV     = bind_add_param('CANDRV',     2, 1)    -- CAN driver to use

--[[
  // @Param: EFI_SP_UPDATE_HZ
  // @DisplayName: SkyPower EFI update rate
  // @Description: SkyPower EFI update rate
  // @Range: 10 200
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_SP_UPDATE_HZ  = bind_add_param('UPDATE_HZ',  3, 200)  -- Script update frequency in Hz

--[[
  // @Param: EFI_SP_THR_FN
  // @DisplayName: SkyPower EFI throttle function
  // @Description: SkyPower EFI throttle function. This sets which SERVOn_FUNCTION to use for the target throttle. This should be 70 for fixed wing aircraft and 31 for helicopter rotor speed control
  // @Values: 0:Disabled,70:FixedWing,31:HeliRSC
  // @User: Standard
--]]
local EFI_SP_THR_FN     = bind_add_param('THR_FN',     4, 0)    -- servo function for throttle

--[[
  // @Param: EFI_SP_THR_RATE
  // @DisplayName: SkyPower EFI throttle rate
  // @Description: SkyPower EFI throttle rate. This sets rate at which throttle updates are sent to the engine
  // @Range: 10 100
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_SP_THR_RATE   = bind_add_param('THR_RATE',   5, 50)   -- throttle update rate

--[[
  // @Param: EFI_SP_START_FN
  // @DisplayName: SkyPower EFI start function
  // @Description: SkyPower EFI start function. This is the RCn_OPTION value to use to find the R/C channel used for controlling engine start
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFI_SP_START_FN   = bind_add_param('START_FN',   6, 0)    -- start control function (RC option)

--[[
  // @Param: EFI_SP_GEN_FN
  // @DisplayName: SkyPower EFI generator control function
  // @Description: SkyPower EFI generator control function. This is the RCn_OPTION value to use to find the R/C channel used for controlling generator start/stop
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFI_SP_GEN_FN     = bind_add_param('GEN_FN',     7, 0)    -- generator control function (RC option)

--[[
  // @Param: EFI_SP_MIN_RPM
  // @DisplayName: SkyPower EFI minimum RPM
  // @Description: SkyPower EFI minimum RPM. This is the RPM below which the engine is considered to be stopped
  // @Range: 1 1000
  // @User: Advanced
--]]
local EFI_SP_MIN_RPM    = bind_add_param('MIN_RPM',    8, 100)  -- min RPM, for engine restart

--[[
  // @Param: EFI_SP_TLM_RT
  // @DisplayName: SkyPower EFI telemetry rate
  // @Description: SkyPower EFI telemetry rate. This is the rate at which extra telemetry values are sent to the GCS
  // @Range: 1 10
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_SP_TLM_RT     = bind_add_param('TLM_RT',     9, 0)    -- rate for extra telemetry values

--[[
  // @Param: EFI_SP_LOG_RT
  // @DisplayName: SkyPower EFI log rate
  // @Description: SkyPower EFI log rate. This is the rate at which extra logging of the SkyPower EFI is performed
  // @Range: 1 50
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_SP_LOG_RT     = bind_add_param('LOG_RT',    10, 10)   -- rate for logging

--[[
  // @Param: EFI_SP_ST_DISARM
  // @DisplayName: SkyPower EFI allow start disarmed
  // @Description: SkyPower EFI allow start disarmed. This controls if starting the engine while disarmed is allowed
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_SP_ST_DISARM  = bind_add_param('ST_DISARM', 11, 0)    -- allow start when disarmed

--[[
  // @Param: EFI_SP_MODEL
  // @DisplayName: SkyPower EFI ECU model
  // @Description: SkyPower EFI ECU model
  // @Values: 0:Default,1:SP_275
  // @User: Standard
--]]
local EFI_SP_MODEL  = bind_add_param('MODEL', 12, MODEL_DEFAULT)

--[[
  // @Param: EFI_SP_GEN_CTRL
  // @DisplayName: SkyPower EFI enable generator control
  // @Description: SkyPower EFI enable generator control
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_SP_GEN_CTRL  = bind_add_param('GEN_CRTL', 13, 1)

--[[
  // @Param: EFI_SP_RST_TIME
  // @DisplayName: SkyPower EFI restart time
  // @Description: SkyPower EFI restart time. If engine should be running and it has stopped for this amount of time then auto-restart. To disable this feature set this value to zero.
  // @Range: 0 10
  // @User: Standard
  // @Units: s
--]]
local EFI_SP_RST_TIME  = bind_add_param('RST_TIME', 14, 2)

if EFI_SP_ENABLE:get() == 0 then
   gcs:send_text(0, string.format("EFISP: disabled"))
   return
end

-- Register for the CAN drivers
local driver1

local CAN_BUF_LEN = 25
if EFI_SP_CANDRV:get() == 1 then
   driver1 = CAN.get_device(CAN_BUF_LEN)
elseif EFI_SP_CANDRV:get() == 2 then
   driver1 = CAN.get_device2(CAN_BUF_LEN)
end

if not driver1 then
    gcs:send_text(0, string.format("EFISP: Failed to load driver"))
    return
end


local now_s = get_time_sec()

function C_TO_KELVIN(temp)
   return temp + 273.15
end

--[[
   we allow the engine to run if either armed or the EFI_SP_ST_DISARM parameter is 1
--]]
function allow_run_engine()
  return arming:is_armed() or EFI_SP_ST_DISARM:get() == 1
end

--[[
   EFI Engine Object
--]]
local function engine_control(_driver)
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
    local inj_time = 0
    local target_load = 0
    local current_load = 0
    local throttle_angle = 0
    local ignition_angle = 0
    local supply_voltage = 0
    local fuel_consumption_lph = 0
    local fuel_total_l = 0
    local last_fuel_s = 0
    local driver = _driver
    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()
    local last_thr_update = get_time_sec()
    local last_telem_update = get_time_sec()
    local last_log_t = get_time_sec()
    local last_stop_message_t = get_time_sec()
    local engine_started = false
    local generator_started = false
    local engine_start_t = 0.0
    local last_throttle = 0.0
    local sensor_error_flags = 0
    local thermal_limit_flags = 0
    local starter_rpm = 0

    -- frames for sending commands
    local FRM_500 = uint32_t(0x500)
    local FRM_505 = uint32_t(0x505)
    local FRM_506 = uint32_t(0x506)

    -- Generator Data Structure
    local gen        = {}
    gen.amps         = 0.0
    gen.rpm          = 0.0
    gen.batt_current = 0.0

    -- Temperature Data Structure
    local temps = {}
    temps.egt = 0.0        -- Exhaust Gas Temperature
    temps.cht = 0.0        -- Cylinder Head Temperature
    temps.imt = 0.0        -- intake manifold temperature
    temps.oilt = 0.0        -- oil temperature
    temps.cht2 = 0.0
    temps.egt2 = 0.0
    temps.imt2 = 0.0
    temps.oil2 = 0.0

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

            -- All Frame IDs for this EFI Engine are not extended
            if not frame:isExtended() then
                self.handle_EFI_packet(frame)
            end
        end
        if last_rpm_t > last_state_update_t then
           -- update state if we have an updated RPM
           last_state_update_t = last_rpm_t
           self.set_EFI_State()
        end
    end

    -- handle an EFI packet
    function self.handle_EFI_packet(frame)
       local id = frame:id_signed()
       if EFI_SP_MODEL:get() == MODEL_SP_275 then
          -- updated telemetry for SP-275 ECU
          if id == 0x100 then
             rpm = get_uint16(frame, 0)
             ignition_angle = get_uint16(frame, 2) * 0.1
             throttle_angle = get_uint16(frame, 4) * 0.1
             last_rpm_t = get_time_sec()
          elseif id == 0x101 then
             air_pressure = get_uint16(frame, 4)
          elseif id == 0x102 then
             inj_time = get_uint16(frame, 4)
             -- inj_ang = get_uint16(frame, 2) * 0.1
          elseif id == 0x104 then
             supply_voltage = get_uint16(frame, 0) * 0.1
          elseif id == 0x105 then
             temps.cht = get_uint16(frame, 0) * 0.1
             temps.imt = get_uint16(frame, 2) * 0.1
             temps.egt = get_uint16(frame, 4) * 0.1
             temps.oilt = get_uint16(frame, 6) * 0.1
          elseif id == 0x107 then
             sensor_error_flags = get_uint16(frame, 0)
             thermal_limit_flags = get_uint16(frame, 2)
          elseif id == 0x107 then
             target_load = get_uint16(frame, 6) * 0.1
          elseif id == 0x10C then
             temps.cht2 = get_uint16(frame, 4) * 0.1
             temps.egt2 = get_uint16(frame, 6) * 0.1
          elseif id == 0x10D then
             current_load = get_uint16(frame, 2) * 0.1
          elseif id == 0x113 then
             gen.amps = get_uint16(frame, 2)
          elseif id == 0x2E0 then
             starter_rpm = get_uint16(frame, 4)
          elseif id == 0x10B then
             fuel_consumption_lph = get_uint16(frame,6)*0.001
             if last_fuel_s > 0 then
                local dt = now_s - last_fuel_s
                local fuel_lps = fuel_consumption_lph / 3600.0
                fuel_total_l = fuel_total_l + fuel_lps * dt
             end
             last_fuel_s = now_s
          end
       else
          -- original SkyPower driver
          if id == 0x100 then
             rpm = get_uint16(frame, 0)
             ignition_angle = get_uint16(frame, 2) * 0.1
             throttle_angle = get_uint16(frame, 4) * 0.1
             last_rpm_t = get_time_sec()
          elseif id == 0x101 then
             current_load = get_uint16(frame, 0) * 0.1
             target_load = get_uint16(frame, 2) * 0.1
             inj_time = get_uint16(frame, 4)
             -- inj_ang = get_uint16(frame, 6) * 0.1
          elseif id == 0x104 then
             supply_voltage = get_uint16(frame, 0) * 0.1
             ecu_temp = get_uint16(frame, 2) * 0.1
             air_pressure = get_uint16(frame, 4)
             fuel_consumption_lph = get_uint16(frame,6)*0.001
             if last_fuel_s > 0 then
                local dt = now_s - last_fuel_s
                local fuel_lps = fuel_consumption_lph / 3600.0
                fuel_total_l = fuel_total_l + fuel_lps * dt
             end
             last_fuel_s = now_s
          elseif id == 0x105 then
             temps.cht = get_uint16(frame, 0) * 0.1
             temps.imt = get_uint16(frame, 2) * 0.1
             temps.egt = get_uint16(frame, 4) * 0.1
             temps.oilt = get_uint16(frame, 6) * 0.1
          end
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(C_TO_KELVIN(temps.cht))
       cylinder_state:exhaust_gas_temperature(C_TO_KELVIN(temps.egt))
       cylinder_state:ignition_timing_deg(ignition_angle)
       cylinder_state:injection_time_ms(inj_time*0.001)

       efi_state:engine_speed_rpm(uint32_t(rpm))
       efi_state:engine_load_percent(math.floor(current_load))

       efi_state:fuel_consumption_rate_cm3pm(fuel_consumption_lph * 1000.0 / 60.0)
       efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_l * 1000.0)
       efi_state:throttle_position_percent(math.floor(throttle_angle*100.0/90.0+0.5))
       efi_state:atmospheric_pressure_kpa(air_pressure*0.1)
       efi_state:ignition_voltage(supply_voltage)
       efi_state:intake_manifold_temperature(C_TO_KELVIN(temps.imt))
       efi_state:throttle_out(last_throttle * 100)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)

       local last_efi_state_time = millis()
       efi_state:last_updated_ms(last_efi_state_time)


        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    --- send throttle command, thr is 0 to 1
    function self.send_throttle(thr)
       last_throttle = thr
       local msg = CANFrame()
       msg:id(FRM_500)
       msg:data(0,1)
       msg:data(1,0)
       thr = math.floor(thr*1000)
       msg:data(2,thr&0xFF)
       msg:data(3,thr>>8)
       msg:dlc(8)
       driver:write_frame(msg, 10000)
    end

    -- send an engine start command
    function self.send_engine_start()
       if EFI_SP_MODEL:get() == MODEL_SP_275 then
          -- the SP-275 needs a stop before a start will work
          self.send_engine_stop()
       end
       local msg = CANFrame()
       msg:id(FRM_505)
       msg:data(0,10)
       msg:dlc(8)
       driver:write_frame(msg, 10000)
    end

    -- send an engine stop command
    function self.send_engine_stop()
       local msg = CANFrame()
       msg:id(FRM_505)
       msg:data(7,10)
       msg:dlc(8)
       driver:write_frame(msg, 10000)
       local now = get_time_sec()
       if now - last_stop_message_t > 0.5 then
          last_stop_message_t = now
          gcs:send_text(0, string.format("EFISP: stopping engine"))
       end
    end

    -- start generator
    function self.send_generator_start()
       local msg = CANFrame()
       msg:id(FRM_506)
       msg:data(2,10)
       msg:dlc(8)
       driver:write_frame(msg, 10000)
    end

    -- stop generator
    function self.send_generator_stop()
       local msg = CANFrame()
       msg:id(FRM_506)
       msg:data(2,0)
       msg:dlc(8)
       driver:write_frame(msg, 10000)
    end
    
    -- update starter control
    function self.update_starter()
       local start_fn = EFI_SP_START_FN:get()
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
          gcs:send_text(0, string.format("EFISP: starting engine"))
          engine_start_t = get_time_sec()
          self.send_engine_start()
          should_be_running = true
       end
       if start_state > 0 and engine_started and allow_run_engine() then
          should_be_running = true
       end
       local min_rpm = EFI_SP_MIN_RPM:get()
       if min_rpm > 0 and engine_started and rpm < min_rpm and allow_run_engine() then
          local now = get_time_sec()
          local dt = now - engine_start_t
          if EFI_SP_RST_TIME:get() > 0 and dt > EFI_SP_RST_TIME:get() then
             gcs:send_text(0, string.format("EFISP: re-starting engine"))
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

    -- update generator control
    function self.update_generator()
       if EFI_SP_GEN_CTRL:get() == 0 then
          return
       end
       local gen_state = rc:get_aux_cached(EFI_SP_GEN_FN:get())
       if gen_state == 0 and generator_started then
          generator_started = false
          gcs:send_text(0, string.format("EFISP: stopping generator"))
          self.send_generator_stop()
       end
       if gen_state == 2 and not generator_started then
          generator_started = true
          gcs:send_text(0, string.format("EFISP: starting generator"))
          self.send_generator_start()
       end
    end
    
    -- update throttle output
    function self.update_throttle()
       local thr_func = EFI_SP_THR_FN:get()
       local thr_rate = EFI_SP_THR_RATE:get()
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
       local rate = EFI_SP_TLM_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_telem_update < 1.0 / rate then
          return
       end
       last_telem_update = now
       gcs:send_named_float('EFI_OILTMP', temps.oilt)
       gcs:send_named_float('EFI_TRLOAD', target_load)
       gcs:send_named_float('EFI_VOLTS', supply_voltage)
       gcs:send_named_float('EFI_GEN_AMPS', gen.amps)
       gcs:send_named_float('EFI_CHT2', temps.cht2)
       gcs:send_named_float('EFI_STARTRPM', starter_rpm)
    end

    -- update custom logging
    function self.update_logging()
       local rate = EFI_SP_LOG_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_log_t < 1.0 / rate then
          return
       end
       last_log_t = now
       logger.write('EFSP','Thr,CLoad,TLoad,OilT,RPM,gRPM,gAmp,gCur,SErr,TLim,STRPM', 'ffffffffHHH',
                    last_throttle, current_load, target_load, temps.oilt, rpm,
                    gen.rpm, gen.amps, gen.batt_current,
                    sensor_error_flags, thermal_limit_flags,
                    starter_rpm)
    end
    
    -- return the instance
    return self
end -- end function engine_control

local engine1 = engine_control(driver1)

function update()
   now_s = get_time_sec()

   if not efi_backend then
      efi_backend = efi:get_backend(0)
      if not efi_backend then
         return
      end
   end

   -- Parse Driver Messages
   engine1.update_telemetry()
   engine1.update_starter()
   engine1.update_generator()
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
        gcs:send_text(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 1000 / EFI_SP_UPDATE_HZ:get()
end

-- start running update loop
return protected_wrapper()
