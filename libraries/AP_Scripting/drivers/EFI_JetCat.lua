--[[ 
 EFI Scripting backend driver for JetCat ECU

 See https://www.jetcat.de/en/productdetails/produkte/jetcat/produkte/zubehoer/PRO/JetCat%20PRO-Interface
--]]

local PARAM_TABLE_KEY = 42
local PARAM_TABLE_PREFIX = "EFI_JC_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local K_THROTTLE = 70
local K_HELIRSC = 31

-- based CAN ID, can be set by user
local BASE0 = 0x100

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

-- Type conversion functions, big-endian data
function get_uint16(frame, ofs)
    return frame:data(ofs+1) + (frame:data(ofs) << 8)
end

function get_int16(frame, ofs)
   v = get_uint16(frame,ofs)
   if v > 32767 then
      v = v - 65536
   end
   return v
end

function get_uint32(frame, ofs)
   return (frame:data(ofs)<<24) + (frame:data(ofs+1)<<16) + (frame:data(ofs+2)<<8) + frame:data(ofs+3)
end

function get_int8(frame, ofs)
   local v = frame:data(ofs)
   if v > 127 then
      v = v - 256
   end
   return v
end

function constrain(v, vmin, vmax)
   return math.max(math.min(v, vmax), vmin)
end

local efi_backend = nil

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not param table')

--[[
  // @Param: EFI_JC_ENABLE
  // @DisplayName: Enable JetCat EFI support
  // @Description: Enable JetCat EFI support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_JC_ENABLE     = bind_add_param('ENABLE',     1, 0)

--[[
  // @Param: EFI_JC_CANDRV
  // @DisplayName: Set JetCat EFI CAN driver
  // @Description: Set JetCat EFI CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local EFI_JC_CANDRV     = bind_add_param('CANDRV',     2, 1)    -- CAN driver to use

--[[
  // @Param: EFI_JC_UPDATE_HZ
  // @DisplayName: JetCat EFI update rate
  // @Description: JetCat EFI update rate
  // @Range: 10 200
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_JC_UPDATE_HZ  = bind_add_param('UPDATE_HZ',  3, 200)  -- Script update frequency in Hz

--[[
  // @Param: EFI_JC_THR_FN
  // @DisplayName: JetCat EFI throttle function
  // @Description: JetCat EFI throttle function. This sets which SERVOn_FUNCTION to use for the target throttle. This should be 70 for fixed wing aircraft and 31 for helicopter rotor speed control
  // @Values: 0:Disabled,70:FixedWing,31:HeliRSC
  // @User: Standard
--]]
local EFI_JC_THR_FN     = bind_add_param('THR_FN',     4, 0)    -- servo function for throttle

--[[
  // @Param: EFI_JC_THR_RATE
  // @DisplayName: JetCat EFI throttle rate
  // @Description: JetCat EFI throttle rate. This sets rate at which throttle updates are sent to the engine
  // @Range: 10 100
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_JC_THR_RATE   = bind_add_param('THR_RATE',   5, 50)   -- throttle update rate

--[[
  // @Param: EFI_JC_START_FN
  // @DisplayName: JetCat EFI start function
  // @Description: JetCat EFI start function. This is the RCn_OPTION value to use to find the R/C channel used for controlling engine start
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFI_JC_START_FN   = bind_add_param('START_FN',   6, 0)    -- start control function (RC option)

--[[
  // @Param: EFI_JC_MIN_RPM
  // @DisplayName: JetCat EFI minimum RPM
  // @Description: JetCat EFI minimum RPM. This is the RPM below which the engine is considered to be stopped
  // @Range: 1 1000
  // @User: Advanced
--]]
local EFI_JC_MIN_RPM    = bind_add_param('MIN_RPM',    7, 100)  -- min RPM, for engine restart

--[[
  // @Param: EFI_JC_TLM_RT
  // @DisplayName: JetCat EFI telemetry rate
  // @Description: JetCat EFI telemetry rate. This is the rate at which extra telemetry values are sent to the GCS
  // @Range: 1 10
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_JC_TLM_RT     = bind_add_param('TLM_RT',     8, 0)    -- rate for extra telemetry values

--[[
  // @Param: EFI_JC_LOG_RT
  // @DisplayName: JetCat EFI log rate
  // @Description: JetCat EFI log rate. This is the rate at which extra logging of the JetCat EFI is performed
  // @Range: 1 50
  // @Units: Hz
  // @User: Advanced
--]]
local EFI_JC_LOG_RT     = bind_add_param('LOG_RT',    9, 10)   -- rate for logging

--[[
  // @Param: EFI_JC_ST_DISARM
  // @DisplayName: JetCat EFI allow start disarmed
  // @Description: JetCat EFI allow start disarmed. This controls if starting the engine while disarmed is allowed
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_JC_ST_DISARM  = bind_add_param('ST_DISARM', 10, 0)    -- allow start when disarmed

if EFI_JC_ENABLE:get() == 0 then
   gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_JC: disabled"))
   return
end

-- Register for the CAN drivers
local driver

local CAN_BUF_LEN = 25
if EFI_JC_CANDRV:get() == 1 then
   driver = CAN.get_device(CAN_BUF_LEN)
elseif EFI_JC_CANDRV:get() == 2 then
   driver = CAN.get_device2(CAN_BUF_LEN)
end

if not driver then
    gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_JC: Failed to load driver"))
    return
end


function C_TO_KELVIN(temp)
   return temp + 273.15
end

--[[
   we allow the engine to run if either armed or the EFI_JC_ST_DISARM parameter is 1
--]]
function allow_run_engine()
  return arming:is_armed() or EFI_JC_ST_DISARM:get() == 1
end

--[[
   EFI Engine Object
--]]
local function engine_control()
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local telem = {}
    telem.setpoint_rpm = 0
    telem.rpm = 0
    telem.egt = 0
    telem.state = 0
    telem.pump_volts = 0
    telem.batt_volt = 0
    telem.batt_amps = 0
    telem.gen_volts = 0
    telem.gen_amps = 0
    telem.flags1 = 0
    telem.batt_pct = 0
    telem.flags2 = 0
    telem.fuel_flow_mlm = 0
    telem.fuel_consumed_ml = 0
    telem.ambient_pressure = 0
    telem.egt_comp = 0
    telem.rem_fuel_pct = 0
    telem.runs = 0
    telem.runs_aborted = 0
    telem.run_total = 0
    telem.last_runtime = 0
    telem.last_off_RPM = 0
    telem.last_off_EGT = 0
    telem.last_off_pump_volts = 0
    telem.last_off_cond = 0

    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()
    local last_thr_update = get_time_sec()
    local last_telem_update = get_time_sec()
    local last_log_t = get_time_sec()
    local engine_started = false
    local engine_start_t = 0.0
    local last_throttle = 0.0

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

            -- All Frame IDs for this EFI Engine are not extended by default
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
       if id == BASE0+0 then
          telem.setpoint_rpm = get_uint16(frame, 0)*10
          telem.rpm = get_uint16(frame, 2)*10
          telem.egt = get_uint16(frame, 4)*0.1
          telem.state = frame:data(6)
          telem.pump_volts = get_int8(frame, 7)
          last_rpm_t = get_time_sec()
       elseif id == BASE0+1 then
          telem.batt_volt = frame:data(0)*0.1
          telem.batt_amps = frame:data(1)*0.1
          telem.gen_volts = frame:data(2)*0.2
          telem.gen_amps = frame:data(3)*0.1
          telem.flags1 = frame:data(4)
          telem.batt_pct = frame:data(5)
          telem.flags2 = frame:data(6)
          telem.mode = frame:data(7)
       elseif id == BASE0+2 then
          telem.fuel_flow_mlm = get_uint16(frame,0)
          telem.fuel_consumed_ml = get_uint16(frame, 2)*10
          telem.ambient_pressure = get_uint16(frame, 4)*0.02
          telem.egt_comp = get_uint16(frame, 6)*0.5 - 30.0
          telem.rem_fuel_pct = frame:data(7)*0.5
       elseif id == BASE0+3 then
          telem.runs = get_uint16(frame, 0)
          telem.runs_aborted = get_uint16(frame, 2)
          telem.run_total = get_uint32(frame, 4)
       elseif id == BASE0+4 then
          telem.last_runtime = get_uint16(frame, 0)
          telem.last_off_RPM = get_uint16(frame, 2)*10
          telem.last_off_EGT = get_int16(frame, 4)
          telem.last_off_pump_volts = frame:data(6)
          telem.last_off_cond = frame:data(7)
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(C_TO_KELVIN(telem.egt))
       cylinder_state:exhaust_gas_temperature(C_TO_KELVIN(telem.egt))
       efi_state:engine_speed_rpm(uint32_t(telem.rpm))

       efi_state:fuel_consumption_rate_cm3pm(telem.fuel_flow_mlm)
       efi_state:estimated_consumed_fuel_volume_cm3(telem.fuel_consumed_ml)
       efi_state:atmospheric_pressure_kpa(telem.ambient_pressure*0.1)
       efi_state:ignition_voltage(telem.batt_volt)
       efi_state:intake_manifold_temperature(telem.egt_comp)
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
       -- disabled for now
    end

    -- send an engine start command
    function self.send_engine_start()
       -- disabled for now
    end

    -- send an engine stop command
    function self.send_engine_stop()
       -- disabled for now
    end

    -- update starter control
    function self.update_starter()
       local start_fn = EFI_JC_START_FN:get()
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
          gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_JC: starting engine"))
          engine_start_t = get_time_sec()
          self.send_engine_start()
          should_be_running = true
       end
       if start_state > 0 and engine_started and allow_run_engine() then
          should_be_running = true
       end
       local min_rpm = EFI_JC_MIN_RPM:get()
       if min_rpm > 0 and engine_started and rpm < min_rpm and allow_run_engine() then
          local now = get_time_sec()
          local dt = now - engine_start_t
          if dt > 2.0 then
             gcs:send_text(MAV_SEVERITY.INFO, string.format("EFI_JC: re-starting engine"))
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
       local thr_func = EFI_JC_THR_FN:get()
       local thr_rate = EFI_JC_THR_RATE:get()
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
       local rate = EFI_JC_TLM_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_telem_update < 1.0 / rate then
          return
       end
       last_telem_update = now
       gcs:send_named_float('JC_PUMPV', telem.pump_volts)
    end

    -- update custom logging
    function self.update_logging()
       local rate = EFI_JC_LOG_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_log_t < 1.0 / rate then
          return
       end
       last_log_t = now
       logger.write('JCAT','Thr,PumpV', 'ff',
                    last_throttle, telem.pump_volts)
    end
    
    -- return the instance
    return self
end

local engine = engine_control()

function update()
   if not efi_backend then
      efi_backend = efi:get_backend(0)
      if not efi_backend then
         return
      end
   end

   -- Parse Driver Messages
   engine.update_telemetry()
   engine.update_starter()
   engine.update_throttle()
   engine.update_telem_out()
   engine.update_logging()
end

gcs:send_text(MAV_SEVERITY.INFO, "EFI JetCat loaded")

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
    return protected_wrapper, 1000 / EFI_JC_UPDATE_HZ:get()
end

-- start running update loop
return protected_wrapper()
