--[[ 
  EFI Scripting backend driver for Halo6000 generator
--]]

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: undefined-field
---@diagnostic disable: missing-parameter
---@diagnostic disable: need-check-nil

-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = 4.3
local SCRIPT_NAME       = "EFI: Halo6000 CAN"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s Requires: %s:%.1f. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 40
PARAM_TABLE_PREFIX = "EFI_H6K_"

UPDATE_RATE_HZ = 40

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
    return frame:data(ofs) + (frame:data(ofs+1) << 8)
end

function constrain(v, vmin, vmax)
   return math.max(math.min(v,vmax),vmin)
end

local efi_backend = nil

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add EFI_H6K param table')

--[[
  // @Param: EFI_H6K_ENABLE
  // @DisplayName: Enable Halo6000 EFI driver
  // @Description: Enable Halo6000 EFI driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_H6K_ENABLE = bind_add_param('ENABLE',  1, 0)

--[[
  // @Param: EFI_H6K_CANDRV
  // @DisplayName: Halo6000 CAN driver
  // @Description: Halo6000 CAN driver. Use 1 for first CAN scripting driver, 2 for 2nd driver
  // @Values: 0:Disabled,1:FirstCAN,2:SecondCAN
  // @User: Standard
--]]
local EFI_H6K_CANDRV = bind_add_param('CANDRV',  2, 0)   -- CAN driver number

--[[
  // @Param: EFI_H6K_START_FN
  // @DisplayName: Halo6000 start auxilliary function
  // @Description: The RC auxilliary function number for start/stop of the generator. Zero to disable start function
  // @Values: 0:Disabled,300:300,301:301,302:302,303:303,304:304,305:305,306:306,307:307
  // @User: Standard
--]]
local EFI_H6K_START_FN = bind_add_param('START_FN', 3, 0)

--[[
  // @Param: EFI_H6K_TELEM_RT
  // @DisplayName: Halo6000 telemetry rate
  // @Description: The rate that additional generator telemetry is sent
  // @Units: Hz
  // @User: Standard
--]]
local EFI_H6K_TELEM_RT = bind_add_param('TELEM_RT', 4, 2)

--[[
  // @Param: EFI_H6K_FUELTOT
  // @DisplayName: Halo6000 total fuel capacity
  // @Description: The capacity of the tank in litres
  // @Units: litres
  // @User: Standard
--]]
local EFI_H6K_FUELTOT = bind_add_param('FUELTOT', 5, 20)

--[[
  // @Param: EFI_H6K_OPTIONS
  // @DisplayName: Halo6000 options
  // @Description: Halo6000 options
  // @Bitmask: 0:LogAllCanPackets
  // @User: Standard
--]]
local EFI_H6K_OPTIONS = bind_add_param('OPTIONS', 6, 0)

local OPTION_LOGALLFRAMES = 0x01

if EFI_H6K_ENABLE:get() == 0 then
   return
end

-- Register for the CAN drivers
local CAN_BUF_LEN = 25
if EFI_H6K_CANDRV:get() == 1 then
   gcs:send_text(0, string.format("EFI_H6K: attaching to CAN1"))
   driver1 = CAN.get_device(CAN_BUF_LEN)
elseif EFI_H6K_CANDRV:get() == 2 then
   gcs:send_text(0, string.format("EFI_H6K: attaching to CAN2"))
   driver1 = CAN.get_device2(CAN_BUF_LEN)
end

if not driver1 then
    gcs:send_text(0, string.format("EFI_H6K: Failed to load driver"))
    return
end

local frame_count = 0

--[[
   frame logging - can be replayed with Tools/scripts/CAN/CAN_playback.py
--]]
local function log_can_frame(frame)
   logger.write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                frame:id(),
                frame:dlc(),
                frame_count,
                frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                frame:data(4), frame:data(5), frame:data(6), frame:data(7))
   frame_count = frame_count + 1
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
    local rpm = 0
    local throttle_pos = 0
    local fuel_pct = 0
    local cyl_temp = 0
    local cool_temp = 0
    local out_volt = 0
    local out_current = 0
    local last_state_update_t = 0
    local last_rpm_t = 0
    local last_telem_update_t = 0
    local last_stop_message_t = 0
    local C_TO_KELVIN = 273.2
    local engine_started = false

    -- read telemetry packets
    function self.update_telemetry()
        local max_packets = 25
        local count = 0
        while count < max_packets do
            frame = driver1:read_frame()
            count = count + 1
            if not frame then
                break
            end

            if EFI_H6K_OPTIONS:get() & OPTION_LOGALLFRAMES ~= 0 then
               log_can_frame(frame)
            end

            -- All Frame IDs for this EFI Engine are in the 11-bit address space
            if not frame:isExtended() then
                self.handle_packet(frame)
            end
        end
        if last_rpm_t > last_state_update_t then
           -- update state if we have an updated RPM
           last_state_update_t = last_rpm_t
           self.set_EFI_State()
        end
    end

    -- handle an EFI packet
    function self.handle_packet(frame)
       local id = frame:id_signed()
       if id == 0x1c1 then
          -- 20Hz telem
          rpm = get_uint16(frame, 0)
          last_rpm_t = get_time_sec()
          throttle_pos = get_uint16(frame, 2)*0.1
          fuel_pct = get_uint8(frame, 4)*0.5
          cyl_temp = get_uint8(frame, 5) - 40
          cool_temp = get_uint8(frame, 6) - 40
       elseif id == 0x1c2 then
          -- 20Hz telem2
          out_volt = get_uint16(frame, 0)*0.2
          out_current = get_uint16(frame, 2)*0.2 - 200.0
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(cyl_temp + C_TO_KELVIN)
       cylinder_state:exhaust_gas_temperature(cool_temp + C_TO_KELVIN)
       efi_state:engine_speed_rpm(uint32_t(rpm))
       efi_state:intake_manifold_temperature(cool_temp + C_TO_KELVIN)
       efi_state:throttle_position_percent(math.floor(throttle_pos))
       efi_state:ignition_voltage(out_volt)
       efi_state:estimated_consumed_fuel_volume_cm3((100.0-fuel_pct)*0.01*EFI_H6K_FUELTOT:get()*1000)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)
       
       efi_state:last_updated_ms(millis())

       -- Set the EFI_State into the EFI scripting driver
       efi_backend:handle_scripting(efi_state)

       logger.write('H6K','Curr,Volt,CoolT,CylT,FuelPct', 'fffff',
                    out_current, out_volt, cool_temp, cyl_temp, fuel_pct)
    end

    -- update telemetry output for extra telemetry values
    function self.update_telem_out()
       local rate = EFI_H6K_TELEM_RT:get()
       if rate <= 0 then
          return
       end
       local now = get_time_sec()
       if now - last_telem_update_t < 1.0 / rate then
          return
       end
       last_telem_update_t = now
       gcs:send_named_float('H6K_FUEL', fuel_pct)
       gcs:send_named_float('H6K_CYLT', cyl_temp)
       gcs:send_named_float('H6K_COOLT', cool_temp)
       gcs:send_named_float('H6K_AMPS', out_current)
    end

    -- send a frame with checksum
    function self.write_frame_checksum(msg)
        local sum = 0
        for i = 0, 6 do
        sum = sum + msg:data(i)
        end
        local checksum = (0 - sum) % 0x100
        msg:data(7, checksum)
        msg:dlc(8)
        driver1:write_frame(msg, 10000)
    end

    -- send an engine start command
    function self.send_engine_start()
       local msg = CANFrame()
       msg:id(0x1A0)
       msg:data(0,1)
       msg:data(7,1)
       self.write_frame_checksum(msg)
    end

    -- send an engine stop command
    function self.send_engine_stop()
       local msg = CANFrame()
       msg:id(0x1A0)
       msg:data(0,0)
       self.write_frame_checksum(msg)
       local now = get_time_sec()
       if now - last_stop_message_t > 0.5 then
          last_stop_message_t = now
          gcs:send_text(0, string.format("EFIH6K: stopping generator"))
       end
    end

    -- update starter control
    function self.update_starter()
       local start_fn = EFI_H6K_START_FN:get()
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
       if start_state == 2 and not engine_started then
          engine_started = true
          gcs:send_text(0, string.format("EFIH6K: starting generator"))
          engine_start_t = get_time_sec()
          self.send_engine_start()
          should_be_running = true
       end
       if start_state > 0 and engine_started then
          should_be_running = true
       end
       local min_rpm = 500
       if min_rpm > 0 and engine_started and rpm < min_rpm then
          local now = get_time_sec()
          local dt = now - engine_start_t
          if dt > 2.0 then
             gcs:send_text(0, string.format("EFIH6K: re-starting generator"))
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
    
    -- return the instance
    return self
end -- end function engine_control()

local engine1 = engine_control()

function update()
   if not efi_backend then
      efi_backend = efi:get_backend(0)
      if not efi_backend then
         return
      end
   end

   -- Parse Driver Messages
   engine1.update_telemetry()
   engine1.update_telem_out()
   engine1.update_starter()
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
    return protected_wrapper, 1000 / UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
