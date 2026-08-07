--[[ 
  EFI Scripting backend driver for HFE based on HFEDCN0191 Rev L
--]]
---@diagnostic disable: param-type-mismatch
---@diagnostic disable: undefined-field
---@diagnostic disable: missing-parameter
---@diagnostic disable: need-check-nil


-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = 4.3
local SCRIPT_NAME       = "EFI: HFE CAN"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s Requires: %s:%.1f. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))

local MAV_SEVERITY_ERROR = 3

PARAM_TABLE_KEY = 37
PARAM_TABLE_PREFIX = "EFI_HFE_"

K_THROTTLE = 70
K_IGNITION = 67

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
    return frame:data(ofs+1) + (frame:data(ofs) << 8)
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
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add EFI_HFE param table')

--[[
  // @Param: EFI_HFE_ENABLE
  // @DisplayName: Enable HFE EFI driver
  // @Description: Enable HFE EFI driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local EFI_HFE_ENABLE = bind_add_param('ENABLE',  1, 0)

--[[
  // @Param: EFI_HFE_RATE_HZ
  // @DisplayName: HFI EFI Update rate
  // @Description: HFI EFI Update rate
  // @Range: 0 400
  // @User: Standard
--]]
local EFI_HFE_RATE_HZ  = bind_add_param('RATE_HZ',  2, 200)

--[[
  // @Param: EFI_HFE_ECU_IDX
  // @DisplayName: HFI EFI ECU index
  // @Description: HFI EFI ECU index, 0 for automatic
  // @Range: 0 10
  // @User: Standard
--]]
local EFI_HFE_ECU_IDX = bind_add_param('ECU_IDX',  3, 0)

--[[
  // @Param: EFI_HFE_FUEL_DTY
  // @DisplayName: HFI EFI fuel density
  // @Description: HFI EFI fuel density in gram per litre
  // @Range: 0 2000
  // @User: Standard
--]]
local EFI_HFE_FUEL_DTY = bind_add_param('FUEL_DTY',  4, 740)

--[[
  // @Param: EFI_HFE_REL_IDX
  // @DisplayName: HFI EFI relay index
  // @Description: HFI EFI relay index
  // @Range: 0 10
  // @User: Standard
--]]
local EFI_HFE_REL_IDX = bind_add_param('REL_IDX',  5, 0)

--[[
  // @Param: EFI_HFE_CANDRV
  // @DisplayName: HFI EFI CAN driver
  // @Description: HFI EFI CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local EFI_HFE_CANDRV = bind_add_param('CANDRV',  6, 0)

--[[
  // @Param: EFI_HFE_OPTIONS
  // @DisplayName: HFI EFI options
  // @Description: HFI EFI options
  // @Bitmask: 1:EnableCANLogging
  // @User: Standard
--]]
local EFI_HFE_OPTIONS = bind_add_param('OPTIONS',  7, 0)

local OPTION_LOGALLFRAMES = 0x01

-- on 4.6.x this will be nil and direct relay support in AP_ICEngine can be used
local ICE_PWM_IGN_ON = nil
if param:get("ICE_PWM_IGN_ON") then
    ICE_PWM_IGN_ON = Parameter("ICE_PWM_IGN_ON")
end

if EFI_HFE_ENABLE:get() == 0 then
   return
end

-- Register for the CAN drivers
local CAN_BUF_LEN = 25
if EFI_HFE_CANDRV:get() == 1 then
   driver1 = CAN.get_device(CAN_BUF_LEN)
elseif EFI_HFE_CANDRV:get() == 2 then
   driver1 = CAN.get_device2(CAN_BUF_LEN)
end

if not driver1 then
    gcs:send_text(0, string.format("EFI_HFE: Failed to load driver"))
    return
end

local frame_count = 0

--[[
   frame logging - can be replayed with Tools/scripts/CAN/CAN_playback.py
--]]
local function log_can_frame(frame)
    logger:write("CANF",'Id,DLC,FC,B0,B1,B2,B3,B4,B5,B6,B7','IBIBBBBBBBB',
                 frame:id(),
                 frame:dlc(),
                 frame_count,
                 frame:data(0), frame:data(1), frame:data(2), frame:data(3),
                 frame:data(4), frame:data(5), frame:data(6), frame:data(7))
    frame_count = frame_count + 1
end

--[[
   in 4.5.x temperatures in EFI state structure were incorrectly
   used as C instead of Kelvin
--]]
local temp_offset = 0.0
if FWVersion:major() == 4 and FWVersion:minor() <= 5 then
    temp_offset = -273.15
end

local now_s = get_time_sec()

--[[
   EFI Engine Object
--]]
local function engine_control(driver)
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
    local map_ratio = 0.0
    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()
    local throttle_pos = 0.0
    local last_thr_t = get_time_sec()
    local C_TO_KELVIN = 273.2
    local fuel_flow_gph = 0.0
    local fuel_total_g = 0.0
    local fuel_press = 0.0
    local last_fuel_s = 0.0
    local ecu_voltage = 0.0
    local injector_duty = 0.0
    local ignition_angle = 0.0

    -- Temperature Data Structure
    local temps = {}
    temps.egt = 0.0        -- Engine Gas Temperature
    temps.iat = 0.0        -- inlet air temperature
    temps.mat = 0.0        -- manifold air temperature
    temps.cht = 0.0        -- cylinder head temperature

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
            if EFI_HFE_OPTIONS:get() & OPTION_LOGALLFRAMES ~= 0 then
                log_can_frame(frame)
            end

            -- All Frame IDs for this EFI Engine are in the 29-bit extended address space
            if frame:isExtended() then
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
       if id >> 24 ~= 0x08 then
          -- not from ECU
          return
       end
       local ecu = id & 0xff
       if ecu ~= 0 and EFI_HFE_ECU_IDX:get() == 0 then
          EFI_HFE_ECU_IDX:set(ecu)
          gcs:send_text(0, string.format("EFI_HFE: found ECU %u", ecu))
       end
       if ecu ~= EFI_HFE_ECU_IDX:get() then
          -- not from correct ECU
          return
       end
       local cmd = (id >> 16) & 0xff
       if cmd == 0x0 then
          -- fast telemetry
          throttle_pos = get_uint8(frame, 0)
          rpm = get_uint16(frame, 1)
          last_rpm_t = get_time_sec()
       elseif cmd == 0x01 then
          -- slow telem0
          air_pressure = get_uint16(frame, 5) * 2
          map_ratio = get_uint8(frame, 7) * 0.01
          temps.cht = get_uint8(frame,3) - 10.0
       elseif cmd == 0x02 then
          -- slow telem1
          temps.iat = get_uint8(frame, 0)
          ecu_voltage = get_uint8(frame, 6) * 0.1
          fuel_press = get_uint16(frame,1)*20.0
       elseif cmd == 0x03 then
          -- slow telem2
          temps.mat = (1.5*get_uint8(frame, 1)) - 128
          fuel_flow_gph = get_uint16(frame, 6)
          if last_fuel_s > 0 then
             local dt = now_s - last_fuel_s
             local fuel_gps = fuel_flow_gph / 3600.0
             fuel_total_g = fuel_total_g + fuel_gps * dt
          end
          injector_duty = get_uint16(frame,2)
          last_fuel_s = now_s
          ignition_angle = get_uint8(frame,4)*2.0
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(temps.cht + C_TO_KELVIN + temp_offset)
       cylinder_state:exhaust_gas_temperature(temps.mat + temp_offset)
       cylinder_state:ignition_timing_deg(ignition_angle)
       if rpm > 0 then
          cylinder_state:injection_time_ms((60.0/rpm)*1000*injector_duty)
       else
          cylinder_state:injection_time_ms(0)
       end

       efi_state:engine_speed_rpm(uint32_t(rpm))

       efi_state:atmospheric_pressure_kpa(air_pressure*0.001)
       efi_state:intake_manifold_pressure_kpa(air_pressure*0.001*map_ratio)
       efi_state:intake_manifold_temperature(temps.mat + C_TO_KELVIN + temp_offset)
       efi_state:throttle_position_percent(math.floor((throttle_pos*100/255)+0.5))
       efi_state:ignition_voltage(ecu_voltage)
       efi_state:fuel_pressure(fuel_press*0.001)
       efi_state:fuel_pressure_status(1) -- Fuel_Pressure_Status::OK

       local gram_to_cm3 = EFI_HFE_FUEL_DTY:get() * 0.001
       efi_state:fuel_consumption_rate_cm3pm((fuel_flow_gph/60.0) * gram_to_cm3)
       efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_g * gram_to_cm3)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)

       efi_state:last_updated_ms(millis())

        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    -- send throttle
    function self.send_throttle()
       if now_s - last_thr_t < 0.02 then
          -- limit to 50Hz
          return
       end
       last_thr_t = now_s
       local thr = SRV_Channels:get_output_scaled(K_THROTTLE)
       local msg = CANFrame()
       msg:id(uint32_t(0x89060000 | EFI_HFE_ECU_IDX:get()))
       msg:data(0,math.floor((thr*255/100)+0.5))
       msg:dlc(2)
       driver:write_frame(msg, 10000)

       -- throttle calibration request, for debug
       --msg = CANFrame()
       --msg:id(uint32_t(0x89050000 | EFI_HFE_ECU_IDX:get()))
       --msg:dlc(0)
       --driver:write_frame(msg, 10000)

       -- map K_IGNITION to relay for enable of engine
       local relay_idx = EFI_HFE_REL_IDX:get()
       if relay_idx > 0 and ICE_PWM_IGN_ON then
          local ignition_pwm = SRV_Channels:get_output_pwm(K_IGNITION)
          if ignition_pwm == ICE_PWM_IGN_ON:get() then
             relay:on(relay_idx-1)
          else
             relay:off(relay_idx-1)
          end
       end
    end
    
    -- return the instance
    return self
end -- end function engine_control(driver)

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
   engine1.send_throttle()
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
    return protected_wrapper, 1000 / EFI_HFE_RATE_HZ:get()
end

-- start running update loop
return protected_wrapper()
