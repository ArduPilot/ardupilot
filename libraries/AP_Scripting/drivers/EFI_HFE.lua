--[[ 
  EFI Scripting backend driver for HFE based on HFEDCN0191 Rev E
--]]

-- Check Script uses a miniumum firmware version
local SCRIPT_AP_VERSION = 4.4
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

-- Register for the CAN drivers
local driver1 = CAN.get_device(25)

if not driver1 then
    gcs:send_text(0, string.format("EFI CAN Telemetry: Failed to load driver"))
    return
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add EFI_HFE param table')

local EFI_HFE_ENABLE = bind_add_param('ENABLE',  1, 0)
local EFI_HFE_RATE_HZ  = bind_add_param('RATE_HZ',  2, 200)    -- Script update frequency in Hz
local EFI_HFE_ECU_IDX = bind_add_param('ECU_IDX',  3, 0)   -- ECU index on CAN bus, 0 for automatic
local EFI_HFE_FUEL_DTY = bind_add_param('FUEL_DTY',  4, 740)   -- fuel density, g/litre
local EFI_HFE_REL_IDX = bind_add_param('REL_IDX',  5, 0)   -- relay number for engine enable
local ICE_PWM_IGN_ON = bind_param("ICE_PWM_IGN_ON")

if EFI_HFE_ENABLE:get() == 0 then
   return
end

local now_s = get_time_sec()

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
    local map_ratio = 0.0
    local driver = _driver
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

    -- Generator Data Structure
    local gen        = {}
    gen.amps         = 0.0
    gen.rpm          = 0.0
    gen.batt_current = 0.0

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
       cylinder_state:cylinder_head_temperature(temps.cht + C_TO_KELVIN)
       cylinder_state:exhaust_gas_temperature(temps.mat)
       cylinder_state:ignition_timing_deg(ignition_angle)
       if rpm > 0 then
          cylinder_state:injection_time_ms((60.0/rpm)*1000*injector_duty)
       else
          cylinder_state:injection_time_ms(0)
       end

       efi_state:engine_speed_rpm(uint32_t(rpm))

       efi_state:atmospheric_pressure_kpa(air_pressure*0.001)
       efi_state:intake_manifold_pressure_kpa(air_pressure*0.001*map_ratio)
       efi_state:intake_manifold_temperature(temps.mat + C_TO_KELVIN)
       efi_state:throttle_position_percent(math.floor((throttle_pos*100/255)+0.5))
       efi_state:ignition_voltage(ecu_voltage)
       efi_state:fuel_pressure(fuel_press*0.001)

       local gram_to_cm3 = EFI_HFE_FUEL_DTY:get() * 0.001
       efi_state:fuel_consumption_rate_cm3pm((fuel_flow_gph/60.0) * gram_to_cm3)
       efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_g * gram_to_cm3)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)

       efi_state:last_updated_ms(millis())

        -- Set the EFI_State into the EFI scripting driver
        efi:handle_scripting(efi_state)
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
       if relay_idx > 0 then
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
end -- end function engine_control(_driver)

local engine1 = engine_control(driver1, 1)

function update()
   now_s = get_time_sec()

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
