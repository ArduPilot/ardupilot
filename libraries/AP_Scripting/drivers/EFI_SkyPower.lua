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
local SCRIPT_AP_VERSION = 4.4
local SCRIPT_NAME       = "EFI: Skypower CAN"

local VERSION = FWVersion:major() + (FWVersion:minor() * 0.1)

assert(VERSION >= SCRIPT_AP_VERSION, string.format('%s Requires: %s:%.1f. Found Version: %s', SCRIPT_NAME, FWVersion:type(), SCRIPT_AP_VERSION, VERSION))


local MAV_SEVERITY_ERROR = 3

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

---- GLOBAL VARIABLES
local GKWH_TO_LBS_HP_HR = 0.0016439868
local LITRES_TO_LBS = 1.6095 -- 6.1 lbs of fuel per gallon -> 1.6095

-- Register for the CAN drivers
local driver1 = CAN.get_device(25)

if not driver1 then
    gcs:send_text(0, string.format("EFI CAN Telemetry: Failed to load drivers"))
    return
end

local efi_backend = nil

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add EFI_SP param table')

local EFI_SP_ENABLE = bind_add_param('ENABLE',  1, 0)
local EFI_SP_UPDATE_HZ  = bind_add_param('UPDATE_HZ',  2, 200)    -- Script update frequency in Hz

if EFI_SP_ENABLE:get() == 0 then
   return
end

local now_s = get_time_sec()

--[[
   EFI Engine Object
--]]
local function engine_control(_driver, _idx)
    local self = {}

    -- Build up the EFI_State that is passed into the EFI Scripting backend
    local efi_state = EFI_State()
    local cylinder_state = Cylinder_Status()

    -- private fields as locals
    local rpm = 0
    local air_pressure = 0
    local inj_ang = 0
    local inj_time = 0
    local target_load = 0
    local current_load = 0
    local throttle_angle = 0
    local ignition_angle = 0
    local sfc = 0
    local sfc_icao = 0
    local last_sfc_t = 0
    local supply_voltage = 0
    local fuel_consumption_lph = 0
    local fuel_total_l = 0
    local last_fuel_s = 0
    local driver = _driver
    local idx = _idx
    local last_rpm_t = get_time_sec()
    local last_state_update_t = get_time_sec()

    -- Generator Data Structure
    local gen        = {}
    gen.amps         = 0.0
    gen.rpm          = 0.0
    gen.batt_current = 0.0

    -- Temperature Data Structure
    local temps = {}
    temps.egt1 = 0.0        -- Engine Gas Temperature: Sensor 1
    temps.egt2 = 0.0        -- Engine Gas Temperature: Sensor 2
    temps.cht1 = 0.0        -- Cylinder Head Temperature: Sensor 1
    temps.cht2 = 0.0        -- Cylinder Head Temperature: Sensor 2

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

            -- All Frame IDs for this EFI Engine are in the 11-bit address space
            if not frame:isExtended() then
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
    function self.handle_EFI_packet(frame, idx)
       local id = frame:id_signed()
       if id == 0x100 then
          rpm = get_uint16(frame, 0)
          ignition_angle = get_uint16(frame, 2) * 0.1
          throttle_angle = get_uint16(frame, 4) * 0.1
          last_rpm_t = get_time_sec()
       elseif id == 0x101 then
          air_pressure = get_uint16(frame, 4)
       elseif id == 0x102 then
          inj_ang = get_uint16(frame, 2) * 0.1
          inj_time = get_uint16(frame, 4) * 0.1
       elseif id == 0x104 then
          supply_voltage = get_uint16(frame, 0) * 0.1
       elseif id == 0x105 then
          temps.cht1 = get_uint16(frame, 0) * 0.1
          temps.egt1 = get_uint16(frame, 4) * 0.1
       elseif id == 0x10A then
          target_load = get_uint16(frame, 6) * 0.1
       elseif id == 0x10C then
          temps.cht2 = get_uint16(frame, 4) * 0.1
          temps.egt2 = get_uint16(frame, 6) * 0.1
       elseif id == 0x10D then
          current_load = get_uint16(frame, 2) * 0.1
       elseif id == 0x10F then
          fuel_consumption_lph = get_uint16(frame,4)*0.1
          if last_fuel_s > 0 then
             local dt = now_s - last_fuel_s
             local fuel_lps = fuel_consumption_lph / 3600.0
             fuel_total_l = fuel_total_l + fuel_lps * dt
          end
          last_fuel_s = now_s
       elseif id == 0x113 then
          gen.amps = get_uint16(frame, 2) * 0.01
       elseif id == 0x114 then
          gen.rpm = get_uint16(frame, 0)
       elseif id == 0x115 then
          gen.batt_current = get_uint16(frame, 4) * 0.01
       end
    end

    -- Build and set the EFI_State that is passed into the EFI Scripting backend
    function self.set_EFI_State()
       -- Cylinder_Status
       cylinder_state:cylinder_head_temperature(0.5 * (temps.cht1 + temps.cht2))
       cylinder_state:exhaust_gas_temperature(0.5 * (temps.egt1 + temps.egt2))
       cylinder_state:ignition_timing_deg(ignition_angle)
       cylinder_state:injection_time_ms(inj_time*0.001)

       efi_state:engine_speed_rpm(uint32_t(rpm))
       efi_state:engine_load_percent(current_load)

       efi_state:fuel_consumption_rate_cm3pm(fuel_consumption_lph * 1000.0 / 60.0)
       efi_state:estimated_consumed_fuel_volume_cm3(fuel_total_l * 1000.0)
       efi_state:throttle_position_percent(math.floor(throttle_angle*100.0/90.0+0.5))
       efi_state:atmospheric_pressure_kpa(air_pressure*0.1)
       efi_state:ignition_voltage(supply_voltage)

       -- copy cylinder_state to efi_state
       efi_state:cylinder_status(cylinder_state)

       last_efi_state_time = millis()
       efi_state:last_updated_ms(last_efi_state_time)


        -- Set the EFI_State into the EFI scripting driver
        efi_backend:handle_scripting(efi_state)
    end

    -- return the instance
    return self
end -- end function engine_control(_driver, _idx)

local engine1 = engine_control(driver1, 1)

local last_efi_state_time = 0.0

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
