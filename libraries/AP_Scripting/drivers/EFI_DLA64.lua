--[[
    DLA64 serial EFI protocol
--]]

local PARAM_TABLE_KEY = 52
local PARAM_TABLE_PREFIX = "EFI_DLA64_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: EFI_DLA64_ENABLE
  // @DisplayName: EFI DLA64 enable
  // @Description: Enable EFI DLA64 driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_DLA64_ENABLE = bind_add_param("ENABLE", 1, 1)

if EFI_DLA64_ENABLE:get() ~= 1 then
   return
end

local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "DLA64: unable to find scripting serial")
   return
end
uart:begin(115200)

local efi_backend = efi:get_backend(0)
if not efi_backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "DLA64: unable to find EFI backend")
   return
end

--[[
    in 4.5.x temperatures were incorrectly used as C instead of Kelvin
--]]
local temp_offset = 0.0
if FWVersion:major() == 4 and FWVersion:minor() <= 5 then
    temp_offset = -273.15
end

--[[
   discard n bytes
--]]
local function discard_bytes(n)
   for _ = 1, n do
      uart:read()
   end
end

local state = {}
state.last_read_us = uint32_t(0)
state.total_fuel_cm3 = 0.0

--[[
   xmodem CRC implementation thanks to https://github.com/cloudwu/skynet
   under MIT license
--]]
local XMODEMCRC16Lookup = {
   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
   0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
   0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
   0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
   0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
   0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
   0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
   0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
   0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
   0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
   0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
   0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
   0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
   0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
   0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
   0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
   0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
   0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
   0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
   0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
   0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
   0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
   0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
   0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
   0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
   0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
   0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
   0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
   0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
   0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
   0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
   0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
}

local function crc16(bytes)
    -- see https://blog.csdn.net/sijia5135/article/details/116450963
    local crc = 0xffff
	for i=1,#bytes do
		local b = string.byte(bytes,i,i)
		crc = ((crc<<8) & 0xffff) ~ XMODEMCRC16Lookup[(((crc>>8)~b) & 0xff) + 1]
	end
    return crc
end

local function farenheight_to_Kelvin(v)
   return (v + 459.67) * 0.55556 + 273.15
end

--[[
   check for input and parse data
--]]
local function check_input()
   local n_bytes = uart:available():toint()
   -- gcs:send_text(MAV_SEVERITY.INFO, string.format("n_bytes=%u %.2f", n_bytes, millis():tofloat()*0.001))
   if n_bytes < 127 then
      return
   end
   if n_bytes > 127 then
      discard_bytes(n_bytes)
      return
   end
   local payload = uart:readstring(127)
   if not payload then
      discard_bytes(n_bytes)
      return
   end
   local head, flen = string.unpack(">Hi", payload)
   local crc, tail = string.unpack(">HH", string.sub(payload, 124, 128))
   local crc2 = crc16(string.sub(payload, 3, 123))
   if head ~= 0xf11f or tail ~= 0xf22f or flen ~= 125 or crc ~= crc2 then
       return
   end
   state.rpm = string.unpack("<H", string.sub(payload, 16, 17))
   state.fcr = string.unpack("<H", string.sub(payload, 108, 109)) * 0.1
   state.ctemp1 = farenheight_to_Kelvin(string.unpack("<H", string.sub(payload, 96, 97)) * 0.1)
   state.ctemp2 = farenheight_to_Kelvin(string.unpack("<H", string.sub(payload, 102, 103)) * 0.1)
   state.atemp = farenheight_to_Kelvin(string.unpack("<H", string.sub(payload, 30, 31)) * 0.1)
   state.apress_kPa = string.unpack("<H", string.sub(payload, 26, 27)) * 0.1
   state.bvolt = string.unpack("<H", string.sub(payload, 36, 37)) * 0.1

   state.last_read_us = micros()
end

--[[
   request more data
--]]
local function request_data()
    local hexSequence = {
        0xF1, 0x1F, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x22, 0x66, 0xD6, 0xF1, 0xF2, 0x2F
    }
    for _, h in ipairs(hexSequence) do
        uart:write(h)
    end
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

   cylinder_state:cylinder_head_temperature(state.ctemp1 + temp_offset)
   cylinder_state:exhaust_gas_temperature(state.ctemp2 + temp_offset)
   efi_state:engine_speed_rpm(state.rpm)

   efi_state:atmospheric_pressure_kpa(state.apress_kPa)
   efi_state:intake_manifold_temperature(state.atemp + temp_offset)
   efi_state:ignition_voltage(state.bvolt)

   local now_us = micros()
   local dt = (now_us - state.last_read_us):tofloat()*1.0e-6
   state.last_read_us = now_us

   local fcr_litres_per_second = state.fcr / 3600.0

   state.total_fuel_cm3 = state.total_fuel_cm3 + fcr_litres_per_second * dt

   efi_state:fuel_consumption_rate_cm3pm(fcr_litres_per_second * 60 * 1000)
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
    if EFI_DLA64_ENABLE:get() > 0 then
        check_input()
        update_EFI()
        request_data()
    end
end

gcs:send_text(MAV_SEVERITY.INFO, "DLA64: loaded")

-- wrapper around update()
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "DLA64: Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 100
end

-- start running update loop
return protected_wrapper()

