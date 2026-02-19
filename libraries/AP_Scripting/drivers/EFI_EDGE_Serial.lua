--[[
    Edge Autonomy serial EFI protocol, as used on PenguinB
--]]

local PARAM_TABLE_KEY = 94
local PARAM_TABLE_PREFIX = "EFI_EDGE_"

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 8), 'could not add param table')

--[[
  // @Param: EFI_EDGE_ENABLE
  // @DisplayName: EFI EDGE enable
  // @Description: Enable EFI EDGE driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
EFI_EDGE_ENABLE = bind_add_param("ENABLE", 1, 1)

--[[
  // @Param: EFI_EDGE_KILL_FN
  // @DisplayName: EFI EDGE kill switch aux function
  // @Description: Aux function for ignition control. Set to -1 to disable. On ArduPilot 4.6.x use a scripting aux function (eg. 300)
  // @Values: -1:Disabled,179:ICE_START_STOP,300:Scripting1,301:Scripting2,302:Scripting3,303:Scripting4
  // @User: Standard
--]]
EFI_EDGE_KILL_FN = bind_add_param("KILL_FN", 2, -1)

--[[
  // @Param: EFI_EDGE_TLM_RT
  // @DisplayName: EFI EDGE telemetry rate
  // @Description: Requested telemetry rate from ECU in Hz
  // @Range: 1 20
  // @Units: Hz
  // @User: Standard
--]]
EFI_EDGE_TLM_RT = bind_add_param("TLM_RT", 3, 10)

if EFI_EDGE_ENABLE:get() ~= 1 then
   return
end

local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "EDGE: unable to find scripting serial")
   return
end
uart:begin(115200)

local EFI_FUEL_DENS = Parameter("EFI_FUEL_DENS")

local efi_backend = efi:get_backend(0)
if not efi_backend then
   gcs:send_text(MAV_SEVERITY.ERROR, "EDGE: unable to find EFI backend")
   return
end

local state = {}
local buffer
state.last_read_us = uint32_t(0)
state.total_fuel_cm3 = 0.0
state.last_kill_status = -1

-- pre-allocate objects to avoid GC pressure
local cylinder_state = Cylinder_Status()
local efi_state = EFI_State()
local UINT32_0 = uint32_t(0)
local UINT32_200 = uint32_t(200)
local UINT32_1000 = uint32_t(1000)

--[[
   discard all bytes
--]]
local function discard_bytes()
   local n = uart:available():toint()
   for _ = 1, n do
      uart:read()
   end
   buffer = nil
end

--[[
   CRC-16/ARC (CRC-16/IBM) lookup table
   poly=0x8005, init=0x0000, refIn=true, refOut=true, xorOut=0x0000
--]]
local CRC16Lookup = {
   0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
   0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
   0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
   0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
   0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
   0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
   0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
   0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
   0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
   0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
   0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
   0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
   0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
   0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
   0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
   0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
   0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
   0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
   0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
   0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
   0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
   0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
   0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
   0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
   0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
   0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
   0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
   0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
   0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
   0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
   0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
   0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
}

local function crc16(bytes, offset, len)
    local crc = 0x0000
    local byte = string.byte
    local tbl = CRC16Lookup
    for i = offset, offset + len - 1 do
        crc = ((crc >> 8) & 0xff) ~ tbl[((crc ~ byte(bytes, i)) & 0xff) + 1]
    end
    return crc
end

local function fahrenheit_to_kelvin(v)
   return (v + 459.67) * (5.0/9.0)
end

function constrain(v, vmin, vmax)
   return math.max(math.min(v,vmax),vmin)
end

--[[
   convert grams to cm3 using EFI_FUEL_DENS, default 0.8 kg/litre
--]]
local function gram_to_cm3(g)
   local density = EFI_FUEL_DENS:get()
   if not density or density <= 0 then
      density = 800.0
   end
   return (g / density) * 1000.0
end

--[[
    check CRC at position pos in buf, CRC covers header+payload (size+4 bytes)
--]]
local function check_crc(buf, pos, size)
    local crc_lo = string.byte(buf, pos + size + 4)
    local crc_hi = string.byte(buf, pos + size + 5)
    local expected_crc = crc_lo + crc_hi * 256
    local computed_crc = crc16(buf, pos, size + 4)
    return computed_crc == expected_crc
end

--[[
   parse telemetry 1 packet (packet type 6, 10 byte payload)
   fuel and engine working time data
--]]
local function parse_telem_1(buf, pos)
    local total_fuel_cons, fuel_since_restart, ewt, remaining_fuel =
        string.unpack("<I2I2I2I2", buf, pos + 4)
    state.total_fuel_consumed_kg = total_fuel_cons * 0.1
    state.fuel_remaining_g = remaining_fuel
    state.ewt_hours = ewt * 0.01
    logger:write('EFE1','TFuel,FSR,EWT,RFuel', 'ffff',
                 total_fuel_cons * 0.1, fuel_since_restart, ewt * 0.01, remaining_fuel)
    -- gcs:send_text(MAV_SEVERITY.ERROR, "EDGE: telem1")
end

--[[
   parse telemetry 2 packet (packet type 7, 60 byte payload)
   main engine telemetry data
   temperatures MAT and COOLANT are in 0.1 degF units
--]]
local function parse_telem_2(buf, pos)
    local time_raw, duct_i, thrl_i, fp_i, ecu_t, gen_t,
          duct_pos, thr_pos, fuel_flow, sys_status,
          thr_source, pulse_w, rpm, advance,
          eng_status, baro, map_p, mat, coolant,
          tps, afr1, ms_volt =
        string.unpack("<I4I2I2I2bbBBI2I2BI2I2I2Bi2i2i2i2i2i2i2", buf, pos + 4)
    state.rpm = rpm
    state.ctemp1 = fahrenheit_to_kelvin(coolant * 0.1)
    state.ctemp2 = ecu_t + 273.15
    state.apress_kPa = baro * 0.1
    state.ipress_kPa = map_p * 0.1
    state.atemp = fahrenheit_to_kelvin(mat * 0.1)
    state.bvolt = ms_volt * 0.1
    state.fcr = fuel_flow
    state.tps = tps * 0.1
    state.advance_deg = advance * 0.1
    state.pulse_w_ms = pulse_w * 0.000666
    state.lambda = afr1 ~= 0 and (afr1 * 0.1 / 14.7) or 0
    state.last_read_us = micros()

    -- report software and hardware kill switch changes
    local sw_kill = (sys_status & 0x04) ~= 0
    local hw_kill = (sys_status & 0x10) ~= 0
    local kill_status = (sw_kill and 1 or 0) + (hw_kill and 2 or 0)
    if kill_status ~= state.last_kill_status then
        state.last_kill_status = kill_status
        gcs:send_text(MAV_SEVERITY.INFO,
                      string.format("EDGE: SW_kill=%s HW_kill=%s",
                                    sw_kill and "on" or "off",
                                    hw_kill and "on" or "off"))
    end

    logger:write('EFE2','RPM,FF,Bar,MAP,MAT,CLT,TPS,AFR,V,ET,GT,ES',
                 'HHhhhhhhhbbB',
                 rpm, fuel_flow, baro, map_p, mat, coolant,
                 tps, afr1, ms_volt, ecu_t, gen_t, eng_status)
    logger:write('EFE3','T,DI,TI,FI,DP,TP,St,TS,PW,Av',
                 'IHHHBBHBHH',
                 time_raw, duct_i, thrl_i, fp_i,
                 duct_pos, thr_pos, sys_status,
                 thr_source, pulse_w, advance)
    -- gcs:send_text(MAV_SEVERITY.ERROR, "EDGE: telem2")
end

--[[
   check for input and parse all available packets
   uses offset-based parsing to minimise string allocations
--]]
local function check_input()
   local n_bytes = uart:available():toint()
   if n_bytes == 0 then
      return
   end
   if n_bytes > 127 then
      discard_bytes()
      return
   end
   local b = uart:readstring(n_bytes)
   if not b then
      return
   end
   -- only concatenate when we have leftover bytes from last call
   if buffer then
      buffer = buffer .. b
   else
      buffer = b
   end

   -- process all complete packets using position offsets
   local pos = 1
   local buf_len = #buffer
   while pos + 5 <= buf_len do
       if string.byte(buffer, pos) ~= 0xA0 or string.byte(buffer, pos + 1) ~= 0x05 then
           pos = pos + 1
       else
           local size = string.byte(buffer, pos + 3)
           if pos + size + 5 > buf_len then
               break -- incomplete packet, wait for more data
           end
           if not check_crc(buffer, pos, size) then
               pos = pos + 1
           else
               local ptype = string.byte(buffer, pos + 2)
               if ptype == 6 then
                   parse_telem_1(buffer, pos)
               elseif ptype == 7 then
                   parse_telem_2(buffer, pos)
               end
               pos = pos + size + 6
           end
       end
   end

   -- keep only unprocessed bytes (one allocation at most)
   if pos > buf_len then
       buffer = nil
   elseif pos > 1 then
       buffer = string.sub(buffer, pos)
   end
   -- if pos == 1 buffer is unchanged, no allocation
end

--[[
   send a packet with given type and payload string
--]]
local function send_packet(ptype, payload)
    local data = string.pack("<BBBB", 0xA0, 0x05, ptype, #payload) .. payload
    uart:writestring(data .. string.pack("<I2", crc16(data, 1, #data)))
end

--[[
   send a pre-built raw packet (no allocation)
--]]
local function send_raw(pkt)
    uart:writestring(pkt)
end

-- pre-build kill switch packets (sent at 5Hz, avoid repeated allocation)
local kill_on_pkt, kill_off_pkt
do
    local function build_packet(ptype, payload)
        local data = string.pack("<BBBB", 0xA0, 0x05, ptype, #payload) .. payload
        return data .. string.pack("<I2", crc16(data, 1, #data))
    end
    kill_on_pkt = build_packet(14, string.pack("B", 1))
    kill_off_pkt = build_packet(14, string.pack("B", 0))
end

--[[
   request telemetry at configured rate, sent every 1s
   packet type 11 (TELEMETRY_PERIOD), period in 50ms units
--]]
local last_telem_req_ms = uint32_t(0)

local function request_telem()
    local now = millis()
    if now - last_telem_req_ms < UINT32_1000 then
        return
    end
    last_telem_req_ms = now
    local rate = EFI_EDGE_TLM_RT:get()
    if rate < 1 then rate = 1 end
    if rate > 20 then rate = 20 end
    local period = math.floor(20 / rate)
    if period < 1 then period = 1 end
    send_packet(11, string.pack("BB", period, period))
end

--[[
   check ICE_START_STOP aux switch at 5Hz, send SW_KILL_SWITCH (type 14)
   aux value 0 (low) = ignition disabled, 1 or 2 (mid/high) = ignition enabled
--]]
local last_kill_sw_ms = uint32_t(0)
local last_ignition = -1

local function check_ice_start()
    local kill_fn = EFI_EDGE_KILL_FN:get()
    if kill_fn < 0 then
        return
    end
    local now = millis()
    if now - last_kill_sw_ms < UINT32_200 then
        return
    end
    last_kill_sw_ms = now
    local sw = rc:get_aux_cached(kill_fn)
    if sw then
        local ignition = (sw == 0) and 0 or 1
        if ignition ~= last_ignition then
            last_ignition = ignition
            gcs:send_text(MAV_SEVERITY.INFO, ignition == 1 and "EDGE: ignition on" or "EDGE: ignition off")
        end
        send_raw(ignition == 1 and kill_on_pkt or kill_off_pkt)
    end
end

--[[
   update EFI state
--]]
local function update_EFI()
   if state.last_read_us == UINT32_0 then
      return
   end

   cylinder_state:cylinder_head_temperature(state.ctemp1)
   cylinder_state:exhaust_gas_temperature(state.ctemp2)
   cylinder_state:ignition_timing_deg(state.advance_deg)
   cylinder_state:injection_time_ms(state.pulse_w_ms)
   cylinder_state:lambda_coefficient(state.lambda)
   efi_state:engine_speed_rpm(state.rpm)

   efi_state:atmospheric_pressure_kpa(state.apress_kPa)
   efi_state:intake_manifold_pressure_kpa(state.ipress_kPa)
   efi_state:intake_manifold_temperature(state.atemp)
   efi_state:coolant_temperature(state.ctemp1)
   efi_state:ignition_voltage(state.bvolt)
   efi_state:throttle_position_percent(constrain(math.floor(state.tps), 0, 100))

   local now_us = micros()
   local dt = (now_us - state.last_read_us):tofloat()*1.0e-6
   state.last_read_us = now_us

   -- fuel_flow from ECU is in grams per hour
   local grams_per_second = state.fcr / 3600.0
   local cm3_per_second = gram_to_cm3(grams_per_second)

   state.total_fuel_cm3 = state.total_fuel_cm3 + cm3_per_second * dt

   efi_state:fuel_consumption_rate_cm3pm(cm3_per_second * 60.0)
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
    if EFI_EDGE_ENABLE:get() > 0 then
        check_input()
        update_EFI()
        request_telem()
        check_ice_start()
    end
end

gcs:send_text(MAV_SEVERITY.INFO, "EFI_EDGE_Serial: loaded")

-- wrapper around update()
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "EDGE Err: " .. string.sub(tostring(err), -40))
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 25
end

-- start running update loop
return protected_wrapper()

