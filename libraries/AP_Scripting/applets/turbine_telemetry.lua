
-- ECU TURBINE TELEMETRY

local PARAM_TABLE_KEY = 73
local PARAM_TABLE_PREFIX = 'ECU_'

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30),
       'could not add param table')

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
           string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- 1 = Jakadofsky
-- 2 = EvoJet
-- 3 = PBS
-- 4 = Hornet
-- 5 = JetCat
-- 6 = KingTech
-- 7 = AMT
-- 8 = Xicoy telem adapter
-- 9 = JetCentral
-- 10 = Kolibri NG
-- 11 = Swiwin
-- 12 = Linton

local ECU_TYPE = bind_add_param('TYPE', 1, 1)
local XICOY_ID = 0x1B --default xicoy address
local VSPEAK_ID = 0x1C --default vspeak address
local port = serial:find_serial(0)
local POLL_START = 0x7E
local POLL_INTERVAL = 0.1
local LOG_INTERVAL = 0.1
local POLL_ID = VSPEAK_ID
local last_poll = 0
local last_report = 0
local rx_bytes = 0
raw_bytes = {}
local unescape_next = false
local EGT = 0
local RPM_val = 0
local Fuel = 0
local Vbat = 0
local Vpump = 0
local status = 0
local status_last = 0
local ECU_status = {}

-- Circular buffer 
local BUFFER_SIZE = 64
local raw_bytes = {}
for i = 1, BUFFER_SIZE do raw_bytes[i] = 0 end

local head_idx = 1
local tail_idx = 1

function get_time()
   return millis():tofloat() * 0.001
end

local function get_buffer_count()
    if head_idx >= tail_idx then
        return head_idx - tail_idx
    else
        return (BUFFER_SIZE - tail_idx) + head_idx
    end
end

local function get_byte_at_offset(offset)
    local idx = tail_idx + offset
    if idx > BUFFER_SIZE then
        idx = idx - BUFFER_SIZE
    end
    return raw_bytes[idx]
end

local function advance_buffer(n)
    tail_idx = tail_idx + n
    if tail_idx > BUFFER_SIZE then
        tail_idx = tail_idx - BUFFER_SIZE
    end
end

local function sport_crc(bytes)
    local crc = 0
    for i = 1, #bytes do
        crc = crc + bytes[i]
        crc = crc + (crc >> 8)
        crc = crc & 0xFF
    end
    return (0xFF - crc) & 0xFF
end

-- status table

local function initECU_status()

    local ecu_type = ECU_TYPE:get()

    ECU_status = {}

    -- ========================================================
    -- XICOY
    -- ========================================================

    if POLL_ID == XICOY_ID then

        ECU_status[0] = "HighTemp"
        ECU_status[1] = "Trim Low"
        ECU_status[2] = "SetIdle!"
        ECU_status[3] = "Ready"
        ECU_status[4] = "Ignition"
        ECU_status[5] = "FuelRamp"
        ECU_status[6] = "Glow Test"
        ECU_status[7] = "Running"
        ECU_status[8] = "Stop"
        ECU_status[9] = "FlameOut"
        ECU_status[10] = "SpeedLow"
        ECU_status[11] = "Cooling"
        ECU_status[12] = "Ignit.Bad"
        ECU_status[13] = "Start.Fail"
        ECU_status[14] = "AccelFail"
        ECU_status[15] = "Start On"
        ECU_status[16] = "UserOff"
        ECU_status[17] = "Failsafe"
        ECU_status[18] = "Low RPM"
        ECU_status[19] = "Reset"
        ECU_status[20] = "RXPwFail"
        ECU_status[21] = "PreHeat"
        ECU_status[22] = "Battery!"
        ECU_status[23] = "Time Out"
        ECU_status[24] = "Overload"
        ECU_status[25] = "Ign.Fail"
        ECU_status[26] = "Burner On"
        ECU_status[27] = "Starting"
        ECU_status[28] = "SwitchOv"
        ECU_status[29] = "Cal.Pump"
        ECU_status[30] = "PumpLimi"
        ECU_status[31] = "NoEngine"
        ECU_status[32] = "PwrBoost"
        ECU_status[33] = "Run-Idle"
        ECU_status[34] = "Run-Max"
        ECU_status[35] = "Restart"

        return
    end

    -- ========================================================
    -- VSPEAK
    -- ========================================================

    if ecu_type == 1 or ecu_type == 2 then

        ECU_status[-30] = " ERROR"
        ECU_status[-20] = "TH:-off"
        ECU_status[-10] = "TH:cool"
        ECU_status[-1] = "TH:lock"
        ECU_status[0] = "TH:stop"
        ECU_status[10] = "TH:run-"
        ECU_status[20] = "TH:rel-"
        ECU_status[25] = "TH:glow"
        ECU_status[30] = "TH:spin"
        ECU_status[40] = "TH:fire"
        ECU_status[45] = "TH:ignt"
        ECU_status[50] = "TH:heat"
        ECU_status[60] = "TH:acce"
        ECU_status[65] = "TH:cal."
        ECU_status[70] = "TH:idle"

    elseif ecu_type == 3 then

        ECU_status[-30] = "ERROR"
        ECU_status[-10] = "COOL"
        ECU_status[-1] = "LOCK"
        ECU_status[0] = "STOP"
        ECU_status[10] = "TH:Idle"
        ECU_status[20] = "TH:-Rel"
        ECU_status[25] = "TEST"
        ECU_status[40] = "FIRE"
        ECU_status[70] = "IDLE"

    elseif ecu_type == 4 then

        ECU_status[-30] = "DEV.DELAY"
        ECU_status[-20] = "FLAME OUT"
        ECU_status[-10] = "EMERGENCY"
        ECU_status[0] = "OFF"
        ECU_status[1] = "ON"
        ECU_status[10] = "Cool Down"
        ECU_status[20] = "Slow Down"
        ECU_status[30] = "STANDBY"
        ECU_status[31] = "PROP IGNIT"
        ECU_status[32] = "PROP HEAT"
        ECU_status[33] = "Pump Start"
        ECU_status[34] = "BURNER ON"
        ECU_status[35] = "FUEL IGNIT"
        ECU_status[36] = "FUEL HEAT"
        ECU_status[37] = "Ramp Delay"
        ECU_status[38] = "RAMP UP"
        ECU_status[40] = "STEADY"
        ECU_status[41] = "CAL IDLE"
        ECU_status[42] = "CALIBRATE"
        ECU_status[43] = "WAIT ACC"
        ECU_status[44] = "GO IDLE"
        ECU_status[50] = "AUTO"
        ECU_status[51] = "AUTO HC"

    elseif ecu_type == 5 then

        ECU_status[-40] = "Failure"
        ECU_status[-32] = "ECU reboot"
        ECU_status[-31] = "ClutchFail"
        ECU_status[-30] = "Low Rpm"
        ECU_status[-28] = "Out Fuel"
        ECU_status[-27] = "Pump Comm"
        ECU_status[-26] = "WrongPump"
        ECU_status[-25] = "No Pump!"
        ECU_status[-24] = "OverCurr"
        ECU_status[-23] = "No-OIL"
        ECU_status[-22] = "2nd Comm"
        ECU_status[-21] = "2nd Diff"
        ECU_status[-20] = "2nd EngF"
        ECU_status[-19] = "Rpm2Fail"
        ECU_status[-18] = "FuelFail"
        ECU_status[-17] = "TempFail"
        ECU_status[-16] = "PowerFail"
        ECU_status[-15] = "IgnTimOut"
        ECU_status[-14] = "FailSafe"
        ECU_status[-13] = "WatchDog"
        ECU_status[-12] = "GlowPlug!"
        ECU_status[-11] = "HiTempOff"
        ECU_status[-10] = "LowTempOff"
        ECU_status[-9] = "OverTemp"
        ECU_status[-8] = "BatteryLow"
        ECU_status[-7] = "Low-Rpm"
        ECU_status[-6] = "Over-Rpm"
        ECU_status[-5] = "Acc. Slow"
        ECU_status[-4] = "AccTimOut"
        ECU_status[-3] = "Manual Off"
        ECU_status[-2] = "Auto-Off"
        ECU_status[-1] = "RC-Off"
        ECU_status[0] = "-OFF-"
        ECU_status[1] = "SlowDown"
        ECU_status[2] = "SwitchOff"
        ECU_status[3] = "Stby/START"
        ECU_status[4] = "PreHeat1"
        ECU_status[5] = "PreHeat2"
        ECU_status[6] = "Ignite..."
        ECU_status[7] = "AccelrDly"
        ECU_status[8] = "MainFStrt"
        ECU_status[9] = "Keros.Full"
        ECU_status[10] = "accelerate"
        ECU_status[11] = "Stabilise"
        ECU_status[12] = "LearnLO"
        ECU_status[13] = "RUN (reg.)"
        ECU_status[14] = "SpeedCtrl"
        ECU_status[15] = "Rpm2Ctrl"

    elseif ecu_type == 6 then

        ECU_status[-20] = "- ERROR -"
        ECU_status[-19] = "Unknown"
        ECU_status[-13] = "CAB-Lost"
        ECU_status[-12] = "FlameOut"
        ECU_status[-11] = "TempHigh"
        ECU_status[-10] = "SpeedLow"
        ECU_status[-9] = "Failsafe"
        ECU_status[-8] = "RxPwFail"
        ECU_status[-7] = "Overload"
        ECU_status[-6] = "Low Batt"
        ECU_status[-5] = "StartBad"
        ECU_status[-4] = "Weak Gas"
        ECU_status[-3] = "Time Out"
        ECU_status[-2] = "Ign.Fail"
        ECU_status[-1] = "Glow Bad"
        ECU_status[0] = "Trim Low"
        ECU_status[1] = "User Off"
        ECU_status[2] = "Stop"
        ECU_status[3] = "Cooling"
        ECU_status[4] = "Ready"
        ECU_status[5] = "GdReady"
        ECU_status[6] = "GlowTest"
        ECU_status[7] = "StickLo!"
        ECU_status[8] = "PrimeVap"
        ECU_status[9] = "BurnerOn"
        ECU_status[10] = "Start On"
        ECU_status[11] = "Ignition"
        ECU_status[12] = "Stage1"
        ECU_status[13] = "Stage2"
        ECU_status[14] = "Stage3"
        ECU_status[20] = "Running"
        ECU_status[22] = "ReStart"

    elseif ecu_type == 7 then

        ECU_status[-20] = "- ERROR -"
        ECU_status[-8] = "Supply ASS"
        ECU_status[-7] = "Supply low"
        ECU_status[-6] = "RPM high"
        ECU_status[-5] = "EGT error"
        ECU_status[-4] = "Throttle fail"
        ECU_status[-3] = "Switch failure"
        ECU_status[-2] = "RPM low"
        ECU_status[-1] = "No serial input"
        ECU_status[0] = "No Start Clear"
        ECU_status[1] = "Start Clear"
        ECU_status[2] = "Starting"
        ECU_status[3] = "Started up"
        ECU_status[4] = "Calibrated"
        ECU_status[5] = "Auto stop"
        ECU_status[6] = "Running"
        ECU_status[7] = "Max. RPM"

    elseif ecu_type == 8 then

        ECU_status[0]  = "HighTemp"
        ECU_status[1]  = "Trim Low"
        ECU_status[2]  = "SetIdle!"
        ECU_status[3]  = "Ready"
        ECU_status[4]  = "Ignition"
        ECU_status[5]  = "FuelRamp"
        ECU_status[6]  = "Glow Test"
        ECU_status[7]  = "Running"
        ECU_status[8]  = "Stop"
        ECU_status[9]  = "FlameOut"
        ECU_status[10] = "SpeedLow"
        ECU_status[11] = "Cooling"
        ECU_status[12] = "Ignit.Bad"
        ECU_status[13] = "Start.Fail"
        ECU_status[14] = "AccelFail"
        ECU_status[15] = "Start On"
        ECU_status[16] = "UserOff"
        ECU_status[17] = "Failsafe"
        ECU_status[18] = "Low RPM"
        ECU_status[19] = "Reset"
        ECU_status[20] = "RXPwFail"
        ECU_status[21] = "PreHeat"
        ECU_status[22] = "Battery!"
        ECU_status[23] = "Time Out"
        ECU_status[24] = "Overload"
        ECU_status[25] = "Ign.Fail"
        ECU_status[26] = "Burner On"
        ECU_status[27] = "Starting"
        ECU_status[28] = "SwitchOv"
        ECU_status[29] = "Cal.Pump"
        ECU_status[30] = "PumpLimi"
        ECU_status[31] = "NoEngine"
        ECU_status[32] = "PwrBoost"
        ECU_status[33] = "Run-Idle"
        ECU_status[34] = "Run-Max "
        ECU_status[35] = "Restart "
        ECU_status[36] = "No status"

    elseif ecu_type == 9 then

        ECU_status[-21] = "ERROR"
        ECU_status[-20] = "- no data -"
        ECU_status[-19] = "MaxPump"
        ECU_status[-18] = "RPMSensorErr"
        ECU_status[-17] = "LowRPM"
        ECU_status[-16] = "MaxAmpers"
        ECU_status[-15] = "MaxTemp"
        ECU_status[-14] = "ComTurbErr"
        ECU_status[-13] = "TempSensErr"
        ECU_status[-12] = "RxSetupError"
        ECU_status[-11] = "Failsafe"
        ECU_status[-10] = "NoRx"
        ECU_status[-9] = "LowECUBatt"
        ECU_status[-8] = "LowRXBatt"
        ECU_status[-7] = "IgniterBad"
        ECU_status[-6] = "AccelerateBad"
        ECU_status[-5] = "ToIdle Error"
        ECU_status[-4] = "StarterError"
        ECU_status[-3] = "SwitchOverErr"
        ECU_status[-2] = "PreheatError"
        ECU_status[-1] = "IgnitionError"
        ECU_status[0] = "OFF"
        ECU_status[1] = "ManCooling"
        ECU_status[2] = "AutoCooling"
        ECU_status[6] = "GlowTest"
        ECU_status[7] = "StarterTest"
        ECU_status[8] = "PrimeFuel"
        ECU_status[9] = "PrimeBurner"
        ECU_status[10] = "Standby"
        ECU_status[11] = "Start"
        ECU_status[12] = "IgniterHeat"
        ECU_status[13] = "Ignition"
        ECU_status[14] = "Preheat"
        ECU_status[15] = "Switchover"
        ECU_status[16] = "To Idle"
        ECU_status[17] = "MinPumpOk"
        ECU_status[18] = "MaxPumpOk"
        ECU_status[19] = "Running"
        ECU_status[20] = "Full"

    elseif ecu_type == 10 then

        ECU_status[-10] = "FLAME OUT"
        ECU_status[-2] = "BATT LOW!"
        ECU_status[-1] = "GLOW PLUG!"
        ECU_status[0] = "OFF"
        ECU_status[1] = "ON"
        ECU_status[2] = "COOL-DOWN"
        ECU_status[3] = "SLOW-DOWN"
        ECU_status[10] = "STANDBY"
        ECU_status[11] = "PROPIGNIT"
        ECU_status[12] = "PROP-HEAT"
        ECU_status[13] = "PUMPSTART"
        ECU_status[14] = "FUELHEAT"
        ECU_status[15] = "RAMP-UP"
        ECU_status[20] = "AUTO"

    elseif ecu_type == 11 then

        ECU_status[-30] = "no data"
        ECU_status[-18] = "Engine Offline"
        ECU_status[-17] = "Current overload"
        ECU_status[-16] = "Clutch failure"
        ECU_status[-15] = "Pump Temp High"
        ECU_status[-14] = "StarterTempHigh"
        ECU_status[-13] = "Lost Signal"
        ECU_status[-12] = "Fuel Valve Bad"
        ECU_status[-11] = "Gas Valve Bad"
        ECU_status[-10] = "TempSensorfail"
        ECU_status[-9] = "Low Temp"
        ECU_status[-8] = "High Temp"
        ECU_status[-7] = "RPM Instability"
        ECU_status[-6] = "RPM Low"
        ECU_status[-5] = "Starter failure"
        ECU_status[-4] = "Pump Anomaly"
        ECU_status[-3] = "GlowPlug Bad"
        ECU_status[-2] = "Low Battery"
        ECU_status[-1] = "Time Out"
        ECU_status[0] = "Stop"
        ECU_status[1] = "Cooling"
        ECU_status[5] = "TestGlowPlug"
        ECU_status[6] = "TestFuelValve"
        ECU_status[7] = "TestGasValve"
        ECU_status[8] = "TestPump"
        ECU_status[9] = "TestStarter"
        ECU_status[10] = "Ready"
        ECU_status[11] = "Ignition"
        ECU_status[12] = "Preheat"
        ECU_status[13] = "Fuelramp"
        ECU_status[20] = "Running"
        ECU_status[21] = "Restart"

    elseif ecu_type == 12 then

        ECU_status[-40] = "-no data-"
        ECU_status[-39] = "Error Alarms"
        ECU_status[-35] = "Power limit"
        ECU_status[-34] = "Restart Fail"
        ECU_status[-27] = "Pump bubble"
        ECU_status[-18] = "Fuel Fail"
        ECU_status[-17] = "RPM Low"
        ECU_status[-16] = "RPM Err"
        ECU_status[-15] = "Rc Lost Off"
        ECU_status[-14] = "Pump Fail"
        ECU_status[-13] = "PumpCurrent"
        ECU_status[-12] = "Pump Open"
        ECU_status[-11] = "CTH Fail"
        ECU_status[-10] = "MotorCurrent"
        ECU_status[-9] = "Motor Open"
        ECU_status[-8] = "IGT Short"
        ECU_status[-7] = "IGT Open"
        ECU_status[-6] = "EGT Warn"
        ECU_status[-5] = "TT Trans"
        ECU_status[-4] = "TT Open"
        ECU_status[-3] = "OverCurrent"
        ECU_status[-2] = "Volt High"
        ECU_status[-1] = "Volt Low"
        ECU_status[0] = "Ready"
        ECU_status[1] = "Ready start"
        ECU_status[2] = "Temp high"
        ECU_status[3] = "Start.."
        ECU_status[4] = "Burner.."
        ECU_status[5] = "Success"
        ECU_status[6] = "Heating1.."
        ECU_status[7] = "Heating2.."
        ECU_status[8] = "Heating3.."
        ECU_status[9] = "Heating4.."
        ECU_status[10] = "Heating5.."
        ECU_status[11] = "Heating6.."
        ECU_status[12] = "Pump Acc.."
        ECU_status[13] = "CTH.."
        ECU_status[18] = "Idling.."
        ECU_status[19] = "Acc.."
        ECU_status[20] = "Dec.."
        ECU_status[21] = "Speed.."
        ECU_status[22] = "Max Speed.."
        ECU_status[23] = "RC Learn"
        ECU_status[24] = "RC Learning.."
        ECU_status[25] = "RC Successful"
        ECU_status[26] = "Restart"
        ECU_status[27] = "Restart.."
        ECU_status[28] = "Cooling.."
        ECU_status[29] = "Error status"
    end
end


-- Parser function

local function parse_value(app_id, value)
    if POLL_ID == VSPEAK_ID then
        if app_id == 0x0400 then
            if value >= 0 and value <= 1000 then EGT = value end
        elseif app_id == 0x0500 then
            if value >= 0 and value <= 250000 then RPM_val = value end
        elseif app_id == 0x0600 then
            if value >= 0 then Fuel = value end
        elseif app_id == 0x0900 then
            if value >= 0 and value < 100000 then Vbat = value / 100 end
        elseif app_id == 0x0910 then
            if value >= 0 and value < 100000 then Vpump = value / 100 end
        elseif app_id == 0x0410 then
            if value >= 0 and value < 100 then status = value end
        end
    elseif POLL_ID == XICOY_ID then
        if app_id == 0x4400 then
            if value >= 0 and value <= 1000 then EGT = value end
        elseif app_id == 0x4401 then
            if value >= 0 and value <= 250000 then RPM_val = value end
        elseif app_id == 0x4403 then
            if value >= 0 and value < 100000 then Vbat = value / 10 end
        elseif app_id == 0x4406 then
            if value >= 0 and value < 100 then status = value end
        end
    end
end

local function process_and_validate_frame()

    local f1 = get_byte_at_offset(0)
    local f2 = get_byte_at_offset(1)
    local f3 = get_byte_at_offset(2)
    local f4 = get_byte_at_offset(3)
    local f5 = get_byte_at_offset(4)
    local f6 = get_byte_at_offset(5)
    local f7 = get_byte_at_offset(6)

    local app_id = f1 | (f2 << 8)
    
    if POLL_ID == VSPEAK_ID then
        local crc_calc = sport_crc({ 0x10, f1, f2, f3, f4, f5, f6 })
        if crc_calc == f7 then
            local value = f3 | (f4 << 8) | (f5 << 16) | (f6 << 24)
            if value > 0x7FFFFFFF then value = value - 0x100000000 end
            parse_value(app_id, value)
            return true
        end

    elseif POLL_ID == XICOY_ID then
        if app_id == 0x4400 or app_id == 0x4401 or 
           app_id == 0x4403 or app_id == 0x4406 then
            
            local value = f3 | (f4 << 8) | (f5 << 16) | (f6 << 24)
            if value > 0x7FFFFFFF then value = value - 0x100000000 end
            
            parse_value(app_id, value)
            return true
        end
    end

    return false
end

local function read_stream()
    if not port then return end

    local available_bytes = port:available():toint()
    if available_bytes <= 0 then 
        return 
    end

    local data = port:readstring(available_bytes)

    if not data then
        return
    end

    for i = 1, #data do

        local b = data:byte(i)
        
        rx_bytes = rx_bytes + 1

        if b == 0x7D then
            unescape_next = true
        else
            if unescape_next then
                if b == 0x5E then b = 0x7E
                elseif b == 0x5D then b = 0x7D end
                unescape_next = false
            end

            --fill the buffer
            raw_bytes[head_idx] = b
            head_idx = head_idx + 1
            if head_idx > BUFFER_SIZE then
                head_idx = 1
            end
        end
    end

    --align frames 
    while get_buffer_count() >= 8 do
        if process_and_validate_frame() then
            advance_buffer(8)
        else
            advance_buffer(1)
        end
    end
end

local function send_poll()
    if not port then
        return
    end
    port:write(POLL_START)
    port:write(POLL_ID)
end

function update()

    if not port then

        gcs:send_text(5, "NO SERIAL PORT")

        return update, 1000
    end

    local now = get_time()

    read_stream()

    if now - last_poll >= POLL_INTERVAL then

        send_poll()

        last_poll = now
    end

    if now - last_report >= LOG_INTERVAL then
        logger:write('TURB', 'EGT,RPM,Fuel,Vbat,Vpmp,Stat', 'ffffff', EGT, RPM_val, Fuel, Vbat, Vpump, status)
        --gcs:send_text(5, string.format("EGT:%d RPM:%d Vbat:%.1f", EGT, RPM_val, Vbat))
        last_report = now
    end

    if status ~= status_last then

        local msg =
            string.format(
                "ECU: %s",
                ECU_status[status] or
                ("Unknown(" .. status .. ")")
            )

        gcs:send_text(5, msg)

        status_last = status
    end

    return update, 1
end

-- Initialization

if port then

    local ecu_type = ECU_TYPE:get()

    if ecu_type == 8 then
        POLL_ID = XICOY_ID
    else
        POLL_ID = VSPEAK_ID
    end

    port:begin(57600)

    initECU_status()

    gcs:send_text(5, string.format("ECU TELEMETRY RUNNING ID:0x%02X", POLL_ID))
    send_poll()
end

return update, 1
