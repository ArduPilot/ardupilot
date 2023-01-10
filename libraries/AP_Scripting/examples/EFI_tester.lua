--[[
   simulator for CAN EFI
   this can be used with a loopback cable between CAN1 and CAN2 to test CAN EFI Drivers
--]]

local driver2 = CAN.get_device2(25)
if not driver2 then
   gcs:send_text(0,string.format("EFISIM: Failed to load CAN driver"))
   return
end

local PARAM_TABLE_KEY = 13
local PARAM_TABLE_PREFIX = "EFISIM_"

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

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

local EFISIM_TYPE  = bind_add_param('TYPE', 1, 0)
local EFISIM_RATE_HZ  = bind_add_param('RATE_HZ', 2, 100)

function get_time_sec()
   return millis():tofloat() * 0.001
end

local FRM_100 = uint32_t(0x80000100)
local FRM_101 = uint32_t(0x80000101)
local FRM_102 = uint32_t(0x80000102)
local FRM_104 = uint32_t(0x80000104)
local FRM_105 = uint32_t(0x80000105)
local FRM_106 = uint32_t(0x80000106)
local FRM_10A = uint32_t(0x8000010A)
local FRM_10C = uint32_t(0x8000010C)
local FRM_10D = uint32_t(0x8000010D)
local FRM_10F = uint32_t(0x8000010F)
local FRM_113 = uint32_t(0x80000113)
local FRM_114 = uint32_t(0x80000114)
local FRM_115 = uint32_t(0x80000115)

function put_u8(msg, ofs, v)
   msg:data(ofs,v&0xFF)
end

function put_u16_LE(msg, ofs, v)
   msg:data(ofs,v&0xFF)
   msg:data(ofs+1,v>>8)
end

function put_u32_LE(msg, ofs, v)
   msg:data(ofs+0,v&0xFF)
   msg:data(ofs+1,(v>>8)&0xFF)
   msg:data(ofs+2,(v>>16)&0xFF)
   msg:data(ofs+3,(v>>24)&0xFF)
end

function put_u16_BE(msg, ofs, v)
   msg:data(ofs+1,v&0xFF)
   msg:data(ofs,v>>8)
end

function put_u32_BE(msg, ofs, v)
   msg:data(ofs+3,v&0xFF)
   msg:data(ofs+2,(v>>8)&0xFF)
   msg:data(ofs+1,(v>>16)&0xFF)
   msg:data(ofs+0,(v>>24)&0xFF)
end

local rev_counter = 0

--[[
   send SkyPower data. Called at 100Hz
--]]
function send_SkyPower(driver)

   --local msg = CANFrame()
   local t = get_time_sec()

   local RPM = 1200 + math.floor(1000*math.sin(t))
   rev_counter = rev_counter + (RPM/60.0)*0.01

   -- 0x100
   local msg = CANFrame()
   msg:id(FRM_100)
   put_u16_LE(msg,0,RPM)
   put_u16_LE(msg,2,13*10) -- ignition angle
   put_u16_LE(msg,4,45*10) -- throttle angle
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x101
   msg = CANFrame()
   msg:id(FRM_101)
   put_u16_LE(msg,2,917) -- air pressure
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x102
   msg = CANFrame()
   msg:id(FRM_102)
   put_u16_LE(msg,0,7*10) -- ingition gap
   put_u16_LE(msg,2,270*10) -- injection angle
   put_u16_LE(msg,4,37000) -- injection time
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x104
   msg = CANFrame()
   msg:id(FRM_104)
   put_u16_LE(msg,0,math.floor(14.8*10)) -- supply voltage
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x105
   msg = CANFrame()
   msg:id(FRM_105)
   put_u16_LE(msg,0,172*10) -- engine temp head 1
   put_u16_LE(msg,2,65*10) -- air temp
   put_u16_LE(msg,4,320*10) -- exhaust temp
   put_u16_LE(msg,6,113*10) -- ecu temp
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x106
   msg = CANFrame()
   msg:id(FRM_106)
   put_u32_LE(msg,0,math.floor(rev_counter))
   put_u8(msg,4,math.floor(t))
   put_u8(msg,5,math.floor(t/60))
   put_u16_LE(msg,6,math.floor(t/3600))
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x10A
   msg = CANFrame()
   msg:id(FRM_10A)
   put_u16_LE(msg,6,72*10) -- target load
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x10C
   msg = CANFrame()
   msg:id(FRM_10C)
   put_u16_LE(msg,4,145*10) -- engine head temp 2
   put_u16_LE(msg,6,315*10) -- exhaust temp 2
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x10D
   msg = CANFrame()
   msg:id(FRM_10D)
   put_u16_LE(msg,4,72*10) -- current load
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x10F
   msg = CANFrame()
   msg:id(FRM_10F)
   put_u16_LE(msg,4,2*10) -- fuel consumption
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x113
   msg = CANFrame()
   msg:id(FRM_113)
   put_u16_LE(msg,2,1*10) -- gen amps
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x114
   msg = CANFrame()
   msg:id(FRM_114)
   put_u16_LE(msg,2,2400*10) -- gen RPM
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- 0x115
   msg = CANFrame()
   msg:id(FRM_115)
   put_u16_LE(msg,4,30*100) -- gen current
   msg:dlc(8)
   driver:write_frame(msg, 10000)
end

--[[
   send HFE data. Called at 100Hz
--]]
function send_HFE(driver)

   --local msg = CANFrame()
   local t = get_time_sec()

   local RPM = 1200 + math.floor(1000*math.sin(t))
   rev_counter = rev_counter + (RPM/60.0)*0.01

   -- fast telem
   local msg = CANFrame()
   local base_id = uint32_t(0x88000005)
   msg:id(base_id | 0x00000)
   put_u16_BE(msg,1,RPM)
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- slow telem0
   msg = CANFrame()
   msg:id(base_id | 0x10000)
   put_u16_BE(msg,5,101324/2) -- air pressure
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- slow telem1
   msg = CANFrame()
   msg:id(base_id | 0x20000)
   put_u8(msg,0,35) -- inlet air temp, signed, deg C
   msg:dlc(8)
   driver:write_frame(msg, 10000)

   -- slow telem2
   msg = CANFrame()
   msg:id(base_id | 0x30000)
   put_u8(msg,1,math.floor((67+128)/1.5)) -- MAT
   put_u16_BE(msg,6,173) -- fuel flow in grams/hr
   msg:dlc(8)
   driver:write_frame(msg, 10000)
   

end

function update()
   if EFISIM_TYPE:get() == 1 then
      send_SkyPower(driver2)
   elseif EFISIM_TYPE:get() == 2 then
      send_HFE(driver2)
   end
   return update, math.floor(1000/EFISIM_RATE_HZ:get())
end

gcs:send_text(0,string.format("EFISIM: loaded"))

return update()
