--[[ 
   Scripting backend driver for InertialLabs external INS
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 45
PARAM_TABLE_PREFIX = "ILABS_"

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

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 6), 'ILabs: could not add param table')

--[[
  // @Param: ILABS_ENABLE
  // @DisplayName: Inertial Labs enable
  // @Description: Enable InertialLabs driver
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local ILABS_ENABLE = bind_add_param('ENABLE',  1, 0)

--[[
  // @Param: ILABS_BAUD
  // @DisplayName: Inertial Labs uart baudrate
  // @Description: Inertial Labs uart baudrate
  // @CopyFieldsFrom: SERIAL1_BAUD
  // @User: Standard
--]]
local ILABS_BAUD = bind_add_param('BAUD',  2, 460800)

--[[
  // @Param: ILABS_DEBUG
  // @DisplayName: Inertial Labs debug enable
  // @Description: Inertial Labs debug enable
  // @Values: 0:Disable,1:Enabled
  // @User: Advanced
--]]
local ILABS_DEBUG = bind_add_param('DEBUG', 3, 0)

if ILABS_ENABLE:get() == 0 then
   gcs:send_text(MAV_SEVERITY.INFO, "ILabs disabled")
   return
end

local uart = serial:find_serial(0) -- first scripting serial
if not uart then
   gcs:send_text(MAV_SEVERITY.ERROR, "ILabs: unable to find scripting serial")
   return
end
uart:begin(ILABS_BAUD:get())

local GRAVITY_MSS = 9.80665

--[[
   discard pending bytes
--]]
local function discard_pending()
   local n = uart:available():toint()
   for _ = 1, n do
      uart:read()
   end
end

local csum = 0

--[[
   read n bytes from the uart, returning as a lua string, and updating csum
--]]
local function read_bytes(n)
   local ret = uart:readstring(n)
   local len = #ret
   for i = 1, len do
      csum = csum + string.byte(ret, i)
   end
   return ret
end

local function scaled_vector3(x,y,z,scale)
   local v = Vector3f()
   v:x(x)
   v:y(y)
   v:z(z)
   v = v:scale(scale)
   return v
end

local state = EAHRS_State()
local last_pkt_ms = uint32_t(0)

--[[
   check for incoming data
--]]
local function update()
   local n_bytes = uart:available():toint()
   -- only handle one packet form at the moment
   local pkt_size = 172
   if n_bytes < pkt_size then
      return
   end
   if n_bytes > pkt_size then
      discard_pending()
      return
   end
   local magic = string.unpack("<H", read_bytes(2))
   if magic ~= 0x55AA then
      discard_pending()
      return
   end
   -- reset checksum after header
   csum = 0
   local msg_type,msg_id,msg_len,num_messages = string.unpack("<BBHB", read_bytes(5))
   if msg_type ~= 1 or msg_id ~= 0x95 or msg_len ~= pkt_size-2 or num_messages ~= 27 then
      discard_pending()
      return
   end
   local expected_messages = { 0x01, 0x3C, 0x23, 0x21, 0x25, 0x24, 0x07, 0x12, 0x10, 0x58, 0x57, 0x53, 0x4a,
                               0x3b, 0x30, 0x32, 0x3e, 0x36, 0x41, 0xc0, 0x28, 0x86, 0x8a, 0x8d, 0x50, 0x52, 0x5a }
   for i = 1, #expected_messages do
      local msg = string.unpack("<B", read_bytes(1))
      if msg ~= expected_messages[i] then
         discard_pending()
         return
      end
   end

   -- lots of unused variables parsed here, we don't want them all as _ as we can't see what they are
   -- luacheck: ignore 211 (Unused local variable)

   local gps_ins_time_ms, gps_week, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = string.unpack("<IHiiiiii", read_bytes(30))
   local pressure_pa2, baro_alt, mag_x, mag_y, mag_z = string.unpack("<Hihhh", read_bytes(12))
   local yaw, pitch, roll = string.unpack("<hhh", read_bytes(6))
   local vel_x, vel_y, vel_z = string.unpack("<iii", read_bytes(12))
   local lat, lon, alt = string.unpack("<iii", read_bytes(12))
   local kf_vel_x, kf_vel_y, kf_vel_z = string.unpack("<BBB", read_bytes(3))
   local kf_pos_x, kf_pos_y, kf_pos_z = string.unpack("<HHH", read_bytes(6))
   local unit_status, fix_type, spoofing_status, num_sats = string.unpack("<HBBB", read_bytes(5))
   local gps_lat, gps_lon, gps_alt = string.unpack("<iii", read_bytes(12))
   local hor_speed, track_over_ground, ver_speed = string.unpack("<iHi", read_bytes(10))
   local gnss_pos_timestamp, gnss_info1, gnss_info2 = string.unpack("<IBB", read_bytes(6))
   local gnss_new_data, gnss_jam_status, differential_pressure, true_airspeed = string.unpack("<BBih", read_bytes(8))
   local wind_x, wind_y, wind_z = string.unpack("<hhh", read_bytes(6))
   local air_data_status, supply_voltage, temperature, unit_status2 = string.unpack("<HHhH", read_bytes(8))
   local crc1 = csum & 0xFFFF
   local crc2 = string.unpack("<H", read_bytes(2))
   if crc1 ~= crc2 then
      return
   end

   local now_ms = millis()
   if ILABS_DEBUG:get() > 0 then
      local dt = (now_ms - last_pkt_ms):tofloat()*0.001
      if dt > 0.01 then
         gcs:send_text(MAV_SEVERITY.INFO, string.format("ILabs: dt=%.3f", dt))
      end
   end
   last_pkt_ms = now_ms

   local accel = scaled_vector3(accel_y, accel_x, -accel_z, GRAVITY_MSS*1.0e-6)
   local gyro = scaled_vector3(gyro_y, gyro_x, -gyro_z, math.rad(1)*1.0e-5)
   local quat = Quaternion()
   quat:from_euler(math.rad(roll*0.01), math.rad(pitch*0.01), math.rad(yaw*0.01))
   local vel = scaled_vector3(vel_y, vel_x, -vel_z, 0.01)
   local loc = Location()
   loc:lat(lat)
   loc:lng(lon)
   loc:alt(alt)

   state:accel(accel)
   state:gyro(gyro)
   state:quat(quat)
   state:velocity(vel)
   state:location(loc)
   state:origin(loc)

   state:have_quaternion(true)
   state:have_velocity(true)
   state:have_location(true)
   state:have_quaternion(true)
   if fix_type > 1 and num_sats > 3 then
      state:have_origin(true)
   end

   local vel_gate = 5
   local pos_gate = 5
   local hgt_gate = 5
   local mag_var = 0
   state:velocity_variance(scaled_vector3(kf_vel_x,kf_vel_y,kf_vel_z,0.001):length()/vel_gate)
   state:pos_horiz_variance(scaled_vector3(kf_pos_x,kf_pos_y,0,0.001):length()/pos_gate)
   state:pos_vert_variance(math.abs(kf_pos_z*0.001)/hgt_gate)
   state:compass_variance(mag_var)
   
   local filter_status = nav_filter_status_flags_t()
   filter_status:attitude(true)
   filter_status:horiz_vel(true)
   filter_status:vert_vel(true)
   filter_status:horiz_pos_rel(true)
   filter_status:horiz_pos_abs(true)
   filter_status:vert_pos(true)
   filter_status:pred_horiz_pos_rel(true)
   filter_status:pred_horiz_pos_abs(true)
   filter_status:using_gps(fix_type>1)
   filter_status:initialized(true)

   -- provide position, velocity, attitude
   ExternalAHRS:handle_scripting(state, filter_status)

   -- provide IMU data (this will be discarded if EAHRS_SENSORS doesn't include IMU)
   local ins_data = ins_data_message_t()
   ins_data:accel(accel)
   ins_data:gyro(gyro)
   ins_data:temperature(temperature*0.1)
   ins:handle_external(ins_data)

   -- provide baro data
   local baro_data = baro_data_message_t()
   baro_data:instance(0)
   baro_data:pressure_pa(pressure_pa2*2)
   baro_data:temperature(temperature*0.1)
   baro:handle_external(baro_data)

   -- provide mag data
   local mag_data = mag_data_message_t()
   mag_data:field(scaled_vector3(mag_y,mag_x,-mag_z,10*0.01))
   compass:handle_external(mag_data)

   -- provide gps data
   if gnss_new_data ~= 0 then
      local gps_data = gps_data_message_t()
      gps_data:gps_week(gps_week)
      gps_data:ms_tow(gps_ins_time_ms)
      gps_data:fix_type(fix_type+1)
      gps_data:satellites_in_view(num_sats)
      gps_data:horizontal_pos_accuracy(0.1)
      gps_data:vertical_pos_accuracy(0.1)
      gps_data:longitude(gps_lon)
      gps_data:latitude(gps_lat)
      gps_data:msl_altitude(gps_alt)
      gps_data:ned_vel_north(vel_y*0.01)
      gps_data:ned_vel_east(vel_x*0.01)
      gps_data:ned_vel_down(-vel_z*0.01)
      local gps_instance = gps:get_first_external_instance()
      if gps_instance ~= nil then
         gps:handle_external(gps_data, gps_instance)
      end
   end

   -- provide airspeed data
   local airspeed_data = airspeed_data_message_t()
   airspeed_data:differential_pressure(differential_pressure*1.0e-4)
   airspeed_data:temperature(temperature*0.1)
   airspeed:handle_external(airspeed_data)
   
end

gcs:send_text(MAV_SEVERITY.INFO, "ILabs loaded")

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs:send_text(MAV_SEVERITY.ERROR, "Internal Error: " .. err)
        return protected_wrapper, 250
    end
    return protected_wrapper, 2
end

-- start running update loop
return protected_wrapper()
