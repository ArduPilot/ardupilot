-- get GPS data from ardupilot's native bindings then resynthesize into a
-- virtual NMEA GPS and feed back through the serial device sim bindings.
-- demonstrates the bindings and provides the opportunity for script-controlled
-- tampering and other such activities.

-- parameters:
-- SCR_ENABLE 1
-- SCR_SDEV_EN 1
-- SCR_SDEV1_PROTO 5
-- SERIAL3_PROTOCOL 5
-- SERIAL4_PROTOCOL -1
-- GPS2_TYPE 5
-- GPS_PRIMARY 1
-- GPS_AUTO_SWITCH 0

local ser_device = serial:find_simulated_device(5, 0)
if not ser_device then
  error("SCR_SDEV_EN must be 1 and SCR_SDEVn_PROTO must be 5")
end

function convert_coord(coord, dir)
  -- convert ardupilot degrees*1e7 to NMEA degrees + decimal minutes + dir.
  -- the first character of dir is used if the coordinate is positive,
  -- the second if negative.

  -- handle sign
  if coord < 0 then
    coord = -coord
    dir = dir:sub(2, 2)
  else
    dir = dir:sub(1, 1)
  end

  local degrees = coord // 10000000 -- integer divide
  coord = coord - (degrees * 10000000) -- remove that portion
  local minutes = coord * (60/10000000) -- float divide

  return ("%03d%08.5f,%s"):format(degrees, minutes, dir)
end

function convert_time(time_week, time_week_ms)
  -- convert ardupilot GPS time to NMEA UTC date/time strings

  -- GPS week 1095 starts on Dec 31 2000
  local seconds_per_week = uint32_t(86400*7)
  timestamp_s = uint32_t(time_week - 1095)*seconds_per_week
  -- subtract one day to get to Jan 1 2001, then 18 additional seconds to
  -- account for the GPS to UTC leap second induced offset
  timestamp_s = timestamp_s - uint32_t(86400 + 18)
  -- add in time within the week
  timestamp_s = timestamp_s + (time_week_ms/uint32_t(1000))

  timestamp_s = timestamp_s:toint() -- seconds since Jan 1 2001

  local ts_year = 2001
  local day_seconds = 86400
  while true do
    local year_seconds = day_seconds * ((ts_year % 4 == 0) and 366 or 365)
    if timestamp_s >= year_seconds then
      timestamp_s = timestamp_s - year_seconds
      ts_year = ts_year + 1
    else
      break
    end
  end

  local month_days = {31, (ts_year % 4 == 0) and 29 or 28,
    31, 30, 31, 30, 31, 31, 30, 31, 30, 31}

  local ts_month = 1
  for _, md in ipairs(month_days) do
    local month_seconds = 86400 * md
    if timestamp_s >= month_seconds then
        timestamp_s = timestamp_s - month_seconds
        ts_month = ts_month + 1
    else
      break
    end
  end

  local ts_day = 1+(timestamp_s // 86400)
  timestamp_s = timestamp_s % 86400

  local ts_hour = timestamp_s // 3600
  timestamp_s = timestamp_s % 3600

  local ts_minute = timestamp_s // 60
  local ts_second = timestamp_s % 60

  local date = ("%02d%02d%02d"):format(ts_year-2000, ts_month, ts_day)
  local time = ("%02d%02d%02d.%01d"):format(ts_hour, ts_minute, ts_second,
    (time_week_ms % 1000):toint()//100)

  return date, time
end

function get_gps_data(instance)
  -- get GPS data from ardupilot scripting bindings in native format
  local data = {
    hdop = gps:get_hdop(instance),
    time_week_ms = gps:time_week_ms(instance),
    time_week = gps:time_week(instance),
    sats = gps:num_sats(instance),
    crs = gps:ground_course(instance),
    spd = gps:ground_speed(instance),
    loc = gps:location(instance),
    status = gps:status(instance),
  }
  if data.status < gps.GPS_OK_FIX_3D then
    return nil -- don't bother with invalid data
  end
  return data
end

function arrange_nmea(data)
  -- convert ardupilot data entries to NMEA-compatible format
  local ts_date, ts_time = convert_time(data.time_week, data.time_week_ms)

  return {
    time = ts_time,
    lat = convert_coord(data.loc:lat(), "NS"),
    lng = convert_coord(data.loc:lng(), "EW"),
    spd = data.spd / 0.514, -- m/s to knots
    crs = data.crs, -- degrees
    date = ts_date,
    sats = data.sats,
    hdop = data.hdop,
    alt = data.loc:alt()/100,
  }
end

function wrap_nmea(msg)
  -- compute checksum and add header and footer
  local checksum = 0
  for i = 1,#msg do
    checksum = checksum ~ msg:byte(i, i)
  end

  return ("$%s*%02X\r\n"):format(msg, checksum)
end

function format_nmea(data)
  -- format data into complete NMEA sentences
  local rmc_raw = ("GPRMC,%s,A,%s,%s,%03f,%03f,%s,000.0,E"):format(
    data.time, data.lat, data.lng, data.spd, data.crs, data.date)

  local gga_raw = ("GPGGA,%s,%s,%s,1,%02d,%05.2f,%06.2f,M,0,M,,"):format(
    data.time, data.lat, data.lng, data.sats, data.hdop/100, data.alt)

  return wrap_nmea(rmc_raw), wrap_nmea(gga_raw)
end

function update()
  -- get data from first instance (we are the second)
  local ardu_data = get_gps_data(0)

  if ardu_data then
    local nmea_data = arrange_nmea(ardu_data)
    local rmc, gga = format_nmea(nmea_data)

    if ser_device:writestring(rmc) ~= #rmc 
        or ser_device:writestring(gga) ~= #gga then
      error("overflow, ardupilot is not processing the data, check config!")
    end
  end

  return update, 200 -- 5Hz like a real GPS
end

return update()
