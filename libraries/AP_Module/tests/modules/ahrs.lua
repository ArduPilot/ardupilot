-- SPDX-License-Identifier: GPL-3.0-or-later
local function number(v)
  return math.floor(v * 1000)
end

local function vector(name, v)
  if v == nil then
    gcs:send_text(6, "LUA AHRS " .. name .. " nil")
  else
    gcs:send_text(6, string.format("LUA AHRS %s %d %d %d", name, number(v:x()), number(v:y()), number(v:z())))
  end
end

local function update()
  vector("G", ahrs:get_gyro())
  vector("V", ahrs:get_velocity_NED())
  vector("P", ahrs:get_relative_position_NED_origin())
  local vel, pos, height, mag, tas = ahrs:get_variances()
  if vel == nil then
    gcs:send_text(6, "LUA AHRS E nil")
  else
    gcs:send_text(6, string.format("LUA AHRS E %d %d %d %d %d %d %d",
      number(vel), number(pos), number(height), number(mag:x()), number(mag:y()), number(mag:z()), number(tas)))
  end
  local q = ahrs:get_quaternion()
  if q == nil then
    gcs:send_text(6, "LUA AHRS Q nil")
  else
    gcs:send_text(6, string.format("LUA AHRS Q %d %d %d %d",
      number(q:q1()), number(q:q2()), number(q:q3()), number(q:q4())))
  end
  return update, 500
end
return update()
