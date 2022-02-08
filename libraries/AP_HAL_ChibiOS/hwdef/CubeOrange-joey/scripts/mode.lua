
-- 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate
-- and arming:is_armed()

local MODE_AUTO = 3
local MODE_LOITER = 5
local MODE_ALT_HOLD = 2
local MODE_RTL = 6
local MODE_LAND = 9
local counter = 0

function update()
  local pos = ahrs:get_location()

  if (vehicle:get_mode() == MODE_AUTO or vehicle:get_mode() == MODE_LOITER) then
    setfence(true)
  elseif vehicle:get_mode() == MODE_ALT_HOLD and not pos then
    setfence(false)
  elseif vehicle:get_mode() == MODE_ALT_HOLD and pos then
    setfence(true)
  end
  
  if (vehicle:get_mode() ~= MODE_AUTO and vehicle:get_mode() ~= MODE_LOITER and vehicle:get_mode() ~= MODE_ALT_HOLD and vehicle:get_mode() ~= MODE_RTL and vehicle:get_mode() ~= MODE_LAND) then
    if not pos then
        vehicle:set_mode(MODE_ALT_HOLD)
        gcs:send_text(0, "JoeyFence: Invalid Mode, Changing no gps")
    else
        vehicle:set_mode(MODE_LOITER)
        gcs:send_text(0, "JoeyFence: Invalid Mode, Changing gps")
    end
  end

  if (counter >= 5 * 5) then
    --gcs:send_text(0, "JoeyFence: Running")
    counter = 0
  end

  counter = counter + 1
  -- run at 5Hz
  return update, 200
end

function setfence(enabled)
  if (enabled) then
    value = param:get('FENCE_ENABLE')
    if(value ~= 1) then
      param:set('FENCE_ENABLE',1)
      gcs:send_text(0, "JoeyFence: Enabled")
    end
  else
    value = param:get('FENCE_ENABLE')
    if(value ~= 0) then
      param:set('FENCE_ENABLE',0)
      gcs:send_text(0, "JoeyFence: Disabled")
    end
  end
end

-- start running update loop
return update()
