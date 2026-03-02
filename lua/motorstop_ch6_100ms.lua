-- motor_stop_control.lua
-- プロポのスイッチ操作により特定のモータ2基を一定時間停止させる

local RC_SWITCH_CH = 6
local MOT_FUNCTION_1 = 33  -- Motor1の関数ID
local MOT_FUNCTION_2 = 34  -- Motor2の関数ID
local PWM_STOP = 1000
local DURATION_MS = 100
local INTERVAL_MS = 100

function update()
  local pwm = rc:get_pwm(RC_SWITCH_CH)

  if pwm and pwm > 1800 then
    -- 関数IDから物理的なチャンネルインデックス (0-indexed) を取得する.
    local chan1 = SRV_Channels:find_channel(MOT_FUNCTION_1)
    local chan2 = SRV_Channels:find_channel(MOT_FUNCTION_2)

    if chan1 and chan2 then
      -- 取得したインデックスに対してPWM出力を上書きする.
      SRV_Channels:set_output_pwm_chan_timeout(chan1, PWM_STOP, DURATION_MS)
      SRV_Channels:set_output_pwm_chan_timeout(chan2, PWM_STOP, DURATION_MS)
      gcs:send_text(4, "Motors stopped: Overriding Ch " .. tostring(chan1) .. " & " .. tostring(chan2))
    else
      gcs:send_text(3, "Error: Motor channels not found")
    end
  end

  return update, INTERVAL_MS
end

gcs:send_text(6, "Motor Stop Script Loaded")
return update()
