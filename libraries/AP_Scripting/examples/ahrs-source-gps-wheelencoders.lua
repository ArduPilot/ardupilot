-- This script helps vehicles move between GPS and Non-GPS environments using GPS and Wheel Encoders
--
-- setup RCx_OPTION = 90 (EKF Source Set) to select the source (low=primary, middle=secondary, high=tertiary)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- setup EK3_SRCn_ parameters so that GPS is the primary source, WheelEncoders are the secondary
--
-- When the auxiliary switch (ZigZag Auto) is pulled high automatic source selection uses these thresholds:
-- SCR_USER2 holds the threshold for GPS speed accuracy (around 0.3 is a good choice)
-- SCR_USER3 holds the threshold for GPS innovations (around 0.3 is a good choice)
--     if GPS speed accuracy <= SCR_USER2 and GPS innovations <= SRC_USER3 then the GPS (primary source set) will be used
--     otherwise wheel encoders (secondary source set) will be used
---@diagnostic disable: need-check-nil

local source_prev = 0               -- previous source, defaults to primary source
local sw_auto_pos_prev = -1         -- previous auto source switch position
local auto_switch = false           -- true when auto switching between sources is active
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local gps_vs_nongps_vote = 0        -- vote counter for GPS vs NonGPS (-20 = GPS, +20 = NonGPS)

-- play tune on buzzer to alert user to change in active source set
function play_source_tune(source)
  if (source) then
    if (source == 0) then
      notify:play_tune("L8C")       -- one long lower tone
    elseif (source == 1) then
      notify:play_tune("L12DD")     -- two fast medium tones
    elseif (source == 2) then
      notify:play_tune("L16FFF")    -- three very fast, high tones
    end
  end
end

-- the main update function
function update()

  -- check switches are configured
  -- at least one switch must be set
  -- source selection from RCx_FUNCTION = 90 (EKF Source Select)
  -- auto source from RCx_FUNCTION = 300 (Scripting1)
  local rc_function_source = rc:find_channel_for_option(90)
  local rc_function_auto = rc:find_channel_for_option(300)
  if (rc_function_source == nil) and (rc_function_auto == nil) then
    gcs:send_text(0, "ahrs-source-gps-wheelencoders.lua: RCx_FUNCTION=90 or 300 not set!")
    return update, 1000
  end

  -- check GPS speed accuracy threshold has been set
  local gps_speedaccuracy_thresh = param:get('SCR_USER2')  -- SCR_USER2 holds GPS speed accuracy threshold
  if (gps_speedaccuracy_thresh == nil) or (gps_speedaccuracy_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-wheelencoders.lua: set SCR_USER2 to GPS speed accuracy threshold")
    return update, 1000
  end

  -- check GPS innovation threshold has been set
  local gps_innov_thresh = param:get('SCR_USER3')  -- SCR_USER3 holds GPS velocity innovation
  if (gps_innov_thresh == nil) or (gps_innov_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-wheelencoders.lua: set SCR_USER3 to GPS innovation threshold")
    return update, 1000
  end

  -- check if GPS speed accuracy is over threshold
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())
  local gps_over_threshold = (gps_speed_accuracy == nil) or (gps:speed_accuracy(gps:primary_sensor()) > gps_speedaccuracy_thresh)

  -- get GPS innovations from ahrs
  local gps_innov = ahrs:get_vel_innovations_and_variances_for_source(3)
  local gps_innov_over_threshold = (gps_innov == nil) or (gps_innov:z() == 0.0) or (math.abs(gps_innov:z()) > gps_innov_thresh)

  -- automatic selection logic --

  -- GPS vs NonGPS vote. "-1" to move towards GPS, "+1" to move to Non-GPS
  if (not gps_over_threshold) and (not gps_innov_over_threshold) then
    -- vote for GPS if GPS accuracy good AND innovations are low
    gps_vs_nongps_vote = math.max(gps_vs_nongps_vote - 1, -vote_counter_max)
  else
    -- otherwise vote for NonGPS (wheel encoders)
    gps_vs_nongps_vote = math.min(gps_vs_nongps_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                         -- auto source undecided if -1
  if gps_vs_nongps_vote <= -vote_counter_max then
    auto_source = 0                              -- GPS
  elseif gps_vs_nongps_vote >= vote_counter_max then
    auto_source = 1                              -- Non-GPS / wheel encoders
  end

  -- read source switch position from RCx_FUNCTION = 90 (EKF Source Select)
  local sw_source_pos = rc_function_source:get_aux_switch_pos()
  if sw_source_pos ~= sw_source_pos_prev then    -- check for changes in source switch position
    sw_source_pos_prev = sw_source_pos           -- record new switch position so we can detect changes
    auto_switch = false                          -- disable auto switching of source
    if source_prev ~= sw_source_pos then         -- check if switch position does not match source (there is a one-to-one mapping of switch to source)
      source_prev = sw_source_pos                -- record what source should now be (changed by ArduPilot vehicle code)
      gcs:send_text(0, "Pilot switched to Source " .. string.format("%d", source_prev+1))
    else
      gcs:send_text(0, "Pilot switched but already Source " .. string.format("%d", source_prev+1))
    end
    play_source_tune(source_prev)                -- alert user of source regardless of whether it has changed or not
  end

  -- read auto source switch position from RCx_FUNCTION = 300 (Scripting1)
  if rc_function_auto then
    local sw_auto_pos = rc_function_auto:get_aux_switch_pos()
    if sw_auto_pos ~= sw_auto_pos_prev  then       -- check for changes in source auto switch position
      sw_auto_pos_prev = sw_auto_pos               -- record new switch position so we can detect changes
      if sw_auto_pos == 0 then                     -- pilot has pulled switch low
        auto_switch = false                        -- disable auto switching of source
        if sw_source_pos ~= source_prev then       -- check if source will change
          source_prev = sw_source_pos              -- record pilot's selected source
          ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
          gcs:send_text(0, "Auto source disabled, switched to Source " .. string.format("%d", source_prev+1))
        else
          gcs:send_text(0, "Auto source disabled, already Source " .. string.format("%d", source_prev+1))
        end
      elseif sw_auto_pos == 2 then                 -- pilot has pulled switch high
        auto_switch = true                         -- enable auto switching of source
        if auto_source < 0 then
          gcs:send_text(0, "Auto source enabled, undecided, Source " .. string.format("%d", source_prev+1))
        elseif auto_source ~= source_prev then     -- check if source will change
          source_prev = auto_source                -- record pilot's selected source
          ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
          gcs:send_text(0, "Auto source enabled, switched to Source " .. string.format("%d", source_prev+1))
        else
          gcs:send_text(0, "Auto source enabled, already Source " .. string.format("%d", source_prev+1))
        end
      end
      play_source_tune(source_prev)
    end
  end

  -- auto switching
  if auto_switch and (auto_source >= 0) and (auto_source ~= source_prev) then
    source_prev = auto_source                  -- record selected source
    ahrs:set_posvelyaw_source_set(source_prev)     -- switch to pilot's selected source
    gcs:send_text(0, "Auto switched to Source " .. string.format("%d", source_prev+1))
    play_source_tune(source_prev)
  end

  return update, 100
end

return update()
