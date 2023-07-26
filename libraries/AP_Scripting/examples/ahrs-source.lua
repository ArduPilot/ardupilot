-- switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
-- this script is intended to help vehicles move between GPS and Non-GPS environments
--
-- setup RCx_OPTION = 90 (EKF Pos Source) to select the source (low=primary, middle=secondary, high=tertiary)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- setup EK3_SRCn_ parameters so that GPS is the primary source, Non-GPS (i.e. T265) is secondary and optical flow tertiary
-- configure a forward or downward facing lidar with a range of more than 5m
--
-- When the auxiliary switch (ZigZag Auto) is pulled high automatic source selection uses these thresholds:
-- SCR_USER1 holds the threshold for rangefinder altitude:
--     if rangefinder distance >= SCR_USER1, source2 (ExtNav) will be used (if Non-GPS vel innovations are good)
--     if rangefinder distance < SCR_USER1, source3 (optical flow) will be used (if Non-GPS vel innovations are not good)
-- SCR_USER2 holds the threshold for GPS speed accuracy (around 0.3 is a good choice)
-- SCR_USER3 holds the threshold for Non-GPS vertical speed innovation (about 0.3 is a good choice)
--     if both GPS speed accuracy <= SCR_USER2 and ExternalNav speed variance >= SCR_USER3, source1 will be used
--     otherwise source2 (T265) or source3 (optical flow) will be used based on rangefinder distance
-- luacheck: only 0

local rangefinder_rotation = 25     -- check downward (25) facing lidar
local source_prev = 0               -- previous source, defaults to primary source
local sw_source_prev = -1           -- previous source switch position
local sw_auto_pos_prev = -1         -- previous auto source switch position
local auto_switch = false           -- true when auto switching between sources is active
local gps_usable_accuracy = 1.0     -- GPS is usable if speed accuracy is at or below this value
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local gps_vs_nongps_vote = 0        -- vote counter for GPS vs NonGPS (-20 = GPS, +20 = NonGPS)
local extnav_vs_opticalflow_vote = 0 -- vote counter for extnav vs optical flow (-20 = extnav, +20 = opticalflow)

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
    gcs:send_text(0, "ahrs-source.lua: RCx_FUNCTION=90 or 300 not set!")
    return update, 1000
  end

  -- check rangefinder distance threshold has been set
  local rangefinder_thresh_dist = param:get('SCR_USER1')  -- SCR_USER1 holds rangefinder threshold
  if (rangefinder_thresh_dist == nil) or (rangefinder_thresh_dist <= 0) then
    gcs:send_text(0, "ahrs-source.lua: set SCR_USER1 to rangefinder threshold")
    return update, 1000
  end

  -- check GPS speed accuracy threshold has been set
  local gps_speedaccuracy_thresh = param:get('SCR_USER2')  -- SCR_USER2 holds GPS speed accuracy threshold
  if (gps_speedaccuracy_thresh == nil) or (gps_speedaccuracy_thresh <= 0) then
    gcs:send_text(0, "ahrs-source.lua: set SCR_USER2 to GPS speed accuracy threshold")
    return update, 1000
  end

  -- check external nav innovation threshold has been set
  local extnav_innov_thresh = param:get('SCR_USER3')  -- SCR_USER3 holds Non-GPS vertical velocity innovation
  if (extnav_innov_thresh == nil) or (extnav_innov_thresh <= 0) then
    gcs:send_text(0, "ahrs-source.lua: set SCR_USER3 to ExtNav innovation threshold")
    return update, 1000
  end

  -- check if GPS speed accuracy is over threshold
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())
  local gps_over_threshold = (gps_speed_accuracy == nil) or (gps:speed_accuracy(gps:primary_sensor()) > gps_speedaccuracy_thresh)
  local gps_usable = (gps_speed_accuracy ~= nil) and (gps_speed_accuracy <= gps_usable_accuracy)

  -- get external nav innovations from ahrs
  local extnav_innov = Vector3f()
  local extnav_var = Vector3f()
  extnav_innov, extnav_var = ahrs:get_vel_innovations_and_variances_for_source(6)
  local extnav_over_threshold = (extnav_innov == nil) or (extnav_innov:z() == 0.0) or (math.abs(extnav_innov:z()) > extnav_innov_thresh)

  -- get rangefinder distance
  local rngfnd_distance_m = 0
  if rangefinder:has_data_orient(rangefinder_rotation) then
    rngfnd_distance_m = rangefinder:distance_cm_orient(rangefinder_rotation) * 0.01
  end
  local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > rangefinder_thresh_dist)

  -- NonGPS is usable if extnav innovations are good or rangefinder distance is short (for optical flow)
  local nongps_usable = (not extnav_over_threshold) or (not rngfnd_over_threshold)

  -- automatic selection logic --

  -- GPS vs NonGPS vote. "-1" to move towards GPS, "+1" to move to Non-GPS
  if (not gps_over_threshold) or (gps_usable and not nongps_usable) then
    -- vote for GPS if GPS accuracy good OR usable GPS and NonGPS unusable
    gps_vs_nongps_vote = math.max(gps_vs_nongps_vote - 1, -vote_counter_max)
  elseif nongps_usable then
    -- vote for NonGPS if extnav or opticalflow is usable
    gps_vs_nongps_vote = math.min(gps_vs_nongps_vote + 1, vote_counter_max)
  end

  -- extnav vs optical flow vote. "-1" to move towards extnav, "+1" to move to opticalflow
  if (not extnav_over_threshold) then
    -- vote for extnav is innovations under threshold
    extnav_vs_opticalflow_vote = math.max(extnav_vs_opticalflow_vote - 1, -vote_counter_max)
  elseif (not rngfnd_over_threshold) then
    -- vote for optical flow if rangefinder is not over threshold
    extnav_vs_opticalflow_vote = math.min(extnav_vs_opticalflow_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                         -- auto source undecided if -1
  if gps_vs_nongps_vote <= -vote_counter_max then
    auto_source = 0                              -- GPS
  elseif gps_vs_nongps_vote >= vote_counter_max then
    if extnav_vs_opticalflow_vote <= -vote_counter_max then
      auto_source = 1                            -- extnav
    elseif extnav_vs_opticalflow_vote >= vote_counter_max then
      auto_source = 2                            -- opticalflow
    end
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
    play_source_tune(source_prev)                -- alert user of source whether changed or not
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
