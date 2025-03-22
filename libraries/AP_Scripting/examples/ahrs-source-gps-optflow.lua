-- switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
-- this script is intended to help vehicles automatically switch between GPS and optical flow
--
-- configure a forward or downward facing lidar with a range of at least 5m
-- setup RCx_OPTION = 90 (EKF Source Set) to select the source (low=GPS, middle=opticalflow, high=Not Used)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- SCR_ENABLE = 1 (enable scripting)
-- setup EK3_SRCn_ parameters so that GPS is the primary source, opticalflow is secondary.
--     EK3_SRC1_POSXY = 3 (GPS)
--     EK3_SRC1_VELXY = 3 (GPS)
--     EK3_SRC1_VELZ  = 3 (GPS)
--     EK3_SRC1_POSZ  = 1 (Baro)
--     EK3_SRC1_YAW   = 1 (Compass)
--     EK3_SRC2_POSXY = 0 (None)
--     EK3_SRC2_VELXY = 5 (OpticalFlow)
--     EK3_SRC2_VELZ  = 0 (None)
--     EK3_SRC2_POSZ  = 1 (Baro)
--     EK3_SRC2_YAW   = 1 (Compass)
--     EK3_SRC_OPTIONS    = 0 (Do not fuse all velocities)
--
-- SCR_USER1 holds the threshold (in meters) for rangefinder altitude (around 15 is a good choice)
--     if rangefinder distance >= SCR_USER1, source1 (GPS) will be used
--     if rangefinder distance < SCR_USER1, source2 (optical flow) will be used if innovations are below SRC_USER3 value
-- SCR_USER2 holds the threshold for GPS speed accuracy (around 0.3 is a good choice)
-- SCR_USER3 holds the threshold for optical flow quality (about 50 is a good choice)
-- SCR_USER4 holds the threshold for optical flow innovations (about 0.15 is a good choice)
--
-- When the 2nd auxiliary switch (300/Scripting1) is pulled high automatic source selection uses these thresholds:
---@diagnostic disable: need-check-nil

local rangefinder_rotation = 25     -- check downward (25) facing lidar
local source_prev = 0               -- previous source, defaults to primary source
local sw_auto_pos_prev = -1         -- previous auto source switch position
local auto_switch = false           -- true when auto switching between sources is active
local gps_usable_accuracy = 1.0     -- GPS is usable if speed accuracy is at or below this value
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local gps_vs_opticalflow_vote = 0   -- vote counter for GPS vs optical (-20 = GPS, +20 = optical flow)

-- initialise parameters
local scr_user1_param = Parameter('SCR_USER1') -- user1 param (rangefinder altitude threshold)
local scr_user2_param = Parameter('SCR_USER2') -- user2 param (GPS speed accuracy threshold)
local scr_user3_param = Parameter('SCR_USER3') -- user3 param (optical flow quality threshold)
local scr_user4_param = Parameter('SCR_USER4') -- user4 param (optical flow innovation threshold)

assert(optical_flow, 'could not access optical flow')

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
  if not rc_function_source and not rc_function_auto then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: RCx_FUNCTION=90 or 300 not set!")
    return update, 1000
  end

  -- check rangefinder distance threshold has been set
  local rangefinder_thresh_dist = scr_user1_param:get()     -- SCR_USER1 holds rangefinder threshold
  if (rangefinder_thresh_dist <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER1 to rangefinder threshold")
    return update, 1000
  end

  -- check GPS speed accuracy threshold has been set
  local gps_speedaccuracy_thresh = scr_user2_param:get()    -- SCR_USER2 holds GPS speed accuracy threshold
  if (gps_speedaccuracy_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER2 to GPS speed accuracy threshold")
    return update, 1000
  end

  -- check optical flow quality threshold has been set
  local opticalflow_quality_thresh = scr_user3_param:get()  -- SCR_USER3 holds opticalflow quality
  if (opticalflow_quality_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER3 to OpticalFlow quality threshold")
    return update, 1000
  end

  -- check optical flow innovation threshold has been set
  local opticalflow_innov_thresh = scr_user4_param:get()    -- SCR_USER4 holds opticalflow innovation
  if (opticalflow_innov_thresh <= 0) then
    gcs:send_text(0, "ahrs-source-gps-optflow.lua: set SCR_USER4 to OpticalFlow innovation threshold")
    return update, 1000
  end

  -- check if GPS speed accuracy is over threshold
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())
  local gps_over_threshold = (gps_speed_accuracy == nil) or (gps:speed_accuracy(gps:primary_sensor()) > gps_speedaccuracy_thresh)
  local gps_usable = (gps_speed_accuracy ~= nil) and (gps_speed_accuracy <= gps_usable_accuracy)

  -- check optical flow quality
  local opticalflow_quality_good = false
  if (optical_flow) then
    opticalflow_quality_good = (optical_flow:enabled() and optical_flow:healthy() and optical_flow:quality() >= opticalflow_quality_thresh)
  end

  -- get opticalflow innovations from ahrs (only x and y values are valid)
  local opticalflow_over_threshold = true
  local opticalflow_innov = ahrs:get_vel_innovations_and_variances_for_source(5)
  if (opticalflow_innov) then
    local opticalflow_xy_innov = math.sqrt(opticalflow_innov:x() * opticalflow_innov:x() + opticalflow_innov:y() * opticalflow_innov:y())
    opticalflow_over_threshold = (opticalflow_xy_innov == 0.0) or (opticalflow_xy_innov > opticalflow_innov_thresh)
  end

  -- get rangefinder distance
  local rngfnd_distance_m = 0
  if rangefinder:has_data_orient(rangefinder_rotation) then
    rngfnd_distance_m = rangefinder:distance_orient(rangefinder_rotation)
  end
  local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > rangefinder_thresh_dist)

  -- opticalflow is usable if quality and innovations are good and rangefinder is in range
  local opticalflow_usable = opticalflow_quality_good and (not opticalflow_over_threshold) and (not rngfnd_over_threshold)

  -- automatic selection logic --

  -- GPS vs opticalflow vote. "-1" to move towards GPS, "+1" to move to Non-GPS
  if (not gps_over_threshold) or (gps_usable and not opticalflow_usable) then
    -- vote for GPS if GPS accuracy good OR GPS is usable and opticalflow is unusable
    gps_vs_opticalflow_vote = math.max(gps_vs_opticalflow_vote - 1, -vote_counter_max)
  elseif opticalflow_usable then
    -- vote for opticalflow if usable
    gps_vs_opticalflow_vote = math.min(gps_vs_opticalflow_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                         -- auto source undecided if -1
  if gps_vs_opticalflow_vote <= -vote_counter_max then
    auto_source = 0                              -- GPS
  elseif gps_vs_opticalflow_vote >= vote_counter_max then
    auto_source = 1                              -- opticalflow
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

  -- if auto switch exists then read auto source switch position from RCx_FUNCTION = 300 (Scripting1)
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
