-- switches between AHRS/EKF sources based on the pilot's source selection switch or using an automatic source selection algorithm
-- this script is intended to help vehicles automatically switch between ExternalNav and optical flow
--
-- configure a downward facing lidar with a range of at least 5m
-- setup RCx_OPTION = 90 (EKF Pos Source) to select the source (low=external nav, middle=opticalflow, high=Not Used)
-- setup RCx_OPTION = 300 (Scripting1).  When this switch is pulled high, the source will be automatically selected
-- SRC_ENABLE = 1 (enable scripting)
-- setup EK3_SRCn_ parameters so that ExternalNav is the primary source, opticalflow is secondary
--     EK3_SRC1_POSXY = 6 (ExternalNav)
--     EK3_SRC1_VELXY = 6 (ExternalNav)
--     EK3_SRC1_VELZ  = 6 (ExternalNav)
--     EK3_SRC1_POSZ  = 6 (ExternalNav) or 1 (Baro)
--     EK3_SRC1_YAW   = 6 (ExternalNav) or 1 (Compass)
--     EK3_SRC2_POSXY = 0 (None)
--     EK3_SRC2_VELXY = 5 (OpticalFlow)
--     EK3_SRC2_VELZ  = 0 (None)
--     EK3_SRC2_POSZ  = 1 (Baro)
--     EK3_SRC2_YAW   = 1 (Compass)
--     EK3_SRC_OPTIONS = 0 (Do not fuse all velocities)
--
-- When the 2nd auxiliary switch (300/Scripting1) is pulled high automatic source selection is used based on the following criteria:
--     ESRC_EXTN_THRESH holds the threshold for ExternalNav innovation threshold (around 0.3 is a good choice)
--     ESRC_EXTN_QUAL holds the ExternalNav quality threshold (about 10 is a good choice)
--     ESRC_FLOW_QUAL holds the optical flow quality threshold (about 50 is a good choice)
--     ESRC_FLOW_THRESH holds the threshold for optical flow innovations (about 0.15 is a good choice)
--     ESRC_RNGFND_MAX holds the threshold (in meters) for rangefinder altitude
--
--     If ExternalNav's quality is above ESRC_EXTN_QUAL and innovations are below ESRC_EXTN_THRESH, ExternalNav is used
--     Optical flow is used if the above is not true and:
--         optical flow's quality is above ESRC_FLOW_QUAL
--         innovations are below ESRC_FLOW_THRESH
--         rangefinder distance is below ESRC_RNGFND_MAX

local PARAM_TABLE_KEY = 81
PARAM_TABLE_PREFIX = "ESRC_"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

-- bind a parameter to a variable
function bind_param(name)
  local p = Parameter()
  assert(p:init(name), string.format('could not find %s parameter', name))
  return p
end

-- add a parameter and bind it to a variable
local function bind_add_param(name, idx, default_value)
  assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
  return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- add param table
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'ahrs-source-extnav-optflow: could not add param table')

--[[
  // @Param: ESRC_EXTN_THRESH
  // @DisplayName: EKF Source ExternalNav Innovation Threshold
  // @Description: ExternalNav may be used if innovations are below this threshold
  // @Range: 0 1
  // @User: Standard
--]]
local ESRC_EXTN_THRESH = bind_add_param('EXTN_THRESH', 1, 0.3)

--[[
  // @Param: ESRC_EXTN_QUAL
  // @DisplayName: EKF Source ExternalNav Quality Threshold
  // @Description: ExternalNav may be used if quality is above this threshold
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local ESRC_EXTN_QUAL = bind_add_param('EXTN_QUAL', 2, 10)

--[[
  // @Param: ESRC_FLOW_THRESH
  // @DisplayName: EKF Source OpticalFlow Innovation Threshold
  // @Description: OpticalFlow may be used if innovations are below this threshold
  // @Range: 0 1
  // @User: Standard
--]]
local ESRC_FLOW_THRESH = bind_add_param('FLOW_THRESH', 3, 0.3)

--[[
  // @Param: ESRC_FLOW_QUAL
  // @DisplayName: EKF Source OpticalFlow Quality Threshold
  // @Description: OpticalFlow may be used if quality is above this threshold
  // @Range: 0 100
  // @Units: %
  // @User: Standard
--]]
local ESRC_FLOW_QUAL = bind_add_param('FLOW_QUAL', 4, 50)

--[[
  // @Param: ESRC_RNGFND_MAX
  // @DisplayName: EKF Source Rangefinder Max
  // @Description: OpticalFlow may be used if rangefinder distance is below this threshold
  // @Range: 0 50
  // @Units: m
  // @User: Standard
--]]
local ESRC_RNGFND_MAX = bind_add_param('RNGFND_MAX', 5, 7)

local last_rc_ekfsrc_pos = 0        -- last known position of EKF source select switch
local last_rc_autosrc_pos = 0       -- last known position of automatic source select switch
local rangefinder_rotation = 25     -- check downward (25) facing lidar
local auto_switch = false           -- true when auto switching between sources is active
local source_prev = 0               -- previous source, defaults to primary source (ExternalNav)
local vote_counter_max = 20         -- when a vote counter reaches this number (i.e. 2sec) source may be switched
local extnav_vs_opticalflow_vote = 0  -- vote counter for external nav vs optical (-20 = external nav, +20 = optical flow)

assert(visual_odom, 'could not access optical flow')
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

-- convert a boolean to a number
function bool_to_int(b)
  if b then
    return 1
  end
  return 0
end

-- the main update function
function update()

  -- check for EKF Source Select switch position change
  local rc_ekfsrc_pos = rc:get_aux_cached(90)  -- RCx_OPTION = 90 (EKF Pos Source)
  if rc_ekfsrc_pos == nil then
    rc_ekfsrc_pos = 0
  end
  local rc_ekfsrc_pos_changed = (rc_ekfsrc_pos ~= last_rc_ekfsrc_pos)
  last_rc_ekfsrc_pos = rc_ekfsrc_pos

  -- check for EKF automatic source select switch position change
  local rc_autosrc_pos = rc:get_aux_cached(300) -- RCx_OPTION = 300 (Scripting1)
  if rc_autosrc_pos == nil then
    rc_autosrc_pos = 0
  end
  local rc_autosrc_pos_changed = (rc_autosrc_pos ~= last_rc_autosrc_pos)
  last_rc_autosrc_pos = rc_autosrc_pos

  -- check external nav quality and innovations
  local extnav_over_threshold = true
  local extnav_innov
  extnav_innov, _ = ahrs:get_vel_innovations_and_variances_for_source(6)
  if (extnav_innov) then
    local extnav_xyz_innov = extnav_innov:length()
    extnav_over_threshold = (extnav_xyz_innov == 0.0) or (extnav_xyz_innov > ESRC_EXTN_THRESH:get())
    gcs:send_named_float("ExtNInnov", extnav_xyz_innov)
  end
  local extnav_usable = (not extnav_over_threshold) and visual_odom and visual_odom:healthy() and (visual_odom:quality() >= ESRC_EXTN_QUAL:get())
  if visual_odom then
    gcs:send_named_float("ExtNQuality", visual_odom:quality())
  end
  gcs:send_named_float("ExtNUsable", bool_to_int(extnav_usable))

  -- check optical flow quality and innovations
  local opticalflow_quality_good = false
  if (optical_flow) then
    opticalflow_quality_good = (optical_flow:enabled() and optical_flow:healthy() and optical_flow:quality() >= ESRC_FLOW_QUAL:get())
  end

  -- get opticalflow innovations from ahrs (only x and y values are valid)
  local opticalflow_over_threshold = true
  local opticalflow_innov
  opticalflow_innov, _ = ahrs:get_vel_innovations_and_variances_for_source(5)
  if (opticalflow_innov) then
    local opticalflow_xy_innov = math.sqrt(opticalflow_innov:x() * opticalflow_innov:x() + opticalflow_innov:y() * opticalflow_innov:y())
    opticalflow_over_threshold = (opticalflow_xy_innov == 0.0) or (opticalflow_xy_innov > ESRC_FLOW_THRESH:get())
    gcs:send_named_float("FlowInnov", opticalflow_xy_innov)
  end

  -- get rangefinder distance
  local rngfnd_distance_m = 0
  if rangefinder:has_data_orient(rangefinder_rotation) then
    rngfnd_distance_m = rangefinder:distance_cm_orient(rangefinder_rotation) * 0.01
  end
  local rngfnd_over_threshold = (rngfnd_distance_m == 0) or (rngfnd_distance_m > ESRC_RNGFND_MAX:get())

  -- opticalflow is usable if quality and innovations are good and rangefinder is in range
  local opticalflow_usable = opticalflow_quality_good and (not opticalflow_over_threshold) and (not rngfnd_over_threshold)
  gcs:send_named_float("FlowUsable", bool_to_int(opticalflow_usable))

  -- automatic selection logic --

  -- ExtNav vs opticalflow vote. "-1" to move towards EXtNav, "+1" to move to opticalflow
  if (extnav_usable) then
    -- vote for external nav if usable and innovations are low
    extnav_vs_opticalflow_vote = math.max(extnav_vs_opticalflow_vote - 1, -vote_counter_max)
  elseif opticalflow_usable then
    -- vote for opticalflow if usable
    extnav_vs_opticalflow_vote = math.min(extnav_vs_opticalflow_vote + 1, vote_counter_max)
  end

  -- auto source vote collation
  local auto_source = -1                         -- auto source undecided if -1
  if extnav_vs_opticalflow_vote <= -vote_counter_max then
    auto_source = 0                              -- external nav
  elseif extnav_vs_opticalflow_vote >= vote_counter_max then
    auto_source = 1                              -- opticalflow
  end

  -- read source switch position from user
  if rc_ekfsrc_pos_changed then                 -- check for changes in source switch position
    auto_switch = false                         -- disable auto switching of source
    if source_prev ~= rc_ekfsrc_pos then        -- check if switch position does not match source (there is a one-to-one mapping of switch position to source)
      source_prev = rc_ekfsrc_pos                -- record what source should now be (changed by ArduPilot vehicle code)
      gcs:send_text(MAV_SEVERITY.INFO, "Pilot switched to Source " .. string.format("%d", source_prev+1))
    else
      gcs:send_text(MAV_SEVERITY.INFO, "Pilot switched but already Source " .. string.format("%d", source_prev+1))
    end
    play_source_tune(source_prev)                -- alert user of source whether changed or not
  end

  -- if auto switch exists then read auto source switch position from RCx_FUNCTION = 300 (Scripting1)
  if rc_autosrc_pos_changed  then                -- check for changes in source auto switch position
    if rc_autosrc_pos == 0 then                  -- pilot has pulled switch low
      auto_switch = false                        -- disable auto switching of source
      if rc_ekfsrc_pos ~= source_prev then       -- check if source will change
        source_prev = rc_ekfsrc_pos              -- record pilot's selected source
        ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
        gcs:send_text(MAV_SEVERITY.INFO, "Auto source disabled, switched to Source " .. string.format("%d", source_prev+1))
      else
        gcs:send_text(MAV_SEVERITY.INFO, "Auto source disabled, already Source " .. string.format("%d", source_prev+1))
      end
    elseif rc_autosrc_pos == 2 then              -- pilot has pulled auto switch high
      auto_switch = true                         -- enable auto switching of source
      if auto_source < 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "Auto source enabled, undecided, Source " .. string.format("%d", source_prev+1))
      elseif auto_source ~= source_prev then     -- check if source will change
        source_prev = auto_source                -- record pilot's selected source
        ahrs:set_posvelyaw_source_set(source_prev)   -- switch to pilot's selected source
        gcs:send_text(MAV_SEVERITY.INFO, "Auto source enabled, switched to Source " .. string.format("%d", source_prev+1))
      else
        gcs:send_text(MAV_SEVERITY.INFO, "Auto source enabled, already Source " .. string.format("%d", source_prev+1))
      end
    end
    play_source_tune(source_prev)
  end

  -- auto switching
  if auto_switch and (auto_source >= 0) and (auto_source ~= source_prev) then
    source_prev = auto_source                  -- record selected source
    ahrs:set_posvelyaw_source_set(source_prev) -- switch to pilot's selected source
    gcs:send_text(MAV_SEVERITY.INFO, "Auto switched to Source " .. string.format("%d", source_prev+1))
    play_source_tune(source_prev)
  end

  return update, 100
end

return update()
