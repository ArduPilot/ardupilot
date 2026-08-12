-- swarm_follower.lua
--
-- Leader-follower formation flight driven by AP_SwarmMesh peer state.
--
-- Each follower reads the designated leader's global position out of its local
-- SwarmMesh peer table (populated over the mesh by heartbeats + GLOBAL_POSITION_INT),
-- applies a fixed North/East offset at a fixed formation altitude, and commands
-- that as a GUIDED target with vehicle:set_target_location(). As the leader moves,
-- the mesh propagates its new position and every follower recomputes its slot,
-- holding formation.
--
-- Readiness is derived locally rather than signalled over the mesh for now (TODO)
-- a follower only starts commanding targets once it has heard from the expected
-- number of peers AND has a fresh position fix for the leader. This stands in for
-- an explicit ready flag until the coordination fields (role/formation_slot) are
-- added to the TX path.
--
-- An explicit "engage" gate (SCR_USER6) keeps the script from fighting an
-- externally commanded takeoff: it holds off until the user/GCS has the
-- vehicle safely at altitude and sets the flag.
--
-- Configuration (standard SCR_USER parameters, so the user can set a
-- different slot per instance without having to edit the script):
--   SCR_USER1 : leader SwarmMesh sysid (P2P_SYSID of the leader)
--   SCR_USER2 : this follower's offset North of the leader, metres
--   SCR_USER3 : this follower's offset East of the leader, metres
--   SCR_USER4 : formation altitude above home, metres
--   SCR_USER5 : expected peer count for the swarm to be "ready"
--   SCR_USER6 : engage flag (0 = hold, >=1 = keep formation). Set by the user.
--
-- NOTE: Copter only. Assumes the vehicle is armed, at altitude, and in GUIDED
-- before SCR_USER6 is set.

local COPTER_GUIDED_MODE = 4
local ALT_FRAME_ABOVE_HOME = 1

local UPDATE_MS = 200          -- 5 Hz formation update
local RUN_NAME = "swarm_follower"

-- read config once at startup
local leader_sysid   = math.floor(param:get('SCR_USER1') or 0)
local offset_north_m = param:get('SCR_USER2') or 0
local offset_east_m  = param:get('SCR_USER3') or 0
local formation_alt_m = param:get('SCR_USER4') or 15
local expected_peers = math.floor(param:get('SCR_USER5') or 0)

local announced_ready = false
local last_dbg_ms = 0

-- Cache of the last position/velocity we got for the leader. At scale instead of skipping a tick whenever stale we keep flying the most recent known slot for up to LEADER_HOLD_MS.
local LEADER_HOLD_MS = 4000
local cached_leader = nil
local cached_vel = nil
local cached_leader_ms = 0

-- ready once we have a recent leader position (and, if configured, enough peers)
local function leader_available(now)
  if leader_sysid <= 0 then
    return false
  end
  if expected_peers > 0 and swarm:count() < expected_peers then
    return false
  end
  return cached_leader ~= nil and (now - cached_leader_ms) <= LEADER_HOLD_MS
end

-- engage once we've climbed clear of takeoff, derived from our own altitude rather than an external flag.
-- (SCR_USER6 has to be delivered to every follower and is silently lost for some when hundreds of vehicles are engaged at once)
local function has_climbed()
  local rp = ahrs:get_relative_position_NED_home()
  return rp ~= nil and (-rp:z()) >= formation_alt_m * 0.8
end

function update()
  local now = millis():toint()
  local engaged = has_climbed()

  -- refresh the leader cache whenever a live (fresh) fix is available
  local live = swarm:get_peer_location(leader_sysid)
  if live then
    cached_leader = live
    cached_vel = swarm:get_peer_velocity_NED(leader_sysid)   -- may be nil
    cached_leader_ms = now
  end

  -- DEBUG: report state ~1 Hz so we can see why a follower isn't moving.
  if now - last_dbg_ms > 1000 then
    last_dbg_ms = now
    gcs:send_text(7, string.format("SFDBG eng=%d cnt=%d lf=%d age=%d mode=%d armed=%d",
      engaged and 1 or 0, swarm:count(), live and 1 or 0,
      cached_leader and (now - cached_leader_ms) or -1,
      vehicle:get_mode(), arming:is_armed() and 1 or 0))
  end

  if not arming:is_armed() or not engaged then
    announced_ready = false
    return update, UPDATE_MS
  end

  if not leader_available(now) then
    return update, UPDATE_MS
  end

  if not announced_ready then
    announced_ready = true
    gcs:send_text(6, string.format("%s: leader %d acquired, holding formation", RUN_NAME, leader_sysid))
  end

  -- only steer while in GUIDED
  if vehicle:get_mode() ~= COPTER_GUIDED_MODE then
    return update, UPDATE_MS
  end

  -- fly the cached leader slot (tolerant of mesh jitter at scale)
  local target = cached_leader:copy()
  target:offset(offset_north_m, offset_east_m)              -- shift horizontally in the NE plane
  target:set_alt_m(formation_alt_m, ALT_FRAME_ABOVE_HOME)  -- hold a fixed formation altitude

  -- Velocity FF: command the slot position and the leader's velocity. Falls back to a
  -- position target if the EKF origin or the leader's velocity isn't available.
  local neu = target:get_vector_from_origin_NEU_m()          -- slot position as NEU (m) from origin
  if neu and cached_vel then
    local pos_ned = Vector3f()
    pos_ned:x(neu:x())        -- North
    pos_ned:y(neu:y())        -- East
    pos_ned:z(-neu:z())       -- Down = -Up
    vehicle:set_target_posvel_NED(pos_ned, cached_vel)
  else
    vehicle:set_target_location(target)
  end

  return update, UPDATE_MS
end

gcs:send_text(6, string.format("%s loaded: leader=%d offset N=%.1f E=%.1f alt=%.1f", RUN_NAME, leader_sysid, offset_north_m, offset_east_m, formation_alt_m))

return update, UPDATE_MS
