-- swarm_coordination.lua
--
-- Publishing and reading coordination state over AP_SwarmMesh.
--
-- The mesh carries vehicle telemetry on its own, but it assigns no meaning to
-- role, task, formation slot or priority. This example shows both halves of the hook.
--
--   swarm:set_coord_state(state)          publish our own state to every peer
--   swarm:get_peer_coord_state(sysid)     read what a peer last published
--
-- The state is broadcast at P2P_SR_COORD Hz until it is replaced by another
-- call, so a script only needs to call set_coord_state() when something
-- changes. Nothing is transmitted until the first call, so vehicles with no
-- coordination logic will not be transmitted.
--
-- Alongside the named fields there are AP_SWARMMESH_COORD_USER_MAX bytes of
-- payload the library never looks at, for coordination the wire format does
-- not name. Every script in the swarm has to agree on what they mean.
--
-- Set P2P_TYPE to a mesh backend and P2P_SR_COORD to a non zero rate to run this.

local ROLE_LEADER   = 1
local ROLE_FOLLOWER = 2

local UPDATE_MS = 1000

-- our own slot in the formation, and who we consider the leader
local MY_SLOT     = 3
local LEADER_SYSID = 1

-- byte offsets we have chosen for our own use inside the opaque payload
local USER_READY   = 0   -- 1 once we are happy to fly the formation
local USER_FUEL_PCT = 1  -- battery remaining, so peers can prefer a fresher vehicle

local function publish()
  local state = SwarmCoordState()

  state:role(param:get('MAV_SYSID') == LEADER_SYSID and ROLE_LEADER or ROLE_FOLLOWER)
  state:task_id(1)
  state:formation_slot(MY_SLOT)
  state:priority(10)

  -- where we intend to be, so peers can deconflict against our intent rather
  -- than only against where we currently are
  local target = vehicle:get_target_location()
  if target then
    state:target_lat(target:lat())
    state:target_lng(target:lng())
    state:target_alt_mm(target:alt() * 10)   -- Location alt is cm, the basket is mm
  end

  -- anything the wire format does not name goes in the opaque bytes
  state:user(USER_READY, arming:is_armed() and 1 or 0)
  local remaining = battery:capacity_remaining_pct(0)
  state:user(USER_FUEL_PCT, remaining or 0)
  state:user_len(2)

  if not swarm:set_coord_state(state) then
    gcs:send_text(3, 'swarm: coordination publish failed, is P2P_TYPE set?')
  end
end

local function report_peers()
  for i = 0, swarm:count() - 1 do
    local sysid = swarm:get_peer_sysid(i)
    -- a peer only has coordination state once it has published within the freshness budget, so nil means "not coordinating", not "not present"
    local coord = sysid > 0 and swarm:get_peer_coord_state(sysid) or nil
    if coord then
      local ready = coord:user_len() > USER_READY and coord:user(USER_READY) or 0
      gcs:send_text(6, string.format('swarm: peer %u role %u slot %u ready %u',
                                     sysid, coord:role(), coord:formation_slot(), ready))
    end
  end
end

function update()
  publish()
  report_peers()
  return update, UPDATE_MS
end

gcs:send_text(6, 'swarm: coordination example running')

return update, UPDATE_MS
