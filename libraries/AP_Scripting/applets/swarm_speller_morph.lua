-- swarm_speller_morph.lua
--
-- Decentralized glyph formation over AP_SwarmMesh: a swarm spells out a word,
-- with every drone deciding for itself which part of which letter it flies to.
--
-- Every vehicle runs this same script and carries the same 5x7 font. The leader
-- publishes the active word, its anchor, and elapsed task time in its coordination
-- basket. It never assigns anyone a position. Each speller then:
--
--   1. builds the point set for the active word locally, from the font,
--      in an order every vehicle agrees on, so slot N means the same cell everywhere
--   2. maps its persistent formation number to the same numbered slot
--   3. publishes its slot plus intended velocity and acceleration
--   4. enters a shared epoch launch wave with a bounded two agent per altitude cohort
--   5. flies a velocity command through a local CBF safety filter
--
-- No leader assigns slots: every peer independently derives the same mapping.
-- Change task_id and the whole swarm recomputes the active word.
--
-- The leader's coordination state carries the task anchor and elapsed task time.
-- Active cohorts advertise their slot and intended motion; only the eight vehicles
-- in the active wave enable their 5 Hz position streams. Every vehicle computes its
-- own command (neither the leader nor the SITL harness sends flight setpoints).
--
-- Configuration (standard SCR_USER parameters):
--   SCR_USER1 : leader mesh sysid (the vehicle whose MAV_SYSID matches leads)
--   SCR_USER2 : formation altitude above home, metres
--   SCR_USER3 : glyph cell spacing, metres
--   SCR_USER4 : leader only -- index into WORDS to spell. 0 = idle, swarm holds station
--   SCR_USER5 : persistent formation number (1..number of glyph vehicles)
--   SCR_USER6 : minimum 3D centre separation, metres
--
-- NOTE: Copter only. Vehicles are expected to be armed, at altitude and in GUIDED;
-- the script engages itself once it has climbed, so it never fights a takeoff.

local COPTER_GUIDED_MODE = 4
local ALT_FRAME_ABOVE_HOME = 1

local RUN_NAME = "swarm_speller_morph"
local FLY_MS = 100      -- 10 Hz local control; peer state still arrives at 5 Hz
local CLAIM_MS = 500    -- 2 Hz active cohort intent/rank cache refresh
local ARRIVED_M = 1.0
local ARRIVED_SPEED_MPS = 0.35

-- Horizontal guidance and robust first order CBF settings. Copter closes the inner
-- attitude/acceleration loops (this script chooses a safe NED velocity setpoint).
local MAX_SPEED_MPS = 1.5
local MAX_ACCEL_MPS2 = 2.0
local POSITION_GAIN = 0.35
local ALTITUDE_GAIN = 0.35
local MAX_VERTICAL_SPEED_MPS = 1.0
local CBF_ALPHA = 0.25
local CLIMB_SETTLE_MS = 50000
local NEIGHBOUR_RADIUS_M = 15.0
local STATE_PERIOD_S = 0.2       -- P2P_SR_POSITION=5 in the experiment
local PEER_HOLD_MS = 600
local CBF_TOLERANCE = 0.02

-- Assignment remains onboard, but bounded cohorts depart in deterministic waves so
-- the safety filter and mesh never see the whole staging cloud at once.
local CLAIM_SETTLE_MS = 1500
local REPLICATED_ASSIGNMENT = true

local ROLE_LEADER = 1
local ROLE_SPELLER = 2

local PHASE_WAITING = 0
local PHASE_TRANSIT = 1
local PHASE_AVOIDING = 2
local PHASE_OCCUPIED = 3
local PHASE_EMERGENCY = 4

-- 5x7 bitmap font. Only the glyphs the words below need.
local GLYPH_W, GLYPH_H, GLYPH_GAP = 5, 7, 1
local FONT = {
  G = {".###.", "#...#", "#....", "#.##.", "#...#", "#...#", ".###."},
  S = {".####", "#....", "#....", ".###.", "....#", "....#", "####."},
  C = {".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."},
  M = {"#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"},
  o = {".....", ".....", ".###.", "#...#", "#...#", "#...#", ".###."},
  w = {".....", ".....", "#...#", "#...#", "#.#.#", "#.#.#", ".#.#."},
  a = {".....", ".....", ".###.", "....#", ".####", "#...#", ".####"},
  r = {".....", ".....", "#.##.", "##..#", "#....", "#....", "#...."},
  m = {".....", ".....", "##.#.", "#.#.#", "#.#.#", "#.#.#", "#.#.#"},
  e = {".....", ".....", ".###.", "#...#", "#####", "#....", ".###."},
  s = {".....", ".....", ".####", "#....", ".###.", "....#", "####."},
  h = {"#....", "#....", "#.##.", "##..#", "#...#", "#...#", "#...#"},
}

-- task_id indexes this list. Keep it identical on every vehicle.
local WORDS = {"GSoC", "CoSG"}

-- Replicated task specific slot permutations for the default SITL staging cloud.
-- The GSoC assignment minimises staging travel, with one pair local swap
-- that removes the only intersecting two agent cohort path. For CoSG, cells shared
-- with GSoC stay assigned to the vehicle already occupying them and the remaining
-- cells minimise travel. Both assignments keep same cohort nominal paths separated.
-- Every vehicle carries this table; the leader does not send runtime assignments.
local GSOC_SLOT_BY_RANK = {
  26, 17, 32, 18, 24, 43, 3, 39, 37, 28, 36, 21, 31, 47,
  9, 34, 25, 22, 52, 16, 35, 38, 8, 50, 11, 23, 45, 6,
  33, 42, 13, 54, 15, 19, 44, 14, 49, 29, 5, 48, 7, 41,
  51, 4, 46, 1, 30, 56, 12, 40, 27, 2, 53, 10, 20, 55,
}

local COSG_SLOT_BY_RANK = {
  20, 30, 29, 26, 31, 40, 3, 51, 33, 37, 48, 15, 25, 44,
  17, 49, 18, 14, 52, 13, 32, 35, 19, 47, 21, 16, 42, 6,
  34, 39, 10, 54, 12, 27, 41, 11, 46, 23, 5, 45, 7, 38,
  50, 4, 43, 1, 24, 56, 9, 36, 22, 2, 53, 8, 28, 55,
}

local leader_sysid   = math.floor(param:get('SCR_USER1') or 1)
local formation_alt_m = param:get('SCR_USER2') or 20
local spacing_m      = param:get('SCR_USER3') or 4
local safe_distance_m = param:get('SCR_USER6') or 3
local cohort_state = {
  size = 2,
  cohorts_per_wave = 4,
  wave_ms = 100000,
  layer_spacing_m = 4.0,
  position_warmup_ms = 3000,
  position_ready_hold_ms = 5000,
  formation_number = math.floor(param:get('SCR_USER5') or 0),
  swarm_size = math.floor(param:get('P2P_SWARM_SIZE') or 0),
  position_hz = math.floor(param:get('P2P_SR_POSITION') or 5),
  coord_hz = math.floor(param:get('P2P_SR_COORD') or 2),
  active_count = 0,
  active_since_ms = 0,
  horizontal_ready_since_ms = 0,
  was_active = false,
  peer_rank = {},
  position_enabled = true,
  coord_enabled = true,
}
if cohort_state.position_hz <= 0 then
  cohort_state.position_hz = 5
end
if cohort_state.coord_hz <= 0 then
  cohort_state.coord_hz = 2
end
-- SCR_USER4 is read every tick, not cached. Its how the operator hands the
-- airborne swarm a new word, and the whole demo turns on that arriving at runtime

local my_sysid = math.floor(param:get('MAV_SYSID') or 0)
local is_leader = (my_sysid == leader_sysid)

local my_slot = 0            -- 0 = unclaimed
local my_rank = 0
local my_slot_since_ms = 0
local task_start_ms = 0
local descending = false
local horizontal_ready = false
local command_north = 0
local command_east = 0
local command_down = 0
local command_accel_north = 0
local command_accel_east = 0
local control_phase = PHASE_WAITING
local active_constraints = 0
local infeasible_count = 0
-- Contests are settled with this rather than a freshly measured one: a peer can only
-- compare against what we published, so if we judged ourselves on a newer number than
-- they can see, every contender would rate itself the strongest and nobody would ever yield.
local my_pub_prio = 0
local my_task = 0            -- word we are currently negotiating for
-- Last claim heard from each peer, sysid -> {slot, prio, t}. A peer's coordination
-- state goes unreadable whenever its latest basket is older than the library's
-- freshness budget, which at swarm scale is most of the time for most peers. Reading
-- the table directly would leave the claims map full of holes, every contender would
-- see its own cell as uncontested, and nobody would ever yield.
local peer_claims = {}
local CLAIM_HOLD_MS = 5000
-- Where each peer actually is, sysid -> {n, e, t} as an offset from the anchor in cells,
-- from the mesh position stream. Claims  can be lost, contradicted or left behind by a
-- vehicle that never made it; a peer sitting on a cell is ground truth.
-- Cached for the same reason claims are: a peer's position is unreadable more often than
-- it is readable once the swarm is large.
local peer_pos = {}
local POS_HOLD_MS = 5000
-- Position/velocity cache used at the 10 Hz control rate. A live mesh accessor means
-- the underlying message is within AP_SwarmMesh's freshness budget. Once it goes nil,
-- retain the last sample only briefly and grow its uncertainty with cache age.
local peer_motion = {}
local OCCUPY_CELLS = 0.6     -- how close a peer must be to count as standing on a cell
local ABANDON_CELLS = 2.5    -- a claim whose owner is this far from it is not being honoured
local slots = nil            -- cell offsets for my_task, in cells not metres
local anchor = nil           -- Location the word is centred on
local arrived = false
local last_claim_ms = 0
local last_report_ms = 0
local leader_task = 0
local leader_task_start_ms = 0

-- inactive and placed spellers silence both streams; only the bounded wave currently moving emits
-- coordination and 5 Hz position. param:set() changes the live AP_SwarmMesh stream
-- scheduler; it does not save the parameter to storage.
local function set_position_stream(enabled)
  if cohort_state.position_enabled == enabled then
    return
  end
  cohort_state.position_enabled = enabled
  param:set('P2P_SR_POSITION', enabled and cohort_state.position_hz or 0)
end

local function set_coord_stream(enabled)
  if cohort_state.coord_enabled == enabled then
    return
  end
  cohort_state.coord_enabled = enabled
  param:set('P2P_SR_COORD', enabled and cohort_state.coord_hz or 0)
end

set_position_stream(false)
if not is_leader then
  set_coord_stream(false)
end

-- Build the cell list for a word. Order is character, then row, then column, so
-- every vehicle numbers cells identically without runtime agreement.
local function build_slots(word)
  local out = {}
  local nchars = #word
  local total_w = nchars * GLYPH_W + (nchars - 1) * GLYPH_GAP
  local x0 = -(total_w - 1) * 0.5      -- centre the word on the anchor
  local y0 = (GLYPH_H - 1) * 0.5
  for ci = 1, nchars do
    local bmp = FONT[word:sub(ci, ci)]
    if bmp then
      local cx = (ci - 1) * (GLYPH_W + GLYPH_GAP)
      for row = 1, GLYPH_H do
        for col = 1, GLYPH_W do
          if bmp[row]:sub(col, col) == "#" then
            -- north increases up the glyph so the word reads correctly from above
            out[#out + 1] = { east = x0 + cx + (col - 1), north = y0 - (row - 1) }
          end
        end
      end
    end
  end
  return out
end

-- claim strength: closer is stronger. 1..255, saturating a metre at a time.
local function priority_for(dist_m)
  local p = 255 - math.floor(dist_m)
  if p < 1 then p = 1 end
  if p > 255 then p = 255 end
  return p
end

-- is claim A better than claim B? closer wins, lower sysid breaks the tie
local function better(prio_a, sysid_a, prio_b, sysid_b)
  if prio_a ~= prio_b then
    return prio_a > prio_b
  end
  return sysid_a < sysid_b
end

local function slot_location(slot)
  local loc = anchor:copy()
  loc:offset(slots[slot].north * spacing_m, slots[slot].east * spacing_m)
  loc:set_alt_m(formation_alt_m, ALT_FRAME_ABOVE_HOME)
  return loc
end

-- publish what we are doing, so peers can negotiate against it and a companion
-- computer or GCS can watch the swarm organise itself
local function signed_cm(value)
  if value >= 0 then
    return math.floor(value * 100 + 0.5)
  end
  return math.ceil(value * 100 - 0.5)
end

local function publish(role, task, slot, prio, task_elapsed_ms)
  local state = SwarmCoordState()
  state:role(role)
  state:task_id(task)
  state:formation_slot(slot)
  state:priority(prio)
  if slot > 0 and slots ~= nil and anchor ~= nil then
    local loc = slot_location(slot)
    state:target_lat(loc:lat())
    state:target_lng(loc:lng())
    state:target_alt_mm(loc:alt() * 10)     -- Location alt is cm, the basket is mm
  elseif role == ROLE_LEADER then
    local here = ahrs:get_location()
    if here then
      state:target_lat(here:lat())
      state:target_lng(here:lng())
      state:target_alt_mm(here:alt() * 10)
    end
  end
  state:target_vel_NED(0, signed_cm(command_north))
  state:target_vel_NED(1, signed_cm(command_east))
  state:target_vel_NED(2, signed_cm(command_down))
  state:target_accel_NED(0, signed_cm(command_accel_north))
  state:target_accel_NED(1, signed_cm(command_accel_east))
  if role == ROLE_LEADER then
    local elapsed = math.max(0, task_elapsed_ms or 0)
    state:user(0, elapsed % 256)
    state:user(1, math.floor(elapsed / 256) % 256)
    state:user(2, math.floor(elapsed / 65536) % 256)
    state:user(3, math.floor(elapsed / 16777216) % 256)
  else
    state:user(0, arrived and 1 or 0)       -- lets the leader count the word as formed
    state:user(1, control_phase)
    state:user(2, math.min(active_constraints, 255))
    state:user(3, horizontal_ready and 1 or 0)
  end
  state:user_len(4)
  swarm:set_coord_state(state)
end

-- the leader is the only vehicle that says what the swarm is doing
local function run_leader(now)
  local leader_word = math.floor(param:get('SCR_USER4') or 0)
  if leader_word ~= leader_task then
    leader_task = leader_word
    leader_task_start_ms = now
  end
  publish(ROLE_LEADER, leader_word, 0, 255, now - leader_task_start_ms)

  if now - last_report_ms > 2000 then
    last_report_ms = now
    if leader_word > 0 and WORDS[leader_word] then
      local want = #build_slots(WORDS[leader_word])
      local placed = 0
      for i = 0, swarm:count() - 1 do
        local sid = swarm:get_peer_sysid(i)
        if sid > 0 then
          local cs = swarm:get_peer_coord_state(sid)
          if cs and cs:task_id() == leader_word and cs:formation_slot() > 0
             and cs:user_len() > 0 and cs:user(0) == 1 then
            placed = placed + 1
          end
        end
      end
      gcs:send_text(6, string.format("%s: '%s' %d/%d in position",
                                     RUN_NAME, WORDS[leader_word], placed, want))
    end
  end
end

-- pull the active task and anchor out of the leader's basket.
-- The leader is one node competing for mesh bandwidth with everyone else, so its basket
-- is regularly older than the freshness budget. Bailing out whenever that happens would
-- stop us commanding a target and leave us parked wherever we were, sometimes on top of
-- another vehicle's cell, so the task and anchor are held across the gaps.
local function refresh_task(now)
  local cs = swarm:get_peer_coord_state(leader_sysid)
  if cs == nil or cs:role() ~= ROLE_LEADER then
    -- A task is a discrete command, not a continuously valid setpoint. Keep the
    -- last valid command through coordination loss while peer motion remains fresh.
    return anchor ~= nil and my_task > 0
  end
  local task = cs:task_id()
  if task == 0 or WORDS[task] == nil then
    return false
  end
  local leader_elapsed_ms = 0
  if cs:user_len() > 3 then
    leader_elapsed_ms = cs:user(0) + cs:user(1) * 256
                        + cs:user(2) * 65536 + cs:user(3) * 16777216
  end
  if task ~= my_task then
    -- the word changed: drop our claim and renegotiate from scratch
    my_task = task
    slots = build_slots(WORDS[task])
    my_slot = 0
    my_slot_since_ms = now
    task_start_ms = now - leader_elapsed_ms
    descending = false
    horizontal_ready = false
    peer_claims = {}
    peer_motion = {}
    cohort_state.peer_rank = {}
    cohort_state.active_since_ms = 0
    cohort_state.horizontal_ready_since_ms = 0
    cohort_state.was_active = false
    set_position_stream(false)
    arrived = false
    command_north, command_east, command_down = 0, 0, 0
    control_phase = PHASE_WAITING
    gcs:send_text(6, string.format("%s: task %d '%s', %d cells",
                                   RUN_NAME, task, WORDS[task], #slots))
  end
  -- Every fresh leader packet carries elapsed task time. Correct the local epoch
  -- continuously so staggered boot clocks and a late first packet cannot put
  -- different vehicles into different launch waves.
  task_start_ms = now - leader_elapsed_ms
  anchor = Location()
  anchor:lat(cs:target_lat())
  anchor:lng(cs:target_lng())
  anchor:set_alt_m(formation_alt_m, ALT_FRAME_ABOVE_HOME)
  return true
end

-- one round of negotiation: read peers claims, defend or take a slot
local function negotiate(now)
  if REPLICATED_ASSIGNMENT then
    -- Formation numbers are persistent identities supplied by the coordination
    -- configuration, not live assignments from the leader.
    for i = 0, swarm:count() - 1 do
      local sid = swarm:get_peer_sysid(i)
      if sid > 0 and sid ~= leader_sysid and sid ~= my_sysid then
        local cs = swarm:get_peer_coord_state(sid)
        if cs and cs:task_id() == my_task then
          -- In the morph controller formation_slot is the task specific target.
          -- priority carries the peer's persistent formation rank so cohort
          -- membership remains unchanged when the target permutation changes.
          local peer_rank = cs:priority()
          if peer_rank > 0 then
            cohort_state.peer_rank[sid] = peer_rank
          end
          peer_claims[sid] = {
            slot = peer_rank,
            prio = cs:priority(),
            vn = cs:target_vel_NED(0) * 0.01,
            ve = cs:target_vel_NED(1) * 0.01,
            an = cs:target_accel_NED(0) * 0.01,
            ae = cs:target_accel_NED(1) * 0.01,
            t = now,
          }
        end
      end
    end
    cohort_state.active_count = math.min(math.max(0, cohort_state.swarm_size - 1), #slots)
    my_rank = cohort_state.formation_number
    local assigned = 0
    if my_rank > 0 and my_rank <= cohort_state.active_count then
      if my_task == 1 then
        assigned = GSOC_SLOT_BY_RANK[my_rank] or 0
      elseif my_task == 2 then
        assigned = COSG_SLOT_BY_RANK[my_rank] or 0
      end
    end
    if assigned ~= my_slot then
      my_slot = assigned
      my_slot_since_ms = now
      descending = false
      horizontal_ready = false
      arrived = false
    end
    -- Replicated assignment does not run a claim contest. Publish the stable rank
    -- here so peers can reconstruct two agent cohorts independently of target slot.
    my_pub_prio = my_rank
    publish(ROLE_SPELLER, my_task, my_slot, my_pub_prio)
    return
  end

  local here = ahrs:get_location()
  if here == nil then
    return
  end
  local old_slot = my_slot
  -- our own position as a north/east offset from the anchor, in cells, so slot
  -- distances can be computed.
  local ne = anchor:get_distance_NE(here)
  local my_n, my_e = ne:x() / spacing_m, ne:y() / spacing_m

  local function dist_to(slot)
    local dn = slots[slot].north - my_n
    local de = slots[slot].east - my_e
    return math.sqrt(dn * dn + de * de) * spacing_m
  end

  -- refresh both caches from whichever peers are readable this round
  for i = 0, swarm:count() - 1 do
    local sid = swarm:get_peer_sysid(i)
    if sid > 0 and sid ~= leader_sysid and sid ~= my_sysid then
      local cs = swarm:get_peer_coord_state(sid)
      if cs and cs:task_id() == my_task then
        peer_claims[sid] = {
          slot = cs:formation_slot(),
          prio = cs:priority(),
          vn = cs:target_vel_NED(0) * 0.01,
          ve = cs:target_vel_NED(1) * 0.01,
          an = cs:target_accel_NED(0) * 0.01,
          ae = cs:target_accel_NED(1) * 0.01,
          t = now,
        }
      end
      local loc = swarm:get_peer_location(sid)
      if loc then
        local pne = anchor:get_distance_NE(loc)
        peer_pos[sid] = { n = pne:x() / spacing_m, e = pne:y() / spacing_m, t = now }
      end
    end
  end

  -- strongest claim we have heard for each slot, from the cache rather than only
  -- from the peers who happen to be readable right now
  local claims = {}
  for sid, c in pairs(peer_claims) do
    if now - c.t <= CLAIM_HOLD_MS and c.slot > 0 and c.slot <= #slots then
      local held = claims[c.slot]
      if held == nil or better(c.prio, sid, held.prio, held.sysid) then
        claims[c.slot] = { prio = c.prio, sysid = sid }
      end
    end
  end

  -- Staging is outside the glyph, so once a peer physically reaches a cell its
  -- position is an authoritative claim even if its coordination packet was lost.
  -- This also breaks up vehicles that arrived together during a stale claim window.
  for sid, p in pairs(peer_pos) do
    if now - p.t <= POS_HOLD_MS then
      local nearest, nearest_dist = 0, nil
      for s = 1, #slots do
        local dn, de = slots[s].north - p.n, slots[s].east - p.e
        local distance = math.sqrt(dn * dn + de * de)
        if nearest_dist == nil or distance < nearest_dist then
          nearest, nearest_dist = s, distance
        end
      end
      if nearest > 0 and nearest_dist <= OCCUPY_CELLS then
        local held = claims[nearest]
        if held == nil or sid < held.sysid then
          claims[nearest] = { prio = 255, sysid = sid }
        end
      end
    end
  end

  -- How far the peer holding each claim actually is from the cell it asked for.
  local claim_dist = {}
  for s, c in pairs(claims) do
    local p = peer_pos[c.sysid]
    if p ~= nil and now - p.t <= POS_HOLD_MS then
      local dn, de = slots[s].north - p.n, slots[s].east - p.e
      claim_dist[s] = math.sqrt(dn * dn + de * de)
    end
  end

  -- defend the slot we hold, if someone has a strictly better claim on it we yield.
  -- judged on the strength we published, which is the same number our rival is judging
  -- us on, so exactly one side of any contest concludes it has to give way
  if my_slot > 0 then
    local rival = claims[my_slot]
    if rival ~= nil and better(rival.prio, rival.sysid, my_pub_prio, my_sysid) then
      my_slot = 0
      arrived = false
    end
  end

  -- Someone else is already standing on the cell we asked for and we are not: they got
  -- there first, so we leave rather than descend on their head. This settles contests the
  -- priority arithmetic cannot, including a rival whose claim never reached us.
  if my_slot > 0 then
    local rival = claims[my_slot]
    local rival_d = claim_dist[my_slot]
    if rival ~= nil and rival.sysid ~= my_sysid
       and rival_d ~= nil and rival_d <= OCCUPY_CELLS
       and dist_to(my_slot) > OCCUPY_CELLS * spacing_m then
      my_slot = 0
      arrived = false
    end
  end

  -- take the nearest cell nobody is holding.
  -- Ranked with a small deterministic vehicle offset, not on distance alone: two
  -- vehicles that have converged on the same point see an identical claims map and
  -- identical distances, so on pure distance they would pick the same cell, collide,
  -- and keep picking together forever. The offset is unique per (sysid, cell) and
  -- stays well under one cell, so it only reorders candidates that were near equal.
  if my_slot == 0 then
    local function pick(take_abandoned)
      local best, best_score = 0, nil
      for s = 1, #slots do
        local free
        if not take_abandoned then
          free = claims[s] == nil
        else
          -- claimed, but its owner is nowhere near it and we do know where they are
          free = claims[s] ~= nil and claim_dist[s] ~= nil and claim_dist[s] > ABANDON_CELLS
        end
        if free then
          local score = dist_to(s) + ((my_sysid * 37 + s * 17) % 16) * (spacing_m * 0.015)
          if best_score == nil or score < best_score then
            best, best_score = s, score
          end
        end
      end
      return best
    end

    my_slot = pick(false)
    if my_slot == 0 then
      -- Every cell is spoken for. Rather than hover uselessly on top of somebody, take
      -- one whose owner asked for it and then never flew there: a claim nobody honoured
      -- is exactly the hole that leaves the word unfinished. An owner still on its way
      -- is closer than we are, so it outranks us and takes it straight back.
      my_slot = pick(true)
    end
  end

  if my_slot ~= old_slot then
    my_slot_since_ms = now
    descending = false
    horizontal_ready = false
    arrived = false
  end
  my_pub_prio = my_slot > 0 and priority_for(dist_to(my_slot)) or 1
  publish(ROLE_SPELLER, my_task, my_slot, my_pub_prio)
end

-- engage off our own altitude rather than an external flag, which is silently lost
-- for some vehicles when a large fleet is commanded at once
local function has_climbed()
  local rp = ahrs:get_relative_position_NED_home()
  return rp ~= nil and (-rp:z()) >= formation_alt_m * 0.8
end

local function clamp(value, low, high)
  return math.max(low, math.min(high, value))
end

local function limit_horizontal(north, east, limit)
  local length = math.sqrt(north * north + east * east)
  if length > limit and length > 0 then
    local scale = limit / length
    return north * scale, east * scale
  end
  return north, east
end

-- Refresh at the local control rate. get_peer_location() itself enforces the mesh
-- freshness budget; PEER_HOLD_MS only bridges the short nil gaps between packets.
local function refresh_peer_motion(now)
  local my_cohort = math.floor((math.max(1, my_rank) - 1) / cohort_state.size)
  for sid, peer_rank in pairs(cohort_state.peer_rank) do
    if sid ~= my_sysid and math.floor((peer_rank - 1) / cohort_state.size) == my_cohort then
      local loc = swarm:get_peer_location(sid)
      if loc then
        local vel = swarm:get_peer_velocity_NED(sid)
        local claim = peer_claims[sid]
        local old = peer_motion[sid]
        local vn = vel and vel:x() or (claim and claim.vn or (old and old.vn or 0))
        local ve = vel and vel:y() or (claim and claim.ve or (old and old.ve or 0))
        local vd = vel and vel:z() or (old and old.vd or 0)
        local an = claim and claim.an or (old and old.an or 0)
        local ae = claim and claim.ae or (old and old.ae or 0)
        local sample_stamp = swarm:get_peer_position_last_update_ms(sid)
        local sample_t = sample_stamp and sample_stamp:toint() or 0
        peer_motion[sid] = {
          loc = loc,
          vn = vn,
          ve = ve,
          vd = vd,
          an = an,
          ae = ae,
          t = now,
          sample_t = sample_t > 0 and sample_t or (old and old.sample_t or now),
        }
      end
    end
  end
  for sid, peer in pairs(peer_motion) do
    if now - peer.t > PEER_HOLD_MS then
      peer_motion[sid] = nil
    end
  end
end

local function build_cbf_constraints(here, own_vel, now)
  local constraints = {}
  for sid, peer in pairs(peer_motion) do
    local ne = peer.loc:get_distance_NE(here)   -- peer -> us = p_i - p_j
    if ne then
      local rn, re = ne:x(), ne:y()
      local rd = (peer.loc:alt() - here:alt()) * 0.01
      local distance_sq = rn * rn + re * re + rd * rd
      local distance = math.sqrt(distance_sq)
      if distance <= 0.05 then
        -- h has no useful gradient at exact colocation. Give the pair a stable,
        -- antisymmetric virtual bearing so the emergency controller can separate it.
        local low, high = math.min(my_sysid, sid), math.max(my_sysid, sid)
        local angle = ((low * 37 + high * 17) % 360) * math.pi / 180
        local sign = my_sysid == low and 1 or -1
        rn, re = math.cos(angle) * 0.05 * sign, math.sin(angle) * 0.05 * sign
        distance_sq, distance = 0.05 * 0.05, 0.05
      end
      if distance >= 0.05 then
        local sample_age_s = math.max(0, now - peer.sample_t) * 0.001
        -- Propagate the last measured position at its measured velocity. The
        -- remaining uncertainty is one stream period plus bounded acceleration.
        rn = rn - peer.vn * sample_age_s
        re = re - peer.ve * sample_age_s
        rd = rd - peer.vd * sample_age_s
        distance_sq = rn * rn + re * re + rd * rd
        distance = math.sqrt(distance_sq)
        local uncertainty_age_s = STATE_PERIOD_S + sample_age_s
        local guard = safe_distance_m + MAX_SPEED_MPS * STATE_PERIOD_S
                      + 0.5 * MAX_ACCEL_MPS2 * uncertainty_age_s * uncertainty_age_s
        if distance <= NEIGHBOUR_RADIUS_M + guard then
          local h = distance_sq - guard * guard
          constraints[#constraints + 1] = {
            an = 2 * rn,
            ae = 2 * re,
            b = 2 * (rn * peer.vn + re * peer.ve + rd * peer.vd)
                - 2 * rd * own_vel:z() - CBF_ALPHA * h,
            rn = rn,
            re = re,
            distance = distance,
            guard = guard,
            sysid = sid,
          }
        end
      end
    end
  end
  return constraints
end

local function velocity_feasible(north, east, constraints)
  if north * north + east * east > MAX_SPEED_MPS * MAX_SPEED_MPS + CBF_TOLERANCE then
    return false
  end
  for _, c in ipairs(constraints) do
    if c.an * north + c.ae * east < c.b - CBF_TOLERANCE then
      return false
    end
  end
  return true
end

-- The QP is only two dimensional, so enumerate every boundary feature on which its
-- optimum can lie: the nominal point, a single barrier projection, two intersecting
-- barriers, or a barrier intersecting the speed circle. This is exact for the
-- convex feasible set and avoids false infeasibility from alternating projections.
local function filter_velocity(north, east, constraints)
  local nominal_north, nominal_east = north, east
  local best_north, best_east, best_cost = 0, 0, nil

  local function consider(candidate_north, candidate_east)
    if velocity_feasible(candidate_north, candidate_east, constraints) then
      local dn = candidate_north - nominal_north
      local de = candidate_east - nominal_east
      local cost = dn * dn + de * de
      if best_cost == nil or cost < best_cost then
        best_north, best_east, best_cost = candidate_north, candidate_east, cost
      end
    end
  end

  north, east = limit_horizontal(north, east, MAX_SPEED_MPS)
  consider(north, east)
  if math.abs(nominal_north) + math.abs(nominal_east) > 0 then
    local radial_north, radial_east = limit_horizontal(
      nominal_north, nominal_east, MAX_SPEED_MPS)
    consider(radial_north, radial_east)
  end

  for first_index, first in ipairs(constraints) do
    local first_norm_sq = first.an * first.an + first.ae * first.ae
    if first_norm_sq > 0 then
      local projection = (first.b - first.an * nominal_north
                          - first.ae * nominal_east) / first_norm_sq
      consider(nominal_north + projection * first.an,
               nominal_east + projection * first.ae)

      local foot_north = first.an * first.b / first_norm_sq
      local foot_east = first.ae * first.b / first_norm_sq
      local tangent_sq = MAX_SPEED_MPS * MAX_SPEED_MPS
                         - (foot_north * foot_north + foot_east * foot_east)
      if tangent_sq >= -CBF_TOLERANCE then
        local tangent_scale = math.sqrt(math.max(0, tangent_sq) / first_norm_sq)
        local tangent_north, tangent_east = -first.ae * tangent_scale, first.an * tangent_scale
        consider(foot_north + tangent_north, foot_east + tangent_east)
        consider(foot_north - tangent_north, foot_east - tangent_east)
      end
    end

    for second_index = first_index + 1, #constraints do
      local second = constraints[second_index]
      local determinant = first.an * second.ae - first.ae * second.an
      if math.abs(determinant) > 1.0e-6 then
        consider((first.b * second.ae - first.ae * second.b) / determinant,
                 (first.an * second.b - first.b * second.an) / determinant)
      end
    end
  end

  if best_cost == nil then
    return north, east, false, true
  end
  local adjusted = best_cost > CBF_TOLERANCE * CBF_TOLERANCE
  return best_north, best_east, true, adjusted
end

local function emergency_velocity(constraints)
  local north, east = 0, 0
  for _, c in ipairs(constraints) do
    local urgency = math.max(0, c.guard + 1.0 - c.distance)
    if urgency > 0 and c.distance > 0 then
      north = north + c.rn * urgency / c.distance
      east = east + c.re * urgency / c.distance
    end
  end
  -- Rotating each peer relative vector the same way gives reciprocal agents opposite
  -- global sidesteps and breaks a head on standstill.
  if math.abs(north) + math.abs(east) < 0.01 and #constraints > 0 then
    local nearest = constraints[1]
    for _, c in ipairs(constraints) do
      if c.distance < nearest.distance then
        nearest = c
      end
    end
    north, east = -nearest.re, nearest.rn
  end
  return limit_horizontal(north, east, MAX_SPEED_MPS)
end

local last_emergency_report_ms = 0
local last_debug_ms = 0

local function fly_safe(now)
  local here = ahrs:get_location()
  local own_vel = ahrs:get_velocity_NED()
  local rp = ahrs:get_relative_position_NED_home()
  if here == nil or own_vel == nil or rp == nil then
    return
  end

  local my_cohort = math.floor((math.max(1, my_rank) - 1) / cohort_state.size)
  local my_wave = math.floor(my_cohort / cohort_state.cohorts_per_wave)
  local elapsed_ms = now - task_start_ms
  local cohort_active = my_slot > 0
                        and elapsed_ms >= CLIMB_SETTLE_MS + my_wave * cohort_state.wave_ms
  if cohort_active and not cohort_state.was_active then
    cohort_state.active_since_ms = now
  end
  cohort_state.was_active = cohort_active
  local position_hold_done = horizontal_ready
                             and now - cohort_state.horizontal_ready_since_ms
                                 >= cohort_state.position_ready_hold_ms
  local motion_streams_enabled = cohort_active and not position_hold_done
  set_position_stream(motion_streams_enabled)
  set_coord_stream(motion_streams_enabled)

  refresh_peer_motion(now)
  local constraints = build_cbf_constraints(here, own_vel, now)
  active_constraints = #constraints

  local nominal_north, nominal_east = 0, 0
  local ready_to_depart = false
  local target = nil
  local horizontal_distance = math.huge
  local transit_alt_m = formation_alt_m
  if my_slot > 0 then
    target = slot_location(my_slot)
    local target_error = here:get_distance_NE(target)
    if target_error then
      horizontal_distance = math.sqrt(target_error:x() * target_error:x()
                                      + target_error:y() * target_error:y())
    end
    if horizontal_distance <= ARRIVED_M and not horizontal_ready then
      horizontal_ready = true
      cohort_state.horizontal_ready_since_ms = now
    end
    local cohort_count = math.ceil(cohort_state.active_count / cohort_state.size)
    local wave_count = math.ceil(cohort_count / cohort_state.cohorts_per_wave)
    descending = horizontal_ready
                 and elapsed_ms >= CLIMB_SETTLE_MS + wave_count * cohort_state.wave_ms
    transit_alt_m = formation_alt_m + my_cohort * cohort_state.layer_spacing_m
    local at_transit_alt = math.abs((-rp:z()) - transit_alt_m) <= 0.75
    ready_to_depart = now - my_slot_since_ms >= CLAIM_SETTLE_MS
                      and now - task_start_ms >= CLIMB_SETTLE_MS
                      and cohort_active
                      and now - cohort_state.active_since_ms >= cohort_state.position_warmup_ms
                      and (at_transit_alt or descending)
    if ready_to_depart then
      local error = here:get_distance_NE(target)
      if error then
        nominal_north = error:x() * POSITION_GAIN
        nominal_east = error:y() * POSITION_GAIN
      end
    end
  end
  nominal_north, nominal_east = limit_horizontal(nominal_north, nominal_east, MAX_SPEED_MPS)

  local north, east, feasible, adjusted = filter_velocity(
    nominal_north, nominal_east, constraints)
  if not feasible then
    north, east = emergency_velocity(constraints)
    control_phase = PHASE_EMERGENCY
    infeasible_count = infeasible_count + 1
    if now - last_emergency_report_ms >= 2000 then
      last_emergency_report_ms = now
      gcs:send_text(4, string.format("%s: CBF infeasible #%d (%d peers)",
                                     RUN_NAME, infeasible_count, #constraints))
    end
  elseif adjusted then
    control_phase = PHASE_AVOIDING
  elseif not ready_to_depart then
    control_phase = PHASE_WAITING
  else
    control_phase = PHASE_TRANSIT
  end

  local target_down = -(descending and formation_alt_m or transit_alt_m)
  local down = clamp((target_down - rp:z()) * ALTITUDE_GAIN,
                     -MAX_VERTICAL_SPEED_MPS, MAX_VERTICAL_SPEED_MPS)
  local dt = FLY_MS * 0.001
  north, east = limit_horizontal(north, east, MAX_SPEED_MPS)
  local accel_north, accel_east = limit_horizontal(
    (north - command_north) / dt, (east - command_east) / dt, MAX_ACCEL_MPS2)
  command_north, command_east, command_down = north, east, down
  command_accel_north, command_accel_east = accel_north, accel_east

  local distance = target and here:get_distance(target) or math.huge
  arrived = my_slot > 0 and distance <= ARRIVED_M
            and math.sqrt(own_vel:x() * own_vel:x() + own_vel:y() * own_vel:y()) <= ARRIVED_SPEED_MPS
  if arrived and control_phase == PHASE_TRANSIT then
    control_phase = PHASE_OCCUPIED
  end

  local command = Vector3f()
  command:x(command_north)
  command:y(command_east)
  command:z(command_down)
  vehicle:set_target_velocity_NED(command)
  publish(ROLE_SPELLER, my_task, my_slot, my_pub_prio)
  if now - last_debug_ms >= 1000 then
    last_debug_ms = now
    gcs:send_text(7, string.format("SFDBG slot=%d coh=%d tx=%d ph=%d cbf=%d inf=%d peers=%d",
                                   my_slot, my_cohort, cohort_state.position_enabled and 1 or 0,
                                   control_phase, active_constraints, infeasible_count,
                                   swarm:count()))
  end
end

function update()
  local now = millis():toint()

  if is_leader then
    run_leader(now)
    return update, FLY_MS
  end

  if not arming:is_armed() or not has_climbed() then
    set_position_stream(false)
    if not is_leader then
      set_coord_stream(false)
    end
    return update, FLY_MS
  end
  if not refresh_task(now) then
    set_position_stream(false)
    set_coord_stream(false)
    return update, FLY_MS
  end

  if now - last_claim_ms >= CLAIM_MS then
    last_claim_ms = now
    negotiate(now)
  end

  if vehicle:get_mode() == COPTER_GUIDED_MODE then
    fly_safe(now)
  end

  return update, FLY_MS
end

gcs:send_text(6, string.format("%s loaded: sysid=%d %s leader=%d alt=%.0f spacing=%.1f",
                               RUN_NAME, my_sysid, is_leader and "LEADER" or "speller",
                               leader_sysid, formation_alt_m, spacing_m))

return update, FLY_MS
