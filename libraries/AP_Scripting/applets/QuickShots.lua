-- QuickShots
-- Dronie, Rocket, Helix
--
-- triggered by MAVLink MAV_CMD_USER_1 (31010)
-- param1 QuickShot(1-Dronie, 2-Rocket, 3-Helix)
-- param2 max distance
-- param3 max altitude
-- param4 ROI distance
--
-- QGC MavlinkActions
-- ~\Documents\QGroundControl\MavlinkActions\FlyViewMavlinkActions.json

local maxDistance = 50
local maxAltitude = 30
local roiDistance = 8
local startAltitude = 4
local sdDistance = 5
local startWP
local roiWP
local bearing
local gcs_location = Location()
local stage = 0
local cmdQuickShot

local mavlink_msgs = require("MAVLink/mavlink_msgs")

local COMMAND_ACK_ID = mavlink_msgs.get_msgid("COMMAND_ACK")
local COMMAND_LONG_ID = mavlink_msgs.get_msgid("COMMAND_LONG")
local GLOBAL_POSITION_INT_ID = mavlink_msgs.get_msgid("GLOBAL_POSITION_INT")

local msg_map = {}
msg_map[COMMAND_ACK_ID] = "COMMAND_ACK"
msg_map[COMMAND_LONG_ID] = "COMMAND_LONG"
msg_map[GLOBAL_POSITION_INT_ID] = "GLOBAL_POSITION_INT"

-- initialize MAVLink rx with buffer depth and number of messages
mavlink:init(2, 2)

-- register message id to receive
mavlink:register_rx_msgid(COMMAND_LONG_ID)
mavlink:register_rx_msgid(GLOBAL_POSITION_INT_ID)

local MAV_COMP_ID_MISSIONPLANNER = 190
local MAV_CMD_USER_1_ID = 31010

-- Block AP parsing MAV_CMD_USER_1 so we can deal with it in the script
-- Prevents "unsupported" ack
mavlink:block_command(MAV_CMD_USER_1_ID)

function handle_command_long(cmd)
	--gcs:send_text(6, string.format("COMMAND_LONG <%d/%d> %d", cmd.sysid, cmd.target_component, cmd.command) )

	local ret = 0
	
	if cmd.command == MAV_CMD_USER_1_ID then
		gcs:send_text(6, string.format("MAV_CMD_USER_1_ID %d %d %d %d", cmd.param1, cmd.param2, cmd.param3, cmd.param4) )
		if cmd.param1 < 1 or cmd.param1 > 3 then
			gcs:send_text(0, string.format("Quick Shot %d not implemented", cmd.param1))
			ret = 4
		end
		if stage ~= 0 then
			gcs:send_text(0, "still running")
			ret = 4
		end
		if arming:is_armed() then
			gcs:send_text(0, "disarm first")
			ret = 4
		end
		if not ahrs:home_is_set() then
			gcs:send_text(0, "waiting for gps")
			ret = 4
		end
		if ret == 0 then
			cmdQuickShot = cmd.param1
			maxDistance = cmd.param2
			maxAltitude = cmd.param3
			roiDistance = cmd.param4
			stage = 1
		end
	end
	
	return ret
end

function handle_global_position_int(msg)
	--gcs:send_text(6, string.format("position<%d.%d> %f %f %d", msg.sysid, msg.compid, msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000) )
	if msg.compid == MAV_COMP_ID_MISSIONPLANNER then
		gcs_location:lat(msg.lat)
		gcs_location:lng(msg.lon)
		gcs_location:alt(msg.relative_alt/10)
		gcs_location:relative_alt(true)
	end
end

function calcWps()
	startWp = ahrs:get_position()
	if not startWp then
		gcs:send_text(6, "ahrs:get_position() null")
	end

	--gcs:send_text(6, string.format("gcs_location %f %f %d", gcs_location:lat()/1e7, gcs_location:lng()/1e7, gcs_location:alt()/1000) )
	if gcs_location:lat() ~= 0 then -- got gcs location
		gcs:send_text(6, "got gcs location")
		roiWP = gcs_location:copy()
		bearing = math.deg(roiWP:get_bearing(startWp))
	else
		gcs:send_text(6, "no gcs location, estimate roi")
		bearing = 180 + math.deg(ahrs:get_yaw())
		roiWP = startWp:copy()
		roiWP:offset_bearing(bearing, -roiDistance)
	end
	roiWP:alt(100)
	
	gcs:send_text(6, "waypoints calculated")
	stage = 2
end

function createRocket()
	local sdAltitude = 6

	local rocketWP = startWp:copy()
	rocketWP:alt(100*maxAltitude)
	local sldWP = startWp:copy()
	sldWP:alt(100*sdAltitude)

	local FRAME_GLOBAL_RELATIVE_ALT = 3
	
	mission:clear()
	
	--WP0 is always home
	local homeWP = ahrs:get_home()
	local home = mavlink_mission_item_int_t()
	home:seq(0)
	home:frame(FRAME_GLOBAL_RELATIVE_ALT)
	home:command(16) -- WP
	home:x(homeWP:lat())
	home:y(homeWP:lng())
	home:z(homeWP:alt() / 100)
	mission:set_item(0, home)

	-- Takeoff
	local to = mavlink_mission_item_int_t()
	to:seq(1)
	to:frame(FRAME_GLOBAL_RELATIVE_ALT)
	to:command(22) -- TAKEOFF
	to:z(startAltitude)
	mission:set_item(1, to)
	
	-- ROI
	local wp = mavlink_mission_item_int_t()
	wp:seq(2)
	wp:frame(FRAME_GLOBAL_RELATIVE_ALT)
	wp:command(201) -- SET_ROI
	wp:x(roiWP:lat())
	wp:y(roiWP:lng())
	wp:z(roiWP:alt() / 100)
	mission:set_item(2, wp)

	-- Rocket
	wp:seq(3)
	wp:command(16) -- WP
	wp:x(0)
	wp:y(0)
	wp:z(rocketWP:alt() / 100)
	mission:set_item(3, wp)

	-- Slow down
	wp:seq(4)
	wp:frame(FRAME_GLOBAL_RELATIVE_ALT)
	wp:z(sldWP:alt() / 100)
	mission:set_item(4, wp)

	-- Change Vertical Speed to 1m/s
	local cs = mavlink_mission_item_int_t()
	cs:seq(5)
	cs:frame(FRAME_GLOBAL_RELATIVE_ALT)
	cs:command(178) -- CHANGE_SPEED
	cs:param1(3) -- down
	cs:param2(1)
	mission:set_item(5, cs)
	
	-- start
	wp:seq(6)
	wp:z(startAltitude)
	mission:set_item(6, wp)

	-- land
	local land = mavlink_mission_item_int_t()
	land:seq(7)
	land:frame(FRAME_GLOBAL_RELATIVE_ALT)
	land:command(21) -- LAND
	mission:set_item(7, land)
	
	gcs:send_text(0, "Rocket mission written")
end

function createHelix()
	local homeWP = ahrs:get_home()
	
	wp = startWp:copy()
	wp:alt(100*startAltitude)

	local FRAME_GLOBAL_RELATIVE_ALT = 3
	local itemIdx = 0
	
	mission:clear()
	
	--WP0 is always home
	local m = mavlink_mission_item_int_t()
	m:seq(itemIdx)
	m:frame(FRAME_GLOBAL_RELATIVE_ALT)
	m:command(16) -- WP
	m:x(homeWP:lat())
	m:y(homeWP:lng())
	m:z(homeWP:alt() / 100)
	mission:set_item(itemIdx, m)
	itemIdx = itemIdx + 1

	-- Takeoff
	m:seq(itemIdx)
	m:command(22) -- TAKEOFF
	m:z(startAltitude)
	mission:set_item(itemIdx, m)
	itemIdx = itemIdx + 1
	
	-- ROI
	m:seq(itemIdx)
	m:command(201) -- SET_ROI
	m:x(roiWP:lat())
	m:y(roiWP:lng())
	m:z(roiWP:alt() / 100)
	mission:set_item(itemIdx, m)
	itemIdx = itemIdx + 1

	local startRadius = roiDistance
	local radius = startRadius
	local alt = startAltitude
	local wp_per_cycle = 20
	local radius_inc = (maxDistance - startRadius) / (wp_per_cycle - 1)
	local alt_inc = (maxAltitude - startAltitude) / (wp_per_cycle - 1)

	while true do
		bearing = bearing + 360 / (wp_per_cycle-1)
		if bearing > 360 then
			bearing = bearing - 360
		end
		radius = radius + radius_inc
		alt = alt + alt_inc
		wp = roiWP:copy()
		wp:offset_bearing(bearing, radius)
		local distance = wp:get_distance(roiWP)
		if (alt-maxAltitude) > 0.10 or (distance-maxDistance) > 0.10 then
			break
		end
		m:seq(itemIdx)
		m:command(16) -- 16 WP
		m:x(wp:lat())
		m:y(wp:lng())
		m:z(alt)
		mission:set_item(itemIdx, m)
		itemIdx = itemIdx + 1
	end

	-- start
	m:seq(itemIdx)
	m:command(16) -- WP
	local lat = startWp:lat()
	local lng = startWp:lng()
	m:x(lat)
	m:y(lng)
	m:z(startAltitude)
	mission:set_item(itemIdx, m)
	itemIdx = itemIdx + 1

	-- land
	m:seq(itemIdx)
	m:command(21) -- LAND
	m:x(0)
	m:y(0)
	m:z(0)
	mission:set_item(itemIdx, m)
	
	gcs:send_text(0, "Helix mission written")
end

function createDronie()
	local FRAME_GLOBAL_RELATIVE_ALT = 3
	
	local dronieWP = startWp:copy()
	dronieWP:offset_bearing(bearing, maxDistance)
	dronieWP:alt(100*maxAltitude)
	
	local sdAlt = sdDistance * (maxAltitude - startAltitude) / maxDistance + startAltitude
	local sldWP = startWp:copy()
	sldWP:offset_bearing(bearing, sdDistance)
	sldWP:alt(100*sdAlt)
	
	mission:clear()
	
	--WP0 is always home
	local homeWP = ahrs:get_home()
	local home = mavlink_mission_item_int_t()
	home:seq(0)
	home:frame(FRAME_GLOBAL_RELATIVE_ALT)
	home:command(16) -- WP
	home:x(homeWP:lat())
	home:y(homeWP:lng())
	home:z(homeWP:alt() / 100)
	mission:set_item(0, home)

	-- Takeoff
	local to = mavlink_mission_item_int_t()
	to:seq(1)
	to:frame(FRAME_GLOBAL_RELATIVE_ALT)
	to:command(22) -- TAKEOFF
	to:z(startAltitude)
	mission:set_item(1, to)
	
	-- ROI
	local wp = mavlink_mission_item_int_t()
	wp:seq(2)
	wp:frame(FRAME_GLOBAL_RELATIVE_ALT)
	wp:command(201) -- SET_ROI
	wp:x(roiWP:lat())
	wp:y(roiWP:lng())
	wp:z(roiWP:alt() / 100)
	mission:set_item(2, wp)

	-- Dronie
	wp:seq(3)
	wp:command(16) -- WP
	wp:x(dronieWP:lat())
	wp:y(dronieWP:lng())
	wp:z(dronieWP:alt() / 100)
	mission:set_item(3, wp)

	-- Slow down
	wp:seq(4)
	wp:frame(FRAME_GLOBAL_RELATIVE_ALT)
	wp:x(sldWP:lat())
	wp:y(sldWP:lng())
	wp:z(sldWP:alt() / 100)
	mission:set_item(4, wp)

	-- Change Speed to 1m/s
	local cs = mavlink_mission_item_int_t()
	cs:seq(5)
	cs:frame(FRAME_GLOBAL_RELATIVE_ALT)
	cs:command(178) -- CHANGE_SPEED
	cs:param2(1)
	mission:set_item(5, cs)
	
	-- start
	wp:seq(6)
	local lat = startWp:lat()
	local lng = startWp:lng()
	wp:x(lat)
	wp:y(lng)
	wp:z(startAltitude)
	mission:set_item(6, wp)

	-- land
	local land = mavlink_mission_item_int_t()
	land:seq(7)
	land:frame(FRAME_GLOBAL_RELATIVE_ALT)
	land:command(21) -- LAND
	mission:set_item(7, land)
	
	gcs:send_text(0, "Dronie mission written")
end

function createMission()
	if cmdQuickShot == 1 then
		createDronie()
	elseif cmdQuickShot == 2 then
		createRocket()
	elseif cmdQuickShot == 3 then
		createHelix()
	end
	stage = 0
end

function update()
    local msg, chan = mavlink:receive_chan()
    if msg then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)
        if parsed_msg then

            local result
            if parsed_msg.msgid == COMMAND_LONG_ID then
                result = handle_command_long(parsed_msg)
            elseif parsed_msg.msgid == GLOBAL_POSITION_INT_ID then
                result = handle_global_position_int(parsed_msg)
            end

            if result then
                -- Send ack if the command is one were interested in
                local ack = {}
                ack.command = parsed_msg.command
                ack.result = result
                ack.progress = 0
                ack.result_param2 = 0
                ack.target_system = parsed_msg.sysid
                ack.target_component = parsed_msg.compid
                mavlink:send_chan(chan, mavlink_msgs.encode("COMMAND_ACK", ack))
            end
        end
    end

	if stage == 1 then
		calcWps()
	elseif stage == 2 then
		createMission()
	end
	
	return update, 200
end

gcs:send_text(6, "QuickShots script starting")
return update, 100
