--[[
Usage for SITL PLANE testing:

- default params already enable rangefinder
- set param RTL_AUTOLAND 2
- set param SCR_ENABLE 1
- set param RNGFND1_TYPE 100
- restart, if needed
- upload mission (jump_tags_calibrate_agl.waypoints)
- launch plane (switch to AUTO and arm)
- Mission will go to a loiter_unlim at wp 2.
- set param SIM_BARO_DRIFT to your taste to simulate a long mission.
- switch mode to RTL, which jumps you back to AUTO at the DO_LAND_START which begins with a loiter_to_alt.

- once the loiter_to_alt is done, it will do an approach pattern and fly over the runway and sample the altitude
    using the rangefinder. The start and end sample points are defined tag 400 and 401 where at 401 it uses the
    average AGL and adjusts BARO_ALT_GND to compensate for any baro drift.
- It then does the rest of the pattern and lands without the lidar needing to do much because the offset
     has already been corrected for



QGC WPL 110
0	1	0	16	0	0	0	0	-35.3629591	149.1647941	584.016148	1
1	0	3	22	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	10.000000	1
2	0	3	17	0.00000000	0.00000000	0.00000000	0.00000000	-35.36299850	149.15860890	200.000000	1
3	0	3	189	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
4	0	3	31	0.00000000	0.00000000	0.00000000	0.00000000	-35.36416220	149.16121600	40.000000	1
5	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36648940	149.16154860	30.000000	1
6	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36614380	149.16553970	30.000000	1
7	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36330910	149.16519370	30.000000	1
8	0	0	600	400.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
9	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36233350	149.16505430	100.000000	1
10	0	0	600	401.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.00000000	0.000000	1
11	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36058800	149.16486380	40.000000	1
12	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36079800	149.16206900	100.000000	1
13	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36720250	149.16250880	40.000000	1
14	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36693130	149.16574360	30.000000	1
15	0	3	16	0.00000000	0.00000000	0.00000000	0.00000000	-35.36566700	149.16559340	30.000000	1
16	0	3	21	0.00000000	0.00000000	0.00000000	1.00000000	-35.36276450	149.16517900	0.000000	1
--]]



local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}


local ROTATION_PITCH_270 = 25

local MISSION_TAG_MEASURE_AGL_START         = 400
local MISSION_TAG_CALIBRATE_BARO            = 401

local agl_samples_count = 0
local agl_samples_sum = 0


function sample_rangefinder_to_get_AGL()
    if (not rangefinder:has_data_orient(ROTATION_PITCH_270)) then
        -- rangefinder not ready
        return
    end

    -- we're actively sampling rangefinder distance to ground
    local distance_raw_m = rangefinder:distance_cm_orient(ROTATION_PITCH_270) * 0.01

    -- correct the range for attitude (multiply by DCM.c.z, which is cos(roll)*cos(pitch))
    local ahrs_get_rotation_body_to_ned_c_z = math.cos(ahrs:get_roll())*math.cos(ahrs:get_pitch())
    local agl_corrected_for_attitude_m = distance_raw_m * ahrs_get_rotation_body_to_ned_c_z

    if (agl_samples_count <= 0) then
        agl_samples_count = 0 -- divide-by-zero sanity check in case it somehow wrapped or initialized wrong
        agl_samples_sum = 0
        gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: AGL measurements started"))
    end

    agl_samples_sum = agl_samples_sum + agl_corrected_for_attitude_m
    agl_samples_count = agl_samples_count + 1

    local agl_average = agl_samples_sum / agl_samples_count
    gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: AGL measurement %u: %.2fm, avg: %.2f", agl_samples_count, agl_corrected_for_attitude_m, agl_average))
end


function update_baro(new_agl_m)
    
    local current_baro_agl_m = baro:get_altitude()
    local alt_error_m = current_baro_agl_m - new_agl_m
    gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: AGL alt_error is: %.2f - %.2f = %.2f", current_baro_agl_m, new_agl_m, alt_error_m))

    local baro_alt_offset = param:get('BARO_ALT_OFFSET')
    local baro_alt_offset_new_value = baro_alt_offset + alt_error_m
    gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: BARO_ALT_OFFSET changed from %.2f to %.2f", baro_alt_offset, baro_alt_offset_new_value))
    param:set('BARO_ALT_OFFSET', baro_alt_offset_new_value)
end


function update()
    if (mission:state() ~= mission.MISSION_RUNNING) or (not arming:is_armed()) or (not vehicle:get_likely_flying()) then
        -- only run landing mission checks if in auto with a valid mission and armed and flying.
        return update, 5000
    end

    local tag, age = mission:get_last_jump_tag()

    if (tag == nil) then
        return update, 1000
    end

    if ((tag == MISSION_TAG_MEASURE_AGL_START) and (age <= 5)) then
        -- we're at or currently on waypoints after the tag so lets start gathering samples
        sample_rangefinder_to_get_AGL()
        -- lets sample at 2 Hz
        return update, 500

    elseif ((tag == MISSION_TAG_CALIBRATE_BARO) and (age <= 3) and (agl_samples_count > 0)) then
        -- finished sampling, use the result to offset baro
        local agl_average_final_m = agl_samples_sum / agl_samples_count
        gcs:send_text(MAV_SEVERITY.INFO, string.format("LUA: AGL measurements stopped: samples = %d, avg = %.2fm", agl_samples_count, agl_average_final_m))
        update_baro(agl_average_final_m)
        agl_samples_count = 0
    else
        agl_samples_count = 0
    end

    return update, 1000
end


gcs:send_text(MAV_SEVERITY.INFO, "LUA: SCRIPT START: Check AGL to calibrate Baro")
return update()


