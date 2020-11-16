-- This script will select a random location (as defined from a lat long coordinate)
-- and will play a set of tunes to navigate you towards that point, and select a new once
-- once the point has been found.
--
-- This script primarily serves to demo how to work with Locations, as well as how to
-- use the tonealarm to play tones, and send status text messages

local ACCEPTANCE_DISTANCE = 20.0

local TUNE_POINT = "MBNT255>A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8"
local TUNE_TOWARDS = "MFT100L8>B"
local TUNE_AWAY = "MFT100L4>A#B#"

local target = Location()
local top_left = Location()
top_left:lat(-353622666)
top_left:lng(1491650479)

local last_distance = 1e10
local notify_interval_ms = uint32_t(5000)
local last_notify_time_ms = millis()
local score = 0

function find_next_point ()
    target:lat(top_left:lat())
    target:lng(top_left:lng())
    target:offset(math.random()*-100, math.random()*10)
    gcs:send_text(6, string.format("New target %d %d", target:lat(), target:lng()))
    local current = ahrs:get_position()
    if current then
        last_distance = current:get_distance(target)
    end
    last_distance = 1e10
    return
end

function update ()
    local current = ahrs:get_position()
    if current then
        local dist = target:get_distance(current)
        local now = millis()
        if dist < ACCEPTANCE_DISTANCE then
            notify:play_tune(TUNE_POINT)
            score = score + 1
            gcs:send_text(6, string.format("Got a point! %d total", score))
            find_next_point()
        elseif (now - last_notify_time_ms) > notify_interval_ms then
            last_notify_time_ms = now
            gcs:send_text(6, string.format("Distance: %.1f %.1f", target:get_distance(current), dist))
            if dist < (last_distance - 1) then
                notify:play_tune(TUNE_TOWARDS)
            elseif dist > (last_distance + 1) then
                notify:play_tune(TUNE_AWAY)
            end
        end
        if math.abs(last_distance - dist) > 1.0 then
            last_distance = dist;
        end

    end
    return update, 100
end

find_next_point()

return update, 100
