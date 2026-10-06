-- This script is an example of printing to a display via scripting
-- Connect a supported display to the autopilot, and configure the NTF_DISPLAY_TYPE parameter seen at https://ardupilot.org/copter/docs/common-display-onboard.html
-- The notify:send_text(text, row) method will override default on the display, disabling the default messages

local switchTimeA
local switchTimeB
local displayWidth = 18
local function update()
    -- Just keep track of when we should switch to a smiley :)
    if switchTimeA == nil then
        switchTimeA = millis() + 5000
        switchTimeB = switchTimeA + 10000
    end

    -- Example of overriding a line keeping some defaults, here we will replace the battery(1) and GPS(2) rows
    if switchTimeA > millis() then
        notify:send_text("Hello, World!", 1)
        notify:send_text(tostring(millis()), 2)

    -- Next demonstrate we can release the text, and the default will be shown again
    elseif switchTimeB > millis() then
        notify:release_text(1)
        notify:release_text(2)

    -- Example of overriding all lines, a smiley, try moving the autopilot around to see it change
    else
        -- Generate the smiley
        local width = (displayWidth / 2)
        local roll = math.floor(width + (ahrs:get_roll_rad() * width)) - 4
        local pitch = math.floor(ahrs:get_pitch_rad() * 6) + 2;
        local sub = 5 - roll
        if sub < 0 then
            sub = 0
        end
        local rows = {}
        if pitch - 2 >= 0 and pitch - 2 <= 5 then 
            rows[pitch - 2] = (string.rep(" ", roll) .. "    ##"):sub(sub);
        end
        if pitch - 1 >= 0 and pitch - 1 <= 5 then 
            rows[pitch - 1] = (string.rep(" ", roll) .. " #    #"):sub(sub);
        end
        if pitch >= 0 and pitch <= 5 then 
            rows[pitch] = (string.rep(" ", roll) .. "      #"):sub(sub);
        end
        if pitch + 1 >= 0 and pitch + 1 <= 5 then 
            rows[pitch + 1] = (string.rep(" ", roll) .. " #    #"):sub(sub);
        end
        if pitch + 2 >= 0 and pitch + 2 <= 5 then 
            rows[pitch + 2] = (string.rep(" ", roll) .. "    ##"):sub(sub);
        end
        if pitch + 3 >= 0 and pitch + 3 <= 5 then 
            rows[pitch + 3] = "";
        end

        -- Send it out to the display
        for i = 0, 5 do
            if rows[i] == nil then
                rows[i] = ""
            end
            notify:send_text(rows[i], i)
        end
    end

    return update, 10
 end
 return update, 1000 -- Wait a few seconds before starting
