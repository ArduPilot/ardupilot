-- This is a simple script example that shows how to use the notify display override.
-- In this example battery voltage and percent remaining is retrieved and displayed 
-- on screen to act as a handy onboard display for checking the batteries.

local function update()

    local voltage = 0
    local remaining = 0
    local healthy = battery:healthy(0)

    -- Update display
    -- First argument is line number.  There are up to 6 lines available.  0 based numbering is used
    -- Second argument is the string to be displayed.  Up to 19 characters can be displayed on each line
    notify:handle_scr_disp(0," --- ArduPilot --- ")

    if (healthy) then
        -- Get voltage and display it on line 3
        voltage = battery:voltage(0)
        notify:handle_scr_disp(2, string.format("  Voltage: %.2f V", voltage))

        -- Get pack remaining and display it on line 5
        remaining = battery:capacity_remaining_pct(0)
        notify:handle_scr_disp(4, string.format("Remaining: %.2f %%", remaining))

    else
        -- voltage monitor is unhealthy 
        notify:handle_scr_disp(3, "  Battery Monitor  ")
        notify:handle_scr_disp(4, "     Unhealthy     ")
    end

    -- Display is updated at 2 Hz so no point in calling any faster than that
    return update, 500

end

-- Wait for 5 sec before starting script
return update, 5000
