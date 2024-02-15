--[[
    Example for getting state of EFI and checking if the RPM is within tolerance
--]]

function do_check()
    local efi_state = efi:get_state()
    local engine_speed = efi_state:engine_speed_rpm()
    if not engine_speed then
        return
    end
    if engine_speed > 7000 then
        gcs:send_text(2, "ECU RPM is too high!")
    end
end

function update()
    do_check()
    return update, 100
end
gcs:send_text(6, "Started EFI speed check");
return update, 0