--script to force arm on startup

function update()
    if not arming:is_armed() then
        arming:arm()
    end
    return update, 1000
end


return update, 1000