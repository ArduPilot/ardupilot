nv:register("HEAD_ANGLE")

function update()
    val = nv:get("HEAD_ANGLE") -- just get the value
    val, ts = nv:get("HEAD_ANGLE") -- get the value and the timestamp
    val, ts, sysid, compid = nv:get("HEAD_ANGLE") -- get the value, the timestamp, the sysid and the compid
    if val ~= nil then
        gcs:send_text(0, "Time:" .. tostring(ts) .. " HEAD_ANGLE: " .. tostring(val) .. " sysid: " .. tostring(sysid) .. " compid: " .. tostring(compid))
    else
        gcs:send_text(0, "HEAD_ANGLE: nil")
    end
    return update, 1000
end

return update()
