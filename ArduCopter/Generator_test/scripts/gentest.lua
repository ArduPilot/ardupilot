
function update()

    local errorcode = generator:get_errorcode()
    gcs:send_text(0, "LUA: Error code: " .. errorcode)  

return update, 1000
end

return update()
