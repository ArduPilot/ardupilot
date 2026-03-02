local function update()
    gcs:send_text(0, "hello, world")
    return update, 1000  -- 1000msごとに実行
end

return update()
