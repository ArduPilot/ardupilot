params.add(270,0,'SCR_TEST',4,3.14,'SCR_TEST2',4,42)

local count = 0

function update()

  if count == 5 then
    count = 0
    param:set('SCR_TEST',param:get('SCR_TEST') + 5)
  end

  local value = param:get('SCR_TEST')
  if not value then
    gcs:send_text(0, 'get SCR_TEST failed')
  else
    gcs:send_text(0, string.format('SCR_TEST: %0.1f', value))
  end

  count = count + 1
  return update, 5000
end

return update, 5000
