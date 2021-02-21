-- This script is a test of param set and get
local count = 0

-- for fast param acess it is better to get a param object,
-- this saves the code searching for the param by name every time
local VM_I_Count = Parameter()
if not VM_I_Count:init('SCR_VM_I_COUNT') then
  gcs:send_text(6, 'get SCR_VM_I_COUNT failed')
end

-- returns null if param cant be found
local fake_param = Parameter()
if not fake_param:init('FAKE_PARAM') then
  gcs:send_text(6, 'get FAKE_PARAM failed')
end

function update() -- this is the loop which periodically runs

  -- get and print all the scripting parameters
  local value = param:get('SCR_ENABLE')
  if value then
    gcs:send_text(6, string.format('LUA: SCR_ENABLE: %i',value))
  else
    gcs:send_text(6, 'LUA: get SCR_ENABLE failed')
  end
  value = param:get('SCR_VM_I_COUNT')
  if value then
    gcs:send_text(6, string.format('LUA: SCR_VM_I_COUNT: %i',value))
  else
    gcs:send_text(6, 'LUA: get SCR_VM_I_COUNT failed')
  end
  value = param:get('SCR_HEAP_SIZE')
  if value then
    gcs:send_text(6, string.format('LUA: SCR_HEAP_SIZE: %i',value))
  else
    gcs:send_text(6, 'LUA: get SCR_HEAP_SIZE failed')
  end
  value = param:get('SCR_DEBUG_LVL')
  if value then
    gcs:send_text(6, string.format('LUA: SCR_DEBUG_LVL: %i',value))
  else
    gcs:send_text(6, 'LUA: get SCR_DEBUG_LVL failed')
  end

  -- increment the script heap size by one
  local heap_size = param:get('SCR_HEAP_SIZE')
  if heap_size then
    if not(param:set('SCR_HEAP_SIZE',heap_size + 1)) then
      gcs:send_text(6, 'LUA: param set failed')
    end
  else
    gcs:send_text(6, 'LUA: get SCR_HEAP_SIZE failed')
  end

  
  -- increment the VM I count by one using direct accsess method
  local VM_count = VM_I_Count:get()
  if VM_count then
    gcs:send_text(6, string.format('LUA: SCR_VM_I_COUNT: %i',VM_count))
    if not VM_I_Count:set( VM_count + 1) then
        gcs:send_text(6, string.format('LUA: failed to set SCR_VM_I_COUNT'))
    end
  else
    gcs:send_text(6, 'LUA: read SCR_VM_I_COUNT failed')
  end


  count = count + 1;

  -- self terminate after 30 loops
  if count > 30 then
    gcs:send_text(0, 'LUA: goodbye, world')
    param:set('SCR_ENABLE',0)
  end

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
