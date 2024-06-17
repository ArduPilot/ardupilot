-- example script for ensuring home is set before allowing arming
-- useful so users don't take off in stabilize before RTL will work.

---@dxiagnostic disable: param-type-mismatch

auth_id = arming:get_aux_auth_id()

function update ()

    if not ahrs:home_is_set() then
      arming:set_aux_auth_failed(auth_id, "Home is not set")
    else
      arming:set_aux_auth_passed(auth_id)
    end

    return update, 5000

end

return update()
