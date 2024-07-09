--[[
Example script to test VTOL approach bindings. Prints a message to the console once in the VTOL land sequence 
indicating which phase of the approach and landing the aircraft is in.
]]--

-- Function to retrieve VTOL status
function get_vtol_status()
    -- Retrieve the status from the QuadPlane methods
    local in_vtol_land_approach = quadplane:in_vtol_land_approach()
    local in_vtol_land_final = quadplane:in_vtol_land_final()
    local in_vtol_land_poscontrol = quadplane:in_vtol_land_poscontrol()
    local in_vtol_airbrake = quadplane:in_vtol_airbrake()

    -- Return the status as a table
    return {
      approach = in_vtol_land_approach,
      final = in_vtol_land_final,
      poscontrol = in_vtol_land_poscontrol,
      airbrake = in_vtol_airbrake
    }
end

-- Function to send VTOL status to GCS
function send_status_to_gcs()
    -- Get the VTOL status
    local status = get_vtol_status()

    -- Create the message to send to the GCS
    local message = string.format("VTOL Status - Approach: %s, Final: %s, PosControl: %s, Airbrake: %s",
                                  tostring(status.approach),
                                  tostring(status.final),
                                  tostring(status.poscontrol),
                                  tostring(status.airbrake))
    -- Send the message to the GCS if in VTOL land sequence
    gcs:send_text(6, message)
end

-- Main update function
function update()

    -- Call the function to send status to GCS
    if quadplane:in_vtol_land_sequence() then
     send_status_to_gcs()
    end

    -- Schedule the next run of this function in 1000 ms (1 Hz)
    return update, 1000
end

-- Start the main loop
return update, 1000
