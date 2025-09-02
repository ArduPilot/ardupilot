function update()
    if arming:is_armed() then
        gcs:send_text(6, "ARMED LUA")
        -- get aileron, rudder, elevator PWM
        local a_pwm = SRV_Channels:get_output_pwm(4)
        local r_pwm = SRV_Channels:get_output_pwm(21)
        local e_pwm = SRV_Channels:get_output_pwm(19)

        if not a_pwm or not r_pwm or not e_pwm then
            return update, 10
        end

        -- get velocity
        local V_ned = ahrs:get_velocity_NED()
        local V = nil
        local V_x, V_y, V_z = nil, nil, nil

        if V_ned ~= nil then
            local V_wind = ahrs:wind_estimate()

            if V_wind == nil then
                V_wind = Vector3f(0,0,0)
            end

            local V_air = V_ned - V_wind
            local V_body = ahrs:earth_to_body(V_air)

            V_x = V_body:x()  -- forward +x
            V_y = V_body:y()  -- right +y
            V_z = V_body:z()  -- down +z
            V = math.sqrt(V_x*V_x + V_y*V_y + V_z*V_z)
            gcs:send_text(6, string.format("Velocity: %f", V))
        end
        

        -- inputs must be between 1000 and 2000
        a_pwm = math.min(math.max(1000, a_pwm), 2000)
        r_pwm = math.min(math.max(1000, r_pwm), 2000)
        e_pwm = math.min(math.max(1000, e_pwm), 2000)

        -- normalize
        local a_norm = (a_pwm - 1500) / 500.0
        local r_norm = -(r_pwm - 1500) / 500.0
        local e_norm = -(e_pwm - 1500) / 500.0

        local yaw_adjustment_factor = 0.5

        -- gridfin deflections
        local deflection_top  = -a_norm * 0.2 + r_norm * 0.2
        local deflection_star = -a_norm * 0.2 + r_norm * yaw_adjustment_factor * 0.2 + e_norm * 0.3
        local deflection_port = -a_norm * 0.2 + r_norm * yaw_adjustment_factor * 0.2 - e_norm * 0.3



        gcs:send_text(6, string.format("Top D: %f", deflection_top))

        -- limit the deflections

        if V ~= nil and V > 3 then
            -- calculate AOA and beta (sideslip)
            local AOA = math.atan(V_z, V_x)
            local beta = math.asin(V_y/V)
            gcs:send_text(6, string.format("AOA = %.2f rad", AOA))
            gcs:send_text(6, string.format("beta = %.2f rad", beta))
            -- limit fin pwm:
            deflection_top=limit_extended(deflection_top, 0, AOA, beta)
            deflection_star=limit_extended(deflection_star, -2*math.pi/3, AOA, beta)
            deflection_port=limit_extended(deflection_port, 2*math.pi/3, AOA, beta)
        else
            deflection_top=limit_failover(deflection_top)
            deflection_star=limit_failover(deflection_star)
            deflection_port=limit_failover(deflection_port)
        end

        -- deflections in PWM
        local g_top_pwm  = math.floor(deflection_top  * 500 + 1500)
        local g_star_pwm = math.floor(deflection_star * 500 + 1500)
        local g_port_pwm = math.floor(deflection_port * 500 + 1500)

        -- set gridfin pwms
        SRV_Channels:set_output_pwm(94, g_top_pwm)  
        SRV_Channels:set_output_pwm(95, g_star_pwm)  
        SRV_Channels:set_output_pwm(96, g_port_pwm)  
    end
    return update, 10
end

function limit_extended(fin_deflection, fin_angle, AOA, beta)
    local eff = fin_deflection - (AOA * math.sin(fin_angle) + beta * math.cos(fin_angle))
    gcs:send_text(6, string.format("eff = %.2f rad", eff))
    local limit = math.rad(20)
    if eff > limit then
        fin_deflection = limit + (AOA * math.sin(fin_angle) + beta * math.cos(fin_angle))
    elseif eff < -limit then
        fin_deflection = -limit + (AOA * math.sin(fin_angle) + beta * math.cos(fin_angle))
    end

    return fin_deflection
end

function limit_failover(fin_deflection)
    local limit = math.rad(10)
    return math.min(math.max(-limit, fin_deflection), limit)
end 

return update, 10