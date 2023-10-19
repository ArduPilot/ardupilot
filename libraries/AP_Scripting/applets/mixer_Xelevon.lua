-- setup param block, reserving 30 params beginning with XELEVON_
local PARAM_TABLE_KEY = 13
local PARAM_TABLE_PREFIX = "XELEVON_"

local Channel_Roll = 4 -- K_AILERON
local Channel_Pitch = 19 -- K_ELEVATOR
local Channel_Yaw = 21 -- K_RUDDER

local Channel_Xelevon_1 = 94 -- K_SCRIPTING1
local Channel_Xelevon_2 = 95 -- K_SCRIPTING2
local Channel_Xelevon_3 = 96 -- K_SCRIPTING3
local Channel_Xelevon_4 = 97 -- K_SCRIPTING4

local VEHICLE_TYPE_PLUS = 1
local VEHICLE_TYPE_X = 2

-- bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
 end
 
 -- add a parameter and bind it to a variable
 function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
 end



-- setup specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 10), 'could not add param table')
--[[
  // @Param: XELEVON_TYPE
  // @DisplayName: XElevon Configuration type
  // @Description: XElevon Configuration flight surfgace mapping select looking from the rear. Plus "+" channel mapping 1,2,3,4 are 0,90,180,270 degrees (Up, Right, Down, Left) respectively. X channel mapping 1,2,3,4 are 315(-45),45,135,225 (Upper Left, Upper Right, Lower Right, Lower Left) respectively.
  // @Values: 1:Plus,2:X
  // @User: Standard
--]]
local vehicle_type = bind_add_param('TYPE', 1, 1)

--[[
  // @Param: XELEVON_CH1
  // @DisplayName: XElevon Ch1 Gain
  // @Description: XElevon gain applied to flight surface on channel 1 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch1 = bind_add_param('CH1', 2, 1.0)

--[[
  // @Param: XELEVON_CH2
  // @DisplayName: XElevon Ch2 Gain
  // @Description: XElevon gain applied to flight surface on channel 2 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch2 = bind_add_param('CH2', 3, 1.0)

--[[
  // @Param: XELEVON_CH3
  // @DisplayName: XElevon Ch3 Gain
  // @Description: XElevon gain applied to flight surface on channel 3 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch3 = bind_add_param('CH3', 4, 1.0)

--[[
  // @Param: XELEVON_CH4
  // @DisplayName: XElevon Ch4 Gain
  // @Description: XElevon gain applied to flight surface on channel 4 after the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_ch4 = bind_add_param('CH4', 5, 1.0)

--[[
  // @Param: XELEVON_ROLL
  // @DisplayName: XElevon Roll Gain
  // @Description: XElevon gain applied to the commanded Roll before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Roll = bind_add_param('ROLL', 6, 1.0)

--[[
  // @Param: XELEVON_PITCH
  // @DisplayName: XElevon Pitch Gain
  // @Description: XElevon gain applied to commanded Pitch before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Pitch = bind_add_param('PITCH', 7, 1.0)

--[[
  // @Param: XELEVON_YAW
  // @DisplayName: XElevon Yaw Gain
  // @Description: XElevon gain applied to commanded Yaw before the mixer. Default is 1.0
  // @Range: -10 10
  // @User: Advanced
--]]
local gain_Yaw = bind_add_param('YAW', 8, 1.0)


function update()

    local roll = SRV_Channels:get_output_scaled(Channel_Roll)
    local pitch = SRV_Channels:get_output_scaled(Channel_Pitch)
    local yaw = SRV_Channels:get_output_scaled(Channel_Yaw)

    roll = roll * gain_Roll:get()
    pitch = pitch * gain_Pitch:get()
    yaw = yaw * gain_Yaw:get()

    local ch1 = 0
    local ch2 = 0
    local ch3 = 0
    local ch4 = 0
    
    if (vehicle_type:get() == VEHICLE_TYPE_PLUS) then
        ch1, ch2, ch3, ch4 = mixer_plus(roll, pitch, yaw)
    elseif (vehicle_type:get() == VEHICLE_TYPE_X) then
        ch1, ch2, ch3, ch4 = mixer_x(roll, pitch, yaw)
    else
        -- no mixer selected, lets slow things down
        return update, 1000
    end

    SRV_Channels:set_output_scaled(Channel_Xelevon_1, ch1 * gain_ch1:get())
    SRV_Channels:set_output_scaled(Channel_Xelevon_2, ch2 * gain_ch2:get())
    SRV_Channels:set_output_scaled(Channel_Xelevon_3, ch3 * gain_ch3:get())
    SRV_Channels:set_output_scaled(Channel_Xelevon_4, ch4 * gain_ch4:get())

    return update, 10 -- run at 100 Hz
end

function mixer_plus(roll, pitch, yaw)
    -- Looking from the rear, the channel orientations are:
    -- ch1 is   0 degrees   (Up)
    -- ch2 is  90 degrees   (Right)
    -- ch3 is 180 degrees   (Down)
    -- ch4 is 270 degrees   (Left)

    local ch1_out = yaw - roll
    local ch2_out = pitch + roll
    local ch3_out = yaw + roll
    local ch4_out = pitch - roll

    return ch1_out, ch2_out, ch3_out, ch4_out
end

function mixer_x(roll, pitch, yaw)
    -- Looking from the rear, the channel orientations are:
    -- ch1 is Upper Left
    -- ch2 is Upper Right
    -- ch3 is Lower Right
    -- ch4 is Lower Left

    local ch1_out = yaw - roll + pitch
    local ch2_out = pitch + roll - yaw
    local ch3_out = yaw + roll + pitch    
    local ch4_out = pitch - roll - yaw

    return ch1_out, ch2_out, ch3_out, ch4_out
end

return update()
