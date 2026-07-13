--[[
    BionicYaw Lua Script
    ArduPlane biomimetic tail mixer

    Replaces AP_BionicYaw C++ implementation.

    Modes:
      0 = Differential VTail
      1 = Rotating Tail

    Outputs:
      Scripting1 -> left tail
      Scripting2 -> right tail
    
    Author: Abderrahim Khouk / OpenFuselage UAV

--]]

local UPDATE_RATE = 50



--------------------------------------------------
-- bionic_yaw.param
--------------------------------------------------
-- 
-- SERVO9_FUNCTION 94
-- SERVO10_FUNCTION 95
-- 
-- SCR_ENABLE 1
-- SCR_HEAP_SIZE 100000
-- 
-- BYAW_ENABLE 1
-- BYAW_MODE 0
-- BYAW_YAW_GAIN 0.80
-- BYAW_ROLL_GAIN 0.15
-- BYAW_PITCH_GAIN 1.00
--------------------------------------------------



--------------------------------------------------
-- Parameters
--------------------------------------------------

local BYAW_ENABLE = Parameter()
local BYAW_MODE = Parameter()

local BYAW_YAW_GAIN = Parameter()
local BYAW_ROLL_GAIN = Parameter()
local BYAW_PITCH_GAIN = Parameter()

local BYAW_ROT_MAX = Parameter()
local BYAW_ROT_FN = Parameter()

local BYAW_PITCH_COMP = Parameter()
local BYAW_ROLL_COMP = Parameter()


BYAW_ENABLE:init("BYAW_ENABLE")
BYAW_MODE:init("BYAW_MODE")

BYAW_YAW_GAIN:init("BYAW_YAW_GAIN")
BYAW_ROLL_GAIN:init("BYAW_ROLL_GAIN")
BYAW_PITCH_GAIN:init("BYAW_PITCH_GAIN")

BYAW_ROT_MAX:init("BYAW_ROT_MAX")
BYAW_ROT_FN:init("BYAW_ROT_FN")

BYAW_PITCH_COMP:init("BYAW_PITCH_COMP")
BYAW_ROLL_COMP:init("BYAW_ROLL_COMP")


--------------------------------------------------
-- Servo functions
--------------------------------------------------

-- Scripting outputs:
-- 94 = Scripting1
-- 95 = Scripting2

local VTAIL_LEFT  = 94
local VTAIL_RIGHT = 95


--------------------------------------------------
-- Input channels
--------------------------------------------------

local AILERON = 4
local ELEVATOR = 19
local RUDDER = 21



--------------------------------------------------
-- Differential V-tail
--------------------------------------------------

local function update_differential()

    local roll =
        SRV_Channels:get_output_scaled(AILERON)

    local pitch =
        SRV_Channels:get_output_scaled(ELEVATOR)

    local yaw =
        SRV_Channels:get_output_scaled(RUDDER)


    local yg =
        BYAW_YAW_GAIN:get()

    local rg =
        BYAW_ROLL_GAIN:get()

    local pg =
        BYAW_PITCH_GAIN:get()


    local yaw_component =
        yaw * yg

    local roll_component =
        roll * rg

    local pitch_component =
        pitch * pg


    local left =
        pitch_component +
        yaw_component +
        roll_component


    local right =
        pitch_component -
        yaw_component -
        roll_component


    SRV_Channels:set_output_scaled(
        VTAIL_LEFT,
        left
    )

    SRV_Channels:set_output_scaled(
        VTAIL_RIGHT,
        right
    )

end



--------------------------------------------------
-- Rotating Tail
--------------------------------------------------

local function update_rotator()

    local yaw =
        SRV_Channels:get_output_scaled(RUDDER)


    -- yaw range:
    -- -4500 ... +4500 centidegrees

    local max_rotation =
        BYAW_ROT_MAX:get()


    local rotation =
        yaw / 4500.0 *
        max_rotation *
        100


    local fn =
        BYAW_ROT_FN:get()


    if fn < 1 then
        fn = 1
    end


    if fn > 16 then
        fn = 16
    end


    local scripting_function =
        93 + fn


    SRV_Channels:set_output_scaled(
        scripting_function,
        rotation
    )


    return rotation

end



--------------------------------------------------
-- Rotating Tail Compensation
--------------------------------------------------

local function update_compensation(rotation)

    if rotation == nil then
        return
    end


    local pitch =
        SRV_Channels:get_output_scaled(ELEVATOR)

    local roll =
        SRV_Channels:get_output_scaled(AILERON)



    --
    -- Compensation model:
    --
    -- Tail rotation creates coupling moments.
    -- The gains are intentionally exposed
    -- as parameters for aircraft tuning.
    --


    local angle =
        rotation / 100.0


    local pitch_delta =
        math.sin(math.rad(angle)) *
        BYAW_PITCH_COMP:get()


    local roll_delta =
        math.sin(math.rad(angle)) *
        BYAW_ROLL_COMP:get()



    SRV_Channels:set_output_scaled(
        ELEVATOR,
        pitch + pitch_delta
    )


    SRV_Channels:set_output_scaled(
        AILERON,
        roll + roll_delta
    )

end



--------------------------------------------------
-- Main loop
--------------------------------------------------

function update()


    if BYAW_ENABLE:get() == 0 then

        return update,
        1000 / UPDATE_RATE

    end



    local mode =
        BYAW_MODE:get()



    if mode == 1 then

        local rotation =
            update_rotator()

        update_compensation(rotation)

    else

        update_differential()

    end



    return update,
    1000 / UPDATE_RATE

end


return update()
