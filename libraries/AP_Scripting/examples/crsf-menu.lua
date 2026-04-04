--[[
--]]

SCRIPT_NAME = "CRSF Menu"
SCRIPT_NAME_SHORT = "CRSFMenu"
SCRIPT_VERSION = "0.1"

MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
CRSF_EVENT = {PARAMETER_READ=1, PARAMETER_WRITE=2}
CRSF_PARAM_TYPE = {
    UINT8 = 0,  -- deprecated
    INT8 = 1,   -- deprecated
    UINT16 = 2, -- deprecated
    INT16 = 3,  -- deprecated
    FLOAT = 8,
    TEXT_SELECTION = 9,
    STRING = 10,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
}

CRSF_COMMAND_STATUS = {
    READY = 0, --               --> feedback
    START = 1, --               <-- input
    PROGRESS = 2, --            --> feedback
    CONFIRMATION_NEEDED = 3, -- --> feedback
    CONFIRM = 4, --             <-- input
    CANCEL = 5, --              <-- input
    POLL = 6 --                 <-- input
}

-- create a CRSF menu float item
function create_float_entry(name, value, min, max, default, dpoint, step, unit)
    return string.pack(">BzllllBlz", CRSF_PARAM_TYPE.FLOAT, name, value, min, max, default, dpoint, step, unit)
end

-- create a CRSF menu text selection item
function create_text_entry(name, options, value, min, max, default, unit)
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options, value, min, max, default, unit)
end

-- create a CRSF menu string item
function create_string_entry(name, value, max)
    return string.pack(">BzzB", CRSF_PARAM_TYPE.STRING, name, value, max)
end

-- create a CRSF menu info item
function create_info_entry(name, info)
    return string.pack(">Bzz", CRSF_PARAM_TYPE.INFO, name, info)
end

-- create a CRSF command entry
function create_command_entry(name, status, timeout, info)
    timeout = timeout or 10 -- 1s
    return string.pack(">BzBBz", CRSF_PARAM_TYPE.COMMAND, name, status, timeout, info)
end

local param1 = create_info_entry("Menu Item 1", "It goes here")
local param2 = create_info_entry("Menu Item 2", "Another one")
local param3 = create_command_entry("Beethoven", CRSF_COMMAND_STATUS.READY, 50, "Command")
local param4 = create_text_entry("Options", "One;Two;Three;Four", 0, 0, 3, 2, "ms")
local param5 = create_string_entry("Change me:", "Some String", 16)
local param6 = create_float_entry("A Number", 12345, 0, 100000, 10000, 3, 5, "nits")

local command, text, astring, afloat

local menu = crsf:add_menu('Example Menu')
local menu2

if menu ~= nil then
    menu:add_parameter(param1)
    menu:add_parameter(param2)
    command = menu:add_parameter(param3)
    text = menu:add_parameter(param4)
    astring = menu:add_parameter(param5)
    afloat = menu:add_parameter(param6)
    menu2 = menu:add_menu("Another Menu")
    if menu2 ~= nil then
        menu2:add_parameter(create_info_entry("Item 1", "Is badass"))
        menu2:add_parameter(create_info_entry("Item 2", "Is extremely badass"))
        menu2:add_parameter(create_text_entry("Options", "Three;Four;Five", 0, 0, 2, 1, "ms"))
    end

    gcs:send_text(MAV_SEVERITY.INFO, string.format("Loaded CRSF menu \'" .. menu:name() .. "\'"))
end

function update()
    local param, payload, events = crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE)
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        if command ~= nil and param == command:id() then
            local command_action = string.unpack(">B", payload)
            if command_action == CRSF_COMMAND_STATUS.START then
                -- we have been asked to start a command, ask for confirmarion
                local new_data = create_command_entry("Beethoven", CRSF_COMMAND_STATUS.CONFIRMATION_NEEDED, 0, "Play?")
                crsf:send_write_response(new_data)
            elseif command_action == CRSF_COMMAND_STATUS.CONFIRM then
                -- we have been asked to start a command, update the parameter to reflect the status
                local new_data = create_command_entry("Beethoven", CRSF_COMMAND_STATUS.PROGRESS, 0, "Playing")
                crsf:send_write_response(new_data)
                notify:play_tune("L16GGGL4E-L16FFFL4D") -- Beethoven's 5th intro
            elseif command_action == CRSF_COMMAND_STATUS.POLL then
                -- we have been asked to start a command, update the parameter to reflect the status
                crsf:send_write_response(param3)
            end
        elseif text ~= nil and param == text:id() then
            local selection = string.unpack(">B", payload)
            gcs:send_text(MAV_SEVERITY.INFO, "Selected option " .. selection)
            crsf:send_write_response(payload)
        elseif astring ~= nil and param == astring:id() then
            local selection = string.unpack(">z", payload)
            gcs:send_text(MAV_SEVERITY.INFO, "New string is " .. selection)
            crsf:send_write_response(payload)
        elseif afloat ~= nil and param == afloat:id() then
            local selection = string.unpack(">i", payload)
            gcs:send_text(MAV_SEVERITY.INFO, "Value is " .. selection / 10*3)
            crsf:send_write_response(payload)
        end
    end
    return update, 100
end

return update, 5000