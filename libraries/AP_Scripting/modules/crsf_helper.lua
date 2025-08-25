-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.
-- Version 1.3: Fixed SELECTION parameter option separator.

local helper = {}

-- MAVLink severity levels for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3}

-- CRSF constants
local CRSF_EVENT = {PARAMETER_READ = 1, PARAMETER_WRITE = 2}
local CRSF_PARAM_TYPE = {
    FLOAT = 8,
    TEXT_SELECTION = 9,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
}
local CRSF_COMMAND_STATUS = { READY = 0, START = 1 }

-- Internal storage for menu items, callbacks, and object references
local menu_items = {}
local menu_objects = {} -- Keep references to menu objects to prevent garbage collection

-- ####################
-- # PACKING FUNCTIONS
-- ####################

-- These functions create the binary packed strings required by the low-level CRSF API.

-- Creates a CRSF menu text selection item
local function create_selection_entry(name, options_table, default_idx)
    -- BUGFIX: The CRSF spec requires options to be separated by a semicolon ';'.
    local options_str = table.concat(options_table, ";")
    local zero_based_idx = default_idx - 1
    local min_val = 0
    local max_val = #options_table - 1
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options_str, zero_based_idx, min_val, max_val, zero_based_idx, "")
end

-- Creates a CRSF menu number item (as float)
local function create_number_entry(name, value, min, max, default, dpoint, step, unit)
    -- Handle floating point values by converting them to fixed-point integers
    local scale = 10^(dpoint or 0)
    local packed_value = math.floor(value * scale + 0.5)
    local packed_min = math.floor(min * scale + 0.5)
    local packed_max = math.floor(max * scale + 0.5)
    local packed_default = math.floor(default * scale + 0.5)
    local packed_step = math.floor(step * scale + 0.5)
    return string.pack(">BzllllBlz", CRSF_PARAM_TYPE.FLOAT, name, packed_value, packed_min, packed_max, packed_default, dpoint or 0, packed_step, unit or "")
end

-- Creates a CRSF menu info item
local function create_info_entry(name, info)
    return string.pack(">Bzz", CRSF_PARAM_TYPE.INFO, name, info)
end

-- Creates a CRSF command entry
local function create_command_entry(name)
    return string.pack(">BzBBz", CRSF_PARAM_TYPE.COMMAND, name, CRSF_COMMAND_STATUS.READY, 10, "Execute")
end

-- ####################
-- # MENU PARSING
-- ####################

-- Recursively parses the user's declarative menu table and builds the CRSF menu structure.
local function parse_menu(menu_definition, parent_menu_obj)
    if not menu_definition.items or type(menu_definition.items) ~= "table" then
        return
    end

    for _, item_def in ipairs(menu_definition.items) do
        local param_obj = nil
        local packed_data = nil

        if item_def.type == 'MENU' then
            param_obj = parent_menu_obj:add_menu(item_def.name)
            if param_obj then
                table.insert(menu_objects, param_obj) -- Keep a reference to the menu object
                parse_menu(item_def, param_obj) -- Recurse into sub-menu
            else
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create menu: " .. item_def.name)
            end

        elseif item_def.type == 'SELECTION' then
            packed_data = create_selection_entry(item_def.name, item_def.options, item_def.default)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'NUMBER' then
            packed_data = create_number_entry(item_def.name, item_def.default, item_def.min, item_def.max, item_def.default, item_def.dpoint, item_def.step, item_def.unit)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'COMMAND' then
            packed_data = create_command_entry(item_def.name)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'INFO' then
            packed_data = create_info_entry(item_def.name, item_def.info)
            param_obj = parent_menu_obj:add_parameter(packed_data)
        end

        if param_obj then
            -- Store the CRSF-assigned ID back into our definition table for easy lookup
            menu_items[param_obj:id()] = item_def
        elseif not param_obj and item_def.type ~= 'MENU' then
            gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create param: " .. item_def.name)
        end
    end
end

-- ####################
-- # EVENT HANDLING
-- ####################

-- This function runs in the background, listens for menu events, and triggers callbacks.
local function event_loop()
    local param_id, payload, events = crsf:get_menu_event(CRSF_EVENT.PARAMETER_WRITE)

    if (events and (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0) then
        local item_def = menu_items[param_id]
        if not item_def or not item_def.callback then
            return event_loop, 100 -- No item or callback found, continue polling
        end

        local new_value = nil
        if item_def.type == 'SELECTION' then
            -- Unpack the 0-indexed selection from the payload
            local selected_index = string.unpack(">B", payload)
            new_value = item_def.options[selected_index + 1] -- Convert to 1-indexed value for Lua

        elseif item_def.type == 'NUMBER' then
            -- Unpack the integer and scale it back to a float
            local raw_value = string.unpack(">l", payload)
            local scale = 10^(item_def.dpoint or 0)
            new_value = raw_value / scale

        elseif item_def.type == 'COMMAND' then
            local command_action = string.unpack(">B", payload)
            if command_action ~= CRSF_COMMAND_STATUS.START then
                return event_loop, 100 -- Ignore anything other than the 'start' command
            end
            -- For commands, the value passed to the callback is simply 'true'
            new_value = true
        end

        -- If we have a new value, call the user's callback function
        if new_value ~= nil then
            local success, err = pcall(item_def.callback, new_value)
            if not success then
                gcs:send_text(MAV_SEVERITY.ERROR, "CRSF Callback Err: " .. tostring(err))
            end
        end

        -- For commands, we must send a response to reset the UI element
        if item_def.type == 'COMMAND' then
            local packed_data = create_command_entry(item_def.name)
            crsf:send_write_response(packed_data)
        end
    end

    return event_loop, 100 -- Reschedule the event loop
end


-- ####################
-- # PUBLIC API
-- ####################

-- The main entry point for the helper library.
-- The user script calls this function with its menu definition table.
function helper.init(menu_definition)
    -- Create the top-level menu
    local top_menu_obj = crsf:add_menu(menu_definition.name)
    if not top_menu_obj then
        gcs:send_text(MAV_SEVERITY.ERROR, "CRSF: Failed to create top-level menu.")
        return
    end
    table.insert(menu_objects, top_menu_obj) -- Keep a reference to the top-level menu object

    -- Parse the rest of the menu structure
    parse_menu(menu_definition, top_menu_obj)

    gcs:send_text(MAV_SEVERITY.INFO, "CRSF Menu '" .. menu_definition.name .. "' initialized.")

    -- Start the background event loop
    return event_loop, 1000 -- Initial delay before starting the loop
end

return helper
