-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.
-- Version 6.4: Adds default handling for COMMAND POLL requests in the event loop,
--              ensuring state synchronization even if script callbacks ignore POLL.

local helper = {}

-- MAVLink severity levels for GCS messages
helper.MAV_SEVERITY = {INFO = 6, WARNING = 4, ERROR = 3, DEBUG = 7}
local MAV_SEVERITY = helper.MAV_SEVERITY -- local alias

-- CRSF constants
local CRSF_EVENT = {PARAMETER_READ = 1, PARAMETER_WRITE = 2}
local CRSF_PARAM_TYPE = {
    FLOAT = 8,
    TEXT_SELECTION = 9,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
}
helper.CRSF_COMMAND_STATUS = {
    READY = 0, --               --> feedback
    START = 1, --               <-- input
    PROGRESS = 2, --            --> feedback
    CONFIRMATION_NEEDED = 3, -- --> feedback
    CONFIRM = 4, --             <-- input
    CANCEL = 5, --              <-- input
    POLL = 6 --                 <-- input
}
local CRSF_COMMAND_STATUS = helper.CRSF_COMMAND_STATUS -- local alias

-- These tables are local, respecting the script sandbox.
-- Each script will have its own instance of this state.
local menu_items = {}
local crsf_objects = {}

-- ####################
-- # PACKING FUNCTIONS
-- ####################

-- These functions create the binary packed strings required by the low-level CRSF API.

-- Creates a CRSF menu text selection item
local function create_selection_entry(name, options_table, current_idx, unit)
    -- The CRSF spec requires options to be separated by a semicolon ';'.
    local options_str = table.concat(options_table, ";")
    local zero_based_idx = current_idx - 1
    local min_val = 0
    local max_val = #options_table - 1
    -- The 4th argument is the current value. The 7th is the default value.
    -- For our purposes, we'll pack the current value into both slots.
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options_str, zero_based_idx, min_val, max_val, zero_based_idx, unit or "")
end

-- Creates a CRSF menu number item (as float)
local function create_number_entry(name, value, min, max, default, dpoint, step, unit)
    -- Per CRSF spec, float values are sent as INT32 with a decimal point indicator.
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
local function create_command_entry(name, status, info)
    status = status or CRSF_COMMAND_STATUS.READY
    info = info or "Execute"
    local timeout = 50 -- 5s timeout, increased from 1s for long-running cals
    return string.pack(">BzBBz", CRSF_PARAM_TYPE.COMMAND, name, status, timeout, info)
end

-- ####################
-- # MENU PARSING
-- ####################

-- Recursively parses a menu definition table and builds the CRSF menu structure.
local function parse_menu(menu_definition, parent_menu_obj)
    if not menu_definition.items or type(menu_definition.items) ~= "table" then
        return
    end

    for _, item_def in ipairs(menu_definition.items) do
        local param_obj = nil
        local packed_data

        if item_def.type == 'MENU' then
            param_obj = parent_menu_obj:add_menu(item_def.name)
            if param_obj then
                parse_menu(item_def, param_obj) -- Recurse into sub-menu
            else
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create menu: " .. item_def.name)
            end

        elseif item_def.type == 'SELECTION' then
            item_def.current_idx = item_def.default -- Store the initial 1-based index
            packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx, item_def.unit)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'NUMBER' then
            item_def.current_val = item_def.default -- Store the initial value
            packed_data = create_number_entry(item_def.name, item_def.current_val, item_def.min, item_def.max, item_def.default, item_def.dpoint, item_def.step, item_def.unit)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'COMMAND' then
            item_def.status = CRSF_COMMAND_STATUS.READY
            item_def.info = item_def.info or "Execute"
            packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'INFO' then
            packed_data = create_info_entry(item_def.name, item_def.info)
            param_obj = parent_menu_obj:add_parameter(packed_data)
        end

        if param_obj then
            -- Store a reference to the CRSF object to prevent garbage collection
            table.insert(crsf_objects, param_obj)
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
local idle_backoff = 0

-- This function runs as an independent loop for each script.
-- It uses the peek/pop API to safely coexist with other menu scripts.
local function event_loop()
    -- Add dynamic polling delays based on arming state
    local IDLE_DELAY
    local ACTIVE_DELAY

    -- Use the correct arming check from the 'arming' object
    if arming:is_armed() then
        -- Vehicle is ARMED: prioritize flight code, slow down UI polling
        IDLE_DELAY = 500  -- 2.0 Hz idle polling
        ACTIVE_DELAY = 100 -- 10 Hz active polling
    else
        -- Vehicle is DISARMED: prioritize UI responsiveness for setup
        IDLE_DELAY = 200  -- 5.0 Hz idle polling
        ACTIVE_DELAY = 20   -- 50 Hz active polling
    end

    -- ## 1. Peek at the event queue to see if there's anything to do ##
    local count, param_id, payload, events = crsf:peek_menu_event()

    -- If the queue is empty, reschedule with the (now dynamic) idle delay.
    if count == 0 then
        idle_backoff = idle_backoff + 1
        if idle_backoff > (1000/ACTIVE_DELAY) then -- no events in a second so switch to idle
            return event_loop, IDLE_DELAY
        else
            return event_loop, ACTIVE_DELAY
        end
    end

    -- ## 2. Check if the event belongs to this script's menu ##
    local item_def = menu_items[param_id]
    if not item_def then
        -- This event is not for us. Yield and let another script's loop handle it.
        return event_loop, ACTIVE_DELAY -- Use active delay as UI might be busy with other script
    end

    idle_backoff = 0    -- no longer idling

    -- ## 3. Pop the event from the queue ##
    -- This is critical: pop the event before handling it.
    crsf:pop_menu_event()

    -- ## 4. Process the event (it's ours) ##

    -- Handle a READ request from the transmitter first.
    if (events & CRSF_EVENT.PARAMETER_READ) ~= 0 then
        if item_def.type == 'SELECTION' then
            local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx, item_def.unit)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'COMMAND' then
            -- Respond with the command's *current* state
            local packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'INFO' then
            local packed_data = create_info_entry(item_def.name, item_def.info)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'NUMBER' then
            -- Respond with the number's *current* state
            local packed_data = create_number_entry(item_def.name, item_def.current_val, item_def.min, item_def.max, item_def.default, item_def.dpoint, item_def.step, item_def.unit)
            crsf:send_write_response(packed_data)
        else
            -- Send generic response if type not handled explicitly for read
            -- Note: send_response() is deprecated, but kept for potential backward compatibility
            -- or if a generic case is truly needed. It uses the popped context.
            crsf:send_response()
        end
    end

    -- Handle a WRITE request from the transmitter.
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        if not item_def.callback then
            -- No callback, but we must still respond if needed
            if item_def.type == 'COMMAND' then
                 -- Respond with current state even if no callback
                 local packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
                 crsf:send_write_response(packed_data)
            elseif item_def.type == 'SELECTION' then
                 local packed_data = string.pack('>B', item_def.current_idx - 1)
                 crsf:send_write_response(packed_data)
            elseif item_def.type == 'NUMBER' then
                 local scale = 10^(item_def.dpoint or 0)
                 local packed_value = math.floor(item_def.current_val * scale + 0.5)
                 local packed_data = string.pack('>l', packed_value)
                 crsf:send_write_response(packed_data)
            end
            goto reschedule -- Skip callback logic
        end

        local new_value = nil

        -- Determine the new value from the payload and call the user's callback
        if item_def.type == 'SELECTION' then
            local selected_index_zero_based = string.unpack(">B", payload)
            item_def.current_idx = selected_index_zero_based + 1
            new_value = item_def.options[item_def.current_idx]
        elseif item_def.type == 'NUMBER' then
            local raw_value = string.unpack(">l", payload)
            local scale = 10^(item_def.dpoint or 0)
            new_value = raw_value / scale
        elseif item_def.type == 'COMMAND' then
            -- Pass the specific command action to the callback
            local command_action = string.unpack(">B", payload)
            -- Check for START, CONFIRM, CANCEL, or POLL
            if command_action == CRSF_COMMAND_STATUS.POLL or
               command_action == CRSF_COMMAND_STATUS.START or
               command_action == CRSF_COMMAND_STATUS.CONFIRM or
               command_action == CRSF_COMMAND_STATUS.CANCEL then
                new_value = command_action
            end
        end

        if new_value ~= nil then
            -- pcall now expects return values for command state
            -- For POLL, callback might return nil or fewer than 2 values if unhandled
            local success, ret_status, ret_info = pcall(item_def.callback, new_value)

            if not success then
                gcs:send_text(MAV_SEVERITY.ERROR, "CRSF Callback Err: " .. tostring(ret_status)) -- ret_status holds error msg on failure
                if item_def.type == 'COMMAND' then
                    -- Reset command to READY on error
                    item_def.status = CRSF_COMMAND_STATUS.READY
                    item_def.info = "Error"
                end
            else
                -- Store new state returned from a successful command callback
                if item_def.type == 'COMMAND' then
                    -- Only update state if callback handled the action (returned non-nil status)
                    -- For POLL, if callback returns nil, status remains unchanged.
                    if ret_status ~= nil then
                        item_def.status = ret_status
                        item_def.info = ret_info or "Execute" -- Use default info if callback only returns status
                    end
                    -- If ret_status is nil (e.g., callback ignored POLL), item_def.status/info remain unchanged.
                elseif item_def.type == 'NUMBER' then
                    -- Store the new value
                    item_def.current_val = new_value
                end
            end
        end

        -- After a write event, we must respond to confirm the new state to the transmitter.
        -- For POLL, respond with current state if callback didn't handle it.
        -- For other actions, respond with state potentially updated by callback.
        if item_def.type == 'COMMAND' then
            -- Respond with the current item_def status/info
            local packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'SELECTION' then
            -- The value is the 0-based index of the selection
            local packed_data = string.pack('>B', item_def.current_idx - 1)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'NUMBER' then
            -- The value is the 4-byte int32_t
            local scale = 10^(item_def.dpoint or 0)
            local packed_value = math.floor(item_def.current_val * scale + 0.5)
            local packed_data = string.pack('>l', packed_value)
            crsf:send_write_response(packed_data)
        end
    end

    ::reschedule::
    -- ## 5. Reschedule the loop ##
    -- Use the (now dynamic) active delay.
    return event_loop, ACTIVE_DELAY
end

-- ####################
-- # PUBLIC API
-- ####################

--- Initializes a CRSF menu system for the calling script.
-- @param menu_definition (table) The menu definition table for this script.
function helper.register_menu(menu_definition)
    if not (menu_definition and menu_definition.name and menu_definition.items) then
        gcs:send_text(MAV_SEVERITY.ERROR, "CRSF: Invalid menu definition passed to helper.register_menu().")
        return
    end

    -- Create the top-level menu for this specific script
    local top_level_menu_obj = crsf:add_menu(menu_definition.name)
    if top_level_menu_obj then
        -- This function now populates the local menu_items and crsf_objects tables
        parse_menu(menu_definition, top_level_menu_obj)
        gcs:send_text(MAV_SEVERITY.INFO, "CRSF: Built menu '" .. menu_definition.name .. "'")
    else
        gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create top-level menu for '" .. menu_definition.name .. "'")
        return -- Do not start the event loop if the menu could not be created
    end

    -- Start this script's independent, persistent event loop.
    return event_loop, 2000
end

return helper