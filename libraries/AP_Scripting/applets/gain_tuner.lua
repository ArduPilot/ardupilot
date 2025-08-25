-- gain_tuner.lua
-- A script to provide a CRSF menu for in-flight PID gain tuning.
-- Allows for increasing or decreasing key PID gains for an entire axis at once.
-- This version includes Save/Revert functionality, stateful step-based adjustments,
-- and a configurable tuning step percentage.

local crsf_helper = require('crsf_helper')

-- MAVLink severity for GCS messages
local MAV_SEVERITY = {INFO = 6, WARNING = 4}

-- The percentage change for each step, can be changed via the menu
local tuning_step_percent -- This will be initialized later from the menu definition

-- ####################
-- # PARAMETER SETUP
-- ####################

-- Group parameters by axis into tables for easier handling.
-- Using assert() ensures the script will halt with a clear error if a parameter name is incorrect.
local axis_gains = {
    Roll = {
        {param = assert(Parameter('ATC_RAT_RLL_P'), "Failed to find ATC_RAT_RLL_P"), name = "Roll P"},
        {param = assert(Parameter('ATC_RAT_RLL_I'), "Failed to find ATC_RAT_RLL_I"), name = "Roll I"},
        {param = assert(Parameter('ATC_RAT_RLL_D'), "Failed to find ATC_RAT_RLL_D"), name = "Roll D"},
        {param = assert(Parameter('ATC_ANG_RLL_P'), "Failed to find ATC_ANG_RLL_P"), name = "Roll Ang P"}
    },
    Pitch = {
        {param = assert(Parameter('ATC_RAT_PIT_P'), "Failed to find ATC_RAT_PIT_P"), name = "Pitch P"},
        {param = assert(Parameter('ATC_RAT_PIT_I'), "Failed to find ATC_RAT_PIT_I"), name = "Pitch I"},
        {param = assert(Parameter('ATC_RAT_PIT_D'), "Failed to find ATC_RAT_PIT_D"), name = "Pitch D"},
        {param = assert(Parameter('ATC_ANG_PIT_P'), "Failed to find ATC_ANG_PIT_P"), name = "Pitch Ang P"}
    },
    Yaw = {
        {param = assert(Parameter('ATC_RAT_YAW_P'), "Failed to find ATC_RAT_YAW_P"), name = "Yaw P"},
        {param = assert(Parameter('ATC_RAT_YAW_I'), "Failed to find ATC_RAT_YAW_I"), name = "Yaw I"},
        {param = assert(Parameter('ATC_RAT_YAW_D'), "Failed to find ATC_RAT_YAW_D"), name = "Yaw D"},
        {param = assert(Parameter('ATC_ANG_YAW_P'), "Failed to find ATC_ANG_YAW_P"), name = "Yaw Ang P"}
    }
}

-- Table to store the original parameter values on script startup for the revert function
local original_gains = {}
-- Table to store the number of tuning steps applied to each axis (e.g., +1, -2, etc.)
local gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }

-- Populate the original_gains table by iterating through all defined parameters
local function populate_original_gains()
    for axis_name, gains in pairs(axis_gains) do
        original_gains[axis_name] = {}
        for _, gain_info in ipairs(gains) do
            table.insert(original_gains[axis_name], {
                param = gain_info.param,
                original_value = gain_info.param:get()
            })
        end
    end
end

-- Initial population of original gains at script start
populate_original_gains()

-- ####################
-- # CORE LOGIC & CALLBACKS
-- ####################

-- This function is called when the "Step %" is changed in the menu.
-- It parses the string (e.g., "10%") and updates the tuning_step_percent variable.
local function on_step_change(new_step_str)
    -- Remove the '%' character and convert the string to a number
    -- The extra parentheses ensure only the first return value of gsub is passed to tonumber
    local percent_num = tonumber((string.gsub(new_step_str, "%%", "")))
    if percent_num then
        tuning_step_percent = percent_num / 100.0
        gcs:send_text(MAV_SEVERITY.INFO, "Tuning step set to: " .. new_step_str)
    end
end

-- This function adjusts all gains for a given axis based on a step counter.
-- @param axis_name (string): The name of the axis to adjust ("Roll", "Pitch", or "Yaw").
-- @param step_change (integer): The change to apply to the step counter (+1 for increase, -1 for decrease).
local function adjust_axis_gains(axis_name, step_change)
    -- Update the step counter for the axis
    gain_steps[axis_name] = gain_steps[axis_name] + step_change

    local original_gain_table = original_gains[axis_name]
    if not original_gain_table then
        gcs:send_text(MAV_SEVERITY.WARNING, "Invalid axis: " .. tostring(axis_name))
        return
    end

    -- Calculate the total multiplier based on the original value and the current step count
    local total_multiplier = 1.0 + (gain_steps[axis_name] * tuning_step_percent)

    -- Iterate through the original parameters for the axis and calculate/set the new value
    for _, gain_info in ipairs(original_gain_table) do
        local new_value = gain_info.original_value * total_multiplier
        gain_info.param:set(new_value)
    end

    -- Send a confirmation message to the GCS for the entire axis
    local total_percent_change = gain_steps[axis_name] * tuning_step_percent * 100
    local gcs_message = string.format("%s Gains %+d%%", axis_name, math.floor(total_percent_change + 0.5))
    gcs:send_text(MAV_SEVERITY.INFO, gcs_message)
    
    -- Also send the new P gain value for quick reference
    local p_gain_val = axis_gains[axis_name][1].param:get()
    local p_gain_msg = string.format("%s P = %.4f", axis_name, p_gain_val)
    gcs:send_text(MAV_SEVERITY.INFO, p_gain_msg)
end

-- Saves all currently set gain values to EEPROM and resets the tuning state.
local function save_all_gains()
    for _, gains in pairs(axis_gains) do
        for _, gain_info in ipairs(gains) do
            local current_value = gain_info.param:get()
            gain_info.param:set_and_save(current_value)
        end
    end
    gcs:send_text(MAV_SEVERITY.INFO, "All tuned gains have been saved.")

    -- After saving, the new values become the baseline for further tuning.
    -- Repopulate the original_gains table and reset the step counters.
    populate_original_gains()
    gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }
    gcs:send_text(MAV_SEVERITY.INFO, "Tuning state reset to new values.")
end

-- Reverts all gains to the values they had when the script was started.
local function revert_all_gains()
    for _, gains in pairs(original_gains) do
        for _, gain_info in ipairs(gains) do
            gain_info.param:set(gain_info.original_value)
        end
    end
    -- Also reset the step counters back to zero
    gain_steps = { Roll = 0, Pitch = 0, Yaw = 0 }
    gcs:send_text(MAV_SEVERITY.INFO, "Gains reverted to startup values.")
end

-- ####################
-- # MENU DEFINITION
-- ####################

-- This table defines the complete menu structure.
local menu_definition = {
    name = "Gain Tuner", -- The root menu name
    items = {
        -- ====== ROLL TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Roll Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Roll", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Roll", -1) end end},
            }
        },
        -- ====== PITCH TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Pitch Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Pitch", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Pitch", -1) end end},
            }
        },
        -- ====== YAW TUNING SUB-MENU ======
        {
            type = 'MENU',
            name = "Yaw Tuning",
            items = {
                {type = 'COMMAND', name = "Increase Gains", callback = function(v) if v then adjust_axis_gains("Yaw", 1) end end},
                {type = 'COMMAND', name = "Decrease Gains", callback = function(v) if v then adjust_axis_gains("Yaw", -1) end end},
            }
        },
        -- ====== SETTINGS SUB-MENU ======
        {
            type = 'MENU',
            name = "Settings",
            items = {
                {
                    type = 'SELECTION',
                    name = "Step %",
                    options = {"1%", "5%", "10%", "25%", "50%"},
                    default = 3, -- 1-based index for "10%"
                    callback = on_step_change
                }
            }
        },
        -- ====== SAVE & REVERT SUB-MENU ======
        {
            type = 'MENU',
            name = "Save & Revert",
            items = {
                {type = 'INFO', name = "Warning", info = "Save is permanent"},
                {type = 'COMMAND', name = "Save All Gains", callback = function(v) if v then save_all_gains() end end},
                {type = 'COMMAND', name = "Revert All Gains", callback = function(v) if v then revert_all_gains() end end},
            }
        },
    }
}

-- ####################
-- # INITIALIZATION
-- ####################

-- Find the settings definition in the menu table to get the default value
local settings_menu = menu_definition.items[4]
local step_selection = settings_menu.items[1]
local default_step_string = step_selection.options[step_selection.default]

-- Initialize the tuning_step_percent with the default value from the menu
on_step_change(default_step_string)

-- Pass the menu definition to the helper library to build the menu and start the event loop.
return crsf_helper.init(menu_definition)
