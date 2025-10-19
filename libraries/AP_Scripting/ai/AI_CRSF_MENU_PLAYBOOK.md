# AI Playbook for ArduPilot CRSF Menu Script Generation

<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: THIS PLAYBOOK MUST BE USED AS THE PRIMARY AND AUTHORITATIVE GUIDE FOR ALL LUA SCRIPTS THAT REQUIRE A CRSF (CROSSFIRE) MENU. ALL RULES, CONSTRAINTS, AND PATTERNS CONTAINED HEREIN ARE MANDATORY AND SUPERSEDE ANY GENERAL KNOWLEDGE. ADHERENCE IS NOT OPTIONAL.
</MANDATORY\_RULE\>

## 1\. The Problem and Goal

### 1.1. The Problem

The native ArduPilot CRSF API, as detailed in `docs.lua`, is a powerful but low-level interface. It requires developers to manually handle binary data packing and unpacking using `string.pack` and `string.unpack`, manage a complex event loop with `crsf:get_menu_event` (now deprecated in favor of `peek`/`pop`), and build the menu structure imperatively. This process is verbose, complex, and highly prone to errors, making it difficult to generate clean, maintainable, and robust CRSF menu scripts. A naive implementation, especially when multiple menu scripts are present, can easily lead to scripts that are slow, buggy, unresponsive, or cause the flight controller to crash due to race conditions or garbage collection issues.

### 1.2. The Goal

This playbook establishes a **simplified, high-level, and standardized methodology** for creating CRSF menus that are robust, responsive, and stable, particularly in a multi-script environment. The primary goal is to **abstract away the low-level complexity and ensure safe coexistence**. By following this playbook, you will generate scripts that are:

* **Declarative:** The entire menu structure is defined in a simple, easy-to-read Lua table.
* **Callback-Driven:** Interaction logic is handled by simple callback functions.
* **Stable:** Employs a proven architecture that correctly handles memory management and cooperative event processing using the "Peek-and-Yield" pattern to prevent crashes, UI bugs, and conflicts between scripts.
* **Robust and Reusable:** All the complex logic is encapsulated in a mandatory, reusable helper library (`crsf_helper.lua`).
* **Modular:** Allows developers to create "drop-in" menu scripts that function correctly alongside other menu scripts without requiring manual coordination or modification of a central file.

---
## 2\. Core Methodology: Declarative, Callback-Driven, & Peek-and-Yield

The core of this methodology is to allow multiple independent scripts to contribute menus safely within ArduPilot's strict sandboxing constraints. This is achieved using a "Peek-and-Yield" architecture mediated by a helper library.

1.  **Menu Definition (Plugin Scripts, e.g., `my_menu.lua`):** Each script defines its menu structure (sub-menus, parameters, ranges, options) in a standard declarative Lua table. It also defines simple callback functions for interaction logic.
2.  **Initialization (`plugin_script.lua`):** Each plugin script `require`s the `crsf_helper.lua` library and calls `helper.register_menu(menu_definition)` at the end, passing its menu definition table.
3.  **Menu Building (`crsf_helper.lua`):** The `helper.register_menu(menu_definition)` function uses the low-level C++ API (`crsf:add_menu`, `crsf:add_parameter`) to build the C++ menu objects for that specific script. It stores references to these objects and the script's item definitions in **local** tables to prevent garbage collection and allow event lookup.
4.  **Independent Event Loops (`crsf_helper.lua`):** The `helper.register_menu(menu_definition)` function starts a persistent `event_loop` function for **each script**. This is necessary due to ArduPilot's sandbox preventing shared Lua state.
5.  **Cooperative Event Handling ("Peek-and-Yield"):**
    * Each running `event_loop` first uses the C++ `crsf:peek_menu_event()` function to look at the next event in the shared queue **without removing it**.
    * It checks if the `param_id` of the peeked event exists in its **local** `menu_items` table.
    * If the ID **matches** an item belonging to this script, the loop processes the event (calling the appropriate callback) and then calls the C++ `crsf:pop_menu_event()` function to remove the event from the queue.
    * If the ID **does not match**, the loop immediately **yields** (returns), leaving the event on the queue for a different script's event loop to handle.
    * An efficiency throttle adjusts the loop's delay based on whether the queue is empty (longer delay) or active (shorter delay).

This architecture ensures that multiple scripts can safely coexist:
* **Sandbox Compliance:** Scripts remain isolated with no shared Lua state.
* **No Race Conditions:** The "Peek-and-Yield" mechanism prevents scripts from destructively stealing events intended for others.
* **Stability:** Each helper instance maintains strong references only to its own C++ menu objects via its local `crsf_objects` table, preventing garbage collection issues.
* **"Drop-in" Functionality:** New menu scripts can be added without modifying existing ones.

<MANDATORY\_RULE\>
CRITICAL DIRECTIVE: EVERY CRSF SCRIPT GENERATED MUST CONSIST OF TWO FILES: THE USER'S SCRIPT (E.G., `my_menu.lua`) AND THE STANDARD `crsf_helper.lua` LIBRARY. THE USER'S SCRIPT MUST `require()` THE HELPER LIBRARY.
</MANDATORY\_RULE\>

## 3\. The `crsf_helper.lua` Library

This reusable library is the heart of the new methodology. You must include this exact code as a separate `crsf_helper.lua` file alongside any CRSF menu script you generate.

### 3.1. `crsf_helper.lua` Full Code (Version 6.3 - Final "Peek-and-Yield")

```lua
-- crsf_helper.lua
-- A reusable helper library to simplify the creation of ArduPilot CRSF menus.
-- This library abstracts away the complexity of binary packing/unpacking and event loop management.
-- Version 6.1: Command & Control. Adds support for multi-step command lifecycle
--              (START, CONFIRM, CANCEL) and stateful command items.

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
-- CHANGED: Exposing CRSF_COMMAND_STATUS and adding all states
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

-- These tables are now local, respecting the script sandbox.
-- Each script will have its own instance of this state.
local menu_items = {}
local crsf_objects = {}

-- ####################
-- # PACKING FUNCTIONS
-- ####################

-- These functions create the binary packed strings required by the low-level CRSF API.

-- Creates a CRSF menu text selection item
local function create_selection_entry(name, options_table, current_idx)
    -- The CRSF spec requires options to be separated by a semicolon ';'.
    local options_str = table.concat(options_table, ";")
    local zero_based_idx = current_idx - 1
    local min_val = 0
    local max_val = #options_table - 1
    -- The 4th argument is the current value. The 7th is the default value.
    -- For our purposes, we'll pack the current value into both slots.
    return string.pack(">BzzBBBBz", CRSF_PARAM_TYPE.TEXT_SELECTION, name, options_str, zero_based_idx, min_val, max_val, zero_based_idx, "%")
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
-- CHANGED: This function now takes status and info to reflect the command's current state
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
        local packed_data = nil

        if item_def.type == 'MENU' then
            param_obj = parent_menu_obj:add_menu(item_def.name)
            if param_obj then
                parse_menu(item_def, param_obj) -- Recurse into sub-menu
            else
                gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create menu: " .. item_def.name)
            end

        elseif item_def.type == 'SELECTION' then
            item_def.current_idx = item_def.default -- Store the initial 1-based index
            packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'NUMBER' then
            packed_data = create_number_entry(item_def.name, item_def.default, item_def.min, item_def.max, item_def.default, item_def.dpoint, item_def.step, item_def.unit)
            param_obj = parent_menu_obj:add_parameter(packed_data)

        elseif item_def.type == 'COMMAND' then
            -- CHANGED: Store the initial state for the command
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
        return event_loop, IDLE_DELAY
    end

    -- ## 2. Check if the event belongs to this script's menu ##
    local item_def = menu_items[param_id]
    if not item_def then
        -- This event is not for us. Yield and let another script's loop handle it.
        -- Per your feedback, use IDLE_DELAY to back off for the active script.
        return event_loop, IDLE_DELAY
    end
    
    -- ## 3. Pop the event from the queue ##
    -- This is critical: pop the event before handling it.
    crsf:pop_menu_event()

    -- ## 4. Process the event (it's ours) ##

    -- Handle a READ request from the transmitter first.
    if (events & CRSF_EVENT.PARAMETER_READ) ~= 0 then
        gcs:send_text(MAV_SEVERITY.INFO, "CRSF DBG: READ " .. tostring(param_id) .. " type " .. tostring(item_def.type))
        if item_def.type == 'SELECTION' then
            local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'COMMAND' then
            -- Respond with the command's *current* state
            local packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'INFO' then
            local packed_data = create_info_entry(item_def.name, item_def.info)
            crsf:send_write_response(packed_data)
        else
            crsf:send_response()
        end
    end

    -- Handle a WRITE request from the transmitter.
    if (events & CRSF_EVENT.PARAMETER_WRITE) ~= 0 then
        if not item_def.callback then
            -- No callback, but we must still pop the event
            return event_loop, ACTIVE_DELAY
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
            if command_action == CRSF_COMMAND_STATUS.START or
               command_action == CRSF_COMMAND_STATUS.CONFIRM or
               command_action == CRSF_COMMAND_STATUS.CANCEL then
                new_value = command_action
            end
        end

        if new_value ~= nil then
            -- pcall now expects return values for command state
            local success, ret1, ret2 = pcall(item_def.callback, new_value)

            if not success then
                gcs:send_text(MAV_SEVERITY.ERROR, "CRSF Callback Err: " .. tostring(ret1))
                if item_def.type == 'COMMAND' then
                    -- Reset command to READY on error
                    item_def.status = CRSF_COMMAND_STATUS.READY
                    item_def.info = "Error"
                end
            else
                -- Store new state returned from a successful command callback
                if item_def.type == 'COMMAND' then
                    item_def.status = ret1 or CRSF_COMMAND_STATUS.READY
                    item_def.info = ret2 or "Execute"
                end
            end
        end

        -- After a write event, we must respond to confirm the new state to the transmitter.
        if item_def.type == 'COMMAND' and new_value then
            -- Respond with the *new* status that the callback returned
            local packed_data = create_command_entry(item_def.name, item_def.status, item_def.info)
            crsf:send_write_response(packed_data)
        elseif item_def.type == 'SELECTION' then
            local packed_data = create_selection_entry(item_def.name, item_def.options, item_def.current_idx)
            crsf:send_write_response(packed_data)
        end
    end


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
        gcs:send_text(MAV_SEVERITY.INFO, "CRSF: Loaded menu '" .. menu_definition.name .. "'")
    else
        gcs:send_text(MAV_SEVERITY.WARNING, "CRSF: Failed to create top-level menu for '" .. menu_definition.name .. "'")
        return -- Do not start the event loop if the menu could not be created
    end

    -- Start this script's independent, persistent event loop.
    return event_loop, 2000
end

return helper
```

## 4\. Declarative Menu Syntax

The menu is defined as a nested Lua table. Each item in the table is a table itself, representing a menu item, sub-menu, or parameter.

### 4.1. Top-Level Menu Table

The root of the definition must be a table with two keys:

  * `name`: (string) The name of the root menu entry that appears on the transmitter.
  * `items`: (table) A list of tables, where each table defines a menu item.

### 4.2. Menu Item Properties

| Property    | Type       | Description                                                                                                                                              | Used By                     |
| :---------- | :--------- | :------------------------------------------------------------------------------------------------------------------------------------------------------- | :-------------------------- |
| `type`      | `string`   | **Required.** The type of menu item. Must be one of: `'MENU'`, `'NUMBER'`, `'SELECTION'`, `'COMMAND'`, `'INFO'`.        | All                         |
| `name`      | `string`   | **Required.** The text displayed for this menu item.                                                                    | All                         |
| `callback`  | `function` | **Required.** The function to call when the value is changed or the command is executed.                                | `NUMBER`, `SELECTION`, `COMMAND` |
| `items`     | `table`    | A list of item definition tables for a sub-menu.                                                                        | `MENU`                      |
| `default`   | `number`   | The initial value of the parameter. For `SELECTION`, this is the **1-based index** of the `options` table.             | `NUMBER`, `SELECTION`       |
| `min`, `max`| `number`   | The minimum and maximum allowed values.                                                                                 | `NUMBER`                    |
| `step`      | `number`   | *Optional.* The increment/decrement step size. Defaults to 1.                                                            | `NUMBER`                    |
| `dpoint`    | `number`   | *Optional.* The number of decimal points to display. Defaults to 0.                                                      | `NUMBER`                    |
| `unit`      | `string`   | *Optional.* A short string for the unit (e.g., "m", "s", "%").                                                           | `NUMBER`                    |
| `options`   | `table`    | A list of strings for the available choices. **Note:** Options must be separated by a semicolon (`;`) in the final packed data, which the helper handles. | `SELECTION`                 |
| `info`      | `string`   | The read-only text to be displayed next to the name.                                                                    | `INFO`                      |

## **5. Implementation Deep Dive & Common Pitfalls**

Understanding the "why" behind the crsf\_helper.lua design is critical to avoid reintroducing subtle bugs.

### **5.1. Garbage Collection and Memory Corruption**

  * **The Problem:** Lua's garbage collector (GC) will free memory for objects that are no longer referenced. In early versions, scripts created CRSF menu and parameter objects but did not keep a persistent reference to them. After some time, the GC would delete these objects. This led to UI corruption (e.g., all menu items taking the name of the last-created item) and crashes when the system tried to access the freed memory.
  * **The Solution:** The `crsf_objects` table (local to each script's helper instance) is mandatory. Every single `CRSFMenu_ud` and `CRSFParameter_ud` object created by `crsf:add_menu()` or `parent_menu_obj:add_parameter()` **must** be inserted into this table. This creates a strong reference that tells the GC these objects are still in use, preventing them from being collected and ensuring menu stability within that script's context.

### **5.2. CRSF State Synchronization**

  * **The Problem:** The CRSF protocol is a two-way conversation. When a user changes a selection on their transmitter, the transmitter sends a `PARAMETER_WRITE` event to the script. To confirm the change and get the latest state, the transmitter may immediately follow up with a `PARAMETER_READ` request. If the script only handles the write and ignores the read, the transmitter will time out, assume the change failed, and revert the UI display to its previous state. This was the cause of the "reverting selection" bug.
  * **The Solution:** The event loop **must** handle both event types. For stateful items like `SELECTION` and `COMMAND`, the script must send a `crsf:send_write_response()` after processing *any* event, whether it's a read or a write. This confirms the current state to the transmitter and keeps the UI perfectly synchronized. The helper must also internally track the current state of selections (in `item_def.current_idx`) to respond to read requests correctly. The order is critical: `pop_menu_event()` must be called *before* `send_write_response()` to ensure the C++ backend has the correct context.

### **5.3. Balanced Event Loop**

  * **The Problem:** A Lua script runs in a sandboxed environment with a limited time slice to execute before it must yield control back to the main flight controller scheduler. An event loop that is too aggressive (e.g., a tight `while true do ... end` loop with no delay) can use up its entire time slice, starve the main scheduler, and cause a watchdog to trigger a crash. A loop that is too slow (e.g., a long delay) will feel unresponsive and laggy to the user.
  * **The Solution:** The helper library uses a balanced approach.
      * **Peek-and-Yield:** It first peeks at the event queue. If the event is not for its script, it yields immediately with a short delay (20ms).
      * **Event Processing:** If an event *does* belong to the script, it processes it fully (including popping and sending a response if needed).
      * **Idle Throttling:** If the event queue is completely empty after peeking, it reschedules itself with a much longer delay (200ms) to save CPU resources. This combination prevents watchdog crashes while remaining highly responsive during active use.

## 6\. Main Script Pattern (`user_script.lua`)

This is the standard pattern you must follow for any script providing a CRSF menu.

1.  **Require the Helper:** The first line must be `local crsf_helper = require('crsf_helper')`.
2.  **Define Callbacks:** Create the functions that will handle value changes. These functions will receive one argument: the new value. For `SELECTION`, this is the selected string. For `NUMBER`, it's the new number. For `COMMAND`, it's simply `true`.
3.  **Define the Menu Table:** Create the `menu_definition` table according to the syntax in section 4.
4.  **Initialize:** The script must return `helper.register_menu(menu_definition)`, passing its menu definition table. This builds the menu and starts the script's independent event loop.

## 7\. CRSF Specification Notes

  * **NUMBER Type:** The CRSF protocol transmits `FLOAT` parameter types as 32-bit signed integers. The `dpoint` property tells the receiver where to place the decimal point. The helper library handles this conversion automatically.
  * **SELECTION Type:** The list of text options is transmitted as a single string with each option separated by a semicolon (`;`). The helper library handles this formatting.
  * **String Lengths:** While the protocol can handle longer strings, transmitter screens are small. It is best practice to keep `name` and `unit` strings short and descriptive.

## 8\. Troubleshooting

  * **Menu Hangs / Items Don't Appear:** This usually indicates a problem in the event loop's handling of `PARAMETER_READ` requests.
      * Check that the event loop is calling `crsf:peek_menu_event()`.
      * Verify the loop correctly checks if the `param_id` belongs to its menu items using the local `menu_items` table.
      * Ensure that if the event *does* belong, the loop calls `crsf:pop_menu_event()` **before** calling `crsf:send_write_response()`.
      * Make sure there is a response handler in the `PARAMETER_READ` block for **all** menu item types used by the script (e.g., `INFO`, `SELECTION`, `COMMAND`). A missing handler will cause a hang when the transmitter requests that item.
      * Look for repeated `PARAMETER_READ` requests for the same ID in the debug logs – this is the "retry storm" symptom of a hang.
  * **Crashes:** Crashes are often caused by Garbage Collection issues. Ensure that every C++ object created (`crsf:add_menu`, `add_parameter`) is stored in the helper's local `crsf_objects` table.
  * **Incorrect Navigation:** Ensure the `parent_id` is correctly set when creating sub-menus (this is handled automatically by the helper's `parse_menu` function). Check the C++ implementation (`AP_CRSF_Telem.cpp`) ensures the correct `parent_id` is packed in folder responses.

## 9\. Mandatory Rules and Checklist

1.  **\[ \] Use Declarative Table:** The entire menu structure **must** be defined in a single Lua table.
2.  **\[ \] Use `crsf_helper.lua`:** The provided `crsf_helper.lua` library **must** be included and used. Do not attempt to re-implement its logic.
3.  **\[ \] Use `require()`:** The main script **must** load the helper using `require('crsf_helper')`.
4.  **\[ \] Use Callbacks:** All menu interactions **must** be handled via callback functions assigned in the menu definition table. The main script must not ontain a `crsf:get_menu_event` loop.
5.  **\[ \] Return `helper.register_menu()`:** The main script **must** conclude by returning the result of the `crsf_helper.register_menu()` function, passing its menu definition table as the argument.
6.  **\[ \] Follow Parameter Syntax:** All parameter types (`NUMBER`, `SELECTION`, etc.) **must** use the exact property names and data types defined in section 4.2.
7. **\[ \] Prevent Garbage Collection:** The helper library **must** maintain a reference to every CRSF object it creates.
