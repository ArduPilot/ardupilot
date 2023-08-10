# Drone Ping-Pong Code

This script makes the drone go forward and backward a defined distance and number of times.


## Stages
- 0: Change to Guided mode
- 1: Takeoff to the height defined by `takeoff_alt`
- 2: Wait until reaching the takeoff altitude
- 3: Go forward to the defined distance
- 4: Go back to the initial position
- 5: Change to Land mode

## Variables:
- `takeoff_alt`: Takeoff height (m)
- `copter_guided_mode_num`: Guided mode number
- `copter_land_mode_num`: Land mode number
- `stage`: current stage
- `count`: Number of times the drone has gone forward
- `max_count`: Maximum number of times the drone should go forward
- `ping_pong_distance`: Distance up to which the drone should go forward (m)
- `vel`: Drone velocity (m/s)

## Understand the code:
First, there is a comment explaining what the code is about and how it works. Then, the local variables that will be used are declared.
 The `copter_guided_mode_num` and `copter_land_mode_num `are standard numbers, defined by ArduPilot, for the Guided and Land flight modes, respectively. The other variables are user-defined settings according to the desired behavior, as indicated in the comments in front of each one.

```lua
-- This script makes the drone go forward and backward at a defined distance and number of times.
-- The stages are:
-- 0) Change to Guided mode
-- 1) Takeoff to the height defined by takeoff_alt
-- 2) Wait until reaching the takeoff altitude
-- 3) Go forward to the defined distance
-- 4) Go back to the initial position
-- 5) Change to Land mode


local takeoff_alt = 3         -- Takeoff height
local copter_guided_mode_num = 4
local copter_land_mode_num = 9
local stage = 0
local count = 0               -- Number of times the drone has gone forward
local max_count = 2           -- Maximum number of times the drone should go forward
local ping_pong_distance = 10 -- Distance up to which the drone should go forward (m)
local vel = 1    

```
Next, there's the main function of the code, the `update()` function, which will be called once the code is started (since it wasn't indicated a waiting time at the last line of the code) and its `return` indicates that this same fucntion will be called again 100ms after finishing its executuion.



```lua
function update()
    -- [Code]
    return update(), 100
end

return update()
```

In the presented if, the code is waiting for the drone to be armed to change to the next stage. The is_armed() function from the arming library is used to check if the drone is armed or not. The send_text() function from the gcs library is used to send a message ("Arming") with severity 6 (information) to the GCS (Ground Control Station). Thus, while the drone is not armed, it remains in stage 0 of the code.

``` lua
if not arming:is_armed() then
    stage = 0
    gcs:send_text(6, "Arming")
```
If the drone is armed, it moves to the else section, which presents a behavior for each stage.
In `stage 0`, the drone flight mode is changed to GUIDED MODE. To do this, it's used the function `set_mode()` of `vehicle` library, which receives the desired flight mode number(copter_guided_mode_num, defined in the local variables). When it verifies the drone has switched to the desired flight mode, it moves to the next stage. 


``` lua
else
    if stage == 0 then
        if vehicle:set_mode(copter_guided_mode_num) then 
            stage = stage + 1
        end
``` 
in stage 1, it's used the `start_takeoff()` function from the `vehicle` library for the drone to take off to a height defined by the variable `takeoff_alt`.

``` lua
elseif stage == 1 then
    gcs:send_text(6, "Taking off")
    if vehicle:start_takeoff(takeoff_alt) then
        stage = stage + 1
    end
``` 

In stage 2, it's used the `ahrs:get_home()` function to get the drone's takeoff location, and the `ahrs:get_position()` function to get the drone's current position. In line 4, it checks that the obtained values are not null. Then, the `home:get_distance_NED()` function stores in `vec_from_home` a 3D vector, starting from `curr_loc` and ending at `home`. In line 7, the code sends to the GCS the value contained in the z-coordinate of `vec_from_home`, i.e., the current altitude of the drone (multiplied by -1 since the vector points towards home, which is at a lower altitude than the current position).

When the difference between `takeoff_alt` and `vec_from_home:z()` is less than 1, indicating that the drone has reached the takeoff altitude, it moves to the next stage (`math.abs()` is used to get the absolute value, and we performed an addition instead of subtraction because the z component is negative).

``` lua
elseif stage == 2 then
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_position()
    if home and curr_loc then 
        local vec_from_home = home:get_distance_NED(curr_loc)
        gcs:send_text(6, "Altitude above home: " .. tostring(math.floor(-vec_from_home:z())))
        if math.abs(takeoff_alt + vec_from_home:z()) < 1 then
            stage = stage + 1
        end
    end
``` 

In `stage3`, first, the drone checks if it has already performed all the requested loops specified in `max_count`. If so, it switches to `stage5`. If not, it executes the commands of `stage3`.

First, it creates a 3D vector `target_vel` to store the desired velocity. Then, it sets the value of each of the components of this created vector (0 for y and z, and `vel` for x).

Then, it uses the `vehicle:set_target_velocity_NED(target_vel)` function to set the drone's velocity as that of `target_vel`. If it encounters any issues, it sends a warning message.


``` lua
elseif (stage == 3) then -- Stage 3: Moving Forward
    -- If the maximum number of times is exceeded, move to stage 5
    if (count >= max_count) then
        stage = stage + 2
    end

    -- Calculate velocity vector
    local target_vel = Vector3f()
    target_vel:x(vel)
    target_vel:y(0)
    target_vel:z(0)

    -- Send velocity request
    if not (vehicle:set_target_velocity_NED(target_vel)) then
        gcs:send_text(6, "Failed to execute velocity command")
    end

``` 

After that, it uses the same strategy as shown before to calculate the distance vector from the takeoff location to verify the distance `x` traveled. When the difference between the distance `x` that the drone traveled and the `ping_pong_distance` is less than 1, it increments the `count` and moves to the next stage.
``` lua

    -- checking if reached stop point
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_position()
    if home and curr_loc then 
        local vec_from_home = home:get_distance_NED(curr_loc)
        gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
        if(math.abs(ping_pong_distance - vec_from_home:x()) < 1) then
            count = count + 1
            stage = stage + 1
        end
    end
end

``` 
In `Stage4`, the velocity in `x` is multiplied by -1 for the drone to fly in the opposite direction, returning to the takeoff position. When the difference between the drone's position `x` and the takeoff position's `x` is less than 1, it goes back to `Stage3` to start the forward movement again (or not, if the `count` has reached `max_count`).

``` lua
elseif (stage == 4) then -- Stage 4: Moving Back
    -- calculate velocity vector
    local target_vel = Vector3f()
    target_vel:x(-vel)
    target_vel:y(0)
    target_vel:z(0)

    -- send velocity request
    if not (vehicle:set_target_velocity_NED(target_vel)) then
        gcs:send_text(6, "Failed to execute velocity command")
    end

    -- checking if reached stop point
    local home = ahrs:get_home()
    local curr_loc = ahrs:get_position()
    if home and curr_loc then 
        local vec_from_home = home:get_distance_NED(curr_loc)
        gcs:send_text(6, "Distance from home: " .. tostring(math.floor(vec_from_home:x())))
        if(math.abs(vec_from_home:x()) < 1) then
            stage = stage - 1
        end
    end
end

``` 
In `stage5`, the drone simply changes to the Land flight mode and lands, indicating the completion of the code through a message.
``` lua
elseif (stage == 5) then -- Stage 5: Change to LAND mode
    vehicle:set_mode(copter_rtl_mode_num)
    stage = stage + 1
    gcs:send_text(6, "Finished pingpong, switching to LAND")
end

``` 

