local THROTTLE_LEFT_id = 73
local THROTTLE_RIGHT_id = 74
local THROTTLE_LEFT_COMMAND = SRV_Channels:find_channel(73)
local THROTTLE_RIGHT_COMMAND = SRV_Channels:find_channel(74)
local CONTROL_OUTPUT_THROTTLE = 3
local CONTROL_OUTPUT_YAW = 4
local step = 0
local wl = 1000
local wr = 1000
local wb1 = 1000
local wb2 = 1000
local steps = 30

function update() -- this is the loop which periodically runs
  

  -- if not arming:is_armed() then
  --  -- param:set('SCR_USER2',1)
  --   -- param:set('SERVO1_FUNCTION',74)
  --   -- param:set('SERVO4_FUNCTION',74)
  --   -- param:set('SERVO5_FUNCTION',35)
  --   -- param:set('SERVO2_FUNCTION',73)
  --   -- param:set('SERVO3_FUNCTION',73)
  --   -- param:set('SERVO6_FUNCTION',36)
    
  --   -- SRV_Channels:set_output_pwm_chan_timeout(1, 1000,5000) 
  --   -- SRV_Channels:set_output_pwm_chan_timeout(2, 1000,5000) 
  --   -- SRV_Channels:set_output_pwm_chan_timeout(0,1000,5000)
  --   -- SRV_Channels:set_output_pwm_chan_timeout(3,1000,5000)
  --   -- SRV_Channels:set_output_pwm_chan_timeout(4,1000,5000)
  --   -- SRV_Channels:set_output_pwm_chan_timeout(5,1000,5000)
  --   return update, 4000
  -- end

  -- if arming:is_armed() and param:get('SCR_USER1')== 1 then

    
  --   gcs:send_text(4, "AUTO_TUNNING is running")

  --   if step==0 then
  --     param:set('SCR_USER2',1)
  --     local myGPS = gps:primary_sensor()
  --     math.randomseed(gps:time_week_ms(myGPS):toint()) 
  --     wl = math.random(1000, 1500)
  --     wr = math.random(1000, 1500)
  --     wb1 = math.random(1000, 1500)
  --     wb2 = math.random(1000, 1500)
  --   elseif step>0 and step<steps then
  --     wl = math.random(1000, 1500)
  --     wr = math.random(1000, 1500)
  --     wb1 = math.random(1000, 1500)
  --     wb2 = math.random(1000, 1500)
  --   elseif step==steps then
  --     param:set('SCR_USER1',0)
  --     param:set('SCR_USER2',0)
  --     gcs:send_text(4, "AUTO_TUNNING is OVER! SEE DATA FILE")
  --     wr = 1000
  --     wl = 1000
  --     wb1 = 1000
  --     wb2 = 1000
  --   end

  --   --SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_LEFT_COMMAND, wl,5000) 
  --   --SRV_Channels:set_output_pwm_chan_timeout(THROTTLE_RIGHT_COMMAND,wr,5000)

  --   SRV_Channels:set_output_pwm_chan_timeout(1, wr,5000) 
  --   SRV_Channels:set_output_pwm_chan_timeout(2, wr,5000) 
  --   SRV_Channels:set_output_pwm_chan_timeout(0,wl,5000)
  --   SRV_Channels:set_output_pwm_chan_timeout(3,wl,5000)
  --   SRV_Channels:set_output_pwm_chan_timeout(4,wb1,5000)
  --   SRV_Channels:set_output_pwm_chan_timeout(5,wb2,5000)


  --   step = step + 1

    
  --   return update, 4000

  -- end


  
return update, 1000 -- reschedules the loop at 5Hz

end


return update(), 1000 -- run immediately before starting to reschedule