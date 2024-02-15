-- mission editing demo lua script.
-- by Buzz 2020
-- luacheck: only 0

current_pos = nil
home = 0
a = {}
demostage = 0 
eventcounter = 0

function update () -- periodic function that will be called
    current_pos = ahrs:get_location()

  -- adds new/extra mission item at the end by copying the last one and modifying it
  -- get number of last mission item
  wp_num = mission:num_commands()-1
  --get last item from mission
  m = mission:get_item(wp_num) 
  -- get first item from mission
  m1 = mission:get_item(1) 

  if wp_num >  0 then
    gcs:send_text(0, string.format("LUA - Please clear misn to continue demo. size:%d",wp_num+1))
    return update, 1000
  end

  -- no mission, just home at [0] means user has cleared any mission in the system, and this demo is clear to write something new.
  -- it's not required that the mission be empty before we do things, but this script is multi-stage demo so its conveneient
  if ( mission:num_commands() == 1) then
      if demostage == 0  then
            demostage = 1
            gcs:send_text(0, string.format("LUA demo stage 1 starting"))
            return stage1, 1000
      end 
      if demostage == 2  then
            demostage = 3
            gcs:send_text(0, string.format("LUA demo stage 3 starting"))
            return stage3, 1000
      end
      if demostage == 4  then
            demostage = 5
            gcs:send_text(0, string.format("LUA demo stage 5 starting"))
            return stage5, 1000
      end
      if demostage == 6  then
            demostage = 7
            gcs:send_text(0, string.format("LUA demo stage 7 starting"))
            return stage7, 1000
      end
      if demostage == 8  then
            demostage = 9
            gcs:send_text(0, string.format("LUA MISSION demo all COMPLETED."))
            --return update, 1000
      end
  end
  return update, 1000
end

function read_table_from_sd()
    -- Opens a file in read mode
    file = io.open("miss.txt", "r")
    -- sets the default input file as xxxx.txt
    io.input(file)

    -- read whole file, or get empty string
    content = io.read("*all")
    
    if (content == nil) then
        gcs:send_text(0, string.format("file not found, skipping read of miss.txt from sd"))
        return update(), 1000
    end

    local pat = "wp:(%S+)%s+lat:(%S+)%s+lon:(%S+)%s+alt:(%S+)"
    for s1, s2 ,s3,s4 in string.gmatch(content, pat) do
        --s = string.format("wp:%s lat:%s lon:%s alt:%s",s1,s2,s3,s4)
        --gcs:send_text(0, s)
        n1 = tonumber(s1)
        n2 = math.floor(tonumber(s2*10000000))
        n3 = math.floor(tonumber(s3*10000000))
        n4 = tonumber(s4)

        -- use previous item as template...
        m = mission:get_item(mission:num_commands()-1)
        m:command(16) -- 16 = normal WAYPOINT
        m:x(n2)
        m:y(n3)
        m:z(n4)
        -- write as a new item to the end of the list.
        mission:set_item(mission:num_commands(),m)
    end

    gcs:send_text(0, '...loaded file from SD')

    -- closes the open file
    io.close(file)

  return update(), 1000
end

function stage1 ()
  -- demo stage 1 implementation.
  if (demostage == 1 ) and ( mission:num_commands() == 1 )  then
    demostage = 2
    return read_table_from_sd(), 1000
  end

end

function stage3 ()
  -- demo stage 3 implementation.
  --get number of 'last' mission item
  wp_num = mission:num_commands()-1
  -- get HOME item from mission as the 'reference' for future items
  m1 = mission:get_item(0) 
  --get last item from mission
  m = mission:get_item(wp_num) 
  if (demostage == 3 ) then
      -- demo stage 3 starts by writing 10 do-JUMPS over 10 seconds, just for fun
      if mission:num_commands() < 10  then
          m:command(177) -- 177 =  DO_JUMP
          m:param1(m:param1()+1) -- some increments for fun/demo
          m:param2(m:param2()+1)
          gcs:send_text(0, string.format("LUA new miss-item DO_JUMP %d ", wp_num+1))
          mission:set_item(mission:num_commands(),m)
          return stage3, 100 -- stay in stage 3 for now
      end

      -- change copy of last item slightly, for giggles and demo.
      -- This is reading a copy of whatever is currently the last item in the mission, do_jump
      --  and changing/ensuring its type is a 'normal' waypoint, and setting its lat/long/alt
      --  to data we earlier took from HOME, adding an offset and then writing it 
      --  as a NEW mission item at the end 
      if mission:num_commands() == 10 then
          m:command(16) -- 16 = normal WAYPOINT
          m:x(m1:x()+200)
          m:y(m1:y()+200)
          m:z(m1:z()+1)
          gcs:send_text(0, string.format("LUA new miss-item WAYPOINT a %d ", wp_num+1))
          mission:set_item(mission:num_commands(),m)
          return stage3, 100 -- stay in stage 3 for now
      end

      -- change copy of last item slightly, for giggles, and append as a new item
      if (mission:num_commands() > 10) and (mission:num_commands() < 20) then
          m:command(16) -- 16 = normal WAYPOINT
          m:x(m:x()+200)
          m:y(m:y()+200)
          m:z(m:z()+1)
          gcs:send_text(0, string.format("LUA new miss-item WAYPOINT b %d ", wp_num+1))
          mission:set_item(mission:num_commands(),m)
          return stage3, 100 -- stay in stage 3 for now
      end

      -- change copy of last item slightly, for giggles.
      if (mission:num_commands() >= 20) and (mission:num_commands() < 30) then
          m:command(16) -- 16 = normal WAYPOINT
          m:x(m:x()+200)
          m:y(m:y()+200)
          m:z(m:z()+1)
          gcs:send_text(0, string.format("LUA new miss-item WAYPOINT c %d ", wp_num+1))
          mission:set_item(mission:num_commands(),m)
          return stage3, 100 -- stay in stage 3 for now
      end
      -- move on at end of this dempo stage
      if (mission:num_commands() >= 30) then
          gcs:send_text(0, string.format("LUA DEMO stage 3 done. ", wp_num+1))
          demostage = 4
          return update, 100 -- drop to next stage via an update() call
      end
  end

end

function stage5 ()
  -- demo stage 5 implementation for when there's only one wp , HOME, in th system
  if (demostage == 5 ) then 
      -- when no mission, uses home as reference point, otherwise its the 'last item'
      m = mission:get_item(mission:num_commands()-1)
      m:x(m:x())
      m:y(m:y()-400)
      m:z(m:z())
      mission:set_item(1,m)
      gcs:send_text(0, string.format("LUA mode 5 single wp nudge %d",eventcounter))
      eventcounter = eventcounter+1
      if eventcounter > 50 then
          demostage = 6
          eventcounter = 0
          gcs:send_text(0, string.format("LUA DEMO stage 5 done. "))
          return update, 100 -- drop to next stage via an update() call
      end
      return stage5, 100 -- stay in stage 3 for now
  end

end

function stage7 ()
  -- demo stage 5 implementation for when there's more than wp in the system
  if (demostage == 7 ) then --and (mission:num_commands() >= 3) and (mission:num_commands() < 50) then
      -- fiurst time in , there's no mission, lets throw a few wps in to play with later..
      -- change copy of last item slightly, for giggles, and append as a new item
      if (mission:num_commands() == 1)  then
          for x = 1, 10 do 
              m:command(16) -- 16 = normal WAYPOINT
              m:x(m:x()+math.random(-10000,10000)) -- add something random 
              m:y(m:y()+math.random(-10000,10000)) 
              m:z(m:z()+1)
              gcs:send_text(0, string.format("LUA stage 7 making 10 new nearby random wp's %d ", wp_num+1))
              mission:set_item(mission:num_commands(),m)
          end
          gcs:send_text(0, string.format("LUA scattering complete. %d ", wp_num+1))
          return stage7, 100 -- stay in stage 3 for now
      end
 
    -- things that are further away from this one than distance X..
    m1 = mission:get_item(1)

    -- leave item 0 alone, always
    for x = 1, mission:num_commands()-1 do 

        mitem = mission:get_item(x)

        -- look at each mission item above 1, and get the distance from it to the copter.
        local target = Location()
        target:lat(mitem:x())
        target:lng(mitem:y())
        local cur_d = current_pos:get_distance(target)
        
        if cur_d > 100 then
            if mitem:x() > m1:x() then
                 mitem:x(mitem:x()+400)           
            end
            if mitem:x() < m1:x() then
                 mitem:x(mitem:x()-400)  
            end
            if mitem:y() > m1:y() then
                 mitem:y(mitem:y()+400)        
            end
            if mitem:y() < m1:y() then
                 mitem:y(mitem:y()-400)         
            end
        end

        -- write as a new item to the end of the list.
        mission:set_item(x,mitem)
    end
    gcs:send_text(0, string.format("LUA mode 7 scattering existing wp's.. %d",eventcounter))
  end

  -- do it 50 times then consider it done
  eventcounter = eventcounter+1
  if eventcounter > 50 then
          demostage = 8
          eventcounter = 0
          gcs:send_text(0, string.format("LUA DEMO stage 7 done. "))
          return update, 100 -- drop to next stage via an update() call
  end

  return stage7, 500
end

function wait_for_home()
  current_pos = ahrs:get_location()
  if current_pos == nil then
     return wait_for_home, 1000
  end

  home = ahrs:get_home()
  if home == nil then
     return wait_for_home, 1000
  end
  if home:lat() == 0 then
     return wait_for_home, 1000
  end

  gcs:send_text(0, string.format("LUA MISSION Waiting for Home."))

  return update, 1000
end

function delayed_boot()
    gcs:send_text(0, string.format("LUA MISSION DEMO START"))
    return wait_for_home, 1000
end
return delayed_boot, 5000 

