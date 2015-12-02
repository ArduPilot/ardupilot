--[[

  KAP UAV Exposure Control Script v3.1
  -- Released under GPL by waterwingz and wayback/peabody
  http://chdk.wikia.com/wiki/KAP_%26_UAV_Exposure_Control_Script

@title KAP UAV 3.1
@param     i Shot Interval (sec)
  @default i 15
  @range   i 2 120
@param     s Total Shots (0=infinite)
  @default s 0
  @range   s 0 10000
@param     j Power off when done?
  @default j 0
  @range   j 0 1
@param     e Exposure Comp (stops)
  @default e 6
  @values  e -2.0 -1.66 -1.33 -1.0 -0.66 -0.33 0.0 0.33 0.66 1.00 1.33 1.66 2.00
@param     d Start Delay Time (sec)
  @default d 0
  @range   d 0 10000
@param     y Tv Min (sec)
  @default y 0
  @values  y None 1/60 1/100 1/200 1/400 1/640
@param     t Target Tv (sec)
  @default t 5
  @values  t 1/100 1/200 1/400 1/640 1/800 1/1000 1/1250 1/1600 1/2000
@param     x Tv Max (sec)
  @default x 3
  @values  x 1/1000 1/1250 1/1600 1/2000 1/5000 1/10000
@param     f Av Low(f-stop)
  @default f 4
  @values  f 1.8 2.0 2.2 2.6 2.8 3.2 3.5 4.0 4.5 5.0 5.6 6.3 7.1 8.0
@param     a Av Target (f-stop)
  @default a 7
  @values  a 1.8 2.0 2.2 2.6 2.8 3.2 3.5 4.0 4.5 5.0 5.6 6.3 7.1 8.0
@param     m Av Max (f-stop)
  @default m 13
  @values  m 1.8 2.0 2.2 2.6 2.8 3.2 3.5 4.0 4.5 5.0 5.6 6.3 7.1 8.0
@param     p ISO Min
  @default p 1
  @values  p 80 100 200 400 800 1250 1600
@param     q ISO Max1
  @default q 2
  @values  q 100 200 400 800 1250 1600
@param     r ISO Max2
  @default r 3
  @values  r 100 200 400 800 1250 1600
@param     n Allow use of ND filter?
  @default n 1
  @values  n No Yes
@param     z Zoom position
  @default z 0
  @values  z Off 0% 10% 20% 30% 40% 50% 60% 70% 80% 90% 100%
@param     c Focus @ Infinity Mode
  @default c 0
  @values  c None @Shot AFL MF
@param     v Video Interleave (shots)
  @default v 0
  @values  v Off 1 5 10 25 50 100
@param     w Video Duration (sec)
  @default w 10
  @range   w 5 300
@param     u USB Shot Control?
  @default u 0
  @values  u None On/Off OneShot PWM
@param     b Backlight Off?
  @default b 0
  @range   b 0 1
@param     l Logging
  @default l 3
  @values  l Off Screen SDCard Both
--]]

    props=require("propcase")
    capmode=require("capmode")

-- convert user parameter to usable variable names and values
    tv_table       = { -320, 576, 640, 736, 832, 896, 928, 960, 992, 1024, 1056, 1180, 1276}
    tv96target     = tv_table[t+3]
    tv96max        = tv_table[x+8]
    tv96min        = tv_table[y+1]
    sv_table       = { 381, 411, 507, 603, 699, 761, 795 }
    sv96min        = sv_table[p+1]
    sv96max1       = sv_table[q+2]
    sv96max2       = sv_table[r+2]
    av_table       = { 171, 192, 218, 265, 285, 322, 347, 384, 417, 446, 477, 510, 543, 576 }
    av96target     = av_table[a+1]
    av96minimum    = av_table[f+1]
    av96max        = av_table[m+1]
    ec96adjust     = (e - 6)*32
    video_table    = { 0, 1, 5, 10, 25, 50, 100 }
    video_mode     = video_table[v+1]
    video_duration = w
    interval =       i*1000
    max_shots =      s
    poff_if_done =   j
    start_delay =    d
    backlight =      b
    log_mode=        l
    focus_mode =     c
    usb_mode =       u
    if ( z==0 ) then zoom_setpoint = nil else zoom_setpoint = (z-1)*10 end

-- initial configuration values
    nd96offset=3*96                   -- ND filter's number of equivalent f-stops  (f * 96)
    infx = 50000                      -- focus lock distance in mm (approximately 55 yards)
    shot_count = 0                    -- shot counter
    blite_timer = 300                 -- backlight off delay in 100mSec increments
    old_console_timeout = get_config_value( 297 )
    shot_request = false              -- pwm mode flag to request a shot be taken

-- check camera Av configuration ( variable aperture and/or ND filter )
    if n==0 then                      -- use of nd filter allowed?
        if get_nd_present()==1 then   -- check for ND filter only
            Av_mode = 0               -- 0 = ND disabled and no iris available
        else
            Av_mode = 1               -- 1 = ND disabled and iris available
        end
    else
        Av_mode = get_nd_present()+1  -- 1 = iris only , 2=ND filter only, 3= both ND & iris
    end

function printf(...)
    if ( log_mode == 0) then return end
    local str=string.format(...)
    if (( log_mode == 1) or (log_mode == 3)) then print(str) end
    if ( log_mode > 1 ) then
    local logname="A/KAP.log"
        log=io.open(logname,"a")
        log:write(os.date("%Y%b%d %X ")..string.format(...),"\n")
        log:close()
    end
end

tv_ref = {    -- note : tv_ref values set 1/2 way between shutter speed values
-608, -560, -528, -496, -464, -432, -400, -368, -336, -304,
-272, -240, -208, -176, -144, -112,  -80,  -48,  -16,   16,
  48,   80,  112,  144,  176,  208,  240,  272,  304,  336,
 368,  400,  432,  464,  496,  528,  560,  592,  624,  656,
 688,  720,  752,  784,  816,  848,  880,  912,  944,  976,
1008, 1040, 1072, 1096, 1129, 1169, 1192, 1225, 1265, 1376  }

tv_str = {
       ">64",
        "64",    "50",    "40",    "32",    "25",    "20",    "16",    "12",     "10",   "8.0",
       "6.0",   "5.0",   "4.0",   "3.2",   "2.5",   "2.0",   "1.6",   "1.3",    "1.0",   "0.8",
       "0.6",   "0.5",   "0.4",   "0.3",   "1/4",   "1/5",   "1/6",   "1/8",   "1/10",  "1/13",
      "1/15",  "1/20",  "1/25",  "1/30",  "1/40",  "1/50",  "1/60",  "1/80",  "1/100", "1/125",
     "1/160", "1/200", "1/250", "1/320", "1/400", "1/500", "1/640", "1/800", "1/1000","1/1250",
    "1/1600","1/2000","1/2500","1/3200","1/4000","1/5000","1/6400","1/8000","1/10000","hi"  }

function print_tv(val)
    if ( val == nil ) then return("-") end
    local i = 1
    while (i <= #tv_ref) and (val > tv_ref[i]) do i=i+1 end
    return tv_str[i]
end

av_ref = {     160, 176,   208,  243,  275,  304,  336,  368,  400,  432,  464,  480,  496,  512,  544,  592,  624,   656,   688,   720,  752,   784 }
av_str = {"n/a","1.8", "2.0","2.2","2.6","2.8","3.2","3.5","4.0","4.5","5.0","5.6","5.9","6.3","7.1","8.0","9.0","10.0","11.0","13.0","14.0","16.0","hi"}

function print_av(val)
    if ( val == nil ) then return("-") end
    local i = 1
    while (i <= #av_ref) and (val > av_ref[i]) do i=i+1 end
    return av_str[i]
end

sv_ref = {   370,  397,  424,  456,  492,  523,  555,  588,  619,  651,  684,  731,   779,   843,   907   }
sv_str = {"n/a","80","100","120","160","200","250","320","400","500","640","800","1250","1600","3200","hi"}

function print_sv(val)
    if ( val == nil ) then return("-") end
    local i = 1
    while (i <= #sv_ref) and (val > sv_ref[i]) do i=i+1 end
    return sv_str[i]
end

 function pline(message, line)     -- print line function
    fg = 258 bg=259

 end

-- switch between shooting and playback modes
function switch_mode( m )
   if ( m == 1 ) then
      if ( get_mode() == false ) then
         set_record(1)                                                  -- switch to shooting mode
         while ( get_mode() == false ) do
            sleep(100)
         end
         sleep(1000)
      end
   else
      if ( get_mode() == true ) then
         set_record(0)                                                  -- switch to playback mode
         while ( get_mode() == true ) do
            sleep(100)
         end
         sleep(1000)
       end
   end
end

-- focus lock and unlock
function lock_focus()
    if (focus_mode > 1) then                                            -- focus mode requested ?
        if     ( focus_mode == 2 ) then                                 -- method 1 :  set_aflock() command enables MF
            if (chdk_version==120) then
                set_aflock(1)
                set_prop(props.AF_LOCK,1)
            else
                set_aflock(1)
            end
            if (get_prop(props.AF_LOCK) == 1) then printf(" AFL enabled") else printf(" AFL failed ***") end
        else                                                            -- mf mode requested
            if (chdk_version==120) then                                 -- CHDK 1.2.0 : call event proc or levents to try and enable MF mode
                if call_event_proc("SS.Create") ~= -1 then
                    if call_event_proc("SS.MFOn") == -1 then
                            call_event_proc("PT_MFOn")
                    end
                elseif call_event_proc("RegisterShootSeqEvent") ~= -1 then
                    if call_event_proc("PT_MFOn") == -1 then
                        call_event_proc("MFOn")
                    end
                end
                if (get_prop(props.FOCUS_MODE) == 0 ) then              -- MF not set - try levent PressSw1AndMF
                    post_levent_for_npt("PressSw1AndMF")
                    sleep(500)
                end
            elseif (chdk_version >= 130) then                           -- CHDK 1.3.0 : set_mf()
                if ( set_mf(1) == 0 ) then set_aflock(1) end            --    as a fall back, try setting AFL is set_mf fails
            end
            if (get_prop(props.FOCUS_MODE) == 1) then printf(" MF enabled") else printf(" MF enable failed ***") end
        end
        sleep(1000)
        set_focus(infx)
        sleep(1000)
    end
end

function unlock_focus()
    if (focus_mode > 1) then                                            -- focus mode requested ?
        if (focus_mode == 2 ) then                                      -- method 1 : AFL
            if (chdk_version==120) then
                set_aflock(0)
                set_prop(props.AF_LOCK,0)
            else
                set_aflock(0)
            end
            if (get_prop(props.AF_LOCK) == 0) then printf(" AFL unlocked") else printf(" AFL unlock failed") end
         else                                                           -- mf mode requested
             if (chdk_version==120) then                                -- CHDK 1.2.0 : call event proc or levents to try and enable MF mode
                if call_event_proc("SS.Create") ~= -1 then
                    if call_event_proc("SS.MFOff") == -1 then
                        call_event_proc("PT_MFOff")
                    end
                elseif call_event_proc("RegisterShootSeqEvent") ~= -1 then
                    if call_event_proc("PT_MFOff") == -1 then
                        call_event_proc("MFOff")
                    end
                end
                if (get_prop(props.FOCUS_MODE) == 1 ) then              -- MF not reset - try levent PressSw1AndMF
                    post_levent_for_npt("PressSw1AndMF")
                    sleep(500)
                end
            elseif (chdk_version >= 130) then                           -- CHDK 1.3.0 : set_mf()
                if ( set_mf(0) == 0 ) then set_aflock(0) end            --    fall back so reset AFL is set_mf fails
            end
            if (get_prop(props.FOCUS_MODE) == 0) then printf(" MF disabled") else printf(" MF disable failed") end
        end
        sleep(100)
    end
end

-- zoom position
function update_zoom(zpos)
    local count = 0
    if(zpos ~= nil) then
        zstep=((get_zoom_steps()-1)*zpos)/100
        printf("setting zoom to "..zpos.." percent step="..zstep)
        sleep(200)
        set_zoom(zstep)
        sleep(2000)
        press("shoot_half")
        repeat
            sleep(100)
            count = count + 1
        until (get_shooting() == true ) or (count > 40 )   
        release("shoot_half")
    end
end

-- restore camera settings on shutdown
function restore()
    set_config_value(121,0)                                             -- USB remote disable
    set_config_value(297,old_console_timeout)                           -- restore console timeout value
    if (backlight==1) then set_lcd_display(1) end                       -- display on
    unlock_focus()
    if( zoom_setpoint ~= nil ) then update_zoom(0) end
    if( shot_count >= max_shots) and ( max_shots > 1) then
       if ( poff_if_done == 1 ) then                                    -- did script ending because # of shots done ?
          printf("power off - shot count at limit")                     -- complete power down
          sleep(2000)
          post_levent_to_ui('PressPowerButton')
       else
          set_record(0) end                                             -- just retract lens
        end
end

-- Video mode
function check_video(shot)
    local capture_mode
    if ((video_mode>0) and(shot>0) and (shot%video_mode == 0)) then
        unlock_focus()
        printf("Video mode started. Button:"..tostring(video_button))
        if( video_button ) then
            click "video"
        else
            capture_mode=capmode.get()
            capmode.set('VIDEO_STD')
            press("shoot_full")
            sleep(300)
            release("shoot_full")
        end
        local end_second = get_day_seconds() + video_duration
        repeat
            wait_click(500)
        until (is_key("menu")) or (get_day_seconds() > end_second)
        if( video_button ) then
            click "video"
        else
            press("shoot_full")
            sleep(300)
            release("shoot_full")
            capmode.set(capture_mode)
        end
        printf("Video mode finished.")
        sleep(1000)
        lock_focus()
        return(true)
    else
        return(false)
    end
end

-- PWM USB pulse functions

    function ch1up()
        printf(" * usb pulse = ch1up")
        shot_request = true
    end

    function ch1mid()
        printf(" * usb pulse = ch1mid")
        if ( get_mode() == false ) then
            switch_mode(1)
            lock_focus()
        end
    end

    function ch1down()
        printf(" * usb pulse = ch1down")
        switch_mode(0)
    end

    function ch2up()
        printf(" * usb pulse = ch2up")
        update_zoom(100)
    end

    function ch2mid()
        printf(" * usb pulse = ch2mid")
        if ( zoom_setpoint ~= nil ) then update_zoom(zoom_setpoint) else update_zoom(50) end
    end

    function ch2down()
        printf(" * usb pulse = ch2down")
        update_zoom(0)
    end

    function pwm_mode(pulse_width)
    if  pulse_width > 0  then
        if     pulse_width < 5  then ch1up()
        elseif pulse_width < 8  then ch1mid()
        elseif pulse_width < 11 then ch1down()
        elseif pulse_width < 14 then ch2up()
        elseif pulse_width < 17 then ch2mid()
        elseif pulse_width < 20 then ch2down()
        else printf(" * usb pulse width error") end
    end
   end

-- Basic exposure calculation using shutter speed and ISO only
--   called for Tv-only and ND-only cameras (cameras without an iris)
function basic_tv_calc()
    tv96setpoint = tv96target
    av96setpoint = nil
    local min_av = get_prop(props.MIN_AV)
 -- calculate required ISO setting
    sv96setpoint = tv96setpoint + min_av - bv96meter
 -- low ambient light ?
    if (sv96setpoint > sv96max2 ) then                                  -- check if required ISO setting is too high
        sv96setpoint = sv96max2                                         -- clamp at max2 ISO if so
        tv96setpoint = math.max(bv96meter+sv96setpoint-min_av,tv96min)  -- recalculate required shutter speed down to Tv min
 -- high ambient light ?
    elseif (sv96setpoint < sv96min ) then                               -- check if required ISO setting is too low
        sv96setpoint = sv96min                                          -- clamp at minimum ISO setting if so
        tv96setpoint = bv96meter + sv96setpoint - min_av                -- recalculate required shutter speed and hope for the best
    end
end


-- Basic exposure calculation using shutter speed, iris and ISO
--   called for iris-only and "both" cameras (cameras with an iris & ND filter)
function basic_iris_calc()
    tv96setpoint = tv96target
    av96setpoint = av96target
 -- calculate required ISO setting
    sv96setpoint = tv96setpoint + av96setpoint - bv96meter
 -- low ambient light ?
    if (sv96setpoint > sv96max1 ) then                                  -- check if required ISO setting is too high
        sv96setpoint = sv96max1                                         -- clamp at first ISO limit
        av96setpoint = bv96meter + sv96setpoint - tv96setpoint          -- calculate new aperture setting
        if ( av96setpoint < av96min ) then                              -- check if new setting is goes below lowest f-stop
            av96setpoint = av96min                                      -- clamp at lowest f-stop
            sv96setpoint = tv96setpoint + av96setpoint - bv96meter      -- recalculate ISO setting
            if (sv96setpoint > sv96max2 ) then                          -- check if the result is above max2 ISO
                sv96setpoint = sv96max2                                 -- clamp at highest ISO setting if so
                tv96setpoint = math.max(bv96meter+sv96setpoint-av96setpoint,tv96min)  -- recalculate required shutter speed down to tv minimum
            end
        end
 -- high ambient light ?
    elseif (sv96setpoint < sv96min ) then                               -- check if required ISO setting is too low
        sv96setpoint = sv96min                                          -- clamp at minimum ISO setting if so
        tv96setpoint = bv96meter + sv96setpoint - av96setpoint          -- recalculate required shutter speed
        if (tv96setpoint > tv96max ) then                               -- check if shutter speed now too fast
            tv96setpoint = tv96max                                      -- clamp at maximum shutter speed if so
            av96setpoint = bv96meter + sv96setpoint - tv96setpoint      -- calculate new aperture setting
            if ( av96setpoint > av96max ) then                          -- check if new setting is goes above highest f-stop
                av96setpoint = av96max                                  -- clamp at highest f-stop
                tv96setpoint = bv96meter + sv96setpoint - av96setpoint  -- recalculate shutter speed needed and hope for the best
            end
        end
    end
end

-- calculate exposure for cams without adjustable iris or ND filter
function exposure_Tv_only()
    insert_ND_filter = nil
    basic_tv_calc()
end

-- calculate exposure for cams with ND filter only
function exposure_NDfilter()
    insert_ND_filter = false
    basic_tv_calc()
    if (tv96setpoint > tv96max ) then                                   -- check if shutter speed now too fast
        insert_ND_filter = true                                         -- flag the ND filter to be inserted
        bv96meter = bv96meter - nd96offset                              -- adjust meter for ND offset
        basic_tv_calc()                                                 -- start over, but with new meter value
        bv96meter = bv96meter + nd96offset                              -- restore meter for later logging
    end
end

-- calculate exposure for cams with adjustable iris only
function exposure_iris()
    insert_ND_filter = nil
    basic_iris_calc()
end

-- calculate exposure for cams with both adjustable iris and ND filter
function exposure_both()
    insert_ND_filter = false                                            -- NOTE : assume ND filter never used automatically by Canon firmware
    basic_iris_calc()
    if (tv96setpoint > tv96max ) then                                   -- check if shutter speed now too fast
        insert_ND_filter = true                                         -- flag the ND filter to be inserted
        bv96meter = bv96meter - nd96offset                              -- adjust meter for ND offset
        basic_iris_calc()                                               -- start over, but with new meter value
        bv96meter = bv96meter + nd96offset                              -- restore meter for later logging
    end
end

--  ========================== Main Program =================================

set_console_layout(1 ,1, 45, 14 )

printf("KAP 3.1 started - press MENU to exit")
bi=get_buildinfo()
printf("%s %s-%s %s %s %s", bi.version, bi.build_number, bi.build_revision, bi.platform, bi.platsub, bi.build_date)
chdk_version= tonumber(string.sub(bi.build_number,1,1))*100 + tonumber(string.sub(bi.build_number,3,3))*10 + tonumber(string.sub(bi.build_number,5,5))
if ( tonumber(bi.build_revision) > 0 ) then
    build = tonumber(bi.build_revision)
else
    build = tonumber(string.match(bi.build_number,'-(%d+)$'))
end

if ((chdk_version<120) or ((chdk_version==120)and(build<3276)) or ((chdk_version==130)and(build<3383))) then
    printf("CHDK 1.2.0 build 3276 or higher required")
else
    if( props.CONTINUOUS_AF == nil ) then caf=-999 else caf = get_prop(props.CONTINUOUS_AF) end
    if( props.SERVO_AF == nil ) then saf=-999 else saf = get_prop(props.SERVO_AF) end
    cmode = capmode.get_name()
    printf("Mode:%s,Continuous_AF:%d,Servo_AF:%d", cmode,caf,saf)  
    printf(" Tv:"..print_tv(tv96target).." max:"..print_tv(tv96max).." min:"..print_tv(tv96min).." ecomp:"..(ec96adjust/96).."."..(math.abs(ec96adjust*10/96)%10) )
    printf(" Av:"..print_av(av96target).." minAv:"..print_av(av96minimum).." maxAv:"..print_av(av96max) )
    printf(" ISOmin:"..print_sv(sv96min).." ISO1:"..print_sv(sv96max1).." ISO2:"..print_sv(sv96max2) )
    printf(" MF mode:"..focus_mode.."  Video:"..video_mode.." USB:"..usb_mode)
    printf(" AvM:"..Av_mode.." int:"..(interval/1000).." Shts:"..max_shots.." Dly:"..start_delay.." B/L:"..backlight)
    sleep(500)

    if (start_delay > 0 ) then
        printf("entering start delay of ".. start_delay.." seconds")
        sleep( start_delay*1000 )
    end

    -- enable USB remote in USB remote moded
    if (usb_mode > 0 ) then
        set_config_value(121,1) -- make sure USB remote is enabled
        if (get_usb_power(1) == 0) then        -- can we start ?
            printf("waiting on USB signal")
            repeat wait_click(20) until ((get_usb_power(1) == 1) or ( is_key("menu")))
        else  sleep(1000) end
        printf("USB signal received")
    end

    -- switch to shooting mode
        switch_mode(1)

    -- set zoom position
        update_zoom(zoom_setpoint)
    -- lock focus at infinity
        lock_focus()
    -- disable flash and AF assist lamp
        set_prop(props.FLASH_MODE, 2)     -- flash off
        set_prop(props.AF_ASSIST_BEAM,0)  -- AF assist off if supported for this camera
        set_config_value( 297, 60)       -- set console timeout to 60 seconds

    if (usb_mode > 2 ) then repeat until (get_usb_power(2) == 0 ) end  -- flush pulse buffer
    next_shot_time = get_tick_count()
    script_exit = false
    if( get_video_button() == 1) then video_button = true else video_button = false end
    set_console_layout(2 ,0, 45, 4 )
    repeat

        if(    ( (usb_mode < 2 )  and ( next_shot_time <= get_tick_count() ) )
            or ( (usb_mode == 2 ) and (get_usb_power(2) > 0 ) )
            or ( (usb_mode == 3 ) and (shot_request == true ) ) ) then

            -- time to insert a video sequence ?
            if( check_video(shot_count) == true) then next_shot_time = get_tick_count() end

            -- intervalometer timing
            next_shot_time = next_shot_time + interval

            -- set focus at infinity ? (maybe redundant for AFL & MF mode but makes sure its set right)
            if (focus_mode > 0) then
                set_focus(infx)
                sleep(100)
            end

            -- check exposure
            local count = 0
            local timeout = false
            press("shoot_half")
            repeat
                sleep(50)
                count = count + 1
                if (count > 40 ) then timeout = true end
            until (get_shooting() == true ) or (timeout == true)

            -- shoot in auto mode if meter reading invalid,  else calculate new desired exposure
            if ( timeout == true ) then
                release("shoot_half")
                repeat sleep(50)  until get_shooting() == false
                shoot()             -- shoot in Canon auto mode if we don't have a valid meter reading
                shot_count = shot_count + 1
                printf(string.format('IMG_%04d.JPG',get_exp_count()).." : shot in auto mode, meter reading invalid")
            else
                -- get meter reading values (and add in exposure compensation)
                bv96raw=get_bv96()
                bv96meter=bv96raw-ec96adjust
                tv96meter=get_tv96()
                av96meter=get_av96()
                sv96meter=get_sv96()

                -- set minimum Av to larger of user input or current min for zoom setting
                av96min= math.max(av96minimum,get_prop(props.MIN_AV))
                if (av96target < av96min) then av96target = av96min end

                -- calculate required setting for current ambient light conditions
                if       (Av_mode == 1) then exposure_iris()
                elseif   (Av_mode == 2) then exposure_NDfilter()
                elseif   (Av_mode == 3) then exposure_both()
                else                         exposure_Tv_only()
                end

                -- set up all exposure overrides
                set_tv96_direct(tv96setpoint)
                set_sv96(sv96setpoint)
                if( av96setpoint ~= nil) then set_av96_direct(av96setpoint) end

                if(Av_mode > 1) and (insert_ND_filter == true) then     -- ND filter available and needed?
                    set_nd_filter(1)                         -- activate the ND filter
                    nd_string="NDin"
                else
                    set_nd_filter(2)                         -- make sure the ND filter does not activate
                    nd_string="NDout"
                end

                -- and finally shoot the image
                press("shoot_full_only")
                sleep(100)
                release("shoot_full")
                repeat sleep(50)  until get_shooting() == false
                shot_count = shot_count + 1

                -- update shooting statistic and log as required
                shot_focus=get_focus()
                if(shot_focus ~= -1) and (shot_focus < 20000) then
                    focus_string=" foc:"..(shot_focus/1000).."."..(((shot_focus%1000)+50)/100).."m"
                    if(focus_mode>0) then
                        error_string="   **WARNING : focus not at infinity**"
                    end
                else
                    focus_string=" foc:infinity"
                    error_string=nil
                end
                printf(string.format('%d) IMG_%04d.JPG',shot_count,get_exp_count()))
                printf(" meter : Tv:".. print_tv(tv96meter)   .." Av:".. print_av(av96meter)   .." Sv:"..print_sv(sv96meter).." "..bv96raw ..":"..bv96meter)
                printf(" actual: Tv:".. print_tv(tv96setpoint).." Av:".. print_av(av96setpoint).." Sv:"..print_sv(sv96setpoint))
                printf("         AvMin:".. print_av(av96min).." NDF:"..nd_string..focus_string )

                if ((max_shots>0) and (shot_count >= max_shots)) then script_exit = true end

                shot_request = false                    -- reset shot request flag

            end
            collectgarbage()
        end

        -- check if USB remote enabled in intervalometer mode and USB power is off  ->  pause if so
        if ((usb_mode == 1 ) and (get_usb_power(1) == 0)) then
            printf("waiting on USB signal")
            unlock_focus()
            switch_mode(0)
            repeat wait_click(20) until ((get_usb_power(1) == 1) or ( is_key("menu")))
            switch_mode(1)
            lock_focus()
            if ( is_key("menu")) then script_exit = true end
            printf("USB wait finished")
            sleep(100)
        end

        if (usb_mode == 3 ) then pwm_mode(get_usb_power(2)) end

        if (blite_timer > 0) then
            blite_timer = blite_timer-1
            if ((blite_timer==0) and (backlight==1)) then set_lcd_display(0) end
        end

        if( error_string ~= nil) then
            draw_string( 16, 144, string.sub(error_string.."                          ",0,42), 258, 259)
        end

        wait_click(100)

        if( not( is_key("no_key"))) then
            if ((blite_timer==0) and (backlight==1)) then set_lcd_display(1) end
            blite_timer=300
            if ( is_key("menu") ) then script_exit = true end
        end

    until (script_exit==true)
    printf("script halt requested")
    restore()
end

   --[[ end of file ]]--


