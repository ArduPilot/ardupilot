! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_mission
    use aerobatics_support
    implicit none
    integer, parameter :: auto_mode=10,nav_script_time=42702
contains
    integer(c_int16_t) function word_pattern(n) result(v)
        integer, intent(in) :: n
        v=int(iand(n,32767),c_int16_t)
        if (btest(n,15)) v=ibset(v,15)
    end function
    integer function mode_get(rt) result(mode)
        type(runtime), intent(inout) :: rt
        integer(c_int8_t) :: raw
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        mode=0
        if (rt%error/=module_ok) return
        start=time_us(rt%ctx)
        do
            code=vehicle_get_mode(rt%ctx,raw)
            if (.not.retry_busy(rt,code,start)) exit
        end do
        mode=iand(int(raw),255)
    end function
    function nav_time_get(rt) result(command)
        type(runtime), intent(inout) :: rt
        type(optional_vehicle_nav_script_timeoutput) :: command
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        command%present=.false.
        if (rt%error/=module_ok) return
        start=time_us(rt%ctx)
        do
            code=vehicle_nav_script_time(rt%ctx,command)
            if (.not.retry_busy(rt,code,start)) exit
        end do
    end function
    integer function nav_index_get(rt) result(index)
        type(runtime), intent(inout) :: rt
        integer(c_int16_t) :: raw
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        index=0
        if (rt%error/=module_ok) return
        start=time_us(rt%ctx)
        do
            code=mission_get_current_nav_index(rt%ctx,raw)
            if (.not.retry_busy(rt,code,start)) exit
        end do
        index=iand(int(raw),65535)
    end function
    function item_get(rt,index) result(item)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: index
        type(optional_mavlink_mission_item_int_t) :: item
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        item%present=.false.
        if (rt%error/=module_ok) return
        if (index<0.or.index>65535) then
            call check(rt,module_unavailable); return
        end if
        start=time_us(rt%ctx)
        do
            code=mission_get_item(rt%ctx,word_pattern(index),item)
            if (.not.retry_busy(rt,code,start)) exit
        end do
    end function
    function mission_location(rt,start_index,forward) result(loc)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: start_index
        logical, intent(in) :: forward
        type(location) :: loc
        type(optional_mavlink_mission_item_int_t) :: item
        integer :: index,k
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        logical(c_bool) :: has_location
        loc=location(.false.,.false.,.false.,.false.,.false.,0,0,0); index=start_index
        do k=1,256
            item=item_get(rt,index)
            if (rt%error/=module_ok) return
            if (.not.item%present) then
                call check(rt,module_unavailable); return
            end if
            start=time_us(rt%ctx)
            do
                code=mission_cmd_has_location(rt%ctx,item%value%command,has_location)
                if (.not.retry_busy(rt,code,start)) exit
            end do
            if (rt%error/=module_ok) return
            if (has_location.and.(item%value%x/=0.or.item%value%y/=0)) then
                code=location_lat_set(rt%ctx,loc,item%value%x); call check(rt,code)
                if (rt%error==module_ok) then
                    code=location_lng_set(rt%ctx,loc,item%value%y); call check(rt,code)
                end if
                return
            end if
            if (forward.and.item%value%command==177) then
                if (.not.finite(item%value%param1).or.item%value%param1<0.0.or.item%value%param1>65535.0) then
                    call check(rt,module_invalid); return
                end if
                index=int(item%value%param1)
            else if (forward) then
                index=index+1
            else
                index=index-1
            end if
            if (index<0.or.index>65535) then
                call check(rt,module_unavailable); return
            end if
        end do
        call check(rt,module_invalid)
    end function
    real(c_float) function mission_heading(rt,angle,course) result(yaw)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: angle,course
        type(location) :: previous,next
        type(optional_location) :: current
        integer :: index
        integer(c_int32_t) :: code
        integer(c_int64_t) :: start
        real(c_float) :: bearing,difference
        yaw=angle
        if (.not.finite(angle).or..not.finite(course)) call check(rt,module_invalid)
        if (rt%error/=module_ok.or.angle/=0.0) return
        index=nav_index_get(rt)
        if (index>1) then
            previous=mission_location(rt,index-1,.false.)
        else
            start=time_us(rt%ctx)
            do
                code=ahrs_get_location(rt%ctx,current)
                if (.not.retry_busy(rt,code,start)) exit
            end do
            if (.not.current%present) call check(rt,module_unavailable)
            previous=current%value
        end if
        if (index==65535) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        next=mission_location(rt,index+1,.true.)
        if (rt%error/=module_ok) return
        start=time_us(rt%ctx)
        do
            code=location_get_bearing(rt%ctx,previous,next,bearing)
            if (.not.retry_busy(rt,code,start)) exit
        end do
        yaw=deg(bearing); difference=wrap180(rt,course-yaw)
        if (index>1.and.abs(difference)>90.0) yaw=wrap180(rt,yaw+180.0)
    end function
end module aerobatics_mission
