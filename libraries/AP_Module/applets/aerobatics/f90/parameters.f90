! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_parameters
    use aerobatics_controller
    implicit none
    type :: parameters
        character(len=16) :: aerom(0:40),trik(0:63)
        real(c_float) :: spd_p,spd_i
        integer :: trick_count
    end type
contains
    real(c_float) function read_parameter(rt,name) result(value)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: name
        type(optional_float) :: output
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        value=0.0
        if (rt%error/=module_ok) return
        if (text_length(name)==0) then
            call check(rt,module_invalid); return
        end if
        started=time_us(rt%ctx)
        do
            code=param_get(rt%ctx,name(1:text_length(name)),output)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.output%present) then
            call check(rt,module_invalid); return
        end if
        value=output%value
        if (.not.finite(value)) call check(rt,module_invalid)
    end function
    subroutine bind_parameter(rt,name)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: name
        type(optional_float) :: output
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=param_get(rt%ctx,name(1:text_length(name)),output)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (rt%error==module_ok.and..not.output%present) call check(rt,module_invalid)
    end subroutine
    subroutine add_parameter(rt,key,prefix,idx,name,default,fullname)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: key,idx
        character(len=*), intent(in) :: prefix,name
        real(c_float), intent(in) :: default
        character(len=*), intent(out) :: fullname
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        logical(c_bool) :: ok
        character(len=96) :: combined
        fullname=''
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=param_add_param(rt%ctx,int(key,c_int8_t),int(idx,c_int8_t),name,default,ok)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.ok) call check(rt,module_host_error)
        combined=joined(prefix,name); fullname=combined
        call bind_parameter(rt,fullname)
    end subroutine
    subroutine create_parameters(rt,params)
        type(runtime), intent(inout) :: rt
        type(parameters), intent(out) :: params
        integer, parameter :: indices(30)=[1,2,4,5,6,8,11,12,13,14,15,16,17,18,19,20,21,22,23,24,26,27,28,29,30,31,32,33,34,35]
        character(len=10), parameter :: names(30)=[character(len=10) :: &
            'ANG_ACCEL','ANG_TC','THR_PIT_FF','SPD_P','SPD_I','ROL_COR_TC','TIME_COR_P','ERR_COR_P','ERR_COR_D', &
            'ENTRY_RATE','THR_LKAHD','DEBUG','THR_MIN','THR_BOOST','YAW_ACCEL','LKAHD','PATH_SCALE','BOX_WIDTH', &
            'STALL_THR','STALL_PIT','KE_RUDD','KE_RUDD_LK','ALT_ABORT','TS_P','TS_I','TS_SPDMAX','TS_RATE','MIS_ANGLE', &
            'OPTIONS','LANG']
        real(c_float), parameter :: defaults(30)=[6000.0,0.1,80.0,5.0,25.0,0.25,1.0,2.0,2.8,60.0,1.0,0.0, &
            0.0,50.0,1500.0,0.5,1.0,400.0,40.0,-20.0,25.0,0.25,15.0,0.33,0.33,0.0,4.0,0.0,0.0,0.0]
        character(len=4), parameter :: suffixes(5)=[character(len=4) :: 'ID','ARG1','ARG2','ARG3','ARG4']
        real(c_float), parameter :: trick_defaults(5)=[-1.0,30.0,0.0,0.0,0.0]
        integer :: k,i,offset
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        logical(c_bool) :: ok
        real(c_float) :: value
        character(len=12) :: digits
        character(len=96) :: name
        params%aerom=''; params%trik=''; params%trick_count=0; params%spd_p=0.0; params%spd_i=0.0
        started=time_us(rt%ctx)
        do
            code=param_add_table(rt%ctx,70_c_int8_t,'AEROM_',40_c_int8_t,ok)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (.not.ok) call check(rt,module_host_error)
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=param_add_table(rt%ctx,71_c_int8_t,'TRIK',63_c_int8_t,ok)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (.not.ok) call check(rt,module_host_error)
        do k=1,size(indices)
            i=indices(k)
            call add_parameter(rt,70,'AEROM_',i,names(k)(1:text_length(names(k))),defaults(k),params%aerom(i))
        end do
        call add_parameter(rt,71,'TRIK',1,'_ENABLE',0.0,params%trik(1))
        params%spd_p=read_parameter(rt,params%aerom(5)); params%spd_i=read_parameter(rt,params%aerom(6))
        call bind_parameter(rt,'AIRSPEED_MIN'); call bind_parameter(rt,'SCALING_SPEED')
        call bind_parameter(rt,'AIRSPEED_CRUISE'); value=read_parameter(rt,params%aerom(1))
        if (rt%error/=module_ok) return
        if (value>0.0 .and. value<100.0) then
            code=param_set_and_save(rt%ctx,'AEROM_ANG_ACCEL',3000.0,ok); call check(rt,code)
        end if
        value=read_parameter(rt,params%aerom(2))
        if (value>1.0 .and. rt%error==module_ok) then
            code=param_set_and_save(rt%ctx,'AEROM_ANG_TC',0.2,ok); call check(rt,code)
        end if
        value=read_parameter(rt,params%trik(1))
        if (value<=0.0 .or. rt%error/=module_ok) return
        call add_parameter(rt,71,'TRIK',2,'_SEL_FN',301.0,params%trik(2))
        call add_parameter(rt,71,'TRIK',3,'_ACT_FN',300.0,params%trik(3))
        call add_parameter(rt,71,'TRIK',4,'_COUNT',3.0,params%trik(4))
        value=read_parameter(rt,params%trik(4)); params%trick_count=int(constrain(value,1.0,11.0))
        do i=1,params%trick_count
            digits=decimal(i)
            do offset=0,4
                name=joined(digits(1:text_length(digits)),'_',suffixes(offset+1)(1:text_length(suffixes(offset+1))))
                call add_parameter(rt,71,'TRIK',5*i+offset,name(1:text_length(name)), &
                    trick_defaults(offset+1),params%trik(5*i+offset))
            end do
        end do
    end subroutine
    function get_configuration(rt,params) result(p)
        type(runtime), intent(inout) :: rt
        type(parameters), intent(in) :: params
        type(configuration) :: p
        p%ang_accel=read_parameter(rt,params%aerom(1)); p%ang_tc=read_parameter(rt,params%aerom(2))
        p%thr_pit_ff=read_parameter(rt,params%aerom(4)); p%spd_p=params%spd_p; p%spd_i=params%spd_i
        p%roll_corr_tc=read_parameter(rt,params%aerom(8)); p%time_corr_p=read_parameter(rt,params%aerom(11))
        p%err_corr_p=read_parameter(rt,params%aerom(12)); p%err_corr_d=read_parameter(rt,params%aerom(13))
        p%entry_rate=read_parameter(rt,params%aerom(14)); p%thr_lookahead=read_parameter(rt,params%aerom(15))
        p%thr_min=read_parameter(rt,params%aerom(17)); p%thr_boost=read_parameter(rt,params%aerom(18))
        p%yaw_accel=read_parameter(rt,params%aerom(19)); p%lookahead=read_parameter(rt,params%aerom(20))
        p%path_scale=read_parameter(rt,params%aerom(21)); p%ke_rudd=read_parameter(rt,params%aerom(26))
        p%ke_rudd_lookahead=read_parameter(rt,params%aerom(27)); p%alt_abort=read_parameter(rt,params%aerom(28))
        p%airspeed_min=read_parameter(rt,'AIRSPEED_MIN'); p%scaling_speed=read_parameter(rt,'SCALING_SPEED')
        p%airspeed_cruise=read_parameter(rt,'AIRSPEED_CRUISE')
    end function
end module aerobatics_parameters
