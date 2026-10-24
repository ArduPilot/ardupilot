! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_runner
    use aerobatics_parameters
    use aerobatics_schedule
    use aerobatics_mission
    use aerobatics_logging
    implicit none
    type :: runner_state
        type(parameters) :: params
        type(path_tree) :: tree
        type(controller) :: control
        type(rc_channeltoken) :: channel
        integer :: loaded_id,owner,owner_id,last_selection,last_action,last_mission_id,last_preload
        real(c_float) :: loaded_args(4),last_named_float
        character(len=48) :: loaded_name
        logical :: active,starting,starting_mission,preloading,stepped,delivering,have_attempted_trick
        type(sharedstatetoken) :: shared
        integer(c_int64_t) :: shared_revision,loaded_hash
        integer(c_int32_t) :: words(0:511)
        real(c_float) :: loaded_speed,loaded_entry,loaded_width
        logical :: shared_held
        logical :: language_selected
        integer :: attempted_trick
        integer(c_int64_t) :: count,last_error,last_command,tick_started
    end type
contains
    function observe(rt) result(input)
        type(runtime), intent(inout) :: rt
        type(observation) :: input
        type(optional_vector3f) :: vector
        type(optional_float) :: airspeed
        type(optional_quaternion) :: attitude
        integer(c_int64_t) :: started,call_started,milliseconds
        integer(c_int32_t) :: code
        input%now=0.0; input%pos=vector3f(0.0,0.0,0.0); input%gyro=input%pos; input%velocity=input%pos
        input%airspeed=0.0; input%eas2tas=0.0; input%attitude=quaternion(1.0,0.0,0.0,0.0); input%throttle=0.0
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx); milliseconds=iand(time_us(rt%ctx)/1000,4294967295_c_int64_t)
        input%now=real(milliseconds,c_float)*0.001
        call_started=time_us(rt%ctx)
        do
            code=ahrs_get_relative_position_ned_origin(rt%ctx,vector)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.vector%present) then
            call check(rt,module_unavailable); return
        end if
        input%pos=vector%value
        call_started=time_us(rt%ctx)
        do
            code=ahrs_get_gyro(rt%ctx,input%gyro)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        call_started=time_us(rt%ctx)
        do
            code=ahrs_get_velocity_ned(rt%ctx,vector)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.vector%present) then
            call check(rt,module_unavailable); return
        end if
        input%velocity=vector%value
        call_started=time_us(rt%ctx)
        do
            code=ahrs_airspeed_eas(rt%ctx,airspeed)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.airspeed%present) then
            call check(rt,module_unavailable); return
        end if
        input%airspeed=airspeed%value
        call_started=time_us(rt%ctx)
        do
            code=ahrs_get_eas2tas(rt%ctx,input%eas2tas)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        call_started=time_us(rt%ctx)
        do
            code=ahrs_get_quaternion(rt%ctx,attitude)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        if (.not.attitude%present) then
            call check(rt,module_unavailable); return
        end if
        input%attitude=attitude%value
        call_started=time_us(rt%ctx)
        do
            code=srv_channels_get_output_scaled(rt%ctx,70_c_int32_t,input%throttle)
            if (.not.retry_busy(rt,code,call_started)) exit
        end do
        if (rt%error/=module_ok) return
        if (max(0_c_int64_t,time_us(rt%ctx)-started)>10000) call check(rt,module_busy)
        call validate_observation(rt,input)
    end function
    integer function aux_get(rt,option) result(action)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: option
        type(optional_uint8_t) :: value
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        action=-1
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=rc_get_aux_cached(rt%ctx,int(option,c_int32_t),value)
            if (.not.retry_busy(rt,code,started)) exit
        end do
        if (rt%error==module_ok.and.value%present) action=iand(int(value%value),255)
    end function
    subroutine find_channel(rt,option,channel)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: option
        type(rc_channeltoken), intent(out) :: channel
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        channel%value=0
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=rc_find_channel_for_option(rt%ctx,int(option,c_int32_t),channel)
            if (.not.retry_busy(rt,code,started)) exit
        end do
    end subroutine
    real(c_float) function channel_norm(rt,channel) result(norm)
        type(runtime), intent(inout) :: rt
        type(rc_channeltoken), intent(in) :: channel
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        norm=0.0
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=rc_channel_norm_input_ignore_trim(rt%ctx,channel,norm)
            if (.not.retry_busy(rt,code,started)) exit
        end do
    end function
    subroutine release(rt,state)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        integer(c_int32_t) :: code
        integer(c_int8_t) :: mode
        logical(c_bool) :: ok
        if (rt%error/=module_ok) return
        select case(state%owner)
        case(2)
            code=vehicle_nav_script_time_done(rt%ctx,word_pattern(state%owner_id)); call check(rt,code)
        case(1)
            code=vehicle_get_mode(rt%ctx,mode); call check(rt,code)
            if (rt%error==module_ok.and.iand(int(mode),255)==state%owner_id) then
                code=vehicle_nav_scripting_enable(rt%ctx,-1_c_int8_t,ok); call check(rt,code)
            end if
        end select
        if (rt%error==module_ok) state%owner=0
    end subroutine
    subroutine load_task(rt,state,id,args,p,input,width,saved_eval)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        integer, intent(in) :: id
        real(c_float), intent(in) :: args(4),width
        type(configuration), intent(in) :: p
        type(observation), intent(in) :: input
        type(evaluation) :: eval
        type(evaluation), intent(in), optional :: saved_eval
        character(len=48) :: name
        character(len=12) :: digits
        character(len=32) :: filename
        character(len=12), parameter :: directories(3)=[character(len=12) :: 'APM/scripts/','scripts/','']
        character(len=96) :: temp
        logical :: found
        integer :: cmd,i,root,cache_count
        if (rt%error/=module_ok) return
        state%loaded_id=0; call free_tree(state%tree)
        if (id<1.or.id>255) then
            call check(rt,module_invalid); return
        end if
        if (present(saved_eval)) then
            eval=saved_eval
        else
            eval=evaluation(target_speed(rt,input,p),p%entry_rate,.true.)
        end if
        if (.not.finite(eval%speed).or.eval%speed<=0.0.or..not.finite(eval%entry_rate).or. &
            eval%entry_rate<=0.0.or..not.finite(width)) then
            call check(rt,module_invalid); return
        end if
        state%loaded_speed=eval%speed; state%loaded_entry=eval%entry_rate; state%loaded_width=width
        state%loaded_hash=0; cmd=builtin(id,name)
        if (cmd/=0) then
            call allocate_tree(rt,state%tree,command_nodes(cmd))
            root=build_maneuver(rt,state%tree,cmd,args,width,eval); state%tree%root=root
        else
            digits=decimal(id); found=.false.
            do i=1,3
                temp=joined(directories(i)(1:text_length(directories(i))),'trick',digits(1:text_length(digits)))
                filename=temp; i_filename: block
                    integer :: n
                    n=text_length(filename); filename(n+1:n+4)='.txt'
                end block i_filename
                call load_schedule(rt,filename(1:text_length(filename)),state%tree,width,eval,found,name,state%loaded_hash)
                if (found.or.rt%error/=module_ok) exit
            end do
            if (.not.found) call check(rt,module_invalid)
        end if
        if (rt%error==module_ok) then
            cache_count=0
            do i=1,state%tree%used
                if (state%tree%nodes(i)%kind==composite) cache_count=cache_count+1
            end do
            if (cache_count>112) call check(rt,module_invalid)
        end if
        if (rt%error/=module_ok) then
            call free_tree(state%tree); return
        end if
        state%loaded_id=id; state%loaded_args=args; state%loaded_name=name
    end subroutine
    logical function task_matches(state,id,args) result(matches)
        type(runner_state), intent(in) :: state
        integer, intent(in) :: id
        real(c_float), intent(in) :: args(4)
        matches=state%loaded_id==id.and.all(state%loaded_args==args)
    end function
    logical function ownership_valid(rt,state) result(valid)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(in) :: state
        type(optional_vehicle_nav_script_timeoutput) :: command
        integer :: mode
        valid=.false.; mode=mode_get(rt)
        if (rt%error/=module_ok) return
        if (state%owner==1) then
            valid=mode==state%owner_id
        else if (state%owner==2.and.mode==auto_mode) then
            command=nav_time_get(rt)
            if (command%present) valid=iand(int(command%value%out0),65535)==state%owner_id
        end if
    end function
    subroutine deliver(rt,o)
        type(runtime), intent(inout) :: rt
        type(control_output), intent(in) :: o
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=vehicle_set_target_throttle_rate_rpy(rt%ctx,o%throttle,o%rates%x,o%rates%y,o%rates%z)
            if (.not.retry_busy(rt,code,started)) exit
        end do
    end subroutine
    subroutine deliver_rudder(rt,o)
        type(runtime), intent(inout) :: rt
        type(control_output), intent(in) :: o
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx)
        do
            code=vehicle_set_rudder_offset(rt%ctx,o%rudder,.true._c_bool)
            if (.not.retry_busy(rt,code,started)) exit
        end do
    end subroutine
    logical function shared_write(rt,state) result(written)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        type(array_uint8_t_2048) :: bytes
        integer(c_int64_t) :: revision
        integer(c_int32_t) :: code
        integer :: i,j
        written=.false.
        if (rt%error/=module_ok) return
        do i=0,511
            do j=0,3
                bytes%data(4*i+j)=int(iand(shiftr(state%words(i),8*j),255),c_int8_t)
            end do
        end do
        code=shared_state_write(rt%ctx,state%shared,0_c_int32_t,2048_c_int16_t,state%shared_revision,bytes,revision)
        call check(rt,code)
        if (rt%error/=module_ok.or.revision==0) return
        state%shared_revision=revision; written=.true.
    end function
    subroutine yield_state(rt,state)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        integer :: cursor,i
        state%words=0; state%words(0)=int(z'4145524f',c_int32_t); state%words(1)=1; state%words(3)=1
        state%words(4)=merge(1,0,state%active); state%words(5)=state%owner; state%words(6)=state%owner_id
        state%words(7)=int(state%last_command,c_int32_t); state%words(8)=int(shiftr(state%last_command,32),c_int32_t)
        state%words(9)=int(state%count,c_int32_t); state%words(10)=state%last_selection
        state%words(11)=state%last_action; state%words(12)=state%last_mission_id; state%words(13)=state%last_preload
        if (state%active) then
            state%words(14)=state%loaded_id
            do i=1,4
                state%words(14+i)=transfer(state%loaded_args(i),0_c_int32_t)
            end do
            state%words(19)=transfer(state%loaded_speed,0_c_int32_t)
            state%words(20)=transfer(state%loaded_entry,0_c_int32_t)
            state%words(21)=transfer(state%loaded_width,0_c_int32_t); state%words(22)=int(state%loaded_hash,c_int32_t)
            call save_controller(state%control,state%words(24:59))
            cursor=64; call path_cache(rt,state%tree,state%tree%root,state%words,cursor,.false.)
            state%words(23)=(cursor-64)/4
        end if
        if (.not.shared_write(rt,state)) then
            call check(rt,module_invalid); return
        end if
        state%shared_held=.false.; state%active=.false.; state%owner=0
        state%loaded_id=0; call free_tree(state%tree)
    end subroutine
    subroutine claim_state(rt,state)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        type(sharedstateread) :: bytes
        type(evaluation) :: eval
        type(configuration) :: p
        type(observation) :: input
        integer(c_int64_t) :: expected_hash
        integer(c_int32_t) :: code
        integer :: i,j,cursor
        real(c_float) :: args(4),width
        code=shared_state_read(rt%ctx,state%shared,0_c_int32_t,2048_c_int16_t,bytes); call check(rt,code)
        if (rt%error/=module_ok) return
        state%shared_revision=bytes%revision; state%words=0
        do i=0,511
            do j=0,3
                state%words(i)=ior(state%words(i),shiftl(iand(int(bytes%data%data(4*i+j),c_int32_t),255),8*j))
            end do
        end do
        if (state%words(0)==0) then
            if (any(state%words/=0)) then
                call check(rt,module_invalid); return
            end if
            state%words(0)=int(z'4145524f',c_int32_t); state%words(1)=1
        end if
        if (state%words(0)/=int(z'4145524f',c_int32_t).or.state%words(1)/=1) then
            call check(rt,module_invalid); return
        end if
        if (state%words(2)/=0) return
        state%words(2)=state%shared%value
        if (.not.shared_write(rt,state)) return
        state%shared_held=.true.
        if (state%words(4)<0.or.state%words(4)>1.or.state%words(5)<0.or.state%words(5)>2.or. &
            state%words(6)<0.or.state%words(6)>65535.or.state%words(10)<0.or. &
            state%words(10)>state%params%trick_count.or.state%words(11)<-1.or.state%words(11)>2.or. &
            state%words(12)<-1.or.state%words(12)>65535.or.state%words(13)<-1.or.state%words(13)>65535) then
            call check(rt,module_invalid); return
        end if
        state%count=iand(int(state%words(9),c_int64_t),4294967295_c_int64_t)
        if (state%words(3)/=0) then
            state%last_selection=state%words(10); state%last_action=state%words(11)
            state%last_mission_id=state%words(12); state%last_preload=state%words(13)
        end if
        state%last_command=ior(iand(int(state%words(7),c_int64_t),4294967295_c_int64_t), &
            shiftl(int(state%words(8),c_int64_t),32))
        if (state%words(4)==0) return
        if (state%last_command<0.or.state%last_command>time_us(rt%ctx)) then
            call check(rt,module_invalid); return
        end if
        if (time_us(rt%ctx)-state%last_command>1000000) return
        if (state%words(5)==0.or.(state%words(5)==1.and.state%words(6)>255).or. &
            state%words(23)<0.or.state%words(23)>112) then
            call check(rt,module_invalid); return
        end if
        eval=evaluation(transfer(state%words(19),0.0_c_float),transfer(state%words(20),0.0_c_float),.false.)
        width=transfer(state%words(21),0.0_c_float)
        do i=1,4
            args(i)=transfer(state%words(14+i),0.0_c_float)
            if (.not.finite(args(i))) call check(rt,module_invalid)
        end do
        expected_hash=iand(int(state%words(22),c_int64_t),4294967295_c_int64_t)
        p=get_configuration(rt,state%params); input%now=0.0
        call load_task(rt,state,state%words(14),args,p,input,width,eval)
        if (rt%error/=module_ok) return
        if (state%loaded_hash/=expected_hash) then
            call check(rt,module_invalid); return
        end if
        cursor=64; call path_cache(rt,state%tree,state%tree%root,state%words,cursor,.true.)
        if (cursor/=64+4*state%words(23)) call check(rt,module_invalid)
        call restore_controller(rt,state%control,state%words(24:59))
        if (rt%error/=module_ok) return
        state%owner=state%words(5); state%owner_id=state%words(6); state%active=.true.
    end subroutine
    subroutine tick(rt,state)
        type(runtime), intent(inout) :: rt
        type(runner_state), intent(inout) :: state
        type(configuration) :: p
        type(observation) :: input
        type(control_output) :: output
        type(optional_vehicle_nav_script_timeoutput) :: command
        type(optional_mavlink_mission_item_int_t) :: item
        type(runtime) :: diagnostics
        integer :: mode,index,id,action,selection,k,step
        real(c_float) :: args(4),course,yaw,norm,value,width
        character(len=96) :: text,temp
        character(len=12) :: digits,errtext
        integer(c_int32_t) :: code
        integer(c_int64_t) :: started,elapsed
        logical :: stale,finish,valid
        logical(c_bool) :: ok
        code=sleep_ms(rt%ctx,25_c_int32_t); call check(rt,code)
        if (rt%error/=module_ok) return
        value=read_parameter(rt,state%params%aerom(35))
        if (rt%error/=module_ok) return
        if (value/=2.0) then
            if (state%shared_held) call yield_state(rt,state)
            state%language_selected=.false.; return
        end if
        if (.not.state%shared_held) then
            call claim_state(rt,state)
            if (rt%error/=module_ok.or..not.state%shared_held) return
            value=read_parameter(rt,state%params%aerom(35))
            if (rt%error/=module_ok.or.value/=2.0) return
        end if
        if (.not.state%language_selected) then
            if (.not.state%active) state%last_action=-1
            call message(rt,6_c_int32_t,'Aerobatics F90 selected')
            if (rt%error/=module_ok) return
            state%language_selected=.true.
        end if
        state%tick_started=time_us(rt%ctx)
        if (state%active.and.max(0_c_int64_t,state%tick_started-state%last_command)>1000000) then
            call message(rt,3_c_int32_t,'Aerobatics: control timeout'); call check(rt,module_invalid); return
        end if
        if (state%owner/=0.and..not.state%active) call release(rt,state)
        mode=mode_get(rt); command%present=.false.
        if (mode==auto_mode) command=nav_time_get(rt)
        if (rt%error/=module_ok) return
        stale=.false.
        if (state%owner==2) then
            stale=.true.
            if (command%present) stale=iand(int(command%value%out0),65535)/=state%owner_id
        else if (state%owner==1) then
            stale=state%owner_id/=mode
        end if
        if (stale) then
            if (state%active) state%count=state%control%count
            state%active=.false.; call release(rt,state)
        end if
        value=read_parameter(rt,state%params%aerom(34))
        if (iand(rust_u32(value),4_c_int64_t)/=0.and.rt%error==module_ok) then
            call release(rt,state); state%active=.false.; return
        end if
        if (rt%error/=module_ok) return
        if (mode==auto_mode) then
            if (command%present) then
                id=iand(int(command%value%out0),65535)
                if (state%last_mission_id/=id) then
                    args=[command%value%out2,command%value%out3, &
                        real(command%value%out4,c_float),real(command%value%out5,c_float)]
                    p=get_configuration(rt,state%params); call validate_configuration(rt,p); input=observe(rt)
                    if (rt%error/=module_ok) return
                    state%last_mission_id=id; state%starting_mission=.true.
                    state%attempted_trick=iand(int(command%value%out1),255); state%have_attempted_trick=.true.
                    if (.not.task_matches(state,state%attempted_trick,args)) then
                        width=read_parameter(rt,state%params%aerom(22))
                        call load_task(rt,state,state%attempted_trick,args,p,input,width)
                    end if
                    course=deg(atan2f(rt,input%velocity%y,input%velocity%x))
                    value=read_parameter(rt,state%params%aerom(33)); yaw=mission_heading(rt,value,course)
                    input=observe(rt)
                    call start_controller(rt,state%control,state%tree,input,p,yaw,state%count)
                    if (rt%error/=module_ok) return
                    valid=mode_get(rt)==auto_mode
                    if (valid) then
                        command=nav_time_get(rt); valid=.false.
                        if (command%present) valid=iand(int(command%value%out0),65535)==id
                    end if
                    if (rt%error/=module_ok) return
                    if (.not.valid) then
                        digits=decimal(state%attempted_trick)
                        call message(rt,4_c_int32_t,joined('Trick ',digits(1:text_length(digits)), &
                            ' skipped: item expired')); return
                    end if
                    state%owner=2; state%owner_id=id; state%last_command=time_us(rt%ctx); state%active=.true.
                    call message(rt,6_c_int32_t,joined('Starting ',state%loaded_name(1:text_length(state%loaded_name)),'!'))
                end if
            else
                index=nav_index_get(rt)
                if (rt%error/=module_ok) return
                if (state%last_preload/=index) then
                    state%last_preload=index; state%preloading=.true.
                    if (index<65535) then
                        item=item_get(rt,index+1)
                        if (rt%error/=module_ok) return
                        if (item%present) then
                            if (iand(int(item%value%command),65535)==nav_script_time) then
                                state%attempted_trick=rust_i32(item%value%param1); state%have_attempted_trick=.true.
                                p=get_configuration(rt,state%params)
                                args=[item%value%param3,item%value%param4,real(item%value%x,c_float),real(item%value%y,c_float)]
                                input=observe(rt); width=read_parameter(rt,state%params%aerom(22))
                                call load_task(rt,state,state%attempted_trick,args,p,input,width)
                            end if
                        end if
                    end if
                end if
                return
            end if
        else
            if (state%params%trick_count==0) return
            value=read_parameter(rt,state%params%trik(3)); action=aux_get(rt,rust_i32(value))
            if (rt%error/=module_ok) return
            if (state%last_action==-1.and.action/=-1) state%last_action=action
            if (action==0.and.state%active) then
                state%count=state%control%count; state%active=.false.; state%last_selection=0
                call release(rt,state); call message(rt,1_c_int32_t,'Trick aborted'); return
            end if
            if (state%channel%value==0) then
                value=read_parameter(rt,state%params%trik(2)); call find_channel(rt,rust_i32(value),state%channel)
            end if
            selection=0
            if (state%channel%value/=0.and.rt%error==module_ok) then
                norm=channel_norm(rt,state%channel); value=read_parameter(rt,state%params%trik(4))
                value=value*constrain(0.5*(norm+1.0),0.0,0.999)+1.0
                if (value>0.0.and.value<real(state%params%trick_count+1,c_float)) selection=int(value)
                if (selection<1.or.selection>state%params%trick_count) selection=0
            end if
            if (rt%error/=module_ok) return
            if (.not.state%active.and.selection>0.and. &
                ((action==1.and.state%last_selection/=selection).or.action/=state%last_action)) then
                state%starting=.true.; state%last_selection=selection; state%last_action=action
                value=read_parameter(rt,state%params%trik(5*selection)); id=rust_i32(value)
                do k=1,4
                    args(k)=read_parameter(rt,state%params%trik(5*selection+k))
                end do
                p=get_configuration(rt,state%params); call validate_configuration(rt,p)
                if (rt%error/=module_ok) return
                if (.not.task_matches(state,id,args)) then
                    input=observe(rt); width=read_parameter(rt,state%params%aerom(22))
                    call load_task(rt,state,id,args,p,input,width)
                    if (rt%error/=module_ok.and.rt%error/=module_stopped.and.rt%error/=module_busy) then
                        errtext=error_name(rt%error); rt%error=module_ok; digits=decimal(id)
                        text=joined('Trick ',digits(1:text_length(digits)),' load: ')
                        call message(rt,3_c_int32_t,joined(text(1:text_length(text)),errtext(1:text_length(errtext))))
                        return
                    end if
                end if
                if (rt%error/=module_ok) return
                if (action==1) then
                    digits=decimal(selection); temp=joined('Trick ',digits(1:text_length(digits)),' selected (')
                    call message(rt,6_c_int32_t,joined(temp(1:text_length(temp)), &
                        state%loaded_name(1:text_length(state%loaded_name)),')'))
                else if (action==2) then
                    input=observe(rt); yaw=wrap180(rt,deg(atan2f(rt,input%velocity%y,input%velocity%x)))
                    call start_controller(rt,state%control,state%tree,input,p,yaw,state%count)
                    if (rt%error/=module_ok) return
                    started=time_us(rt%ctx)
                    do
                        code=vehicle_nav_scripting_enable(rt%ctx,int(mode,c_int8_t),ok)
                        if (.not.retry_busy(rt,code,started)) exit
                    end do
                    if (rt%error/=module_ok) return
                    if (.not.ok) then
                        call message(rt,1_c_int32_t,'Tricks not available in mode'); return
                    end if
                    state%owner=1; state%owner_id=mode; state%last_command=time_us(rt%ctx)
                    digits=decimal(selection); temp=joined('Trick ',digits(1:text_length(digits)),' started (')
                    call message(rt,6_c_int32_t,joined(temp(1:text_length(temp)), &
                        state%loaded_name(1:text_length(state%loaded_name)),')'))
                    state%last_selection=0; state%active=.true.; return
                end if
            end if
        end if
        if (rt%error/=module_ok.or..not.state%active) return
        p=get_configuration(rt,state%params); input=observe(rt)
        if (max(0_c_int64_t,time_us(rt%ctx)-state%last_command)>1000000) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        started=time_us(rt%ctx); state%stepped=.true.
        call step_controller(rt,state%control,state%tree,input,p,step,output); state%count=state%control%count
        elapsed=max(0_c_int64_t,time_us(rt%ctx)-started); finish=.false.
        if (rt%error==module_stopped) return
        if (rt%error/=module_ok) then
            rt%error=module_ok; call message(rt,0_c_int32_t,'Invalid path - aborting'); step=step_finished
        end if
        select case(step)
        case(step_waiting); return
        case(step_finished); finish=.true.
        case(step_abort)
            call message(rt,0_c_int32_t,'Too low altitude, aborting'); finish=.true.
        end select
        if (step==step_command.or.step==step_abort) then
            state%delivering=.not.finish; valid=ownership_valid(rt,state)
            if (rt%error/=module_ok) return
            if (.not.valid) then
                state%active=.false.; state%owner=0; return
            end if
            call deliver(rt,output)
            if (rt%error/=module_ok) return
            state%last_command=time_us(rt%ctx); call deliver_rudder(rt,output)
            if (rt%error/=module_ok) return
            state%delivering=.false.; diagnostics=runtime(rt%ctx,module_ok)
            call log_output(diagnostics,input,output)
            if (diagnostics%error==module_stopped) then
                call check(rt,module_stopped); return
            end if
            if (output%path_rate_fallback) call message(rt,0_c_int32_t,'path_rate_ef_rads: invalid')
            if (input%now-state%last_named_float>0.5) then
                state%last_named_float=input%now; code=gcs_send_named_float(rt%ctx,'PERR',output%position_error)
            end if
            diagnostics%error=module_ok
            call log_timing(diagnostics,elapsed,max(0_c_int64_t,time_us(rt%ctx)-state%tick_started),output)
        end if
        if (finish) then
            call release(rt,state)
            if (rt%error/=module_ok) return
            call message(rt,6_c_int32_t,joined('Finishing ',state%loaded_name(1:text_length(state%loaded_name)),'!'))
            state%active=.false.
        end if
    end subroutine
    function run(ctx) bind(C,name='ap_module_fortran_run') result(status)
        type(c_ptr), value :: ctx
        integer(c_int32_t) :: status
        type(runtime) :: rt,cleanup
        type(runner_state) :: state
        integer :: previous_selection,previous_action
        character(len=96) :: text
        character(len=12) :: digits,errtext
        integer(c_int64_t) :: now
        real(c_float) :: value
        rt=runtime(ctx,module_ok)
        nullify(state%tree%nodes); state%tree%allocation=c_null_ptr
        state%tree%used=0; state%tree%capacity=0; state%tree%root=0
        state%loaded_id=0; state%loaded_args=0.0; state%loaded_name=''; state%active=.false.
        state%channel%value=0; state%owner=0; state%owner_id=0; state%last_selection=0; state%last_action=-1
        state%last_mission_id=-1; state%last_preload=-1; state%count=0; state%last_named_float=0.0
        state%last_error=0; state%last_command=0
        state%language_selected=.false.; state%shared_held=.false.; state%shared_revision=0
        code_open: block
            integer(c_int32_t) :: code
            code=shared_state_open(ctx,int(z'4145524f',c_int32_t),2048_c_int32_t,state%shared)
            call check(rt,code)
        end block code_open
        call create_parameters(rt,state%params)
        if (rt%error/=module_ok) then
            errtext=error_name(rt%error)
            call message(rt,3_c_int32_t,joined('Aerobatics parameters: ',errtext(1:text_length(errtext))))
            status=rt%error; return
        end if
        call message(rt,6_c_int32_t,'Loaded plane_aerobatics Fortran')
        if (state%params%trick_count>0) then
            digits=decimal(state%params%trick_count)
            call message(rt,6_c_int32_t,joined('Enabled ',digits(1:text_length(digits)),' tricks'))
            value=read_parameter(rt,state%params%trik(3)); state%last_action=aux_get(rt,rust_i32(value))
            if (rt%error/=module_ok) then
                state%last_action=-1; rt%error=module_ok
            end if
        end if
        do
            previous_selection=state%last_selection; previous_action=state%last_action
            state%starting=.false.; state%starting_mission=.false.; state%preloading=.false.
            state%attempted_trick=0; state%have_attempted_trick=.false.
            state%stepped=.false.; state%delivering=.false.
            call tick(rt,state)
            if (rt%error==module_ok) cycle
            if (rt%error==module_busy) then
                if (state%starting_mission.and.state%owner==0) state%last_mission_id=-1
                if (state%preloading) state%last_preload=-1
            end if
            if (state%starting.and.(rt%error==module_busy.or.rt%error==module_unavailable).and.state%owner==0) then
                state%last_selection=previous_selection; state%last_action=previous_action
            end if
            if (rt%error==module_busy.and.(.not.state%stepped.or.state%delivering)) then
                rt%error=module_ok; cycle
            end if
            if (state%active) state%count=state%control%count
            state%active=.false.; cleanup=runtime(ctx,module_ok); call release(cleanup,state)
            if (rt%error==module_stopped) exit
            now=time_us(ctx); errtext=error_name(rt%error); rt%error=module_ok
            if (state%have_attempted_trick) then
                digits=decimal(state%attempted_trick)
                text=joined('Aerobatics error: trick ',digits(1:text_length(digits)),': ')
                call message(rt,3_c_int32_t,joined(text(1:text_length(text)),errtext(1:text_length(errtext))))
            else if (max(0_c_int64_t,now-state%last_error)>1000000) then
                state%last_error=now
                call message(rt,3_c_int32_t,joined('Aerobatics error ',errtext(1:text_length(errtext))))
            end if
            if (rt%error==module_stopped) exit
        end do
        status=rt%error; call free_tree(state%tree)
        close_shared: block
            integer(c_int32_t) :: code
            code=shared_state_close(ctx,state%shared)
        end block close_shared
    end function
end module aerobatics_runner
