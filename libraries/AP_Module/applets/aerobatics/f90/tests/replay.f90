! SPDX-License-Identifier: GPL-3.0-or-later
! Same observations and output fields as tests/modules/aerobatics_controller.rs.
module aerobatics_replay
    use aerobatics_schedule
    use aerobatics_controller
    use aerobatics_logging
    use aerobatics_mission
    implicit none
    integer, parameter :: variant=0
contains
    function inputs(rt,i) result(input)
        type(runtime), intent(inout) :: rt
        integer, intent(in) :: i
        type(observation) :: input
        real(c_float) :: n,si,co,roll,pitch
        integer(c_int32_t) :: code
        n=real(i,c_float); si=fsin(rt,n*0.03); co=fcos(rt,n*0.01); roll=si*0.2; pitch=co*0.1
        input%attitude=quaternion(1.0,0.0,0.0,0.0)
        code=quaternion_from_euler(rt%ctx,input%attitude,roll,pitch,0.3); call check(rt,code)
        input%now=real(10000+i*25,c_float)*0.001
        si=fsin(rt,n*0.01); co=fcos(rt,n*0.02)
        input%pos=vector3f(n*0.45,si*2.0,-30.0+co); input%gyro=vector3f(0.1,0.2,0.3)
        input%velocity=vector3f(18.0,0.02,-0.01); input%airspeed=18.0; input%eas2tas=1.0; input%throttle=40.0
        if (variant/=0) then
            input%velocity%x=10.0+real(mod(i,21),c_float)
            if (mod(i,37)==0) input%velocity%x=-5.0
            input%airspeed=4.0+real(mod(i,37),c_float)
            if (mod(i,43)==0) input%eas2tas=5.0
            input%gyro%x=fsin(rt,n*0.02)
        end if
    end function
    subroutine trace(rt,name,labels,id,values)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: name,labels
        integer, intent(in) :: id
        real(c_float), intent(in) :: values(:)
        type(logrecord) :: record
        character(len=16) :: format
        integer(c_int8_t) :: bytes(2)
        integer :: k
        format='H'
        do k=1,size(values)
            format(k+1:k+1)='f'
        end do
        call init_record(record,name,labels,format(1:size(values)+1))
        bytes=transfer(int(id,c_int16_t),bytes); record%payload%data(0:1)=bytes; record%payload_len=2
        do k=1,size(values)
            call append_float(record,values(k))
        end do
        call write_record(rt,record)
    end subroutine
    subroutine primitive_checks(rt,tree,p,eval)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        type(configuration), intent(in) :: p
        type(evaluation), intent(in) :: eval
        character(len=48) :: name
        integer :: capacity,k
        character(len=64), parameter :: bad(10)=[character(len=64) :: '', '#empty', 'name: Only', 'unknown 5', &
            'loop 1 2 3 4 5','loop NaN','loop inf','function triangle(r)','thr_boost: maybe','name: two words']
        if (sgn(0.0)/=-1.0.or.sgn(0.000001)/=0.0.or.sgn(0.000002)/=1.0) call check(rt,module_invalid)
        if (constrain(2.0,3.0,1.0)/=1.0) call check(rt,module_invalid)
        if (roll_get(roll_entry_exit,90.0,0.25,0.0,60.0)/=45.0) call check(rt,module_invalid)
        if (roll_get(roll_entry,90.0,0.1,10.0,30.0)/=(90.0*0.1/((90.0/30.0)/10.0))) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        do k=1,size(bad)
            call parse_schedule(rt,bad(k)(1:text_length(bad(k))),tree,400.0,eval,.false.,name,capacity)
            if (rt%error/=module_invalid) then
                rt%error=module_host_error; return
            end if
            rt%error=module_ok
        end do
        call parse_schedule(rt,'loop 25'//achar(10)//'message: dangling',tree,400.0,eval,.false.,name,capacity)
        if (rt%error/=module_invalid) then
            rt%error=module_host_error; return
        end if
        rt%error=module_ok
        call parse_schedule(rt,'name: Test'//achar(13)//achar(10)//achar(9)//'loop'//achar(9)//'25 0 1', &
            tree,400.0,eval,.false.,name,capacity)
        if (.not.same_text(name(1:text_length(name)),'Test').or.capacity/=3) call check(rt,module_host_error)
    end subroutine
    subroutine mission_checks(rt)
        type(runtime), intent(inout) :: rt
        type(mavlink_mission_item_int_t) :: item
        type(location) :: loc
        integer :: i
        integer(c_int32_t) :: code
        logical(c_bool) :: ok
        real(c_float) :: yaw
        if (rt%error/=module_ok) return
        yaw=mission_heading(rt,123.0,0.0)
        if (yaw/=123.0) call check(rt,module_host_error)
        code=mission_clear(rt%ctx,ok); call check(rt,code)
        item=mavlink_mission_item_int_t(0.0,0.0,0.0,0.0,0,0,40.0,0_c_int16_t,16_c_int16_t, &
            0_c_int8_t,0_c_int8_t,3_c_int8_t,0_c_int8_t,1_c_int8_t,0_c_int8_t)
        do i=0,5
            item%command=16; item%x=100
            if (i==2) then
                item%command=word_pattern(nav_script_time); item%x=13; item%param1=72.0
            else if (i==3) then
                item%command=177; item%x=0; item%param1=5.0
            else if (i==4) then
                item%x=200
            else if (i==5) then
                item%x=300
            end if
            item%seq=int(i,c_int16_t)
            code=mission_set_item(rt%ctx,int(i,c_int16_t),item,ok); call check(rt,code)
            if (.not.ok) call check(rt,module_host_error)
        end do
        loc=mission_location(rt,2,.false.)
        if (loc%lat/=100) call check(rt,module_host_error)
        loc=mission_location(rt,3,.true.)
        if (loc%lat/=300) call check(rt,module_host_error)
        if (rt%error/=module_ok) return
        item%command=177; item%x=0; item%param1=1.0
        code=mission_set_item(rt%ctx,1_c_int16_t,item,ok); call check(rt,code)
        loc=mission_location(rt,1,.true.)
        if (rt%error/=module_invalid) then
            rt%error=module_host_error; return
        end if
        rt%error=module_ok; loc=mission_location(rt,65535,.false.)
        if (rt%error/=module_unavailable) then
            rt%error=module_host_error; return
        end if
        rt%error=module_ok
    end subroutine
    function run(ctx) bind(C,name='ap_module_fortran_run') result(status)
        type(c_ptr), value :: ctx
        integer(c_int32_t) :: status,code
        type(runtime) :: rt
        type(path_tree) :: tree
        type(controller) :: state
        type(configuration) :: p,fallback_p
        type(observation) :: first,input,bad
        type(control_output) :: out
        type(evaluation) :: eval
        character(len=48) :: name
        integer :: i,step,node
        integer(c_int64_t) :: count,hash
        integer(c_int32_t) :: words(0:511)
        integer :: cursor,saved_cursor,j,k
        logical :: found
        rt=runtime(ctx,module_ok); code=sleep_ms(ctx,2000_c_int32_t); call check(rt,code)
        nullify(tree%nodes); tree%allocation=c_null_ptr
        tree%used=0; tree%capacity=0; tree%root=0
        p=configuration(6000.0,0.1,80.0,5.0,25.0,0.25,1.0,2.0,2.8,60.0,1.0,0.0,50.0,1500.0, &
            0.5,1.0,25.0,0.25,15.0,9.0,15.0,18.0)
        count=0
        if (variant==2) then
            p%path_scale=-1.2; p%lookahead=0.0; p%ke_rudd=0.0; p%ang_accel=0.0; p%yaw_accel=500.0
            p%err_corr_p=1.5; count=200
        end if
        first=inputs(rt,0); eval=evaluation(target_speed(rt,first,p),p%entry_rate,.true.)
        call primitive_checks(rt,tree,p,eval)
        if (rt%error/=module_ok) goto 900
        call load_schedule(rt,'trick72.txt',tree,400.0,eval,found,name)
        if (.not.found) call check(rt,module_invalid)
        if (rt%error/=module_ok) goto 900
        call start_controller(rt,state,tree,first,p,0.0,count)
        do i=1,512
            if (rt%error/=module_ok) exit
            if (variant/=0) state%path_t=real(i,c_float)/520.0
            input=inputs(rt,i); call step_controller(rt,state,tree,input,p,step,out)
            if (step/=step_command) call check(rt,module_invalid)
            if (rt%error/=module_ok) exit
            call trace(rt,'AFCT','Id,Thr,R,P,Y,Rudd,Perr,Pt,Dt,Ert,Qct',i, &
                [out%throttle,out%rates%x,out%rates%y,out%rates%z,out%rudder,out%position_error, &
                out%path_t,out%dt,out%path_error_t,out%q_change_t])
            call trace(rt,'AFCS','Id,X,Y,Z,Q1,Q2,Q3,Q4,Cx,Cy,Cz,Px,Py,Pz,Ex',i, &
                [out%track_pos%x,out%track_pos%y,out%track_pos%z,out%track_attitude%q1,out%track_attitude%q2, &
                out%track_attitude%q3,out%track_attitude%q4,out%correction_rates%x,out%correction_rates%y, &
                out%correction_rates%z,out%path_rates%x,out%path_rates%y,out%path_rates%z,out%roll_error_rate])
            call trace(rt,'FASP','Id,Targ,Curr,P,I,Total,Add',i,out%speed_log)
            words=0; call save_controller(state,words(24:59)); cursor=64
            call path_cache(rt,tree,tree%root,words,cursor,.false.); saved_cursor=cursor
            hash=2166136261_c_int64_t
            do j=0,cursor-1
                do k=0,3
                    hash=iand(ieor(hash,int(iand(shiftr(words(j),8*k),255),c_int64_t))* &
                        16777619_c_int64_t,4294967295_c_int64_t)
                end do
            end do
            call trace(rt,'AFHS','Id,Lo,Hi,N',i,[real(iand(hash,65535_c_int64_t),c_float), &
                real(shiftr(hash,16),c_float),real(cursor,c_float)])
            call restore_controller(rt,state,words(24:59))
            if (mod(i,17)==0) then
                eval%announce=.false.
                call load_schedule(rt,'trick72.txt',tree,400.0,eval,found,name)
            end if
            cursor=64; call path_cache(rt,tree,tree%root,words,cursor,.true.)
            if (cursor/=saved_cursor) call check(rt,module_invalid)
            code=sleep_ms(ctx,5_c_int32_t); call check(rt,code)
        end do
        if (rt%error/=module_ok) goto 900
        input=inputs(rt,512); call step_controller(rt,state,tree,input,p,step,out)
        if (step/=step_waiting) call check(rt,module_host_error)
        bad=input; bad%attitude=quaternion(0.0,0.0,0.0,0.0)
        call step_controller(rt,state,tree,bad,p,step,out)
        if (rt%error/=module_invalid) then
            rt%error=module_host_error; goto 900
        end if
        rt%error=module_ok; input=inputs(rt,513); input%pos%z=100.0
        call step_controller(rt,state,tree,input,p,step,out)
        if (step/=step_abort) call check(rt,module_host_error)
        state%path_t=1.0; input=inputs(rt,514); call step_controller(rt,state,tree,input,p,step,out)
        if (step/=step_finished) call check(rt,module_host_error)
        if (rt%error/=module_ok) goto 900
        count=state%count; call start_controller(rt,state,tree,first,p,0.0,count)
        if (state%count/=count+1) call check(rt,module_host_error)
        bad=first; bad%now=bad%now-1.0; call step_controller(rt,state,tree,bad,p,step,out)
        if (rt%error/=module_invalid) then
            rt%error=module_host_error; goto 900
        end if
        rt%error=module_ok; call allocate_tree(rt,tree,1)
        node=new_node(rt,tree,straight); tree%root=node
        if (node==0) goto 900
        tree%nodes(node)%distance=100.0; call finish_leaf(rt,tree%nodes(node))
        input=first; input%pos=vector3f(2000000.0,0.0,-30.0); input%velocity=vector3f(18.0,0.0,0.0)
        input%eas2tas=1.0; input%airspeed=18.0; input%attitude=quaternion(1.0,0.0,0.0,0.0)
        fallback_p=p; fallback_p%path_scale=1.0; fallback_p%lookahead=epsilon(1.0_c_float)
        fallback_p%time_corr_p=0.0
        call start_controller(rt,state,tree,input,fallback_p,0.0,200_c_int64_t)
        input%now=input%now+0.025; call step_controller(rt,state,tree,input,fallback_p,step,out)
        if (step/=step_command.or..not.out%path_rate_fallback.or..not.out%lookahead_fallback.or. &
            out%path_rates%x/=0.0.or.out%path_rates%y/=0.0.or.out%path_rates%z/=0.0) call check(rt,module_host_error)
        call mission_checks(rt)
        if (rt%error==module_ok) call message(rt,6_c_int32_t,'AEROP Fortran controller done')
900     status=rt%error; call free_tree(tree)
    end function
end module aerobatics_replay
