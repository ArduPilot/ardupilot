! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_controller
    use aerobatics_path
    implicit none
    real(c_float), parameter :: loop_rate=40.0
    integer, parameter :: step_waiting=0,step_command=1,step_abort=2,step_finished=3
    type :: configuration
        real(c_float) :: ang_accel,ang_tc,thr_pit_ff,spd_p,spd_i,roll_corr_tc,time_corr_p,err_corr_p,err_corr_d
        real(c_float) :: entry_rate,thr_lookahead,thr_min,thr_boost,yaw_accel,lookahead,path_scale
        real(c_float) :: ke_rudd,ke_rudd_lookahead,alt_abort,airspeed_min,scaling_speed,airspeed_cruise
    end type
    type :: observation
        real(c_float) :: now,airspeed,eas2tas,throttle
        type(vector3f) :: pos,gyro,velocity
        type(quaternion) :: attitude
    end type
    type :: control_output
        real(c_float) :: throttle,rudder,position_error,path_t,path_error_t,q_change_t,dt,roll_error_rate,angle_error
        type(vector3f) :: rates,track_pos,correction_rates,path_rates
        type(quaternion) :: track_attitude,measured_attitude
        logical :: path_rate_fallback,lookahead_fallback,have_lookahead
        real(c_float) :: lookahead_rates(4),speed_log(6)
    end type
    type :: controller
        integer(c_int64_t) :: count
        real(c_float) :: target_speed,length,total_time,last_time,path_t,roll
        type(quaternion) :: initial_orientation,accumulated_orientation
        type(vector3f) :: initial_pos,last_rates,pos,shift,tangent
        logical :: last_shift_xy
        real(c_float) :: speed_p,speed_i,speed_integral,speed_total,speed_last_time
    end type
contains
    subroutine save_controller(state,words)
        type(controller), intent(in) :: state
        integer(c_int32_t), intent(out) :: words(0:35)
        words(0)=int(state%count,c_int32_t)
        words(1)=transfer(state%target_speed,0_c_int32_t)
        words(2)=transfer(state%length,0_c_int32_t)
        words(3)=transfer(state%total_time,0_c_int32_t)
        words(4)=transfer(state%initial_orientation%q1,0_c_int32_t)
        words(5)=transfer(state%initial_orientation%q2,0_c_int32_t)
        words(6)=transfer(state%initial_orientation%q3,0_c_int32_t)
        words(7)=transfer(state%initial_orientation%q4,0_c_int32_t)
        words(8)=transfer(state%initial_pos%x,0_c_int32_t)
        words(9)=transfer(state%initial_pos%y,0_c_int32_t)
        words(10)=transfer(state%initial_pos%z,0_c_int32_t)
        words(11)=transfer(state%accumulated_orientation%q1,0_c_int32_t)
        words(12)=transfer(state%accumulated_orientation%q2,0_c_int32_t)
        words(13)=transfer(state%accumulated_orientation%q3,0_c_int32_t)
        words(14)=transfer(state%accumulated_orientation%q4,0_c_int32_t)
        words(15)=transfer(state%last_time,0_c_int32_t)
        words(16)=transfer(state%last_rates%x,0_c_int32_t)
        words(17)=transfer(state%last_rates%y,0_c_int32_t)
        words(18)=transfer(state%last_rates%z,0_c_int32_t)
        words(19)=transfer(state%path_t,0_c_int32_t)
        words(20)=transfer(state%pos%x,0_c_int32_t)
        words(21)=transfer(state%pos%y,0_c_int32_t)
        words(22)=transfer(state%pos%z,0_c_int32_t)
        words(23)=transfer(state%roll,0_c_int32_t)
        words(24)=transfer(state%shift%x,0_c_int32_t)
        words(25)=transfer(state%shift%y,0_c_int32_t)
        words(26)=transfer(state%shift%z,0_c_int32_t)
        words(27)=merge(1,0,state%last_shift_xy)
        words(28)=transfer(state%tangent%x,0_c_int32_t)
        words(29)=transfer(state%tangent%y,0_c_int32_t)
        words(30)=transfer(state%tangent%z,0_c_int32_t)
        words(31)=transfer(state%speed_p,0_c_int32_t)
        words(32)=transfer(state%speed_i,0_c_int32_t)
        words(33)=transfer(state%speed_integral,0_c_int32_t)
        words(34)=transfer(state%speed_total,0_c_int32_t)
        words(35)=transfer(state%speed_last_time,0_c_int32_t)
    end subroutine
    subroutine restore_controller(rt,state,words)
        type(runtime), intent(inout) :: rt
        type(controller), intent(out) :: state
        integer(c_int32_t), intent(in) :: words(0:35)
        integer :: i
        real(c_float) :: norm
        do i=1,35
            if (i==27) cycle
            if (.not.finite(transfer(words(i),0.0_c_float))) call check(rt,module_invalid)
        end do
        if (words(27)<0.or.words(27)>1) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        state%count=iand(int(words(0),c_int64_t),4294967295_c_int64_t)
        state%target_speed=transfer(words(1),0.0_c_float)
        state%length=transfer(words(2),0.0_c_float)
        state%total_time=transfer(words(3),0.0_c_float)
        state%initial_orientation%q1=transfer(words(4),0.0_c_float)
        state%initial_orientation%q2=transfer(words(5),0.0_c_float)
        state%initial_orientation%q3=transfer(words(6),0.0_c_float)
        state%initial_orientation%q4=transfer(words(7),0.0_c_float)
        state%initial_pos%x=transfer(words(8),0.0_c_float)
        state%initial_pos%y=transfer(words(9),0.0_c_float)
        state%initial_pos%z=transfer(words(10),0.0_c_float)
        state%accumulated_orientation%q1=transfer(words(11),0.0_c_float)
        state%accumulated_orientation%q2=transfer(words(12),0.0_c_float)
        state%accumulated_orientation%q3=transfer(words(13),0.0_c_float)
        state%accumulated_orientation%q4=transfer(words(14),0.0_c_float)
        state%last_time=transfer(words(15),0.0_c_float)
        state%last_rates%x=transfer(words(16),0.0_c_float)
        state%last_rates%y=transfer(words(17),0.0_c_float)
        state%last_rates%z=transfer(words(18),0.0_c_float)
        state%path_t=transfer(words(19),0.0_c_float)
        state%pos%x=transfer(words(20),0.0_c_float)
        state%pos%y=transfer(words(21),0.0_c_float)
        state%pos%z=transfer(words(22),0.0_c_float)
        state%roll=transfer(words(23),0.0_c_float)
        state%shift%x=transfer(words(24),0.0_c_float)
        state%shift%y=transfer(words(25),0.0_c_float)
        state%shift%z=transfer(words(26),0.0_c_float)
        state%last_shift_xy=words(27)/=0
        state%tangent%x=transfer(words(28),0.0_c_float)
        state%tangent%y=transfer(words(29),0.0_c_float)
        state%tangent%z=transfer(words(30),0.0_c_float)
        state%speed_p=transfer(words(31),0.0_c_float)
        state%speed_i=transfer(words(32),0.0_c_float)
        state%speed_integral=transfer(words(33),0.0_c_float)
        state%speed_total=transfer(words(34),0.0_c_float)
        state%speed_last_time=transfer(words(35),0.0_c_float)
        if (state%target_speed<=0.0.or.state%length<=0.0.or.state%total_time<=0.0) call check(rt,module_invalid)
        norm=state%initial_orientation%q1**2+state%initial_orientation%q2**2+ &
            state%initial_orientation%q3**2+state%initial_orientation%q4**2
        if (abs(norm-1.0)>0.01) call check(rt,module_invalid)
        norm=state%accumulated_orientation%q1**2+state%accumulated_orientation%q2**2+ &
            state%accumulated_orientation%q3**2+state%accumulated_orientation%q4**2
        if (abs(norm-1.0)>0.01) call check(rt,module_invalid)
    end subroutine
    subroutine validate_configuration(rt,p)
        type(runtime), intent(inout) :: rt
        type(configuration), intent(in) :: p
        real(c_float) :: values(22)
        integer :: k
        values=[p%ang_accel,p%ang_tc,p%thr_pit_ff,p%spd_p,p%spd_i,p%roll_corr_tc,p%time_corr_p,p%err_corr_p, &
            p%err_corr_d,p%entry_rate,p%thr_lookahead,p%thr_min,p%thr_boost,p%yaw_accel,p%lookahead,p%path_scale, &
            p%ke_rudd,p%ke_rudd_lookahead,p%alt_abort,p%airspeed_min,p%scaling_speed,p%airspeed_cruise]
        do k=1,22
            if (.not.finite(values(k))) call check(rt,module_invalid)
        end do
        if (p%roll_corr_tc<=0.0.or.p%ang_tc<0.0.or.p%entry_rate<=0.0.or.p%path_scale==0.0.or. &
            p%airspeed_min<=0.0.or.p%scaling_speed<=0.0.or.p%airspeed_cruise<=0.0.or. &
            p%thr_min<0.0.or.p%thr_min>100.0.or.p%thr_boost<0.0.or.p%thr_boost>100.0) call check(rt,module_invalid)
    end subroutine
    subroutine validate_observation(rt,input)
        type(runtime), intent(inout) :: rt
        type(observation), intent(in) :: input
        real(c_float) :: norm
        norm=input%attitude%q1**2+input%attitude%q2**2+input%attitude%q3**2+input%attitude%q4**2
        if (.not.finite(input%now).or..not.finite_v(input%pos).or..not.finite_v(input%gyro).or. &
            .not.finite_v(input%velocity).or..not.finite(input%airspeed).or.input%airspeed<0.0.or. &
            .not.finite(input%eas2tas).or.input%eas2tas<=0.0.or..not.finite(input%throttle).or. &
            .not.finite(norm).or.abs(norm-1.0)>0.01) call check(rt,module_invalid)
    end subroutine
    real(c_float) function target_speed(rt,input,p) result(speed)
        type(runtime), intent(inout) :: rt
        type(observation), intent(in) :: input
        type(configuration), intent(in) :: p
        real(c_float) :: length
        length=vlength(rt,input%velocity); speed=lua_max(input%eas2tas*p%airspeed_cruise,length)
    end function
    subroutine sample(rt,state,tree,t,input,p,point,angle,attrs)
        type(runtime), intent(inout) :: rt
        type(controller), intent(in) :: state
        type(path_tree), intent(inout) :: tree
        real(c_float), intent(in) :: t
        type(observation), intent(in) :: input
        type(configuration), intent(in) :: p
        type(vector3f), intent(out) :: point
        real(c_float), intent(out) :: angle
        type(attributes), intent(out) :: attrs
        real(c_float) :: u,corr
        type(evaluation) :: eval
        type(quaternion) :: inverse,twice
        point=vector3f(0.0,0.0,0.0); angle=0.0; attrs=empty_attributes()
        if (rt%error/=module_ok) return
        u=constrain(t,0.0,1.0); eval=evaluation(target_speed(rt,input,p),p%entry_rate,.true.)
        point=position(rt,tree,tree%root,u,eval)
        angle=path_roll(rt,tree,tree%root,u,state%total_time,eval); corr=correction(rt,tree,tree%root,u,eval)
        attrs=path_attributes(rt,tree,tree%root,u,eval)
        point=vadd(point,state%shift); point=rotate(rt,state%initial_orientation,point)
        point=vscale(point,abs(p%path_scale))
        if (p%path_scale<0.0) then
            point%y=-point%y; corr=-corr; angle=-angle
            inverse=qinverse(rt,state%initial_orientation); twice=qmul(rt,inverse,inverse)
            point=unrotate(rt,twice,point)
        end if
        point=vadd(point,state%initial_pos); angle=rad(angle+corr)
    end subroutine
    subroutine start_controller(rt,state,tree,input,p,yaw,count)
        type(runtime), intent(inout) :: rt
        type(controller), intent(out) :: state
        type(path_tree), intent(inout) :: tree
        type(observation), intent(in) :: input
        type(configuration), intent(in) :: p
        real(c_float), intent(in) :: yaw
        integer(c_int64_t), intent(in) :: count
        type(vector3f) :: point
        type(attributes) :: attrs
        real(c_float) :: angle
        call validate_configuration(rt,p); call validate_observation(rt,input)
        if (.not.finite(yaw)) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        state%target_speed=target_speed(rt,input,p); state%length=tree%nodes(tree%root)%length*abs(p%path_scale)
        if (.not.finite(state%target_speed).or.state%target_speed<=0.0.or. &
            .not.finite(state%length).or.state%length<=0.0) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        state%total_time=state%length/state%target_speed
        state%initial_orientation=orient(rt,0.0,0.0,yaw); call normalize_q(rt,state%initial_orientation)
        tree%nodes(tree%root)%highest=0
        state%count=min(count+1,4294967295_c_int64_t); state%initial_pos=input%pos
        state%accumulated_orientation=state%initial_orientation; state%last_time=input%now-1.0/loop_rate
        state%last_rates=vscale(input%gyro,deg(1.0)); state%path_t=0.0; state%pos=input%pos; state%roll=0.0
        state%shift=vector3f(0.0,0.0,0.0); state%last_shift_xy=.false.; state%tangent=vector3f(0.0,0.0,0.0)
        state%speed_p=p%spd_p; state%speed_i=p%spd_i; state%speed_integral=input%throttle
        state%speed_total=0.0; state%speed_last_time=input%now
        call sample(rt,state,tree,0.1/(state%total_time*loop_rate),input,p,point,angle,attrs)
        state%tangent=vsub(point,state%pos)
    end subroutine
    function ortho(rt,x,unit_vector) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: x,unit_vector
        type(vector3f) :: v,temp
        temp=vcross(rt,unit_vector,x); v=vcross(rt,unit_vector,temp)
    end function
    subroutine speed_update(rt,state,now,target,current,pitch,ff,throttle,log_values)
        type(runtime), intent(inout) :: rt
        type(controller), intent(inout) :: state
        real(c_float), intent(in) :: now,target,current,pitch,ff
        real(c_float), intent(out) :: throttle,log_values(6)
        real(c_float) :: dt,error,proportional,feedforward
        dt=now-state%speed_last_time; state%speed_last_time=now; error=target-current
        proportional=state%speed_p*error
        if ((state%speed_total<100.0.and.state%speed_total>0.0).or. &
            (state%speed_total>=100.0.and.error<0.0).or.(state%speed_total<=0.0.and.error>0.0)) &
            state%speed_integral=state%speed_integral+state%speed_i*error*dt
        state%speed_integral=constrain(state%speed_integral,-100.0,100.0)
        state%speed_total=constrain(proportional+state%speed_integral,0.0,100.0)
        feedforward=fsin(rt,pitch)*ff; throttle=state%speed_total+feedforward
        log_values=[target,current,proportional,state%speed_integral,state%speed_total,feedforward]
    end subroutine
    subroutine step_controller(rt,state,tree,input,p,step,output)
        type(runtime), intent(inout) :: rt
        type(controller), intent(inout) :: state
        type(path_tree), intent(inout) :: tree
        type(observation), intent(in) :: input
        type(configuration), intent(in) :: p
        integer, intent(out) :: step
        type(control_output), intent(out) :: output
        type(quaternion) :: attitude,delta,tempq,rolled,desired,roll_error,change,anticipated,inverse
        type(vector3f) :: p0,p1,p2,point,current,path_point,shift,tangent1,tangent2,tangent3,tv,pos_error
        type(vector3f) :: b,b_dot,acceleration,acc_body,corr,path_rate,rate_earth,rate_body,axis,axis_angle,error_axis
        type(vector3f) :: error_rate,error_body,rate,ahead,total,difference,rates,alt_delta
        type(attributes) :: attrs,lookahead_attrs
        real(c_float) :: velocity_length,dt,local_dt,airspeed,r0,r1,unused,path_distance,path_delta,last_path_t
        real(c_float) :: distance_error,path_error_t,q_change_t,alpha,tas,angle,ratio,max_delta,max_yaw
        real(c_float) :: rudder,scale,pitch,throttle,minimum,pitch1,pitch2
        logical :: shift_xy,path_fallback,lookahead_fallback
        integer(c_int32_t) :: code
        step=step_waiting
        call validate_configuration(rt,p); call validate_observation(rt,input)
        if (rt%error/=module_ok) return
        state%count=min(state%count+1,4294967295_c_int64_t)
        attitude=qinverse(rt,input%attitude); velocity_length=vlength(rt,input%velocity); dt=input%now-state%last_time
        if (dt<0.0) call check(rt,module_invalid)
        if (dt<0.25/loop_rate.or.rt%error/=module_ok) return
        if (.not.finite(dt)) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        state%last_time=input%now; local_dt=(1.0/loop_rate)/state%total_time
        if (state%path_t+local_dt>1.0) then
            step=step_finished; return
        end if
        airspeed=lua_max(p%airspeed_min,input%airspeed); p0=state%pos; r0=state%roll
        call sample(rt,state,tree,state%path_t+local_dt,input,p,p1,unused,attrs)
        shift_xy=attrs%have_shift.and.attrs%shift_xy
        if (shift_xy.and..not.state%last_shift_xy) then
            current=unrotate(rt,state%initial_orientation,input%pos)
            path_point=unrotate(rt,state%initial_orientation,p1); shift=vsub(current,path_point); shift%z=0.0
            state%shift=vadd(state%shift,shift); point=rotate(rt,state%initial_orientation,shift); p1=vadd(p1,point)
            p0%x=p1%x; p0%y=p1%y
        end if
        state%last_shift_xy=shift_xy; tangent1=state%tangent; tangent2=vsub(p1,p0); tv=unit(rt,tangent2)
        path_distance=vdot(rt,input%velocity,tv)*dt
        path_delta=constrain(path_distance/state%length,0.2*local_dt,4.0*local_dt)
        call sample(rt,state,tree,constrain(state%path_t+path_delta,0.0,1.0),input,p,p1,unused,attrs)
        last_path_t=state%path_t; state%path_t=state%path_t+path_delta
        tangent2=vsub(p1,p0); tv=unit(rt,tangent2); pos_error=vsub(input%pos,p1)
        distance_error=vdot(rt,tv,pos_error)
        path_error_t=constrain(distance_error/state%length,-0.9*path_delta,2.0*path_delta)
        state%path_t=state%path_t+p%time_corr_p*path_error_t
        call sample(rt,state,tree,constrain(state%path_t,0.0,1.0),input,p,point,r1,attrs)
        p1=point; tangent2=vsub(p1,p0); q_change_t=(state%path_t-last_path_t)*state%total_time
        r1=state%roll+wrappi(rt,r1-state%roll); alpha=q_change_t/(q_change_t+p%ang_tc/(pi*2.0))
        r1=(1.0-alpha)*state%roll+alpha*r1; r1=wrappi(rt,r1)
        state%tangent=tangent2; state%pos=p1; state%roll=r1
        b=ortho(rt,pos_error,tv); b_dot=ortho(rt,input%velocity,tv)
        acceleration=vadd(vscale(b,p%err_corr_p),vscale(b_dot,p%err_corr_d))
        if (attrs%have_gain) acceleration=vscale(acceleration,attrs%pos_gain_mul)
        if (attrs%have_corr) acceleration=vscale(acceleration,attrs%pos_corr)
        acc_body=rotate(rt,attitude,acceleration); tas=constrain(input%eas2tas*airspeed,3.0,100.0)
        corr=vector3f(0.0,-acc_body%z/tas,acc_body%y/tas)
        if (nan_v(corr)) corr=vector3f(0.0,0.0,0.0)
        corr=vscale(corr,deg(1.0))
        if (state%count<2) corr=vector3f(0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        path_rate=tangents_rate(rt,tangent1,tangent2,dt); path_fallback=.false.
        if (rt%error==module_invalid .or. (rt%error==module_ok.and..not.finite_v(path_rate))) then
            rt%error=module_ok; path_rate=vector3f(0.0,0.0,0.0); path_fallback=.true.
        end if
        if (rt%error/=module_ok) return
        rate_earth=vscale(path_rate,deg(1.0))
        if (state%count<3) rate_earth%z=0.0
        rate_body=rotate(rt,attitude,rate_earth); rate_body%x=deg(wrappi(rt,r1-r0)/dt)
        delta=angular_velocity(rt,path_rate,dt); tempq=qmul(rt,delta,state%accumulated_orientation)
        state%accumulated_orientation=tempq; call normalize_q(rt,state%accumulated_orientation)
        axis=rotate(rt,state%accumulated_orientation,vector3f(1.0,0.0,0.0)); rolled=axisq(rt,axis,r1)
        desired=qmul(rt,rolled,state%accumulated_orientation); roll_error=qmul(rt,desired,attitude)
        call normalize_q(rt,roll_error)
        if (rt%error/=module_ok) return
        code=quaternion_to_axis_angle(rt%ctx,roll_error,axis_angle); call check(rt,code)
        angle=vlength(rt,axis_angle)
        if (angle<0.00001) then
            angle=0.0; error_axis=vector3f(1.0,0.0,0.0)
        else
            error_axis=vscale(axis_angle,1.0/angle)
        end if
        error_rate=vscale(error_axis,angle/p%roll_corr_tc); error_body=rotate(rt,attitude,error_rate)
        error_body=vscale(error_body,deg(1.0)); error_body%y=0.0; error_body%z=0.0
        output%have_lookahead=.false.; output%lookahead_rates=0.0; lookahead_fallback=.false.
        if (p%lookahead>0.0) then
            call sample(rt,state,tree,constrain(state%path_t+p%lookahead/state%total_time,0.0,1.0), &
                input,p,p2,unused,lookahead_attrs)
            tangent3=vsub(p2,p1)
            if (rt%error/=module_ok) return
            rate=tangents_rate(rt,tangent2,tangent3,0.5*(p%lookahead+1.0/loop_rate))
            if (rt%error==module_invalid) then
                rt%error=module_ok; lookahead_fallback=.true.
            else if (rt%error==module_ok) then
                ratio=velocity_length/state%target_speed; rate=vscale(rate,ratio*ratio)
                ahead=rotate(rt,attitude,rate); ahead=vscale(ahead,deg(1.0))
                output%have_lookahead=.true.; output%lookahead_rates=[rate_body%y,ahead%y,rate_body%z,ahead%z]
                if (finite_v(ahead)) then
                    rate_body%y=ahead%y; rate_body%z=ahead%z
                else
                    lookahead_fallback=.true.
                end if
            end if
        end if
        total=vadd(rate_body,corr); total=vadd(total,error_body); total=vadd(total,vector3f(0.0,0.0,0.0))
        difference=vsub(total,state%last_rates); max_delta=p%ang_accel*dt; max_yaw=max_delta
        if (p%yaw_accel>0.0.and.(p%yaw_accel<p%ang_accel.or.p%ang_accel<=0.0)) max_yaw=p%yaw_accel*dt
        if (max_delta>0.0) then
            difference%x=constrain(difference%x,-max_delta,max_delta)
            difference%y=constrain(difference%y,-max_delta,max_delta)
        end if
        if (max_yaw>0.0) difference%z=constrain(difference%z,-max_yaw,max_yaw)
        rates=vadd(state%last_rates,difference); state%last_rates=rates; rudder=0.0
        if (p%ke_rudd>0.0) then
            change=angular_velocity(rt,input%gyro,-p%ke_rudd_lookahead); anticipated=qmul(rt,change,attitude)
            scale=p%scaling_speed/airspeed; inverse=qinverse(rt,anticipated)
            rudder=constrain(-2.0*(inverse%q3*inverse%q4+inverse%q1*inverse%q2)*p%ke_rudd*(scale*scale),-100.0,100.0)
        end if
        change=angular_velocity(rt,path_rate,p%thr_lookahead); anticipated=qmul(rt,change,desired)
        pitch1=qpitch(rt,anticipated); pitch2=qpitch(rt,desired); pitch=lua_max(pitch1,pitch2)
        call speed_update(rt,state,input%now,state%target_speed,velocity_length,pitch,p%thr_pit_ff,throttle,output%speed_log)
        minimum=p%thr_min
        if (attrs%have_boost.and.attrs%thr_boost) minimum=lua_max(minimum,p%thr_boost)
        throttle=constrain(throttle,minimum,100.0)
        if (.not.finite(throttle).or..not.finite_v(rates).or..not.finite(rudder)) call check(rt,module_invalid)
        output%throttle=throttle; output%rates=rates; output%rudder=rudder; output%position_error=vlength(rt,pos_error)
        output%path_rate_fallback=path_fallback; output%lookahead_fallback=lookahead_fallback
        output%track_pos=p1; output%track_attitude=desired; output%measured_attitude=input%attitude
        output%path_t=state%path_t; output%path_error_t=path_error_t; output%q_change_t=q_change_t; output%dt=dt
        output%correction_rates=corr; output%path_rates=rate_body; output%roll_error_rate=error_body%x
        output%angle_error=wrap180(rt,deg(angle)); alt_delta=vsub(input%pos,state%initial_pos)
        step=step_command
        if (alt_delta%z>p%alt_abort) step=step_abort
    end subroutine
end module aerobatics_controller
