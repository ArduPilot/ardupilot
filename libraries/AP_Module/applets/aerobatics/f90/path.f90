! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_path
    use aerobatics_support
    implicit none
    integer, parameter :: straight=1, reverse=2, vertical=3, horizontal=4, cylinder=5, composite=6, alignment=7
    integer, parameter :: roll_angle=1, roll_entry_exit=2, roll_entry=3, roll_exit=4, roll_sequence=5
    integer, parameter :: cmd_loop=1, cmd_reverse_cuban=2, cmd_figure8=3, cmd_immelmann=4, cmd_roll=5, cmd_split=6
    integer, parameter :: cmd_circle=7, cmd_humpty=8, cmd_cuban=9, cmd_barrel=10, cmd_tophat=11, cmd_triangle=12
    integer, parameter :: cmd_align_box=13, cmd_align_center=14
    type, bind(C) :: attributes
        logical(c_bool) :: have_roll,have_orient,have_boost,have_corr,have_gain,have_shift
        real(c_float) :: roll_ref,pos_corr,pos_gain_mul
        type(quaternion) :: set_orient
        logical(c_bool) :: thr_boost,shift_xy
    end type
    type :: evaluation
        real(c_float) :: speed,entry_rate
        logical :: announce
    end type
    ! A compact linked arena retains both levels of the Rust path hierarchy.
    ! Each child's segment-start data lives on that child. Indices are one-based.
    type, bind(C) :: path_node
        integer(c_int32_t) :: kind,first,last,next,highest,last_index,roll_kind
        real(c_float) :: radius,angle,height,distance,spirals,gamma,roll_value,length
        real(c_float) :: start_time,start_angle,start_correction,low,high,barrel_radius
        type(vector3f) :: start_pos
        type(quaternion) :: start_orientation,final_orientation
        type(attributes) :: attrs
        logical(c_bool) :: have_extents,rebuild,barrel,align_present
        character(c_char) :: message(50)
        integer(c_int32_t) :: message_length
    end type
    type :: path_tree
        type(path_node), pointer :: nodes(:)
        type(c_ptr) :: allocation
        integer :: used,capacity,root
    end type
contains
    recursive subroutine path_cache(rt,tree,node,words,cursor,restoring)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: node
        integer(c_int32_t), intent(inout) :: words(0:511)
        integer, intent(inout) :: cursor
        logical, intent(in) :: restoring
        integer :: child,n,index,last,flags
        real(c_float) :: bank
        if (rt%error/=module_ok.or.node==0) return
        if (tree%nodes(node)%kind/=composite) return
        if (cursor+4>512) then
            call check(rt,module_invalid); return
        end if
        child=tree%nodes(node)%first; n=0; last=-1
        do while(child/=0)
            if (child==tree%nodes(node)%last_index) last=n
            n=n+1; child=tree%nodes(child)%next
        end do
        flags=n+merge(65536,0,tree%nodes(node)%barrel)
        if (restoring) then
            bank=transfer(words(cursor+3),0.0_c_float)
            if (words(cursor)/=flags.or.words(cursor+1)<0.or.words(cursor+1)>n.or. &
                words(cursor+2)<0.or.words(cursor+2)>=n.or..not.finite(bank)) then
                call check(rt,module_invalid); return
            end if
            tree%nodes(node)%highest=words(cursor+1)
            child=tree%nodes(node)%first
            do index=1,words(cursor+2)
                child=tree%nodes(child)%next
            end do
            tree%nodes(node)%last_index=child
            if (tree%nodes(node)%barrel) then
                tree%nodes(tree%nodes(node)%first)%roll_value=-bank
                tree%nodes(tree%nodes(node)%last)%roll_value=bank
            end if
        else
            if (last<0) then
                call check(rt,module_invalid); return
            end if
            bank=0.0
            if (tree%nodes(node)%barrel) bank=-tree%nodes(tree%nodes(node)%first)%roll_value
            words(cursor:cursor+3)=[flags,tree%nodes(node)%highest,last,transfer(bank,0_c_int32_t)]
        end if
        cursor=cursor+4; child=tree%nodes(node)%first
        do while(child/=0)
            call path_cache(rt,tree,child,words,cursor,restoring)
            child=tree%nodes(child)%next
        end do
    end subroutine
    function empty_attributes() result(a)
        type(attributes) :: a
        a%have_roll=.false.; a%have_orient=.false.; a%have_boost=.false.
        a%have_corr=.false.; a%have_gain=.false.; a%have_shift=.false.
        a%roll_ref=0.0; a%pos_corr=0.0; a%pos_gain_mul=0.0
        a%set_orient=quaternion(1.0,0.0,0.0,0.0); a%thr_boost=.false.; a%shift_xy=.false.
    end function
    subroutine free_tree(tree)
        type(path_tree), intent(inout) :: tree
        if (c_associated(tree%allocation)) call arena_free(tree%allocation)
        tree%allocation=c_null_ptr; nullify(tree%nodes); tree%used=0; tree%capacity=0; tree%root=0
    end subroutine
    subroutine allocate_tree(rt,tree,capacity)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: capacity
        type(path_node) :: node
        call free_tree(tree)
        if (rt%error/=module_ok) return
        tree%allocation=arena_allocate(int(capacity,c_size_t)*c_sizeof(node))
        if (.not.c_associated(tree%allocation)) then
            call check(rt,module_no_memory); return
        end if
        call c_f_pointer(tree%allocation,tree%nodes,[capacity])
        tree%capacity=capacity
    end subroutine
    integer function new_node(rt,tree,kind) result(i)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: kind
        type(path_node) :: n
        i=0
        if (rt%error/=module_ok) return
        if (tree%used>=tree%capacity) then
            call check(rt,module_no_memory); return
        end if
        tree%used=tree%used+1; i=tree%used
        n%kind=kind; n%first=0; n%last=0; n%next=0; n%highest=0; n%last_index=0; n%roll_kind=roll_angle
        n%radius=0.0; n%angle=0.0; n%height=0.0; n%distance=0.0; n%spirals=0.0; n%gamma=0.0
        n%roll_value=0.0; n%length=0.0; n%start_time=0.0; n%start_angle=0.0; n%start_correction=0.0
        n%low=0.0; n%high=0.0; n%barrel_radius=0.0; n%start_pos=vector3f(0.0,0.0,0.0)
        n%start_orientation=quaternion(1.0,0.0,0.0,0.0); n%final_orientation=n%start_orientation
        n%attrs=empty_attributes(); n%have_extents=.false.; n%rebuild=.false.; n%barrel=.false.
        n%align_present=.false.; n%message=' '; n%message_length=0
        tree%nodes(i)=n
    end function
    subroutine append_node(tree,parent,child)
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent,child
        integer :: previous
        if (parent==0 .or. child==0) return
        previous=tree%nodes(parent)%last
        if (previous==0) then
            tree%nodes(parent)%first=child
        else
            tree%nodes(previous)%next=child
        end if
        tree%nodes(parent)%last=child
    end subroutine
    real(c_float) function roll_get(kind,angle,t,duration,entry_rate) result(v)
        integer, intent(in) :: kind
        real(c_float), intent(in) :: angle,t,duration,entry_rate
        real(c_float) :: entry_t,local
        v=0.0
        select case(kind)
        case(roll_angle)
            if (angle/=0.0) v=angle*t
        case(roll_entry_exit)
            entry_t=lua_min((abs(angle)/entry_rate)/duration,0.5)
            if (t<=0.0) return
            if (t<entry_t) then
                v=angle*t/entry_t
            else if (t<1.0-entry_t) then
                v=angle
            else if (angle/=0.0 .and. t<1.0) then
                v=(1.0-(t-(1.0-entry_t))/entry_t)*angle
            end if
        case(roll_entry)
            entry_t=lua_min((abs(angle)/entry_rate)/duration,0.5)
            if (t<entry_t) then
                v=angle*t/entry_t
            else
                v=angle
            end if
        case(roll_exit)
            entry_t=(abs(angle)/entry_rate)/duration
            if (t<1.0-entry_t .or. angle==0.0) return
            v=((t-(1.0-entry_t))/entry_t)*angle
        case(roll_sequence)
            ! The two custom maneuvers have precisely [0,angle,0] equal-weight segments.
            if (t<=1.0/3.0) then
                v=0.0*t
            else if (t<=2.0/3.0) then
                local=(t-1.0/3.0)/(1.0/3.0); v=local*angle
            else
                v=angle
            end if
        end select
    end function
    function geometry_position(rt,n,t) result(v)
        type(runtime), intent(inout) :: rt
        type(path_node), intent(in) :: n
        real(c_float), intent(in) :: t
        type(vector3f) :: v,w
        type(quaternion) :: q
        real(c_float) :: a,si,co
        v=vector3f(0.0,0.0,0.0)
        select case(n%kind)
        case(straight)
            v%x=n%distance*t
        case(reverse)
            if (t<0.5) then
                v%x=n%distance*t*2.0
            else
                v%x=n%distance-n%distance*(t-0.5)*2.0
            end if
        case(vertical)
            a=wrap2pi(rt,t*rad(n%angle)); si=fsin(rt,a); co=fcos(rt,a)
            v=vector3f(abs(n%radius)*si,0.0,-n%radius*(1.0-co))
        case(horizontal)
            a=t*rad(n%angle); si=fsin(rt,a); co=fcos(rt,a)
            v=vector3f(abs(n%radius)*si,n%radius*(1.0-co),-n%height*t)
        case(cylinder)
            a=t*n%spirals*pi*2.0; si=fsin(rt,a+pi); co=fcos(rt,a)
            w=vector3f(n%distance*t,abs(n%radius)*si,-abs(n%radius)*(1.0-co))
            q=axisq(rt,vector3f(0.0,0.0,1.0),0.5*pi-n%gamma); v=rotate(rt,q,w)
            if (n%radius<0.0) v%y=-v%y
        case default
            call check(rt,module_invalid)
        end select
    end function
    real(c_float) function geometry_correction(rt,n,t) result(v)
        type(runtime), intent(inout) :: rt
        type(path_node), intent(in) :: n
        real(c_float), intent(in) :: t
        real(c_float) :: gamma,si
        v=0.0
        if (n%kind==horizontal .and. n%height/=0.0) then
            gamma=atan2f(rt,n%height*(360.0/n%angle)/(2.0*pi*n%radius),1.0)
            si=fsin(rt,gamma); v=-t*n%angle*si
        else if (n%kind==cylinder) then
            si=fsin(rt,n%gamma); v=sgn(n%radius)*t*360.0*si*n%spirals
        end if
    end function
    subroutine finish_leaf(rt,n)
        type(runtime), intent(inout) :: rt
        type(path_node), intent(inout) :: n
        real(c_float) :: circumference,h,a
        n%final_orientation=quaternion(1.0,0.0,0.0,0.0)
        select case(n%kind)
        case(straight)
            n%length=n%distance
        case(reverse)
            n%length=n%distance*2.0
        case(vertical)
            n%length=abs(n%radius)*2.0*pi*abs(n%angle)/360.0
            a=sgn(n%radius)*rad(wrap180(rt,n%angle))
            n%final_orientation=axisq(rt,vector3f(0.0,1.0,0.0),a)
            call normalize_q(rt,n%final_orientation)
        case(horizontal)
            circumference=2.0*pi*abs(n%radius); h=n%height*360.0/abs(n%angle)
            n%length=fsqrt(rt,h*h+circumference*circumference)*abs(n%angle)/360.0
            n%final_orientation=axisq(rt,vector3f(0.0,0.0,1.0),sgn(n%radius)*rad(n%angle))
        case(cylinder)
            n%gamma=atan2f(rt,(n%distance/n%spirals)/(2.0*pi*abs(n%radius)),1.0)
            circumference=2.0*pi*abs(n%radius); h=n%distance/n%spirals
            n%length=fsqrt(rt,h*h+circumference*circumference)*n%spirals
        end select
    end subroutine
    real(c_float) function barrel_bank(rt,radius,speed) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: radius,speed
        v=deg(atan2f(rt,(speed*speed)/(radius*9.80665),1.0))
    end function
    subroutine activate(rt,tree,i,speed)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: speed
        integer :: first,last,middle
        real(c_float) :: bank
        if (tree%nodes(i)%kind/=composite) return
        tree%nodes(i)%highest=0; tree%nodes(i)%last_index=tree%nodes(i)%last
        if (tree%nodes(i)%barrel) then
            first=tree%nodes(i)%first; middle=tree%nodes(first)%next; last=tree%nodes(i)%last
            if (middle==0) then
                call check(rt,module_invalid); return
            end if
            if (tree%nodes(middle)%next/=last .or. first==last) then
                call check(rt,module_invalid); return
            end if
            bank=barrel_bank(rt,tree%nodes(i)%barrel_radius,speed)
            tree%nodes(first)%roll_value=-bank; tree%nodes(last)%roll_value=bank
        end if
    end subroutine
    subroutine select_segment(rt,tree,i,t,eval,j,u)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t
        type(evaluation), intent(in) :: eval
        integer, intent(out) :: j
        real(c_float), intent(out) :: u
        integer :: rank,k,n
        character(len=50) :: text
        j=tree%nodes(i)%first; rank=1; u=0.0
        if (.not.finite(eval%speed) .or. eval%speed<=0.0 .or. &
            .not.finite(eval%entry_rate) .or. eval%entry_rate<=0.0 .or. j==0) then
            call check(rt,module_invalid); return
        end if
        do while (tree%nodes(j)%next/=0)
            if (t<tree%nodes(j)%start_time+tree%nodes(j)%length/tree%nodes(i)%length) exit
            j=tree%nodes(j)%next; rank=rank+1
        end do
        if (tree%nodes(i)%last_index/=j) then
            if (tree%nodes(j)%rebuild) call activate(rt,tree,j,eval%speed)
            tree%nodes(i)%last_index=j
        end if
        u=(t-tree%nodes(j)%start_time)/(tree%nodes(j)%length/tree%nodes(i)%length)
        if (rank>tree%nodes(i)%highest .and. t>0.0 .and. t<1.0) then
            n=tree%nodes(j)%message_length
            if (eval%announce .and. n>0) then
                text=''
                do k=1,n
                    text(k:k)=tree%nodes(j)%message(k)
                end do
                call message(rt,1_c_int32_t,text(1:n))
            end if
            tree%nodes(i)%highest=rank
        end if
    end subroutine
    recursive function position(rt,tree,i,t,eval) result(v)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t
        type(evaluation), intent(in) :: eval
        type(vector3f) :: v,w
        integer :: j
        real(c_float) :: u
        v=vector3f(0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        if (tree%nodes(i)%kind==composite) then
            call select_segment(rt,tree,i,t,eval,j,u)
            if (rt%error/=module_ok) return
            w=position(rt,tree,j,u,eval); v=rotate(rt,tree%nodes(j)%start_orientation,w)
            v=vadd(v,tree%nodes(j)%start_pos)
        else
            v=geometry_position(rt,tree%nodes(i),t)
        end if
    end function
    recursive real(c_float) function path_roll(rt,tree,i,t,duration,eval) result(v)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t,duration
        type(evaluation), intent(in) :: eval
        integer :: j
        real(c_float) :: u
        v=0.0
        if (rt%error/=module_ok) return
        if (tree%nodes(i)%kind==composite) then
            call select_segment(rt,tree,i,t,eval,j,u)
            if (rt%error/=module_ok) return
            v=path_roll(rt,tree,j,u,tree%nodes(j)%length/eval%speed,eval)+tree%nodes(j)%start_angle
        else if (tree%nodes(i)%kind==alignment) then
            call check(rt,module_invalid)
        else
            v=roll_get(tree%nodes(i)%roll_kind,tree%nodes(i)%roll_value,t,duration,eval%entry_rate)
            v=wrap180(rt,v)
        end if
    end function
    recursive real(c_float) function correction(rt,tree,i,t,eval) result(v)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t
        type(evaluation), intent(in) :: eval
        integer :: j
        real(c_float) :: u
        v=0.0
        if (rt%error/=module_ok) return
        if (tree%nodes(i)%kind==composite) then
            call select_segment(rt,tree,i,t,eval,j,u)
            if (rt%error/=module_ok) return
            v=correction(rt,tree,j,u,eval)+tree%nodes(j)%start_correction
        else if (tree%nodes(i)%kind==alignment) then
            call check(rt,module_invalid)
        else
            v=geometry_correction(rt,tree%nodes(i),t)
        end if
    end function
    recursive function path_attributes(rt,tree,i,t,eval) result(a)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t
        type(evaluation), intent(in) :: eval
        type(attributes) :: a,outer
        integer :: j
        real(c_float) :: u
        a=empty_attributes()
        if (rt%error/=module_ok) return
        if (tree%nodes(i)%kind/=composite) then
            a=tree%nodes(i)%attrs; return
        end if
        call select_segment(rt,tree,i,t,eval,j,u)
        if (rt%error/=module_ok) return
        outer=tree%nodes(j)%attrs; a=path_attributes(rt,tree,j,u,eval)
        if (outer%have_roll) then
            a%have_roll=.true.; a%roll_ref=outer%roll_ref
        end if
        if (outer%have_orient) then
            a%have_orient=.true.; a%set_orient=outer%set_orient
        end if
        if (outer%have_boost .and. outer%thr_boost) then
            a%have_boost=.true.; a%thr_boost=.true.
        end if
        if (outer%have_corr) then
            a%have_corr=.true.; a%pos_corr=outer%pos_corr
        end if
        if (outer%have_gain) then
            a%have_gain=.true.; a%pos_gain_mul=outer%pos_gain_mul
        end if
        if (outer%have_shift .and. outer%shift_xy) then
            a%have_shift=.true.; a%shift_xy=.true.
        end if
    end function
    recursive real(c_float) function next_segment_start(rt,tree,i,t,eval) result(v)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        real(c_float), intent(in) :: t
        type(evaluation), intent(in) :: eval
        integer :: j
        real(c_float) :: u,fraction
        v=1.0
        if (rt%error/=module_ok .or. tree%nodes(i)%kind/=composite) return
        call select_segment(rt,tree,i,t,eval,j,u)
        if (rt%error/=module_ok) return
        fraction=tree%nodes(j)%length/tree%nodes(i)%length
        if (tree%nodes(j)%kind==composite) then
            v=next_segment_start(rt,tree,j,u,eval)
            v=tree%nodes(j)%start_time+v*fraction
        else
            v=tree%nodes(j)%start_time+fraction
        end if
    end function
    subroutine extents(rt,tree,i,eval,low,high)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: i
        type(evaluation), intent(in) :: eval
        real(c_float), intent(out) :: low,high
        real(c_float) :: t
        type(vector3f) :: p
        if (tree%nodes(i)%have_extents) then
            low=tree%nodes(i)%low; high=tree%nodes(i)%high; return
        end if
        p=position(rt,tree,i,0.0,eval); low=p%x; high=p%x; t=0.0
        do while (t<=1.0 .and. rt%error==module_ok)
            p=position(rt,tree,i,t,eval); low=lua_min(low,p%x); high=lua_max(high,p%x); t=t+0.02
        end do
        tree%nodes(i)%low=low; tree%nodes(i)%high=high; tree%nodes(i)%have_extents=.true.
    end subroutine
    subroutine compose(rt,tree,parent,width,eval)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent
        real(c_float), intent(in) :: width
        type(evaluation), intent(in) :: eval
        type(quaternion) :: orientation,q,tempq
        type(vector3f) :: pos,v,w
        real(c_float) :: angle,corr,total,low,high,d,start_x,time,fraction,yaw
        integer :: i,next
        if (rt%error/=module_ok) return
        if (.not.finite(width) .or. .not.finite(eval%speed) .or. eval%speed<=0.0 .or. &
            .not.finite(eval%entry_rate) .or. eval%entry_rate<=0.0) then
            call check(rt,module_invalid); return
        end if
        orientation=quaternion(1.0,0.0,0.0,0.0); pos=vector3f(0.0,0.0,0.0)
        angle=0.0; corr=0.0; total=0.0; i=tree%nodes(parent)%first
        if (i==0) then
            call check(rt,module_invalid); return
        end if
        do while(i/=0 .and. rt%error==module_ok)
            next=tree%nodes(i)%next
            if (tree%nodes(i)%kind==alignment) then
                if (next==0) then
                    call check(rt,module_invalid); return
                end if
                call extents(rt,tree,next,eval,low,high)
                yaw=deg(qyaw(rt,orientation)); start_x=-pos%x
                if (abs(yaw)>90.0) start_x=pos%x
                if (tree%nodes(i)%align_present) then
                    d=(width/2.0)*tree%nodes(i)%radius+start_x-high
                else
                    d=start_x-(low+high)*0.5
                end if
                d=lua_max(d,0.01); tree%nodes(i)%kind=straight; tree%nodes(i)%distance=d
                tree%nodes(i)%length=d; tree%nodes(i)%rebuild=.false.
            end if
            if (.not.finite(tree%nodes(i)%length) .or. tree%nodes(i)%length<0.0) then
                call check(rt,module_invalid); return
            end if
            tree%nodes(i)%start_pos=pos; tree%nodes(i)%start_orientation=orientation
            tree%nodes(i)%start_angle=angle; tree%nodes(i)%start_correction=corr
            total=total+tree%nodes(i)%length
            v=position(rt,tree,i,1.0,eval); w=rotate(rt,orientation,v); pos=vadd(pos,w)
            tempq=qmul(rt,tree%nodes(i)%final_orientation,orientation); orientation=tempq
            call normalize_q(rt,orientation)
            angle=angle+path_roll(rt,tree,i,1.0,tree%nodes(i)%length/eval%speed,eval)
            corr=corr+correction(rt,tree,i,1.0,eval)
            if (tree%nodes(i)%attrs%have_orient) orientation=tree%nodes(i)%attrs%set_orient
            if (tree%nodes(i)%attrs%have_roll) then
                q=axisq(rt,vector3f(1.0,0.0,0.0),rad(tree%nodes(i)%attrs%roll_ref))
                tempq=qmul(rt,orientation,q); orientation=tempq; call normalize_q(rt,orientation)
            end if
            i=next
        end do
        if (.not.finite(total) .or. total<=0.0) call check(rt,module_invalid)
        if (tree%nodes(tree%nodes(parent)%last)%length<=0.0) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        tree%nodes(parent)%length=total
        q=axisq(rt,vector3f(1.0,0.0,0.0),rad(wrap180(rt,angle)))
        tree%nodes(parent)%final_orientation=qmul(rt,q,orientation)
        time=0.0; i=tree%nodes(parent)%first
        do while (i/=0)
            tree%nodes(i)%start_time=time; fraction=tree%nodes(i)%length/total
            time=time+fraction; i=tree%nodes(i)%next
        end do
        tree%nodes(parent)%last_index=tree%nodes(parent)%last; tree%nodes(parent)%highest=0
    end subroutine
    integer function add_leaf(rt,tree,parent,kind,r,a,h,roll,roll_kind) result(i)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent,kind
        real(c_float), intent(in) :: r,a,h,roll
        integer, intent(in), optional :: roll_kind
        i=new_node(rt,tree,kind)
        if (i==0) return
        tree%nodes(i)%radius=r; tree%nodes(i)%angle=a; tree%nodes(i)%height=h
        tree%nodes(i)%distance=r; tree%nodes(i)%spirals=a
        tree%nodes(i)%roll_value=roll
        if (present(roll_kind)) tree%nodes(i)%roll_kind=roll_kind
        call finish_leaf(rt,tree%nodes(i)); call append_node(tree,parent,i)
    end function
    subroutine add_straight(rt,tree,parent,d,roll)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent
        real(c_float), intent(in) :: d,roll
        integer :: i
        i=add_leaf(rt,tree,parent,straight,d,0.0,0.0,roll)
    end subroutine
    subroutine add_vertical(rt,tree,parent,r,a)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent
        real(c_float), intent(in) :: r,a
        integer :: i
        i=add_leaf(rt,tree,parent,vertical,r,a,0.0,0.0)
    end subroutine
    subroutine add_horizontal(rt,tree,parent,r,a,bank)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: parent
        real(c_float), intent(in) :: r,a,bank
        integer :: i
        i=add_leaf(rt,tree,parent,horizontal,r,a,0.0,bank,roll_entry_exit)
    end subroutine
    integer function command_named(name) result(cmd)
        character(len=*), intent(in) :: name
        cmd=0
        if (same_text(name,'loop')) then
            cmd=cmd_loop
        else if (same_text(name,'half_reverse_cuban_eight')) then
            cmd=cmd_reverse_cuban
        else if (same_text(name,'scale_figure_eight')) then
            cmd=cmd_figure8
        else if (same_text(name,'immelmann_turn')) then
            cmd=cmd_immelmann
        else if (same_text(name,'straight_roll')) then
            cmd=cmd_roll
        else if (same_text(name,'split_s')) then
            cmd=cmd_split
        else if (same_text(name,'rolling_circle')) then
            cmd=cmd_circle
        else if (same_text(name,'humpty_bump')) then
            cmd=cmd_humpty
        else if (same_text(name,'half_cuban_eight')) then
            cmd=cmd_cuban
        else if (same_text(name,'barrel_roll')) then
            cmd=cmd_barrel
        else if (same_text(name,'crossbox_tophat')) then
            cmd=cmd_tophat
        else if (same_text(name,'triangular_loop')) then
            cmd=cmd_triangle
        else if (same_text(name,'align_box')) then
            cmd=cmd_align_box
        else if (same_text(name,'align_center')) then
            cmd=cmd_align_center
        end if
    end function
    integer function builtin(id,name) result(cmd)
        integer, intent(in) :: id
        character(len=*), intent(out) :: name
        cmd=0; name=''
        select case(id)
        case(2); cmd=cmd_loop; name='Loop'
        case(7); cmd=cmd_roll; name='Axial Roll'
        case(8); cmd=cmd_circle; name='Rolling Circle'
        case(9); cmd=cmd_cuban; name='Half Cuban Eight'
        case(10); cmd=cmd_reverse_cuban; name='Half Reverse Cuban Eight'
        case(12); cmd=cmd_humpty; name='Humpty Bump'
        case(14); cmd=cmd_figure8; name='Scale Figure Eight'
        case(15); cmd=cmd_immelmann; name='Immelmann Turn'
        case(16); cmd=cmd_split; name='Split-S'
        case(26); cmd=cmd_barrel; name='Barrel Roll'
        end select
    end function
    integer function command_nodes(cmd) result(n)
        integer, intent(in) :: cmd
        select case(cmd)
        case(cmd_loop,cmd_roll,cmd_circle); n=2
        case(cmd_reverse_cuban,cmd_figure8); n=6
        case(cmd_cuban); n=7
        case(cmd_immelmann,cmd_split); n=3
        case(cmd_humpty); n=9
        case(cmd_barrel); n=4
        case(cmd_tophat); n=8
        case(cmd_triangle); n=19
        case(cmd_align_box,cmd_align_center); n=1
        case default; n=0
        end select
    end function
    integer function build_maneuver(rt,tree,cmd,args,width,eval) result(parent)
        type(runtime), intent(inout) :: rt
        type(path_tree), intent(inout) :: tree
        integer, intent(in) :: cmd
        real(c_float), intent(in) :: args(4),width
        type(evaluation), intent(in) :: eval
        real(c_float) :: r,a,arg2,arg3,loops,gamma,bank,r2,h3,side,base,corner,root2,co
        integer :: i,k
        parent=0
        do k=1,4
            if (.not.finite(args(k))) call check(rt,module_invalid)
        end do
        if (.not.finite(eval%speed) .or. eval%speed<=0.0 .or. &
            .not.finite(eval%entry_rate) .or. eval%entry_rate<=0.0) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        r=args(1); a=abs(r); arg2=args(2); arg3=args(3)
        if (cmd==cmd_align_box .or. cmd==cmd_align_center) then
            parent=new_node(rt,tree,alignment)
            if (parent/=0) then
                tree%nodes(parent)%radius=r; tree%nodes(parent)%align_present=cmd==cmd_align_box
            end if
            return
        end if
        parent=new_node(rt,tree,composite)
        if (parent==0) return
        select case(cmd)
        case(cmd_loop)
            loops=1.0
            if (arg3>0.0) loops=arg3
            i=add_leaf(rt,tree,parent,vertical,r,360.0*loops,0.0,arg2,roll_entry_exit)
        case(cmd_reverse_cuban)
            call add_vertical(rt,tree,parent,r,45.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,0.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,180.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,0.0)
            call add_vertical(rt,tree,parent,-r,225.0)
        case(cmd_cuban)
            root2=fsqrt(rt,2.0); call add_straight(rt,tree,parent,2.0*a*root2,0.0)
            call add_vertical(rt,tree,parent,r,225.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,0.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,180.0)
            call add_straight(rt,tree,parent,2.0*a/3.0,0.0)
            call add_vertical(rt,tree,parent,-r,45.0)
        case(cmd_figure8)
            call add_straight(rt,tree,parent,a,0.0)
            call add_horizontal(rt,tree,parent,r,90.0,arg2)
            call add_horizontal(rt,tree,parent,-r,360.0,-arg2)
            call add_horizontal(rt,tree,parent,r,270.0,arg2)
            call add_straight(rt,tree,parent,3.0*a,0.0)
        case(cmd_immelmann)
            call add_vertical(rt,tree,parent,r,180.0); call add_straight(rt,tree,parent,a,180.0)
        case(cmd_roll)
            call add_straight(rt,tree,parent,r,arg2*360.0)
        case(cmd_split)
            call add_straight(rt,tree,parent,a/2.0,180.0); call add_vertical(rt,tree,parent,-r,180.0)
        case(cmd_circle)
            i=add_leaf(rt,tree,parent,horizontal,r,360.0,0.0,360.0*arg2)
            if (i/=0) then
                tree%nodes(i)%attrs%have_boost=.true.; tree%nodes(i)%attrs%thr_boost=.true.
            end if
        case(cmd_humpty)
            if (arg2<2.0*r) then
                call check(rt,module_invalid); return
            end if
            call add_vertical(rt,tree,parent,r,90.0)
            call add_straight(rt,tree,parent,(arg2-2.0*a)/3.0,0.0)
            call add_straight(rt,tree,parent,(arg2-2.0*a)/3.0,180.0)
            call add_straight(rt,tree,parent,(arg2-2.0*a)/3.0,0.0)
            call add_vertical(rt,tree,parent,-r,180.0)
            call add_straight(rt,tree,parent,arg2-2.0*a,0.0)
            call add_vertical(rt,tree,parent,-r,90.0)
            call add_straight(rt,tree,parent,2.0*a,0.0)
        case(cmd_barrel)
            if (r==0.0 .or. arg3<=0.0) then
                call check(rt,module_invalid); return
            end if
            gamma=deg(atan2f(rt,(arg2/arg3)/(2.0*pi*a),1.0)); bank=barrel_bank(rt,r,eval%speed)
            co=fcos(rt,rad(90.0-gamma)); r2=r/(1.0-co)
            call add_horizontal(rt,tree,parent,-r2,90.0-gamma,-bank)
            i=new_node(rt,tree,cylinder)
            if (i/=0) then
                tree%nodes(i)%radius=r; tree%nodes(i)%distance=arg2; tree%nodes(i)%spirals=arg3
                call finish_leaf(rt,tree%nodes(i)); call append_node(tree,parent,i)
            end if
            call add_horizontal(rt,tree,parent,r2,90.0-gamma,bank)
            tree%nodes(parent)%barrel=.true.; tree%nodes(parent)%barrel_radius=r
        case(cmd_tophat)
            call add_vertical(rt,tree,parent,r,90.0)
            i=add_leaf(rt,tree,parent,straight,arg2-2.0*r,0.0,0.0,90.0,roll_sequence)
            if (i/=0) then
                tree%nodes(i)%attrs%have_orient=.true.
                tree%nodes(i)%attrs%set_orient=orient(rt,0.0,90.0,90.0)
            end if
            i=add_leaf(rt,tree,parent,vertical,-r,90.0,0.0,0.0)
            if (i/=0) then
                tree%nodes(i)%attrs%have_orient=.true.
                tree%nodes(i)%attrs%set_orient=orient(rt,180.0,0.0,90.0)
            end if
            call add_straight(rt,tree,parent,arg3-2.0*r,0.0)
            i=add_leaf(rt,tree,parent,vertical,r,90.0,0.0,0.0)
            if (i/=0) then
                tree%nodes(i)%attrs%have_orient=.true.
                tree%nodes(i)%attrs%set_orient=orient(rt,0.0,-90.0,90.0)
            end if
            i=add_leaf(rt,tree,parent,straight,arg2-2.0*r,0.0,0.0,-90.0,roll_sequence)
            if (i/=0) then
                tree%nodes(i)%attrs%have_orient=.true.
                tree%nodes(i)%attrs%set_orient=orient(rt,0.0,-90.0,180.0)
            end if
            i=add_leaf(rt,tree,parent,vertical,r,90.0,0.0,0.0)
            if (i/=0) then
                tree%nodes(i)%attrs%have_orient=.true.
                tree%nodes(i)%attrs%set_orient=orient(rt,0.0,0.0,180.0)
            end if
        case(cmd_triangle)
            h3=arg2-2.0*r; co=fcos(rt,rad(45.0)); side=h3/co; base=2.0*(h3+r)-2.0*r
            call add_straight(rt,tree,parent,base*1.0/5.0,180.0)
            call add_straight(rt,tree,parent,base*2.0/5.0,0.0); call add_vertical(rt,tree,parent,r,135.0)
            do k=1,2
                corner=90.0
                if (k==2) corner=135.0
                call add_straight(rt,tree,parent,side*2.0/9.0,0.0)
                call add_straight(rt,tree,parent,side*2.0/9.0,90.0)
                call add_straight(rt,tree,parent,side*1.0/9.0,0.0)
                call add_straight(rt,tree,parent,side*2.0/9.0,90.0)
                call add_straight(rt,tree,parent,side*2.0/9.0,0.0); call add_vertical(rt,tree,parent,r,corner)
            end do
            call add_straight(rt,tree,parent,base*2.0/5.0,0.0)
            call add_straight(rt,tree,parent,base*1.0/5.0,180.0); call add_straight(rt,tree,parent,base*2.0/5.0,0.0)
        case default
            call check(rt,module_invalid)
        end select
        call compose(rt,tree,parent,width,eval)
    end function
end module aerobatics_path
