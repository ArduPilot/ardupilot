! SPDX-License-Identifier: GPL-3.0-or-later
! AI-assisted translation of the adjacent Rust implementation.
module aerobatics_support
    use ap_module_bindings
    use ap_module, only: time_us, sleep_ms
    implicit none
    real(c_float), parameter :: pi = 3.14159265358979323846_c_float
    type :: runtime
        type(c_ptr) :: ctx
        integer(c_int32_t) :: error
    end type
    interface
        function arena_allocate(bytes) bind(C, name='aerobatics_allocate') result(p)
            import
            integer(c_size_t), value :: bytes
            type(c_ptr) :: p
        end function
        subroutine arena_free(p) bind(C, name='aerobatics_free')
            import
            type(c_ptr), value :: p
        end subroutine
    end interface
contains
    subroutine check(rt, code)
        type(runtime), intent(inout) :: rt
        integer(c_int32_t), intent(in) :: code
        if (rt%error == module_ok) rt%error = code
    end subroutine
    logical function finite(x)
        real(c_float), intent(in) :: x
        finite = abs(x) <= huge(x)
    end function
    logical function finite_v(v)
        type(vector3f), intent(in) :: v
        finite_v = finite(v%x) .and. finite(v%y) .and. finite(v%z)
    end function
    logical function nan_v(v)
        type(vector3f), intent(in) :: v
        nan_v = v%x /= v%x .or. v%y /= v%y .or. v%z /= v%z
    end function
    real(c_float) function rad(x)
        real(c_float), intent(in) :: x
        rad = x * (pi / 180.0)
    end function
    real(c_float) function deg(x)
        real(c_float), intent(in) :: x
        deg = x * (180.0 / pi)
    end function
    real(c_float) function lua_min(a,b) result(v)
        real(c_float), intent(in) :: a,b
        v=a
        if (b<a) v=b
    end function
    real(c_float) function lua_max(a,b) result(v)
        real(c_float), intent(in) :: a,b
        v=a
        if (b>a) v=b
    end function
    real(c_float) function constrain(x, low, high) result(v)
        real(c_float), intent(in) :: x,low,high
        v=x
        if (v<low) v=low
        if (v>high) v=high
    end function
    real(c_float) function sgn(x) result(v)
        real(c_float), intent(in) :: x
        v=0.0
        if (x>0.000001) v=1.0
        if (x<0.000001) v=-1.0
    end function
    integer function text_length(text) result(n)
        character(len=*), intent(in) :: text
        integer :: k
        n=0
        do k=1,len(text)
            if (iachar(text(k:k))/=32) n=k
        end do
    end function
    logical function same_text(a,b) result(equal)
        character(len=*), intent(in) :: a,b
        integer :: k
        equal=.false.
        if (len(a)/=len(b)) return
        do k=1,len(a)
            if (iachar(a(k:k))/=iachar(b(k:k))) return
        end do
        equal=.true.
    end function
    integer function find_char(text,ch) result(p)
        character(len=*), intent(in) :: text
        character, intent(in) :: ch
        integer :: k
        p=0
        do k=1,len(text)
            if (text(k:k)==ch) then
                p=k; return
            end if
        end do
    end function
    integer(c_int32_t) function rust_i32(x) result(n)
        real(c_float), intent(in) :: x
        n=0
        if (x/=x) return
        if (x>=2147483648.0_c_float) then
            n=huge(n)
        else if (x<=-2147483648.0_c_float) then
            n=-huge(n)-1
        else
            n=int(x,c_int32_t)
        end if
    end function
    integer(c_int64_t) function rust_u32(x) result(n)
        real(c_float), intent(in) :: x
        n=0
        if (x/=x.or.x<=0.0) return
        if (x>=4294967296.0_c_float) then
            n=4294967295_c_int64_t
        else
            n=int(x,c_int64_t)
        end if
    end function
    function error_name(code) result(text)
        integer(c_int32_t), intent(in) :: code
        character(len=12) :: text
        text='Unknown'
        select case(code)
        case(module_ok); text='Ok'
        case(module_stopped); text='Stopped'
        case(module_invalid); text='Invalid'
        case(module_no_memory); text='NoMemory'
        case(module_unavailable); text='Unavailable'
        case(module_host_error); text='Host'
        case(module_busy); text='Busy'
        end select
    end function
    function decimal(n) result(text)
        integer, intent(in) :: n
        character(len=12) :: text
        integer :: k,j
        integer(c_int64_t) :: v
        character(len=12) :: reverse
        v=abs(int(n,c_int64_t)); k=0; text=''; reverse=''
        do
            k=k+1; reverse(k:k)=achar(iachar('0')+int(mod(v,10_c_int64_t))); v=v/10
            if (v==0) exit
        end do
        j=0
        if (n<0) then
            j=1; text(1:1)='-'
        end if
        do v=k,1,-1
            j=j+1; text(j:j)=reverse(v:v)
        end do
    end function
    function joined(a,b,c) result(text)
        character(len=*), intent(in) :: a,b
        character(len=*), intent(in), optional :: c
        character(len=96) :: text
        integer :: na,nb,nc
        text=''; na=len(a); nb=len(b); nc=0
        if (present(c)) nc=len(c)
        if (na+nb+nc>len(text)) return
        text(1:na)=a; text(na+1:na+nb)=b
        if (present(c)) text(na+nb+1:na+nb+nc)=c
    end function
    subroutine message(rt,severity,text)
        type(runtime), intent(inout) :: rt
        integer(c_int32_t), intent(in) :: severity
        character(len=*), intent(in) :: text
        integer(c_int32_t) :: code
        if (rt%error==module_stopped) return
        code=gcs_send_text(rt%ctx,severity,text(1:text_length(text)))
        if (code==module_stopped) rt%error=code
    end subroutine
    logical function retry_busy(rt,code,start) result(again)
        type(runtime), intent(inout) :: rt
        integer(c_int32_t), intent(in) :: code
        integer(c_int64_t), intent(in) :: start
        integer(c_int32_t) :: status
        again=.false.
        if (code==module_busy .and. max(0_c_int64_t,time_us(rt%ctx)-start)<100000) then
            status=sleep_ms(rt%ctx,1_c_int32_t)
            if (status==module_ok) then
                again=.true.
            else
                call check(rt,status)
            end if
        else
            call check(rt,code)
        end if
    end function
    function fsin(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=libm_sinf(rt%ctx,a,v)
        call check(rt,code)
    end function
    function fcos(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=libm_cosf(rt%ctx,a,v)
        call check(rt,code)
    end function
    function fsqrt(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=libm_sqrtf(rt%ctx,a,v)
        call check(rt,code)
    end function
    function atan2f(rt,a,b) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: a,b
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=libm_atan2f(rt%ctx,a,b,v)
        call check(rt,code)
    end function
    function fmodf(rt,a,b) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: a,b
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=libm_fmodf(rt%ctx,a,b,v)
        call check(rt,code)
    end function
    function vlength(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=vector3f_length(rt%ctx,a,v)
        call check(rt,code)
    end function
    function vdot(rt,a,b) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: a,b
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=vector3f_dot(rt%ctx,a,b,v)
        call check(rt,code)
    end function
    function vcross(rt,a,b) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: a,b
        type(vector3f) :: v
        integer(c_int32_t) :: code
        v=vector3f(0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        code=vector3f_cross(rt%ctx,a,b,v)
        call check(rt,code)
    end function
    function qmul(rt,a,b) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a,b
        type(quaternion) :: v
        integer(c_int32_t) :: code
        v=quaternion(1.0,0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        code=quaternion_mul(rt%ctx,a,b,v)
        call check(rt,code)
    end function
    function qinverse(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a
        type(quaternion) :: v
        integer(c_int32_t) :: code
        v=quaternion(1.0,0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        code=quaternion_inverse(rt%ctx,a,v)
        call check(rt,code)
    end function
    function qlength(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=quaternion_length(rt%ctx,a,v)
        call check(rt,code)
    end function
    function qpitch(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=quaternion_get_euler_pitch(rt%ctx,a,v)
        call check(rt,code)
    end function
    function qroll(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=quaternion_get_euler_roll(rt%ctx,a,v)
        call check(rt,code)
    end function
    function qyaw(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: a
        real(c_float) :: v
        integer(c_int32_t) :: code
        v=0.0
        if (rt%error/=module_ok) return
        code=quaternion_get_euler_yaw(rt%ctx,a,v)
        call check(rt,code)
    end function
    function vadd(a,b) result(v)
        type(vector3f), intent(in) :: a,b
        type(vector3f) :: v
        v=vector3f(a%x+b%x,a%y+b%y,a%z+b%z)
    end function
    function vsub(a,b) result(v)
        type(vector3f), intent(in) :: a,b
        type(vector3f) :: v
        v=vector3f(a%x-b%x,a%y-b%y,a%z-b%z)
    end function
    function vscale(a,s) result(v)
        type(vector3f), intent(in) :: a
        real(c_float), intent(in) :: s
        type(vector3f) :: v
        v=vector3f(a%x*s,a%y*s,a%z*s)
    end function
    subroutine normalize_q(rt,q)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(inout) :: q
        real(c_float) :: n
        integer(c_int32_t) :: code
        n=qlength(rt,q)
        if (.not.finite(n) .or. n<epsilon(n)) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        code=quaternion_normalize(rt%ctx,q); call check(rt,code)
    end subroutine
    function unit(rt,a) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: a
        type(vector3f) :: v
        real(c_float) :: n
        integer(c_int32_t) :: code
        v=a; n=vlength(rt,a)
        if (.not.finite(n) .or. n<=0.0) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        code=vector3f_normalize(rt%ctx,v); call check(rt,code)
    end function
    function rotate(rt,q,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: q
        type(vector3f), intent(in) :: a
        type(vector3f) :: v
        integer(c_int32_t) :: code
        v=a
        if (rt%error/=module_ok) return
        code=quaternion_earth_to_body(rt%ctx,q,v); call check(rt,code)
    end function
    function unrotate(rt,q,a) result(v)
        type(runtime), intent(inout) :: rt
        type(quaternion), intent(in) :: q
        type(vector3f), intent(in) :: a
        type(vector3f) :: v
        type(quaternion) :: inverse
        inverse=qinverse(rt,q); v=rotate(rt,inverse,a)
    end function
    function axisq(rt,axis,angle) result(q)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: axis
        real(c_float), intent(in) :: angle
        type(quaternion) :: q
        integer(c_int32_t) :: code
        q=quaternion(1.0,0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        code=quaternion_from_axis_angle(rt%ctx,q,axis,angle); call check(rt,code)
    end function
    function orient(rt,roll,pitch,yaw) result(q)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: roll,pitch,yaw
        type(quaternion) :: q
        integer(c_int32_t) :: code
        q=quaternion(1.0,0.0,0.0,0.0)
        if (rt%error/=module_ok) return
        code=quaternion_from_euler(rt%ctx,q,rad(roll),rad(pitch),rad(yaw)); call check(rt,code)
    end function
    function angular_velocity(rt,v,dt) result(q)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: v
        real(c_float), intent(in) :: dt
        type(quaternion) :: q
        real(c_float) :: x,y,z
        integer(c_int32_t) :: code
        q=quaternion(1.0,0.0,0.0,0.0)
        x=0.5*dt*v%x; y=0.5*dt*v%y; z=0.5*dt*v%z
        if (.not.finite(1.0+x*x+y*y+z*z)) call check(rt,module_invalid)
        if (rt%error/=module_ok) return
        code=quaternion_from_angular_velocity(rt%ctx,q,v,dt); call check(rt,code)
    end function
    function tangents_rate(rt,a,b,dt) result(v)
        type(runtime), intent(inout) :: rt
        type(vector3f), intent(in) :: a,b
        real(c_float), intent(in) :: dt
        type(vector3f) :: v,ua,ub,axis
        type(quaternion) :: q
        real(c_float) :: dot
        integer(c_int32_t) :: code
        v=vector3f(0.0,0.0,0.0)
        if (.not.finite(dt) .or. dt<=0.0) call check(rt,module_invalid)
        if (dt>0.0) then
            if (.not.finite(1.0/dt)) call check(rt,module_invalid)
        end if
        if (rt%error/=module_ok) return
        ua=unit(rt,a); ub=unit(rt,b); dot=vdot(rt,ua,ub); axis=vcross(rt,ua,ub)
        q=quaternion(1.0+dot,axis%x,axis%y,axis%z); call normalize_q(rt,q)
        if (rt%error/=module_ok) return
        code=quaternion_to_axis_angle(rt%ctx,q,v); call check(rt,code)
        v=vscale(v,1.0/dt)
    end function
    real(c_float) function wrap360(rt,x) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: x
        v=fmodf(rt,x,360.0)
        if (v<0.0) v=v+360.0
    end function
    real(c_float) function wrap180(rt,x) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: x
        v=wrap360(rt,x)
        if (v>180.0) v=v-360.0
    end function
    real(c_float) function wrappi(rt,x) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: x
        v=rad(wrap180(rt,deg(x)))
    end function
    real(c_float) function wrap2pi(rt,x) result(v)
        type(runtime), intent(inout) :: rt
        real(c_float), intent(in) :: x
        v=rad(wrap360(rt,deg(x)))
    end function
end module aerobatics_support
