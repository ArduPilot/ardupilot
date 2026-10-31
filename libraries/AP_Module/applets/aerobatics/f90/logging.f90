! SPDX-License-Identifier: GPL-3.0-or-later
module aerobatics_logging
    use aerobatics_controller
    implicit none
contains
    subroutine init_record(record,name,labels,format)
        type(logrecord), intent(out) :: record
        character(len=*), intent(in) :: name,labels,format
        integer :: k
        record%name%data=0; record%labels%data=0; record%format%data=0
        record%units%data=0; record%multipliers%data=0; record%have_units=.false.
        record%payload%data=0; record%payload_len=0
        do k=1,len(name)
            record%name%data(k-1)=int(iachar(name(k:k)),c_int8_t)
        end do
        do k=1,len(labels)
            record%labels%data(k-1)=int(iachar(labels(k:k)),c_int8_t)
        end do
        do k=1,len(format)
            record%format%data(k-1)=int(iachar(format(k:k)),c_int8_t)
        end do
    end subroutine
    subroutine append_float(record,value)
        type(logrecord), intent(inout) :: record
        real(c_float), intent(in) :: value
        integer(c_int8_t) :: bytes(4)
        integer :: offset,k
        bytes=transfer(value,bytes); offset=int(record%payload_len)
        do k=1,4
            record%payload%data(offset+k-1)=bytes(k)
        end do
        record%payload_len=record%payload_len+4
    end subroutine
    subroutine append_int(record,value)
        type(logrecord), intent(inout) :: record
        integer(c_int32_t), intent(in) :: value
        integer(c_int8_t) :: bytes(4)
        integer :: offset,k
        bytes=transfer(value,bytes); offset=int(record%payload_len)
        do k=1,4
            record%payload%data(offset+k-1)=bytes(k)
        end do
        record%payload_len=record%payload_len+4
    end subroutine
    subroutine write_record(rt,record)
        type(runtime), intent(inout) :: rt
        type(logrecord), intent(in) :: record
        integer(c_int32_t) :: code
        if (rt%error/=module_ok) return
        code=logger_write(rt%ctx,record)
        if (code/=module_unavailable) call check(rt,code)
    end subroutine
    subroutine log_floats(rt,name,labels,values)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: name,labels
        real(c_float), intent(in) :: values(:)
        type(logrecord) :: record
        character(len=16) :: format
        integer :: k
        format=''
        do k=1,size(values)
            format(k:k)='f'
        end do
        call init_record(record,name,labels,format(1:size(values)))
        do k=1,size(values)
            call append_float(record,values(k))
        end do
        call write_record(rt,record)
    end subroutine
    subroutine log_pose(rt,name,pos,q)
        type(runtime), intent(inout) :: rt
        character(len=*), intent(in) :: name
        type(vector3f), intent(in) :: pos
        type(quaternion), intent(in) :: q
        type(optional_location) :: origin
        type(logrecord) :: record
        real(c_float) :: values(10)
        integer(c_int32_t) :: code
        integer :: k
        if (rt%error/=module_ok) return
        code=ahrs_get_origin(rt%ctx,origin); call check(rt,code)
        if (rt%error/=module_ok.or..not.origin%present) return
        code=location_offset(rt%ctx,origin%value,pos%x,pos%y); call check(rt,code)
        values=[pos%x,pos%y,pos%z,q%q1,q%q2,q%q3,q%q4,deg(qroll(rt,q)),deg(qpitch(rt,q)),deg(qyaw(rt,q))]
        call init_record(record,name,'px,py,pz,q1,q2,q3,q4,r,p,y,Lat,Lon','ffffffffffLL')
        do k=1,10
            call append_float(record,values(k))
        end do
        call append_int(record,origin%value%lat); call append_int(record,origin%value%lng)
        call write_record(rt,record)
    end subroutine
    subroutine log_output(rt,input,o)
        type(runtime), intent(inout) :: rt
        type(observation), intent(in) :: input
        type(control_output), intent(in) :: o
        if (o%have_lookahead) call log_floats(rt,'AELK','Py,Ly,Pz,Lz',o%lookahead_rates)
        call log_pose(rt,'POSM',input%pos,o%measured_attitude); call log_pose(rt,'POST',o%track_pos,o%track_attitude)
        call log_floats(rt,'AETM','T,Terr,QCt,Adt',[o%path_t,o%path_error_t,o%q_change_t,o%dt])
        call log_floats(rt,'AERT','Cx,Cy,Cz,Px,Py,Pz,Ex,Tx,Ty,Tz,Perr,Aerr,Yff,Rofs', &
            [o%correction_rates%x,o%correction_rates%y,o%correction_rates%z,o%path_rates%x,o%path_rates%y,o%path_rates%z, &
            o%roll_error_rate,o%rates%x,o%rates%y,o%rates%z,o%position_error,o%angle_error,0.0,o%rudder])
        call log_floats(rt,'AESP','Targ,Curr,P,I,Total,Add',o%speed_log)
    end subroutine
    subroutine log_timing(rt,elapsed,total,o)
        type(runtime), intent(inout) :: rt
        integer(c_int64_t), intent(in) :: elapsed,total
        type(control_output), intent(in) :: o
        type(logrecord) :: record
        integer :: flags
        integer(c_int64_t) :: us
        integer(c_int32_t) :: bits
        call init_record(record,'ARUN','Us,Total,Perr,T,Flags','IIffB')
        us=min(max(0_c_int64_t,elapsed),4294967295_c_int64_t)
        bits=int(iand(us,2147483647_c_int64_t),c_int32_t)
        if (btest(us,31)) bits=ibset(bits,31)
        call append_int(record,bits)
        us=min(max(0_c_int64_t,total),4294967295_c_int64_t)
        bits=int(iand(us,2147483647_c_int64_t),c_int32_t)
        if (btest(us,31)) bits=ibset(bits,31)
        call append_int(record,bits)
        call append_float(record,o%position_error); call append_float(record,o%path_t)
        flags=0
        if (o%path_rate_fallback) flags=flags+1
        if (o%lookahead_fallback) flags=flags+2
        record%payload%data(int(record%payload_len))=int(flags,c_int8_t); record%payload_len=record%payload_len+1
        call write_record(rt,record)
    end subroutine
end module aerobatics_logging
