! SPDX-License-Identifier: GPL-3.0-or-later
! Generated clients exercised against real SITL and H7 ModuleMath firmware.
function run(ctx) bind(C, name="ap_module_fortran_run") result(status)
    use ap_module, only: send_text
    use ap_module_bindings
    implicit none
    type(c_ptr), value :: ctx
    integer(c_int32_t) :: status
    type(vector3f) :: vector, gyro
    real(c_float) :: magnitude
    type(array_uint8_t_256) :: path
    type(optional_filesystem_statoutput) :: info
    type(optional_filesystem_read_atoutput) :: bytes
    character(len=19) :: filename
    character(len=26) :: message
    integer :: i

    vector = vector3f(3.0_c_float, 4.0_c_float, 12.0_c_float)
    status = vector3f_length(ctx, vector, magnitude)
    if (status /= module_ok .or. magnitude /= 13.0_c_float) goto 900
    status = vector3f_normalize(ctx, vector)
    if (status /= module_ok) goto 900
    status = vector3f_length(ctx, vector, magnitude)
    if (status /= module_ok .or. abs(magnitude - 1.0_c_float) > 0.00001_c_float) goto 900
    status = libm_sqrtf(ctx, 169.0_c_float, magnitude)
    if (status /= module_ok .or. magnitude /= 13.0_c_float) goto 900
    gyro = vector3f(99.0_c_float, 99.0_c_float, 99.0_c_float)
    status = ahrs_get_gyro(ctx, gyro)
    if (status == module_ok) then
        status = gcs_send_text(ctx, mavseverity_info, "Fortran AHRS checked")
        if (status /= module_ok .and. status /= module_unavailable) goto 900
    else
        if (status /= module_unavailable) goto 900
        if (gyro%x /= 0 .or. gyro%y /= 0 .or. gyro%z /= 0) goto 900
    end if
    filename = "fortran-fixture.bin"
    path%data = 0_c_int8_t
    do i = 1, len(filename)
        path%data(i - 1) = iachar(filename(i:i), kind=c_int8_t)
    end do
    status = filesystem_stat(ctx, path, info)
    if (status /= module_ok .or. .not. info%present) goto 900
    if (info%value%size /= 3 .or. info%value%directory) goto 900
    status = filesystem_read_at(ctx, path, 1_c_int32_t, 2_c_int16_t, bytes)
    if (status /= module_ok .or. .not. bytes%present) goto 900
    if (bytes%value%length /= 2) goto 900
    if (bytes%value%data%data(0) /= iachar('b') .or. bytes%value%data%data(1) /= iachar('c')) goto 900
    path%data(0) = iachar('!')
    status = filesystem_stat(ctx, path, info)
    if (status /= module_ok .or. info%present .or. info%value%size /= 0) goto 900
    message = "XFortran bindings checkedY"
    status = gcs_send_text(ctx, mavseverity_info, message(2:25))
    if (status /= module_ok .and. status /= module_unavailable) goto 900
    status = send_text(ctx, "MATH ok")
    return
900 status = module_host_error
end function
