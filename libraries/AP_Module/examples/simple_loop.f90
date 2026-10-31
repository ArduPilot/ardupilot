! SPDX-License-Identifier: GPL-3.0-or-later
! Runtime-free loop using generated bindings and the firmware formatter.
function module_run(context) bind(C, name="ap_module_fortran_run") result(status)
    use, intrinsic :: iso_c_binding
    use ap_module, only: time_us, sleep_ms, stop_requested, send_text_floats
    use ap_module_bindings
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    type(vector3f) :: gyro
    real(c_float) :: seconds, magnitude

    status = gcs_send_text(context, mavseverity_info, "Fortran ready")
    if (status /= module_ok) return
    do while (.not. stop_requested(context))
        status = ahrs_get_gyro(context, gyro)
        if (status /= module_ok) return
        seconds = real(time_us(context), c_float) * 1.0e-6_c_float
        magnitude = sqrt(gyro%x * gyro%x + gyro%y * gyro%y + gyro%z * gyro%z)
        status = send_text_floats(context, "Fortran tick %.1f gyro %.3f", [seconds, magnitude])
        if (status /= module_ok) return
        status = sleep_ms(context, 1000_c_int32_t)
        if (status == module_stopped) exit
        if (status /= module_ok) return
    end do
    status = module_ok
end function
