! SPDX-License-Identifier: GPL-3.0-or-later
#ifndef MODULE_PROBE_TEXT
#define MODULE_PROBE_TEXT "ready"
#endif
function module_run(context) bind(C, name="ap_module_fortran_run") result(status)
    use, intrinsic :: iso_c_binding
    use ap_module
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    real(c_float) :: vector(3), magnitude, seconds

    vector = [3.0_c_float, 4.0_c_float, 12.0_c_float]
    magnitude = sqrt(sum(vector * vector))
    status = module_invalid
    if (magnitude /= 13.0_c_float) return
    seconds = real(time_us(context), c_float) * 1.0e-6_c_float
    status = send_text_floats(context, "Fortran check %.1f time %.1f", [magnitude, seconds])
    if (status /= module_ok) return
    status = send_text(context, MODULE_PROBE_TEXT)
    if (status /= module_ok) return
    do while (.not. stop_requested(context))
        status = sleep_ms(context, 1000_c_int32_t)
        if (status == module_stopped) exit
        if (status /= module_ok) return
    end do
    status = module_ok
end function
