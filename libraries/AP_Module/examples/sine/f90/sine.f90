! SPDX-License-Identifier: GPL-3.0-or-later
! AI-assisted sine demo sharing SINE_FREQ with Lua, Rust and Ada.
function module_run(context) bind(C, name="ap_module_fortran_run") result(status)
    use ap_module, only: time_us, sleep_ms
    use ap_module_bindings
    implicit none
    type(c_ptr), value :: context
    integer(c_int32_t) :: status
    real(c_float), parameter :: pi = 3.141592653589793_c_float

    do
        status = register_parameters()
        if (status /= module_busy) exit
        status = sleep_ms(context, 10_c_int32_t)
        if (status /= module_ok) return
    end do
    if (status /= module_ok) return
    do
        status = sample()
        if (status /= module_ok .and. status /= module_busy) return
        status = sleep_ms(context, 100_c_int32_t)
        if (status /= module_ok) return
    end do
contains
    function register_parameters() result(code)
        integer(c_int32_t) :: code
        logical(c_bool) :: added
        type(optional_float) :: frequency
        code = param_get(context, "SINE_FREQ", frequency)
        if (code /= module_ok) return
        if (frequency%present) return
        code = param_add_table(context, 120_c_int8_t, "SINE_", 1_c_int8_t, added)
        if (code /= module_ok) return
        if (.not. added) then
            ! Lua may have created its table but not yet published the entry.
            code = module_busy
            return
        end if
        code = param_add_param(context, 120_c_int8_t, 1_c_int8_t, "FREQ", 0.2_c_float, added)
        if (code == module_ok .and. .not. added) code = module_host_error
    end function

    function sample() result(code)
        integer(c_int32_t) :: code
        type(optional_float) :: frequency
        real(c_float) :: seconds, value
        code = param_get(context, "SINE_FREQ", frequency)
        if (code /= module_ok) return
        if (.not. frequency%present) then
            code = module_invalid
            return
        end if
        seconds = real(iand(time_us(context) / 1000, 4294967295_c_int64_t), c_float) * 0.001_c_float
        code = libm_sinf(context, 2.0_c_float * pi * frequency%value * seconds + 1.5_c_float * pi, value)
        if (code /= module_ok) return
        code = gcs_send_named_float(context, "Sine_f90", value)
    end function
end function
