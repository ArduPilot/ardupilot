! SPDX-License-Identifier: GPL-3.0-or-later
! Core context operations and formatting helpers. Generated host-call clients
! and types are in ap_module_bindings, built from the shared binding plans.
module ap_module
    use, intrinsic :: iso_c_binding
    implicit none
    private
    integer(c_int32_t), parameter, public :: module_ok = 0, module_stopped = 1
    integer(c_int32_t), parameter, public :: module_invalid = -1
    integer(c_int32_t), parameter, public :: severity_info = 6
    public :: time_us, sleep_ms, stop_requested, get_gyro, send_text, send_text_floats

    ! The opaque context borrows the worker's entry frame. Do not retain it
    ! after module_run returns or use it from another thread.
    interface
        function time_us(context) bind(C, name="ap_module_fortran_time_us") result(value)
            import :: c_ptr, c_int64_t
            type(c_ptr), value :: context
            integer(c_int64_t) :: value
        end function
        function sleep_ms(context, milliseconds) bind(C, name="ap_module_fortran_sleep_ms") result(status)
            import :: c_ptr, c_int32_t
            type(c_ptr), value :: context
            integer(c_int32_t), value :: milliseconds
            integer(c_int32_t) :: status
        end function
        function stop_requested(context) bind(C, name="ap_module_fortran_stop_requested") result(value)
            import :: c_ptr, c_bool
            type(c_ptr), value :: context
            logical(c_bool) :: value
        end function
        function get_gyro(context, gyro) bind(C, name="ap_module_fortran_get_gyro") result(status)
            import :: c_ptr, c_float, c_int32_t
            type(c_ptr), value :: context
            real(c_float), intent(out) :: gyro(3)
            integer(c_int32_t) :: status
        end function
        function raw_text(context, severity, message, length) bind(C, name="ap_module_fortran_text") result(status)
            import :: c_ptr, c_int32_t, c_char
            type(c_ptr), value :: context
            integer(c_int32_t), value :: severity, length
            character(kind=c_char), intent(in) :: message(*)
            integer(c_int32_t) :: status
        end function
        function raw_format(context, severity, format, length, values, count) &
                bind(C, name="ap_module_fortran_text_floats") result(status)
            import :: c_ptr, c_int32_t, c_char, c_float
            type(c_ptr), value :: context
            integer(c_int32_t), value :: severity, length, count
            character(kind=c_char), intent(in) :: format(*)
            real(c_float), intent(in) :: values(*)
            integer(c_int32_t) :: status
        end function
    end interface
contains
    function send_text(context, message) result(status)
        type(c_ptr), value :: context
        character(kind=c_char, len=*), intent(in) :: message
        integer(c_int32_t) :: status
        status = raw_text(context, severity_info, message, len(message, kind=c_int32_t))
    end function

    function send_text_floats(context, format, values) result(status)
        type(c_ptr), value :: context
        character(kind=c_char, len=*), intent(in) :: format
        real(c_float), contiguous, intent(in) :: values(:)
        integer(c_int32_t) :: status
        status = raw_format(context, severity_info, format, len(format, kind=c_int32_t), &
                            values, size(values, kind=c_int32_t))
    end function
end module
