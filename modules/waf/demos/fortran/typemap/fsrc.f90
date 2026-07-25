module example_mod
    use basetypes
    implicit none

    ! uses the kind defined in basetypes.
    integer(kind=int_kind) :: an_int

    ! Useless example to demonstrate that arg_kind is set to a compile-time
    ! expression; the expression could be arbitrary, and would yield different
    ! values for different compilers.
    integer, parameter :: arg_kind = kind(an_int)

    contains

    subroutine sub1(a, b, c, d)
        implicit none
        integer(kind=arg_kind), intent(inout) :: a,b,c,d

        a = 1
        b = 2
        c = 3
        d = 4
    end subroutine sub1

end module example_mod
