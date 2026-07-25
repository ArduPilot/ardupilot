module MOD1
end module MOD1

 module mod2
 use mod1
    integer :: mod2_int

    ! FIXME:
    ! replace the following line with the commented version, and recompile.
    ! mod/uses_two_mods.f90 should be recompiled, but currently isn't.

    integer, parameter :: mod2_param = 5
    ! integer, parameter :: mod2_param = 6

    interface mod2_proc
        module procedure mod2_proc1, mod2_proc2
    end interface

    contains

    subroutine mod2_proc1(a)
        implicit none
        integer, intent(inout) :: a
        a = 10
    end subroutine

    subroutine mod2_proc2(a)
        implicit none
        real, intent(inout) :: a
        a = 10.0
    end subroutine

 end module mod2

