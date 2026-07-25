

! FIXME:
! modifying two_mods.f90 should trigger this file's recompilation, too.

module uses_TWO_mods
      use mod2
      implicit none

      integer, parameter :: uses_two_mods_param = mod2_param * 2

      contains

      subroutine printer
        implicit none

        print *, uses_two_mods_param

      end subroutine printer

end module uses_TWO_mods
